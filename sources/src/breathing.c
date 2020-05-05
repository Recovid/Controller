#include "common.h"
#include <stdint.h>
#include <stdint.h>
#include <stdint.h>
#include <inttypes.h>
#include "controller.h"
#include "breathing.h"
#include "platform.h"
#include "platform_config.h"
#include "stdint.h"

#include <compute_motor.h>
#include <stdint.h>
#include <math.h>


static float EoI_ratio;       
static float FR_pm;           
static float VTe_mL; 
static float VTi_mL; 
static float VMe_Lpm; 
static float Pcrete_cmH2O; 
static float Pplat_cmH2O; 
static float PEP_cmH2O; 
static float PEP_cmH2O_samples[MAX_PEP_SAMPLES]; 
static unsigned int PEP_cmH2O_samples_index; 
static float Pplat_cmH2O_samples[MAX_PPLAT_SAMPLES]; 
static unsigned int Pplat_cmH2O_samples_index; 

#define NB_SLICE	(20)
static uint32_t Tslice; //Duration of a slice
static float error_slice_samples[NB_SLICE+2];
static uint32_t t_end_of_slice_us[NB_SLICE+2];

//Set point
static float set_point_per_slice[2][NB_SLICE];
static float* current_set_point_per_slice  = set_point_per_slice[0];
static float* previous_set_point_per_slice = set_point_per_slice[1];

//V per slice
static float V_per_slice[2][NB_SLICE];
static float* current_V_per_slice  = V_per_slice[0];
static float* previous_V_per_slice = V_per_slice[1];

//NB steps per slice
static uint32_t  nb_steps_per_slice[2][NB_SLICE];
static uint32_t* current_nb_steps_per_slices  = nb_steps_per_slice[0];
static uint32_t* previous_nb_steps_per_slices = nb_steps_per_slice[1];

//Nb total steps
static uint32_t current_nb_steps                = 0;
static uint32_t previous_nb_steps = 0;

static uint32_t  current_nb_slices  = 0;
static uint32_t  previous_nb_slices = 0;

static uint32_t current_slice_start_t_ms =0;

static void init_sample_PEP_cmH2O()
{
    //Samples PEP for a rolling average 
  PEP_cmH2O_samples_index = 0;
  for(int i = 0; i < MAX_PEP_SAMPLES; i++)
    PEP_cmH2O_samples[i] = 0;

}

static void sample_PEP_cmH2O( float Paw_cmH2O)
{
  PEP_cmH2O_samples[PEP_cmH2O_samples_index] = Paw_cmH2O;
  PEP_cmH2O_samples_index = (PEP_cmH2O_samples_index + 1) % MAX_PEP_SAMPLES; 
}

static float get_PEP_avg_cmH2O()
{
  float sum_PEP = 0;
  for(int i=0; i < MAX_PEP_SAMPLES; i++)
  {
    sum_PEP += PEP_cmH2O_samples[i];
  }
  return (sum_PEP / MAX_PEP_SAMPLES);
}

static void init_sample_Pplat_cmH2O()
{
    //Samples Pplat for a rolling average 
  Pplat_cmH2O_samples_index = 0;
  for(int i = 0; i < MAX_PPLAT_SAMPLES; i++)
    Pplat_cmH2O_samples[i] = 0;

}

static void sample_Pplat_cmH2O( float Paw_cmH2O)
{
  Pplat_cmH2O_samples[Pplat_cmH2O_samples_index] = Paw_cmH2O;
  Pplat_cmH2O_samples_index = (Pplat_cmH2O_samples_index + 1) % MAX_PPLAT_SAMPLES; 
}

static float get_Pplat_avg_cmH2O()
{
  float sum_Pplat = 0;
  for(int i=0; i < MAX_PPLAT_SAMPLES; i++)
  {
    sum_Pplat += Pplat_cmH2O_samples[i];
  }
  return (sum_Pplat / MAX_PPLAT_SAMPLES);
}
static float VTe_start_mL=0.;



typedef enum { Insufflation, Plateau, Exhalation, Finished } BreathingState;

static BreathingState _state; 

static uint32_t       _state_start_ms; 
static uint32_t       _cycle_start_ms;





static uint32_t t_motor_steps_us[MAX_MOTOR_STEPS] = {0};  // TODO: Make it configurable with a define. This represent a physical limit a the system.


// static float A_calibrated;
// static float B_calibrated;

#define MAX_FLOW_SAMPLES	        400
// #define FLOW_SAMPLING_PERIOD_MS   5
static float _flow_samples[MAX_FLOW_SAMPLES];		//  used as buffer for flow analysis and motor command computing
static uint32_t _flow_samples_count;

static void compute_error_slice_samples(float*    _flow_samples,              //input:     Samples of flow array
                                        uint32_t* _flow_samples_count,        //input:     size of samples array
                                        float     flow_set_point,             //input:     Flow set point
				        uint32_t  new_Tslice,                 //input:     Duration of the future slice
                                        float     *error_slice_samples,       //ouput:     error during each slice
                                        uint32_t  *t_end_of_slice_us,         //output:    Computed time of end of the slice array (Only one is updated)
                                        uint32_t  *slice_idx,                 //in/output: Current slice (will be incremented)
                                        uint32_t  *slice_start_t_ms,          //output:    Start time of the next slice
                                        uint32_t  *Tslice);                   //output:    Duration of the next slice

static void compute_set_point_per_slice(float     flow_set_point,             //input: Global set point
			                float*    error_slice_samples,        //input: Error per slices array
			                uint32_t* slice_t_end,                //input: Endtime of slcie array
				        uint32_t  nb_slices,                  //input: nb slices within arrays
			                float*    set_point_per_slice);       //output: new setpoint per slice array


static void compute_seed(float      V_set_point,                              //input:  Volume setting
		         float      flow_set_point,                           //input:  Setpoint in flow
		         float*     set_point_per_slice,                      //output: Setpoint per slice
		         float*     V_per_slice,                              //output: Speed per slice
		         uint32_t*  nb_steps_per_slice,                       //output: nb steps per slice array
		         uint32_t*  t_motor_steps_us,                         //output: motor steps array
		         uint32_t*  current_nb_steps);                        //output: current_nb_steps with motor_step_array


static void apply_correction(float*    set_point_per_slice,                   //input:  set point per slice array
		             uint32_t* t_end_of_slice,                        //input:  t_end_of_slice of the end of the slice array
		             float*    previous_set_point_per_slice,          //input:  Previous set point per slice array
		             float*    previous_V_per_slice,                  //input:  Previous mean speed per slice array
		             uint32_t* previous_nb_steps_at_this_slice,       //input:  Previous cycle nb steps in this slice
		             uint32_t  previous_nb_steps,                     //input:  Previous nb total steps during cycle
		             float     Pcrete_previous_cycle,                 //input:  Pcrete Recorded during previous cycle
		             uint32_t  nb_slices,                             //input:  Nb slices within array
		             uint32_t  Ti,                                    //input:  Time of inhalation
		             float*    V_per_slice,                           //output: mean speed per slice array
		             uint32_t* nb_steps_per_slice,                    //output: nb steps per slice array
		             uint32_t* t_motor_steps_us,                      //output: motor steps array
		             uint32_t* current_nb_steps);                     //output: nb steps within array





// static void adaptation(float target_flow_Lm, float* flow_samples, uint32_t nb_samples, float time_step_sec, float* A, float* B);
// static float linear_fit(float* samples, uint32_t samples_len, float time_step_sec, float* slope);
// static int32_t get_plateau(float* samples, uint32_t samples_len, float time_step_sec, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound);


static void enter_state(BreathingState newState);


void breathing_run(void *args) {
  UNUSED(args);
  EventBits_t events;

  while(true) {
    brth_printf("BRTH: Standby\n");
    events= xEventGroupWaitBits(ctrlEventFlags, BREATHING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    brth_printf("BRTH: Started\n");

    motor_enable(true);

    EoI_ratio=0;
    FR_pm=0;
    VTe_mL=0;
    Pcrete_cmH2O=0;
    Pplat_cmH2O=0;
    PEP_cmH2O=0;



    do  {
      brth_printf("BRTH: start cycle\n");
      // Get current controller settings
      uint32_t T        = get_setting_T_ms      ();
      float    VT       = get_setting_VT_mL     ();
      float    VM       = get_setting_Vmax_Lpm  ();
      float    Pmax     = get_setting_Pmax_cmH2O();
      uint32_t Tplat    = get_setting_Tplat_ms  ();
	 
      init_sample_PEP_cmH2O();
      init_sample_Pplat_cmH2O();

      brth_printf("BRTH: T     : %ld\n", T);
      brth_printf("BRTH: VT    : %ld\n", (uint32_t)(VT*100));
      brth_printf("BRTH: VM    : %ld\n", (uint32_t)(VM*100));
      brth_printf("BRTH: Pmax  : %ld\n", (uint32_t)(Pmax*100));
      brth_printf("BRTH: Tplat : %ld\n", Tplat);

      // Compute adaptation based on current settings and previous collected data if any.
      uint32_t Ti = T*1/3;  // TODO: Check how to calculate Tinsuflation
      brth_printf("BRTH: Ti    : %ld\n", Ti);
      enter_state(Insufflation);

      //Is there any slicing done ?
      if(current_nb_slices == 0) //NO
      {
	compute_seed(get_setting_VT_mL(),         //input : Volume setting
		     get_setting_Vmax_Lpm(),      //input : Setpoint in flow
		     current_set_point_per_slice, //output: Setpoint per slice
		     current_V_per_slice,         //output: Speed per slice
		     current_nb_steps_per_slices, //output: nb step per slice
		     t_motor_steps_us,            //output: motor steps array
		     &current_nb_steps);          //output: nb steps within array
      }
      //YES
      if(current_nb_slices > 0)
      {
	compute_set_point_per_slice(get_setting_Vmax_Lpm(),
				    error_slice_samples,
				    t_end_of_slice_us,
			            current_nb_slices,
				    current_set_point_per_slice);

	for(unsigned int i = 0; i < current_nb_slices-2; i++)
	{
	  printf("Set_point[%d] = %d\n", i, (int)(current_set_point_per_slice[i]*100.f));
	}


	apply_correction(current_set_point_per_slice,           //input:  set point per slice array
			 t_end_of_slice_us,                     //input:  t_end_of_slice of the end of the slice array
			 previous_set_point_per_slice,          //input:  Previous set point per slice array
			 previous_V_per_slice,                  //input:  Previous mean speed per slice array
			 previous_nb_steps_per_slices,          //input:  Previous cycle nb steps in this slice
			 previous_nb_steps,                     //input:  Previous nb total steps during cycle
			 get_breathing_Pcrete_cmH2O(),          //input:  Pcrete Recorded during previous cycle
		         current_nb_slices,                     //input:  Nb slices within array
			 Ti,                                    //input:  Time of inhalation
			 current_V_per_slice,                   //output: mean speed per slice array
			 current_nb_steps_per_slices,           //output: nb steps per slice array
			 t_motor_steps_us,                      //output: motor steps array
			 &current_nb_steps);                    //output: nb steps within array

	print_steps(t_motor_steps_us, current_nb_steps);
	//for(unsigned int i = 0; i < nb_slices; i++)
	//{
	//  printf("Error %d = %d end at %"PRIu32"\n", i, (int) (100.f* error_slice_samples[i]), previous_t_end_of_slice_us[i]);
	//}

      }
      //Swap set_point_per_slice, V_per_slice and nb_steps_per_slice between current and previous buffer
      float* tmp_set_point_per_slice    = previous_set_point_per_slice;
      float* tmp_V_per_slice            = previous_V_per_slice;
      uint32_t* tmp_nb_steps_per_slices = previous_nb_steps_per_slices;
      previous_set_point_per_slice      = current_set_point_per_slice;
      previous_V_per_slice              = current_V_per_slice;
      current_set_point_per_slice       = tmp_set_point_per_slice;
      current_V_per_slice               = tmp_V_per_slice;
      current_nb_steps_per_slices       = tmp_nb_steps_per_slices;
      previous_nb_slices		= current_nb_slices;
      current_nb_slices = 0;

      unsigned int Ta = 80;	    //Duration of the acceleration phase (computed)
      Tslice = Ta;
      current_slice_start_t_ms = get_time_ms();

      // Start Inhalation
      valve_inhale();

      motor_press(t_motor_steps_us, current_nb_steps);
      previous_nb_steps = current_nb_steps;
      reset_Vol_mL();
      brth_print("BRTH: Insuflation\n");      
      while(Insufflation == _state ) {

	  //if (Pmax <= read_Paw_cmH2O()) {
          //    brth_printf("BRTH: Paw [%ld]> Pmax --> Exhalation\n", (int32_t)(read_Paw_cmH2O()));
          //    enter_state(Exhalation);
          //    break;
          //} else if (VT <= read_Vol_mL()) {
          //    brth_printf("BRTH: vol [%ld]>= VT --> Plateau\n", (int32_t)(read_Vol_mL()));
          //    enter_state(Plateau);
          //    break;
          //} else
	  if( Ti <= (get_time_ms() - _cycle_start_ms) ) {
              brth_printf("BRTH: dt [%lu]>= Ti\n", (get_time_ms() - _cycle_start_ms));
              enter_state(Plateau);
              break;
          }
	  //Sample flow for later adaptation.
          if(_flow_samples_count<MAX_FLOW_SAMPLES) {
            _flow_samples[_flow_samples_count] = read_Pdiff_Lpm();  // in sls
            ++_flow_samples_count;
          }

          if(Tslice <= (get_time_ms() - current_slice_start_t_ms) + PERIOD_BREATING_MS)
          {
	    compute_error_slice_samples(_flow_samples,
					&_flow_samples_count,
					get_setting_Vmax_Lpm(),
					(Ti-Ta)/NB_SLICE,
					error_slice_samples,
					t_end_of_slice_us,
					&current_nb_slices,
					&current_slice_start_t_ms,
					&Tslice);
	  }
          wait_ms(PERIOD_BREATING_MS);
      }
      compute_error_slice_samples(_flow_samples,
	                          &_flow_samples_count,
	                          get_setting_Vmax_Lpm(),
	                          (Ti-Ta)/NB_SLICE,
	                          error_slice_samples,
	                          t_end_of_slice_us,
	                          &current_nb_slices,
	                          &current_slice_start_t_ms,
	                          &Tslice);

      motor_release();
      while(Plateau == _state) {
        if(_flow_samples_count<MAX_FLOW_SAMPLES) {
          _flow_samples[_flow_samples_count] = read_Pdiff_Lpm()/60.;  // in sls
          ++_flow_samples_count;
        }

	if (Pmax <= read_Paw_cmH2O()) {
            brth_print("BRTH: Paw > Pmax --> Exhalation\n");
            Pplat_cmH2O = get_Pplat_avg_cmH2O();
            enter_state(Exhalation);
        } else if ( is_command_Tpins_expired() && (Tplat <= (get_time_ms() - _state_start_ms)) ) {
            brth_print("BRTH: Tpins expired && (dt > Tplat)\n");
            Pplat_cmH2O = get_Pplat_avg_cmH2O();
            enter_state(Exhalation);
        }
        sample_Pplat_cmH2O(read_Paw_cmH2O());
        Pcrete_cmH2O = MAX(Pcrete_cmH2O, read_Paw_cmH2O());
        wait_ms(PERIOD_BREATING_MS);
      }
      VTi_mL= read_Vol_mL();
      compute_error_slice_samples(_flow_samples,
				  &_flow_samples_count,
				  get_setting_Vmax_Lpm(),
				  (Ti-Ta)/NB_SLICE,
				  error_slice_samples,
				  t_end_of_slice_us,
				  &current_nb_slices,
				  &current_slice_start_t_ms,
				  &Tslice);


      valve_exhale();
      while(Exhalation == _state) { 
          if ( T <= (get_time_ms() - _cycle_start_ms )) { 
              uint32_t t_ms = get_time_ms();

	      PEP_cmH2O = get_PEP_avg_cmH2O();
              EoI_ratio =  (float)(t_ms-_cycle_start_ms)/(_state_start_ms-_cycle_start_ms);
              FR_pm     = 1./(((float)(t_ms-_cycle_start_ms))/1000/60);
              VTe_mL = VTe_start_mL - read_Vol_mL();
              VMe_Lpm   = (VTe_mL/1000) * FR_pm;

              xEventGroupSetBits(brthCycleState, BRTH_RESULT_UPDATED);
              //regulation_pep();
              enter_state(Finished);
          }
	  sample_PEP_cmH2O(read_Paw_cmH2O());
	  wait_ms(PERIOD_BREATING_MS);
      }





      events= xEventGroupGetBits(ctrlEventFlags);
    } while ( ( events & BREATHING_RUN_FLAG ) != 0 );

    brth_printf("BRTH: Stopping\n");      

    wait_ms(200);
    xEventGroupSetBits(ctrlEventFlags, BREATHING_STOPPED_FLAG);

  }
}

static void enter_state(BreathingState newState) {
  _state= newState;
  _state_start_ms = get_time_ms();
  EventBits_t brthState =0;
  switch(_state) {
    case Insufflation:
      _cycle_start_ms = get_time_ms();
      brthState =  BRTH_CYCLE_INSUFLATION;
      brth_printf("BRTH: Insuflation\n");
      break;
    case Plateau:
      brthState =  BRTH_CYCLE_PLATEAU;
      brth_printf("BRTH: Plateau\n");
      break;
    case Exhalation:
      VTe_start_mL = read_Vol_mL();
      brthState =  BRTH_CYCLE_EXHALATION;
      brth_printf("BRTH: Exhalation\n");
      break;
    case Finished:
      brthState =  BRTH_CYCLE_FINISHED;
      brth_printf("BRTH: Finished\n");
      break;
  }
  // Inform system about current state
  xEventGroupClearBits(brthCycleState, (BRTH_CYCLE_INSUFLATION | BRTH_CYCLE_PLATEAU | BRTH_CYCLE_EXHALATION | BRTH_CYCLE_FINISHED) );
  xEventGroupSetBits(brthCycleState, brthState);
}



// static float compte_motor_step_time(uint32_t step_number, float desired_flow_Ls, float A, float B, float speed) {
// 	float res = (0.8*A*speed*speed*step_number) + B * speed;
// 	res = res / desired_flow_Ls;
// 	if (res * 1000000 < 200) {return 200;}
// 	else {return res * 1000000.;}
// }


// static void adaptation(float target_flow_Lm, float* flow_samples, uint32_t nb_samples, float time_step_sec, float* A, float* B) {
//   if(nb_samples==0) return;
// //************************************************* PID ZONE ********************************************//
// 		// Compute average flow and slope to adjust A_calibrated and B_calibrated
// 		float P_plateau_slope = 0.1;
// 		float P_plateau_mean = 0.2;
// 		uint32_t low;
// 		uint32_t high;
// 		if(get_plateau(flow_samples, nb_samples, time_step_sec, 10, &low, &high) == 0) {
// //			brth_printf("plateau found from sample %lu to %lu\n", low, high);
// 		} else {
// //			brth_printf("plateau NOT found, considering from sample %lu to %lu\n", low, high);
// 		}
// 		float plateau_slope = linear_fit(flow_samples+low, high-low-1, time_step_sec, &plateau_slope);
// 		float plateau_mean = 0;
// 		for(uint32_t i=low; i<high; i++) {
// 			plateau_mean += flow_samples[i];
// 		}
// 		plateau_mean = plateau_mean/(high-low);
// //		brth_printf("plateau slope : %ld\n",(int32_t)(1000*plateau_slope));
// //		brth_printf("plateau mean : %ld\n",(int32_t)(1000*plateau_mean));

// 		float error_mean = plateau_mean - (target_flow_Lm/60.);

// 		*A += plateau_slope * P_plateau_slope;
// 		*B += error_mean * P_plateau_mean;
// //		brth_printf("A = %ld\n", (int32_t)(1000*(*A)));
// //		brth_printf("B = %ld\n", (int32_t)(1000*(*B)));

// }

// // Compute slope of samples fetched with specified time_step
// // Returns 	R  if fit is ok
// // 			-1 if fit is not possible
// static float linear_fit(float* samples, uint32_t samples_len, float time_step_sec, float* slope){
// 	float sumx=0,sumy=0,sumxy=0,sumx2=0, sumy2=0;
// 	for(uint32_t i=0;i<samples_len;i++) {
// 		sumx  = sumx + (float)i * time_step_sec;
// 		sumx2 = sumx2 + (float)i*time_step_sec*(float)i*time_step_sec;
// 		sumy  = sumy + *(samples+i);
// 		sumy2 = sumy2 + (*(samples+i)) * (*(samples+i));
// 		sumxy = sumxy + (float)i*(time_step_sec)* (*(samples+i));
// 	}
// 	float denom = (samples_len * sumx2 - (sumx * sumx));
// 	if(denom == 0.) {
// //		brth_printf("Calibration of A is not possible\n");
// 		return 1;
// 	}
// 	// compute slope a
// 	*slope = (samples_len * sumxy  -  sumx * sumy) / denom;

// 	// compute correlation coefficient
// 	return (sumxy - sumx * sumy / samples_len) / sqrtf((sumx2 - (sumx*sumx)/samples_len) * (sumy2 - (sumy*sumy)/samples_len));
// }

// static int32_t get_plateau(float* samples, uint32_t samples_len, float time_step_sec, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound){
// 	if(windows_number < 2 || windows_number > 30) {return -1;}
// 	float slopes[30];
// 	*high_bound = samples_len-1;
// 	// Compute slope for time windows to detect when signal start increasing/decreasing
// 	for(uint32_t window=0; window<windows_number; window++) {
// 		float r = linear_fit(samples+window*(samples_len/windows_number), samples_len/windows_number, time_step_sec, slopes+window);
// //		brth_printf("%ld    ", (int32_t)(*(slopes+window) * 1000));
// 	}
// //	brth_printf("\n");
// 	for(uint32_t window=1; window<windows_number; window++) {
// 		float delta_slope = slopes[window-1] - slopes[window];
// 		if(delta_slope > 1.) {
// 			*low_bound = (uint32_t)((samples_len/windows_number)*(window+1));
// //			brth_printf("plateau begin at %lu over %lu points\n", *low_bound, (uint32_t)samples_len);
// 			return 0;
// 		}
// 	}
// 	*low_bound = (uint32_t)(samples_len/2);
// //	brth_printf("No plateau found\n");
// 	return 1;
// }


static void regulation_pep() {
  float pep_objective = get_setting_PEP_cmH2O();
  float current_pep = get_PEP_cmH2O();
  //Do not do regulation if the current pep is irrelevant
  if(current_pep < 0.0f)
    return;
  int relative_pep = (pep_objective*10.f - current_pep*10.f);
  if(abs(relative_pep) > 3) {
    motor_pep_move( (int) ((float)relative_pep/MOTOR_TO_PEP_FACTOR));
  }
}




float get_breathing_EoI_ratio()     { return EoI_ratio; }
float get_breathing_FR_pm()         { return FR_pm; }
float get_breathing_VTe_mL()        { return VTe_mL; }
float get_breathing_VTi_mL()        { return VTi_mL; }
float get_breathing_VMe_Lpm()       { return VMe_Lpm; }
float get_breathing_Pcrete_cmH2O()  { return Pcrete_cmH2O; }
float get_Pplat_cmH20()             { return Pplat_cmH2O; }
float get_PEP_cmH2O()               { return PEP_cmH2O; }

static void compute_error_slice_samples(float* _flow_samples,          //input:     Samples of flow array
                                 uint32_t* _flow_samples_count, //input:     size of samples array
                                 float     flow_set_point,      //input:     Flow set point
				 uint32_t  new_Tslice,          //input:     Duration of the future slice
                                 float*    error_slice_samples, //ouput:     error during each slice
                                 uint32_t  *t_end_of_slice_us,  //output:    Computed time of end of the slice array (Only one is updated)
                                 uint32_t  *slice_idx,          //in/output: Current slice (will be incremented)
                                 uint32_t  *slice_start_t_ms,   //output:    Start time of the next slice
                                 uint32_t  *Tslice)             //output:    Duration of the next slice
{
  float error = 0.0;
  for(unsigned int i=0; i < *_flow_samples_count; i++)
  {
      error += (_flow_samples[i] - flow_set_point) / *_flow_samples_count;
  }
  error_slice_samples[*slice_idx] = error;
  t_end_of_slice_us[*slice_idx] = get_time_ms()-_cycle_start_ms;
  *slice_start_t_ms = get_time_ms();
  (*slice_idx)++;
  //Reset sampling per slice
  *_flow_samples_count=0;
  *Tslice = new_Tslice;
}

static void compute_set_point_per_slice(float     flow_set_point,
			         float*    error_slice_samples,
			         uint32_t* previous_t_end_of_slice_us,
				 uint32_t  nb_slices,
			         float*    set_point_per_slice)
{
  float error_total = 0.0f;
  for(unsigned int i = 0; i < nb_slices; i++)
  {
    error_total += error_slice_samples[i];
  }
  uint32_t t_corrected = previous_t_end_of_slice_us[nb_slices-1] - previous_t_end_of_slice_us[1];
  float kp = 0.2;

  float new_set_point_global = flow_set_point - ( (error_total/ t_corrected) * kp);
  for(unsigned int i=0; i < nb_slices-2; i++)
  {
     set_point_per_slice[i] = new_set_point_global - error_slice_samples[i+1];
  }
}

static float abaque(uint32_t current_step)
{
  return (current_step/MAX_MOTOR_STEPS)*4; //Output between 0.0 and 4.0
}

static void compute_SaMaxSdMax(uint32_t previous_nb_steps_at_this_slice,
		    uint32_t previous_nb_steps,
		    float Pcrete_previous_cycle,
		    float* SaMax,
		    float* SdMax
		    )
{
  if(SaMax != NULL)
  {
    *SaMax = 0.f;
  }
  if(SdMax != NULL)
  {
    *SdMax = 0.f;
  }
}



static void apply_correction(float*    set_point_per_slice,                   //input:  set point per slice array
		      uint32_t* t_end_of_slice,                        //input:  t_end_of_slice of the end of the slice array
		      float*    previous_set_point_per_slice,          //input:  Previous set point per slice array
		      float*    previous_V_per_slice,                  //input:  Previous mean speed per slice array
		      uint32_t* previous_nb_steps_at_this_slice,       //input:  Previous cycle nb steps in this slice
		      uint32_t  previous_nb_steps,                     //input:  Previous nb total steps during cycle
		      float     Pcrete_previous_cycle,                 //input:  Pcrete Recorded during previous cycle
		      uint32_t  nb_slices,                             //input:  Nb slices within array
		      uint32_t  Ti,                                    //input:  Time of inhalation
		      float*    V_per_slice,                           //output: mean speed per slice array
		      uint32_t* nb_steps_per_slice,                    //output: nb steps per slice array
		      uint32_t* t_motor_steps_us,                      //output: motor steps array
		      uint32_t* current_nb_steps)                      //output: nb steps within array
{
  float Sa[NB_SLICE+2];//Acceleration per slice
  uint32_t steps_offset_per_slice[NB_SLICE+2];
  float SaMax, SdMax;

  V_per_slice[0] = previous_V_per_slice[0]; //V0 du seed
  for(int k = 1; k < NB_SLICE+1;k++)
  {
    V_per_slice[k] = previous_V_per_slice[k] * (set_point_per_slice[k]/previous_set_point_per_slice[k]) * 1.f/abaque(k);
    printf("V_per_slice[%d] = %d\n", k, ((int)V_per_slice[k]));
  }

  compute_SaMaxSdMax(previous_nb_steps_at_this_slice[0], previous_nb_steps, Pcrete_previous_cycle, &Sa[0], NULL);
  for(unsigned int k = 1; k < nb_slices+1; k++)
  {
    Sa[k] = (V_per_slice[k] + V_per_slice[k-1])/2 * (t_end_of_slice[k] - t_end_of_slice[k-1]);
    compute_SaMaxSdMax(previous_nb_steps_at_this_slice[0], previous_nb_steps, Pcrete_previous_cycle, &SaMax, &SdMax);

    if(! TEST_FLT_EQUALS(Sa[k], 0.f) )
    {
      if(Sa[k] > 0) {
	Sa[k] = MAX(SaMax, Sa[k]);
      }
      else { //Sa is actually a Sd (deceleration)
	Sa[k] = MAX(SdMax, Sa[k]);
      }
    }
  }


  uint32_t t_accumulated_steps = 0;
  uint32_t current_step = 0;
  //Pour la tranche 0
  current_nb_steps[0] = 1/2*t_end_of_slice[0]*t_end_of_slice[0]/Sa[0];
  for(uint32_t j = 1; j < current_nb_steps[0]+1; j++)
  {
    t_motor_steps_us[j-1] = 1000000 /(sqrtf(2*j*Sa[j]));
    t_accumulated_steps+=t_motor_steps_us[j-1];
    current_step++;
  }

  //Pour les autres tranches
  uint32_t k;
  for(k = 1; k < nb_slices; k++)
  {
    if(TEST_EQUALS(Sa[k], 0))
    {
      //We will apply the mean speed
      nb_steps_per_slice[k] = ((V_per_slice[k] - V_per_slice[k-1])/2) * (t_end_of_slice[k] - t_end_of_slice[k-1]);
      steps_offset_per_slice[k] = 1;
    }
    else {
      nb_steps_per_slice[k]   = ((V_per_slice[k]*V_per_slice[k])-(V_per_slice[k-1]*V_per_slice[k-1])) / (2*Sa[k]);
      steps_offset_per_slice[k] = (V_per_slice[k-1]*V_per_slice[k-1])/(2*Sa[k]);
    }

    for(uint32_t j = steps_offset_per_slice[k]; j < nb_steps_per_slice[k] + steps_offset_per_slice[k]; j++)
    {
      if(! TEST_FLT_EQUALS(Sa[k], 0.f) )
      {
	t_motor_steps_us[current_step] = 1000000 / ( sqrt(2 * j * fabs(Sa[k])) );
      }
      else { //Acceleration is 0. so speed is constant for all the slice
	//Mean speed
	t_motor_steps_us[current_step] = 1000000 / V_per_slice[k];
      }
      t_accumulated_steps += t_motor_steps_us[current_step];
      current_step++;
      //We computed steps until the Ti
      if(t_accumulated_steps > Ti)
      {
	break;
      }
    }
    //We computed steps until the Ti
    //This event should occurs only on the last slice
    //This code maybe useless
    if(t_accumulated_steps > Ti)
    {
      break;
    }
  }

  //Pour la dernier tranche
  uint32_t steps_end = current_step-1;
  float Sd;
  compute_SaMaxSdMax(previous_nb_steps_at_this_slice[k], previous_nb_steps, Pcrete_previous_cycle, NULL, &Sd);
  uint32_t nb_step_end = 1/(t_motor_steps_us[steps_end]*t_motor_steps_us[steps_end] * 2 * Sd);
  uint32_t t_stop = (1/(t_motor_steps_us[steps_end])) * (1/Sd);
  current_step -= nb_step_end;
  uint32_t new_step_end = current_step + nb_step_end;
  for(; current_step < new_step_end; current_step++)
  {
    //Remove the previously computed time
    t_accumulated_steps -= t_motor_steps_us[current_step];
    t_motor_steps_us[current_step] = (1000000) / ( sqrtf(2*(nb_step_end-current_step) * Sd) );
  }
}

static void compute_seed(float      V_set_point,                              //input:  Volume setting
		  float      flow_set_point,                           //input:  Setpoint in flow
		  float*     set_point_per_slice,                      //output: Setpoint per slice
		  float*     V_per_slice,                              //output: Speed per slice
		  uint32_t*  nb_steps_per_slice,                       //output: nb steps per slice array
		  uint32_t*  t_motor_steps_us,                         //output: motor steps array
		  uint32_t*  current_nb_steps)                         //output: nb steps with motor_step_array
{

}
