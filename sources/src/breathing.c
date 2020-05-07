#include "common.h"
#include <stdint.h>
#include <stdint.h>
#include <stdint.h>
#include <inttypes.h>
#include "controller.h"
#include "breathing.h"
#include "platform.h"
#include "platform_defs.h"
#include "config.h"
#include "compute_motor.h"

#include <stdint.h>
#include <math.h>


//----------------------------------------------------------
// Private defines
//----------------------------------------------------------

#define PERIOD_BREATING_MS            (10)
#define MAX_PEP_SAMPLES               (100 / PERIOD_BREATING_MS) // moyenne glissante sur les 100ms dernieres de l'expi
#define MAX_PPLAT_SAMPLES             (50  / PERIOD_BREATING_MS) // moyenne glissante sur les 50ms dernieres de plat
#define NB_SLICE                      (30)
#define MAX_FLOW_SAMPLES              (400)
#define DEPHASAGE_MS		      (30)

//----------------------------------------------------------
// Private typedefs
//----------------------------------------------------------

typedef enum { Insufflation, Plateau, Exhalation, Finished } BreathingState;


//----------------------------------------------------------
// Private variables
//----------------------------------------------------------
static const uint32_t T_omegamax_ms      = 200;
static const float Ax                    = 0.000204;
static const float Px                    = 2.1505;
static const float k_phi                 = 0.7; //limit de survitesse motor
static const uint32_t NORMALISATION_STEP = 140;

static float          g_EoI_ratio;       
static float          g_FR_pm;           
static float          g_VTe_mL; 
static float          g_VTi_mL; 
static float          g_VMe_Lpm; 
static float          g_Pcrete_cmH2O; 
static float          g_Pplat_cmH2O; 
static float          g_PEP_cmH2O; 
static float          g_PEP_cmH2O_samples[MAX_PEP_SAMPLES]; 
static uint32_t       g_PEP_cmH2O_samples_index; 
static float          g_Pplat_cmH2O_samples[MAX_PPLAT_SAMPLES]; 
static uint32_t       g_Pplat_cmH2O_samples_index; 


static BreathingState g_state; 

static uint32_t       g_state_start_ms; 
static uint32_t       g_cycle_start_ms;
static uint32_t       g_cycle_counter = 0;

static uint32_t       g_motor_steps_us[MAX_MOTOR_STEPS] = {0};  // TODO: Make it configurable with a define. This represent a physical limit a the system.

static uint32_t       g_Tslice_ms; //Duration of a slice
static float          g_error_slice_samples[NB_SLICE+2];
static uint32_t       g_t_end_of_slice_us[NB_SLICE+2];

//Set point
static float	      g_global_flow_set_point;
static float          g_set_point_per_slice[2][NB_SLICE+2];
static float*         g_current_set_point_per_slice  = g_set_point_per_slice[0];
static float*         g_previous_set_point_per_slice = g_set_point_per_slice[1];

//V per slice
static float          g_V_per_slice[2][NB_SLICE+2];
static float*         g_current_V_per_slice  = g_V_per_slice[0];
static float*         g_previous_V_per_slice = g_V_per_slice[1];

//NB steps per slice
                      
static uint32_t       g_step_idx_end_of_slice[2][NB_SLICE+2];
static uint32_t*      g_current_step_idx_end_of_slice  = g_step_idx_end_of_slice[0];
static uint32_t*      g_previous_step_idx_end_of_slice = g_step_idx_end_of_slice[1];

//Nb total steps
static uint32_t        g_current_nb_steps         = 0;
static uint32_t        g_previous_nb_steps        = 0;

static uint32_t        g_current_nb_slices        = 0;
static uint32_t        g_previous_nb_slices       = 0;

static uint32_t        g_theorical_current_slice_end_t_ms = 0;



static float           g_flow_samples[MAX_FLOW_SAMPLES]; //  used as buffer for flow analysis and motor command computing
static uint32_t        g_flow_samples_t_ms[MAX_FLOW_SAMPLES]; //  used as buffer for flow analysis and motor command computing
static uint32_t        g_flow_samples_count;
//----------------------------------------------------------
// Private functions prototypes
//----------------------------------------------------------
static void   regulation_pep();
static void   init_sample_PEP_cmH2O();
static void   sample_PEP_cmH2O( float Paw_cmH2O);
static float  get_PEP_avg_cmH2O();
static void   init_sample_Pplat_cmH2O();
static void   sample_Pplat_cmH2O( float Paw_cmH2O);
static float  get_Pplat_avg_cmH2O();
static void   enter_state(BreathingState newState);
static void   compute_error_slice_samples(float*    flow_samples,              //input:     Samples of flow array
                                          uint32_t* flow_samples_count,        //input:     size of samples array
                                          float*    flow_set_point,             //input:     Flow set point
                                          uint32_t  new_Tslice_ms,              //input:     Duration of the future slice
                                          float     *error_slice_samples,       //ouput:     error during each slice
                                          uint32_t  *t_end_of_slice_us,         //output:    Computed time of end of the slice array (Only one is updated)
                                          uint32_t  *slice_idx,                 //in/output: Current slice (will be incremented)
                                          uint32_t  *slice_theorical_end_t_ms,          //output:    Start time of the next slice
                                          uint32_t  *Tslice_ms);                //output:    Duration of the next slice

//static void compute_set_point_per_slice(float*     flow_set_point,              //input: Global set point
//					float     vol_set_point,               //input: vol set point
//					float 	  Vti_mL,                      //Measured Volume
//                                        float*    error_slice_samples,         //input: Error per slices array
//                                        uint32_t* previous_t_end_of_slice_us,  //input: Endtime of slcie array
//                                        uint32_t  nb_slices,                   //input: nb slices within arrays
//                                        float*    set_point_per_slice);        //output: new setpoint per slice array
//

static void   apply_correction(uint32_t* t_end_of_slice_us,                     //input:  t_end_of_slice of the end of the slice array
                               float*    previous_set_point_per_slice,          //input:  Previous set point per slice array
                               float*    previous_V_per_slice,                  //input:  Previous mean speed per slice array
                               uint32_t* previous_nb_steps_at_this_slice,       //input:  Previous cycle nb steps in this slice
                               uint32_t  previous_nb_steps,                     //input:  Previous nb total steps during cycle
                               float     Pcrete_previous_cycle,                 //input:  Pcrete Recorded during previous cycle
			       float*    error_slice_samples,                   //input: Error per slices array
                               uint32_t  nb_slices,                             //input:  Nb slices within array
                               uint32_t  Ti_ms,                                 //input:  Time of inhalation
                               float*    V_per_slice,                           //output: mean speed per slice array
                               uint32_t* nb_steps_per_slice,                    //output: nb steps per slice array
                               uint32_t* t_motor_step_us,                       //output: motor steps array
                               uint32_t* current_nb_steps);                     //output: nb steps within array


void compute_seed( float     flow_set_point_initial_Lmin, //input:  spec v8 20-60   L/min
                   float     set_point_volume_mL,         //input:  spec v8 100-600 mL
                   uint32_t  nb_slices,                   //input:  ex 1 à 20 (k)
                   uint32_t  Ti_us,                       //input:  temps d'insuflation moteur arrete
                   float*    flow_set_point_per_slice,    //output: flowsp per slice 
                   float*    V_per_slice,                 //output: V per slice
                   uint32_t* t_end_of_slice_us,           //output: T end of slice array
                   uint32_t* step_idx_end_of_slice,       //output: Step idx at the end of the slice 
                   uint32_t* t_motor_step_us,             //output: Array of steps time [MAX_MOTOR_STEPS]
                   uint32_t* nb_steps);                   //output: nb_steps
		
//----------------------------------------------------------
// Public functions
//----------------------------------------------------------


void breathing_run(void *args) {
  UNUSED(args);
  EventBits_t events;
  
  while(true) {
    brth_printf("BRTH: Standby\n");
    events= xEventGroupWaitBits(ctrlEventFlags, BREATHING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    brth_printf("BRTH: Started\n");

    motor_enable(true);

    g_EoI_ratio=0;
    g_FR_pm=0;
    g_VTe_mL=0;
    g_Pcrete_cmH2O=0;
    g_Pplat_cmH2O=0;
    g_PEP_cmH2O=0;



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

      brth_printf("BRTH: T     : %lu\n", T);
      brth_printf("BRTH: VT    : %lu\n", (uint32_t)(VT*100));
      brth_printf("BRTH: VM    : %lu\n", (uint32_t)(VM*100));
      brth_printf("BRTH: Pmax  : %lu\n", (uint32_t)(Pmax*100));
      brth_printf("BRTH: Tplat : %lu\n", Tplat);

      // Compute adaptation based on current settings and previous collected data if any.
      uint32_t Ti_ms = (get_setting_VT_mL()*(60*1000/1000))/get_setting_Vmax_Lpm(); //T*1/3;  // TODO: Check how to calculate Tinsuflation
      brth_printf("BRTH: Ti_ms    : %ld\n", Ti_ms);
      //Is there any slicing done ?
      uint32_t Ta_us;
      if(g_current_nb_slices == 0) //NO
      {

	g_global_flow_set_point = get_setting_Vmax_Lpm();
        compute_seed( g_global_flow_set_point, //input:  spec v8 20-60   L/min
                      get_setting_VT_mL(),    //input:  spec v8 100-600 mL
                      NB_SLICE,                      //input:  ex 1 à 20 (k)
                      (get_setting_VT_mL()*(60*1000000/1000))/get_setting_Vmax_Lpm(),       //input:  temps d'insuflation moteur arrete in us )
                      g_current_set_point_per_slice, //output: flowsp per slice 
                      g_current_V_per_slice,         //output: V per slice
                      g_t_end_of_slice_us,           //output: T end of slice array
                      g_current_step_idx_end_of_slice,
                      g_motor_steps_us,              //output: Array of steps time [MAX_MOTOR_STEPS]
                      &g_current_nb_steps);          //output: nb_steps

	Ta_us = g_t_end_of_slice_us[0];  //Duration of the acceleration phase (computed)
      }
      //YES
      if(g_current_nb_slices > 0)
      {
        //compute_set_point_per_slice(&g_global_flow_set_point,
	//			    get_setting_VT_mL(),
	//			    get_breathing_VTi_mL(),
        //                            g_error_slice_samples,
        //                            g_t_end_of_slice_us,
        //                            g_current_nb_slices,
        //                            g_current_set_point_per_slice);
        //for(unsigned int i = 0; i < g_current_nb_slices-2; i++)
        //{
        //  printf("Set_point[%d] = %d\n", i, (int)(g_current_set_point_per_slice[i]*100.f));
        //}

        apply_correction(g_t_end_of_slice_us,                     //input:  t_end_of_slice of the end of the slice array
                         g_previous_set_point_per_slice,          //input:  Previous set point per slice array
                         g_previous_V_per_slice,                  //input:  Previous mean speed per slice array
                         g_previous_step_idx_end_of_slice,        //input:  Previous Step idx at the end of slice  
                         g_previous_nb_steps,                     //input:  Previous nb total steps during cycle
                         get_breathing_Pcrete_cmH2O(),            //input:  Pcrete Recorded during previous cycle
			 g_error_slice_samples,                   //input: Error per slices array
                         g_current_nb_slices,                     //input:  Nb slices within array
                         Ti_ms,                                   //input:  Time of inhalation
                         g_current_V_per_slice,                   //output: mean speed per slice array
                         g_current_step_idx_end_of_slice,         //output: New Step idx at the end of slice 
                         g_motor_steps_us,                        //output: motor steps array
                         &g_current_nb_steps);                    //output: nb steps within array

	uint32_t k = 0;
	//for(uint32_t i = 0; i < g_current_nb_steps; i++)
	//{
	//  if(i >g_current_step_idx_end_of_slice[k])
	//    k++;
	//  printf("%d;%d;%d\n", i, g_motor_steps_us[i],k);
	//}
        //for(unsigned int i = 0; i < nb_slices; i++)
        //{
        //  printf("Error %d = %d end at %"PRIu32"\n", i, (int) (100.f* error_slice_samples[i]), previous_t_end_of_slice_us[i]);
        //}
      }
      //if(g_cycle_counter == 20)
      //  while(true);

      //Swap set_point_per_slice, V_per_slice and nb_steps_per_slice between current and previous buffer
      float* tmp_set_point_per_slice      = g_previous_set_point_per_slice;
      float* tmp_V_per_slice              = g_previous_V_per_slice;
      uint32_t* tmp_step_idx_end_of_slice = g_previous_step_idx_end_of_slice;
      g_previous_set_point_per_slice      = g_current_set_point_per_slice;
      g_previous_V_per_slice              = g_current_V_per_slice;
      g_previous_step_idx_end_of_slice    = g_current_step_idx_end_of_slice;
      g_current_set_point_per_slice       = tmp_set_point_per_slice;
      g_current_V_per_slice               = tmp_V_per_slice;
      g_current_step_idx_end_of_slice     = tmp_step_idx_end_of_slice;
      g_previous_nb_slices                = g_current_nb_slices;
      g_current_nb_slices                 = 0;

  
      uint32_t Ta_ms = Ta_us / 1000;
      g_Tslice_ms = Ta_ms;
      enter_state(Insufflation);
      g_theorical_current_slice_end_t_ms = (get_time_ms() - g_cycle_start_ms)  + Ta_ms;
      // Start Inhalation
      valve_inhale();
      
      motor_press(g_motor_steps_us, g_current_nb_steps);
      g_previous_nb_steps = g_current_nb_steps;
      reset_Vol_mL();
      brth_print("BRTH: Insuflation\n");      
      while (Insufflation == g_state)
      {
        wait_ms(PERIOD_BREATING_MS);
        g_Pcrete_cmH2O = MAX(g_Pcrete_cmH2O, read_Paw_cmH2O());
        //if (Pmax <= read_Paw_cmH2O())
        //{
        //  brth_printf("BRTH: Paw [%ld]> Pmax --> Exhalation\n", (int32_t)(read_Paw_cmH2O()));
        //  enter_state(Exhalation);
        //  break;
        //}
        //else if (VT <= read_Vol_mL())
        //{
        //  brth_printf("BRTH: vol [%ld]>= VT --> Plateau\n", (int32_t)(read_Vol_mL()));
        //  enter_state(Plateau);
        //  break;
        //}
        //else 
	if (Ti_ms <= (get_time_ms() - g_cycle_start_ms))
        {
          brth_printf("BRTH: dt [%lu]>= Ti\n", (get_time_ms() - g_cycle_start_ms));
          enter_state(Plateau);
          break;
        }
	if(g_flow_samples_count<MAX_FLOW_SAMPLES && DEPHASAGE_MS <= ((get_time_ms() - g_cycle_start_ms))) {
            g_flow_samples[g_flow_samples_count] = read_Pdiff_Lpm();  // in sls
	    g_flow_samples_t_ms[g_flow_samples_count] = get_time_ms() - g_cycle_start_ms;  // in ms
            ++g_flow_samples_count;
          }

	if( g_theorical_current_slice_end_t_ms + DEPHASAGE_MS <= (get_time_ms() - g_cycle_start_ms) + PERIOD_BREATING_MS )
	{
	  compute_error_slice_samples(g_flow_samples,
	      &g_flow_samples_count,
	      g_previous_set_point_per_slice,
	      (Ti_ms-Ta_ms)/NB_SLICE,
	      g_error_slice_samples,
	      g_t_end_of_slice_us,
	      &g_current_nb_slices,
	      &g_theorical_current_slice_end_t_ms,
	      &g_Tslice_ms);
	  //printf("Error tranche %d = %d%\n",g_current_nb_slices-1, ((int)(g_error_slice_samples[g_current_nb_slices-1] * 100.f)));
	}
      }
      motor_release(MOTOR_RELEASE_STEP_US);
      while (Plateau == g_state)
      {
        wait_ms(PERIOD_BREATING_MS);
        sample_Pplat_cmH2O(read_Paw_cmH2O());
        g_Pcrete_cmH2O = MAX(g_Pcrete_cmH2O, read_Paw_cmH2O());
        if (Pmax <= read_Paw_cmH2O())
        {
          brth_print("BRTH: Paw > Pmax --> Exhalation\n");
          g_Pplat_cmH2O = get_Pplat_avg_cmH2O();
          enter_state(Exhalation);
        }
        else if (is_command_Tpins_expired() && (Tplat <= (get_time_ms() - g_state_start_ms)))
        {
          brth_print("BRTH: Tpins expired && (dt > Tplat)\n");
          g_Pplat_cmH2O = get_Pplat_avg_cmH2O();
          enter_state(Exhalation);
        }
	if(g_flow_samples_count<MAX_FLOW_SAMPLES) {
          g_flow_samples[g_flow_samples_count] = read_Pdiff_Lpm();  // in sls
          g_flow_samples_t_ms[g_flow_samples_count] = get_time_ms() - g_cycle_start_ms;  // in ms
          ++g_flow_samples_count;
        }
      }
      compute_error_slice_samples(g_flow_samples,
                           &g_flow_samples_count,
        		   g_previous_set_point_per_slice,
                           (Ti_ms-Ta_ms)/NB_SLICE,
                           g_error_slice_samples,
                           g_t_end_of_slice_us,
                           &g_current_nb_slices,
                           &g_theorical_current_slice_end_t_ms,
                           &g_Tslice_ms);
      //printf("Error last tranche %d = %d%\n",g_current_nb_slices-1, ((int)(g_error_slice_samples[g_current_nb_slices-1] * 100.f)));
      g_VTi_mL = read_Vol_mL();
      printf("VTI = %d\n", (int) g_VTi_mL);

      for(uint32_t i=0; i < g_flow_samples_count; i++)
      {
	  printf("@%ld : %dmL/s\n", g_flow_samples_t_ms[i], (int) (g_flow_samples[i]*1000.f));
      //    printf("@%d : %dmL/m\n", g_flow_samples_t_ms[i], (int) (g_flow_samples[i]*1000.f));
      }
      for(uint32_t i=0; i < g_flow_samples_count; i++)
      {
	g_flow_samples_t_ms[i] = 0;
        g_flow_samples[i] = 0.0f;
      }
      g_flow_samples_count = 0;

      valve_exhale();
      
      float VTe_start_mL = 0.;
      while (Exhalation == g_state)
      {
        wait_ms(PERIOD_BREATING_MS);
        sample_PEP_cmH2O(read_Paw_cmH2O());
        if( !is_command_Tpexp_expired() ) {
          valve_inhale();
        } 
        else 
        {
          valve_exhale();
          if (T <= (get_time_ms() - g_cycle_start_ms))
          {
            brth_print("BRTH: Tpexp expired && (T <= dt)\n");
            uint32_t t_ms = get_time_ms();

            g_PEP_cmH2O = get_PEP_avg_cmH2O();
            g_EoI_ratio = (float)(t_ms - g_cycle_start_ms) / (g_state_start_ms - g_cycle_start_ms);
            g_FR_pm = 1. / (((float)(t_ms - g_cycle_start_ms)) / 1000 / 60);
            g_VTe_mL = VTe_start_mL - read_Vol_mL();
            g_VMe_Lpm = (g_VTe_mL / 1000) * g_FR_pm;

            xEventGroupSetBits(brthCycleState, BRTH_RESULT_UPDATED);

            //regulation_pep();
            enter_state(Finished);
          }
        }
      }

      g_cycle_counter++;
      events = xEventGroupGetBits(ctrlEventFlags);
    } while ((events & BREATHING_RUN_FLAG) != 0);

    brth_printf("BRTH: Stopping\n");

    wait_ms(200);
    xEventGroupSetBits(ctrlEventFlags, BREATHING_STOPPED_FLAG);
  }
}

static void enter_state(BreathingState newState)
{
  g_state = newState;
  g_state_start_ms = get_time_ms();
  EventBits_t brthState = 0;
  switch (g_state)
  {
  case Insufflation:
    g_cycle_start_ms = get_time_ms();
    brthState = BRTH_CYCLE_INSUFLATION;
    brth_printf("BRTH: Insuflation\n");
    break;
  case Plateau:
    brthState = BRTH_CYCLE_PLATEAU;
    brth_printf("BRTH: Plateau\n");
    break;
  case Exhalation:
    brthState = BRTH_CYCLE_EXHALATION;
    brth_printf("BRTH: Exhalation\n");
    break;
  case Finished:
    brthState = BRTH_CYCLE_FINISHED;
    brth_printf("BRTH: Finished\n");
    break;
  }
  // Inform system about current state
  xEventGroupClearBits(brthCycleState, (BRTH_CYCLE_INSUFLATION | BRTH_CYCLE_PLATEAU | BRTH_CYCLE_EXHALATION | BRTH_CYCLE_FINISHED));
  xEventGroupSetBits(brthCycleState, brthState);
}

float get_breathing_EoI_ratio() { return g_EoI_ratio; }
float get_breathing_FR_pm() { return g_FR_pm; }
float get_breathing_VTe_mL() { return g_VTe_mL; }
float get_breathing_VTi_mL() { return g_VTi_mL; }
float get_breathing_VMe_Lpm() { return g_VMe_Lpm; }
float get_breathing_Pcrete_cmH2O() { return g_Pcrete_cmH2O; }
float get_Pplat_cmH20() { return g_Pplat_cmH2O; }
float get_PEP_cmH2O() { return g_PEP_cmH2O; }

//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

static void regulation_pep()
{
  float pep_objective = get_setting_PEP_cmH2O(); // TODO: is it really what we want ? Should we use the setting retreived at the beginning of the cycle instead ?
  float current_pep = get_PEP_avg_cmH2O();
  int relative_pep = (pep_objective * 10.f - current_pep * 10.f);
  if (abs(relative_pep) > 3)
  {
    motor_pep_move((int)((float)relative_pep / MOTOR_PEP_PEP_TO_MM_FACTOR));
  }
}

static void init_sample_PEP_cmH2O()
{
  //Samples PEP for a rolling average
  g_PEP_cmH2O_samples_index = 0;
  for (int i = 0; i < MAX_PEP_SAMPLES; i++)
    g_PEP_cmH2O_samples[i] = 0;
}

static void sample_PEP_cmH2O(float Paw_cmH2O)
{
  g_PEP_cmH2O_samples[g_PEP_cmH2O_samples_index] = Paw_cmH2O;
  g_PEP_cmH2O_samples_index = (g_PEP_cmH2O_samples_index + 1) % MAX_PEP_SAMPLES;
}

static float get_PEP_avg_cmH2O()
{
  float sum_PEP = 0;
  for (int i = 0; i < MAX_PEP_SAMPLES; i++)
  {
    sum_PEP += g_PEP_cmH2O_samples[i];
  }
  return (sum_PEP / MAX_PEP_SAMPLES);
}

static void init_sample_Pplat_cmH2O()
{
  //Samples Pplat for a rolling average
  g_Pplat_cmH2O_samples_index = 0;
  for (int i = 0; i < MAX_PPLAT_SAMPLES; i++)
    g_Pplat_cmH2O_samples[i] = 0;
}

static void sample_Pplat_cmH2O(float Paw_cmH2O)
{
  g_Pplat_cmH2O_samples[g_Pplat_cmH2O_samples_index] = Paw_cmH2O;
  g_Pplat_cmH2O_samples_index = (g_Pplat_cmH2O_samples_index + 1) % MAX_PPLAT_SAMPLES;
}

static float get_Pplat_avg_cmH2O()
{
  float sum_Pplat = 0;
  for (int i = 0; i < MAX_PPLAT_SAMPLES; i++)
  {
    sum_Pplat += g_Pplat_cmH2O_samples[i];
  }
  return (sum_Pplat / MAX_PPLAT_SAMPLES);
}

static void compute_error_slice_samples(float*    flow_samples,        //input:     Samples of flow array
                                        uint32_t* flow_samples_count,  //input:     size of samples array
                                        float*    flow_set_point,      //input:     Flow set point
                                        uint32_t  new_Tslice_ms,       //input:     Duration of the future slice
                                        float*    error_slice_samples, //ouput:     error during each slice
                                        uint32_t  *t_end_of_slice_us,  //output:    Computed time of end of the slice array (Only one is updated)
                                        uint32_t  *slice_idx,          //in/output: Current slice (will be incremented)
                                        uint32_t  *slice_theorical_end_t_ms,   //output:    Start time of the next slice
                                        uint32_t  *Tslice_ms)          //output:    Duration of the next slice
{
  float error = 0.0;
  //float mean_flow = 0.0;

  for(unsigned int i=0; i < *flow_samples_count; i++)
  {
      error += (flow_samples[i] - flow_set_point[*slice_idx]) / *flow_samples_count;
      //mean_flow = (flow_samples[i]) / *flow_samples_count;
  }
  printf("error before div [%ld] = %d\n", *slice_idx, (int) error);


  //printf("Mean flow %d = %d\n", *slice_idx, (int) mean_flow);
  if(TEST_FLT_EQUALS(flow_set_point[*slice_idx], 0.f))
    error_slice_samples[*slice_idx] = 0;
  else
    error_slice_samples[*slice_idx] = error/(flow_set_point[*slice_idx]);

  t_end_of_slice_us[*slice_idx] = (get_time_ms() - g_cycle_start_ms) * 1000;
  *slice_theorical_end_t_ms = *slice_theorical_end_t_ms + new_Tslice_ms;
  //printf("End of %d will be at %dms\n",  *slice_idx+1, *slice_theorical_end_t_ms);
  (*slice_idx)++;
  //Reset sampling per slice
  *flow_samples_count=0;
  *Tslice_ms = new_Tslice_ms;
}

//static void compute_set_point_per_slice(float*    global_flow_set_point,
//					float     vol_set_point,
//					float 	  Vti_mL,
//                                        float*    error_slice_samples,
//                                        uint32_t* previous_t_end_of_slice_us,
//                                        uint32_t* step_idx_end_of_slice_us,
//                                        uint32_t  nb_slices,
//                                        float*    set_point_per_slice)
//{
//  float kg = 0.2f;
//  float kl = 0.2f;
//  //printf("VTarget %d vs Vrealisé = %d\n", (int) vol_set_point, (int) Vti_mL);
//  float error_global = (Vti_mL - vol_set_point) / vol_set_point;
//  float new_global_flow_set_point = *global_flow_set_point * ( (1 - (kg * error_global)));
//
//  //printf("OLD Global = %d vs  NEW %d\n", (int) *global_flow_set_point,  (int) new_global_flow_set_point);
//  for(unsigned int i=1; i < nb_slices; i++)
//  {
//    //printf("Error %d = %d%\n",i, ( (int) ((error_slice_samples[i]) * 100.f)));
//    float new_local_set_point = new_global_flow_set_point *  (1.f - (kl*error_slice_samples[i]));
//    //printf("set point %d NEW %d\n", i, (int) new_local_set_point);
//    set_point_per_slice[i] = new_local_set_point;
//  }
//  *global_flow_set_point = new_global_flow_set_point;
//}

static float abaque(uint32_t step)
{
  return (pow(step, Px-1)) / (pow(NORMALISATION_STEP, Px-1));
}

static void compute_SaMaxSdMax(uint32_t previous_nb_steps_at_this_slice,
                               uint32_t previous_nb_steps,
                               float Pcrete_previous_cycle,
                               float* SaMax,
                               float* SdMax)
{
  if(SaMax != NULL)
  {
    *SaMax = 30000.f;
  }
  if(SdMax != NULL)
  {
    *SdMax = -30000.f;
  }
}


uint32_t T_minobj(uint32_t current_step, uint32_t flow_set_point_Lpm)
{
  float flow_set_point_Lpm_f = flow_set_point_Lpm;
  return ( (uint32_t) (k_phi *(1000.f*Ax*Px*powf(current_step,Px-1)) / (flow_set_point_Lpm_f/60.f)));
}

static void apply_correction(uint32_t* t_end_of_slice_us,                     //input:  t_end_of_slice_us of the end of the slice array
                             float*    previous_set_point_per_slice,          //input:  Previous set point per slice array
                             float*    previous_VM_per_slice,                 //input:  Previous mean speed per slice array
                             uint32_t* previous_step_idx_end_of_slice,       //input:  Previous cycle nb steps in this slice
                             uint32_t  previous_nb_steps,                     //input:  Previous nb total steps during cycle
                             float     Pcrete_previous_cycle,                 //input:  Pcrete Recorded during previous cycle
			     float*    error_slice_samples,		      //input:  Error splice samples
			     uint32_t  nb_slices,                             //input:  Nb slices within array
                             uint32_t  Ti_ms,                                 //input:  Time of inhalation
                             float*    VM_per_slice,                          //output: mean speed per slice array
                             uint32_t* step_idx_end_of_slice,       //output: nb steps p
                             uint32_t* t_motor_step_us,                       //output: motor steps array
                             uint32_t* nb_steps)                              //output: nb steps within array
{
  float Sa[NB_SLICE+2];//Acceleration per slice
  uint32_t t_M_us[NB_SLICE+2];//mean time of slice k
  uint32_t steps_offset_per_slice[NB_SLICE+2];

  float SaMax, SdMax;

  VM_per_slice[0] = previous_VM_per_slice[0]; //V0 du seed
  //printf("VM[%d] = %d\n", 0, (int)VM_per_slice[0]); 
  float kg = 0.30f;
  float kl = 0.20f;
  float error_global = ( get_breathing_VTi_mL() - get_setting_VT_mL()) / get_setting_VT_mL();
  printf("Error global = (%d - %d)/factor = %d\n", (int) get_breathing_VTi_mL(), (int) get_setting_VT_mL(), (int) (error_global * 100.f));
  for(int k = 1; k < NB_SLICE+1;k++)
  {

    //printf("Set point %d\n", (int) set_point_per_slice[k]);
    //printf("Previous Set point %d\n", (int) previous_set_point_per_slice[k]);
    //printf("Set point ration %d\n", (int) (100.f * (set_point_per_slice[k]/previous_set_point_per_slice[k])));
    //printf("Previous VM[%d] = %d\n", k, (int) previous_VM_per_slice[k]); 
    //printf("Set point ration *Previous VM %d\n", (int) (previous_VM_per_slice[k] * (set_point_per_slice[k]/previous_set_point_per_slice[k])));
    //printf("d[%d] = %d\n",k, (t_end_of_slice_us[k]-t_end_of_slice_us[k-1])/1000);
    float correctif = (((error_global*kg)+(error_slice_samples[k]*kl))/abaque(g_previous_step_idx_end_of_slice[k-1]));
    if (fabs(correctif) > 0.2f)
    {
      if(correctif > 0.0f)
      {
	correctif = 0.2;
      }
      else {
	correctif = -0.2;
      }
    }
    correctif = (1 - correctif);
    printf("correction = %d\n",  (int) (correctif*100.f) );
    VM_per_slice[k] = previous_VM_per_slice[k] * (correctif) ;
    t_M_us[k]       = t_end_of_slice_us[k-1] + 0.5*(t_end_of_slice_us[k] - t_end_of_slice_us[k-1]);
    //printf("VM-1[%d] = %d, VM[%d] = %d  \n", k, (int)previous_VM_per_slice[k], k, (int) VM_per_slice[k]); 
    //printf("TM[%d] = %d\n", k, (int)t_M_us[k]); 
  }
  compute_SaMaxSdMax(previous_step_idx_end_of_slice[0], previous_nb_steps, Pcrete_previous_cycle, &Sa[0], NULL);
  
  
  
  uint32_t t_accumulated_steps = 0;
  uint32_t current_step = 1;
  //Pour la tranche 0
  uint32_t k=0;
  float V_cur = 0;
  while(V_cur < VM_per_slice[1])
  {
    t_motor_step_us[current_step-1] = 1000000 /(sqrtf(2*current_step*Sa[0]));
    t_accumulated_steps += t_motor_step_us[current_step-1];
    V_cur = 1000000.f/t_motor_step_us[current_step-1];
    if(t_accumulated_steps > t_end_of_slice_us[1]) {
      break;
    }
    current_step++;
  }

  if(t_accumulated_steps < t_end_of_slice_us[0])
  {
    t_end_of_slice_us[0] = t_accumulated_steps;
    uint32_t t_step_constant = t_motor_step_us[current_step -2];
    while(t_accumulated_steps < t_M_us[1])
    {
      t_motor_step_us[current_step-1] = t_step_constant; 
      t_accumulated_steps += t_step_constant;
    }
  }
  else if(t_accumulated_steps <= t_M_us[1])
  {
    t_end_of_slice_us[0] = t_accumulated_steps;
    uint32_t t_step_constant = t_motor_step_us[current_step -1];
    while(t_accumulated_steps < t_M_us[1])
    {
      printf("ICI 3\n");
      t_motor_step_us[current_step-1] = t_step_constant; 
      t_accumulated_steps += t_step_constant;
    }
  }
  else if(t_M_us[1] <= t_accumulated_steps && t_accumulated_steps < t_end_of_slice_us[1])
  {
    t_end_of_slice_us[0] = t_accumulated_steps;
    t_M_us[1]            = (t_end_of_slice_us[1] + t_end_of_slice_us[0]) / 2;
    uint32_t t_step_constant = t_motor_step_us[current_step -1];
    while(t_accumulated_steps < t_M_us[1])
    {
      printf("ICI 2\n");
      t_motor_step_us[current_step-1] = t_step_constant; 
      t_accumulated_steps += t_step_constant;
    }
  }
  else if(t_accumulated_steps > t_end_of_slice_us[1])
  {
    t_end_of_slice_us[0] = t_accumulated_steps;
    t_M_us[1]            = t_accumulated_steps;
    printf("ICI 1\n");
    t_end_of_slice_us[1] = t_accumulated_steps;
  }

  k++;
  //printf("%d;%d;%d; %d vs %d\n", current_step, t_motor_step_us[current_step],k,t_accumulated_steps, t_end_of_slice_us[k]);
  //printf("%d;%d;%d\n", current_step, t_motor_step_us[current_step],k,t_accumulated_steps, t_end_of_slice_us[k]);
  //printf("VF_0 %d tpF0 %d\n", (int) VF_per_slice[0], t_motor_step_us[j-2]); 
  
  printf("Sa[%ld] = %d\n", 0, (int)Sa[0]);
  //Pour les autres tranches
  for(; k < nb_slices-1; k++)
  {

    printf("VM_per_slice[%d] = %d\n", k, (int) VM_per_slice[k]);
    printf("VM_per_slice[%d] = %d\n", k-1, (int) VM_per_slice[k-1]);
    printf("TM_per_slice[%d] = %d\n", k, (int) t_M_us[k]);
    printf("TM_per_slice[%d] = %d\n", k-1, (int) t_M_us[k-1]);
    Sa[k] = 1000000*(VM_per_slice[k] - VM_per_slice[k-1]) / (t_M_us[k] - t_M_us[k-1]);
    printf("Sa[%ld] = %d\n", k, (int)Sa[k]);

    compute_SaMaxSdMax(previous_step_idx_end_of_slice[0], previous_nb_steps, Pcrete_previous_cycle, &SaMax, &SdMax);
    if(! TEST_FLT_EQUALS(Sa[k], 0.f) )
    {
      if(Sa[k] > 0) {
        Sa[k] = MIN(SaMax, Sa[k]);
      }
      else { //Sa is actually a Sd (deceleration)
        Sa[k] = MAX(SdMax, Sa[k]);
      }
    }
    else {
      //Beware of slice 1 => V[0] is not a mean speed TODO
      VM_per_slice[k] = VM_per_slice[k-1];
    }
    //printf("VM_per_slice[%ld] = %d VM_per_slice[%ld] = %d\n", k, (int) VM_per_slice[k], k-1, (int)VM_per_slice[k-1]);
    //printf("TM_per_slice[%d] = %d TM_per_slice[%d] = %d\n", k, (int) t_M_us[k], k-1, (int)t_M_us[k-1]);
    printf("Sa[%ld] = %d\n", k, (int)Sa[k]);
        
    if(TEST_EQUALS(Sa[k], 0))
    {
      //We will apply the mean speed
      steps_offset_per_slice[k] = 1;
    }
    else {
      float VMpow2;
      VMpow2 = (VM_per_slice[k-1]*VM_per_slice[k-1]);
      float Sax2 = (2. * Sa[k]);
      float offset = fabs(VMpow2 / Sax2);
      steps_offset_per_slice[k] = (int) offset;
      //printf("off [%d] = %d Sa[%d] = %d\n", k, steps_offset_per_slice[k],k, (int) Sa[k]);	
    }
    for(uint32_t j = 0; t_accumulated_steps < t_end_of_slice_us[k]; j++, current_step++)
    {
      if(! TEST_FLT_EQUALS(Sa[k], 0.f) )
      {
        if(Sa[k] > 0.f)
        {
          t_motor_step_us[current_step] = 1000000 / ( sqrt(2 * ( j + steps_offset_per_slice[k]) * Sa[k]) );
        }
        else {
          t_motor_step_us[current_step] = 1000000 / ( sqrt(2 * (steps_offset_per_slice[k] -j) * fabs(Sa[k])));
        }
      }
      else { //Acceleration is 0. so speed is constant for all the slice
        //Mean speed
        t_motor_step_us[current_step] = 1000000 / VM_per_slice[k];
      }
      //printf("T_omegamax_ms = %d\n", T_omegamax_ms);
      //printf("Tminobj(%d, %d) = %d\n", current_step, get_setting_Vmax_Lpm(), T_minobj(current_step, get_setting_Vmax_Lpm())); 
      //printf("T_cmp(%d) = %d\n", current_step,t_motor_step_us[current_step]); 
      t_motor_step_us[current_step] = MAX(T_omegamax_ms,  MAX(T_minobj(current_step, get_setting_Vmax_Lpm()), t_motor_step_us[current_step]));

      //if(t_motor_step_us[current_step] <   && t_motor_step_us[current_step] < 10000)
      //{
      //  printf("VM[%d] = %d VM[%d] = %d\n", k, (int) VM_per_slice[k], k-1, (int) VM_per_slice[k-1]);
      //  printf("Sa[%d] = %d j = %d steps_offset_per_slice[%d] = %d\n", k, (int) Sa[k], j, k, steps_offset_per_slice[k]);
      //  printf("K = %d j = %d t_motor_step_us[%d] = %d\n", k, j, current_step, t_motor_step_us[current_step]);
      //  while(true) {};
      //}

      t_accumulated_steps = t_accumulated_steps + t_motor_step_us[current_step];
      //
      //We computed steps until the Ti
      //printf("%d;%d;%d; %d vs %d\n", current_step, t_motor_step_us[current_step],k,t_accumulated_steps, t_end_of_slice_us[k]);
      //printf("%d;%d;%d;\n", current_step, t_motor_step_us[current_step],k);
      if(t_accumulated_steps > Ti_ms*1000)
      {
        break;
      }
    }

    printf("t_end_of_slice_us[%d] = %d\n", k,(int)  t_end_of_slice_us[k]);
    printf("VM-1[%d] = %d, VM_computed [%d] = %d  VM_real [%d] = %d\n", k, (int)previous_VM_per_slice[k], 
									k, (int) VM_per_slice[k],
									k, (int) (1000000.f/t_motor_step_us[current_step-1])); 

    VM_per_slice[k] = 1000000.f/t_motor_step_us[current_step-1];
    //printf("VM [%ld] = %d\n", k, (int) VM_per_slice[k]);
    step_idx_end_of_slice[k] = current_step - 1; 
    //
    //
    //We computed steps until the Ti
    //This event should occurs only on the last slice
    //This code maybe useless
    if(t_accumulated_steps > Ti_ms*1000)
    {
      break;
    }
  }

  float VMpow2;
  const float final_deceleration = 300000.f;
  VMpow2 = (VM_per_slice[k-1]*VM_per_slice[k-1]);
  float Sax2 = (2. * final_deceleration);
  float offset = fabs(VMpow2 / Sax2);
  for(uint32_t j = 1; t_accumulated_steps < t_end_of_slice_us[k]; j++, current_step++)
  {
    t_motor_step_us[current_step] = 1000000 / ( sqrt(2 * (offset -j) * fabs(final_deceleration)));
    t_accumulated_steps += t_motor_step_us[current_step];
    //printf("%d;%d;%d; %d vs %d\n", current_step, t_motor_step_us[current_step],k,t_accumulated_steps, t_end_of_slice_us[k]);
    //printf("%d;%d;%d\n", current_step, t_motor_step_us[current_step],k,t_accumulated_steps, t_end_of_slice_us[k]);
  }
  step_idx_end_of_slice[k] = current_step - 1; 
  *nb_steps = step_idx_end_of_slice[k]; 
  //uint32_t t_acc=0;
  //k = 0;
  //for(uint32_t i=0; i < steps_end; i++)
  //{
  //  t_acc += t_motor_step_us[i];
  //  if (t_acc > t_end_of_slice_us[k]) k++;
  //  printf("%d;%d;%d;%d;%d\n", i, t_motor_step_us[i],k,t_acc,t_end_of_slice_us[k]);
  //}

}

char* ullx(uint64_t val)
{
    static char buf[34] = { [0 ... 33] = 0 };
    char* out = &buf[33];
    uint64_t hval = val;
    unsigned int hbase = 16;

    do {
        *out = "0123456789abcdef"[hval % hbase];
        --out;
        hval /= hbase;
    } while(hval);

    *out-- = 'x', *out = '0';
    return out;
}


void compute_seed( float     flow_set_point_initial_Lmin, //input:  spec v8 20-60   L/min
                   float     set_point_volume_mL,         //input:  spec v8 100-600 mL
                   uint32_t  nb_slices,                   //input:  ex 1 à 20 (k)
                   uint32_t  Ti_us,                       //input:  temps d'insuflation moteur arrete
                   float*    flow_set_point_per_slice,    //output: flowsp per slice 
                   float*    VM_per_slice,                 //output: V per slice
                   uint32_t* t_end_of_slice_us,           //output: T end of slice array
                   uint32_t* step_idx_end_of_slice,         //output: Nb steps of slice array 
                   uint32_t* t_motor_step_us,             //output: Array of steps time [MAX_MOTOR_STEPS]
                   uint32_t* nb_steps)                    //output: nb_steps
{											
  //Constantes initiales
  const uint32_t Vmax_tour_par_min    = 250;        //Vmax_tour_par_min
  const uint32_t pas_par_tour_mot     = 1000;       //pas_par_tour_mot
  const uint32_t Sa_pas_par_s2        = 45000;      //Sa_pas_par_s²,
  const uint32_t speed_down_t_ns      = 250;        //speed_down_t_ns,
  const uint32_t nb_steps_down_offset = 280;        //npas_down_offset pour 60Lmin/!\ à tester pour les autres débits
  const uint32_t speed_down_t2_ps     = 1150;       //speed_down_t2_us, /!\ pour 60mL à tester pour les autres débits
  const uint32_t Sd_pas_par_s2        = 300000;     //Sd_pas_par_s²,
  const float AxC50R5                 = 0.000204 ;  //constante courbe Vol(mL) de pas
  const float PxC50R5                 = 2.1605 ;    //puissance sur le numéro de pas Vol (pas)
  const float k_vol                   = 0.9f;       // We target 90% of the target volume for the seed
  const float k_debit                 = 1.f;         // 1 = we target the correct volume
  uint32_t t_pente_acc, t_debit_constant, t_pente_dec;
  uint32_t minimal_step_t_ns          = T_omegamax_ms * 1000;     //step_t_minQK_ns pour 600 l/min **** paramétrer la courbe en fonction du débit
  uint32_t step_total_us              = 0;
  uint32_t Vmax_pas_par_s             = ( (float) Vmax_tour_par_min / 60.f) * pas_par_tour_mot;
  uint32_t t_min_vmax_us              = 1000000/ Vmax_pas_par_s;
  uint32_t npas_acc                   = (Vmax_pas_par_s*Vmax_pas_par_s)/(2*Sa_pas_par_s2);
  uint32_t npas_dec                   = (Vmax_pas_par_s*Vmax_pas_par_s)/(2*Sd_pas_par_s2);
  uint32_t npas_stop                  = (uint32_t) powf(((set_point_volume_mL * k_vol)/AxC50R5), 1.f/PxC50R5);

  uint32_t max_steps                  = MIN(npas_stop, MAX_MOTOR_STEPS);

  //variabnpas_accles float intermédiaires
  float t_debit_constant_f;
  float n_pas_f ;
  float debit_cible_Ls                = flow_set_point_initial_Lmin/60.f;
  //fin variables float intermédiaires

  
  uint32_t k = 0; 
  uint32_t total_nb_steps   = 0;
  uint32_t t_end_acc = 0;

  for(unsigned int n_pas = 1; n_pas <= max_steps; n_pas++)
  {
	if(n_pas <= npas_acc) {
	  t_pente_acc = 1000000/sqrt(2*n_pas*Sa_pas_par_s2);
	}
	else {
	  if(t_end_acc == 0)
	  {
	    t_end_acc = step_total_us;
	  }
	  t_pente_acc = t_min_vmax_us;
	}
	
	//Pas QK
	if(n_pas<npas_stop)
	{
		n_pas_f = (float) n_pas ;
		t_debit_constant_f=1000.f*AxC50R5*PxC50R5/debit_cible_Ls*k_debit*(powf (n_pas_f, PxC50R5-1.f)) ;
		if(t_debit_constant_f < t_min_vmax_us)
		{
			t_debit_constant = t_min_vmax_us; //plafond vitesse moteur atteint
		}
		else {
			t_debit_constant = (unsigned int) t_debit_constant_f;
		}
	}
	else {
		t_debit_constant = t_min_vmax_us;
	}

	//Pas décélération
	if( n_pas>=npas_stop-npas_dec && n_pas<=npas_stop) {
		t_pente_dec = (1000000/sqrt(2*(npas_stop+1-n_pas)*Sd_pas_par_s2));
	}
	else {
		t_pente_dec = t_min_vmax_us;
	}
	t_motor_step_us[n_pas-1] = MAX(t_pente_acc, MAX(t_debit_constant, t_pente_dec));
	total_nb_steps++;
	step_total_us += t_motor_step_us[n_pas-1];
  }

  //Compute t end of slice and VM per slice
  uint32_t Ti = step_total_us;
  uint32_t t_slice = (Ti - t_end_acc) / NB_SLICE;
  k = 0;
  float VM_acc = 0.0;
  uint32_t t_acc;
  uint32_t step_start_of_slice;
  for(uint32_t j = 0; j < total_nb_steps; j++)
  {
    if(t_acc > (t_end_acc + k*t_slice) )
    {
      VM_per_slice[k] = VM_acc /   ( (float)  (j - step_start_of_slice));
      flow_set_point_per_slice[k] = flow_set_point_initial_Lmin;
      step_idx_end_of_slice[k] = j-1;
      step_start_of_slice = j;
      k++;
      VM_acc = 0;
    }
    t_end_of_slice_us[k] += t_motor_step_us[j];
    VM_acc += 1000000/t_motor_step_us[j];
    t_acc += t_motor_step_us[j];
  }
  flow_set_point_per_slice[0] = 0;
  flow_set_point_per_slice[NB_SLICE+1] = 0;
  *nb_steps = total_nb_steps;
};

