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
#define NB_SLICE                      (20)
#define MAX_FLOW_SAMPLES              (400)


//----------------------------------------------------------
// Private typedefs
//----------------------------------------------------------

typedef enum { Insufflation, Plateau, Exhalation, Finished } BreathingState;


//----------------------------------------------------------
// Private variables
//----------------------------------------------------------

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
static uint32_t       g_motor_steps_us[MAX_MOTOR_STEPS] = {0};  // TODO: Make it configurable with a define. This represent a physical limit a the system.

static uint32_t       g_Tslice; //Duration of a slice
static float          g_error_slice_samples[NB_SLICE+2];
static uint32_t       g_t_end_of_slice_us[NB_SLICE+2];

//Set point
static float          g_set_point_per_slice[2][NB_SLICE];
static float*         g_current_set_point_per_slice  = g_set_point_per_slice[0];
static float*         g_previous_set_point_per_slice = g_set_point_per_slice[1];

//V per slice
static float          g_V_per_slice[2][NB_SLICE];
static float*         g_current_V_per_slice  = g_V_per_slice[0];
static float*         g_previous_V_per_slice = g_V_per_slice[1];

//NB steps per slice
static uint32_t       g_nb_steps_per_slice[2][NB_SLICE];
static uint32_t*      g_current_nb_steps_per_slices  = g_nb_steps_per_slice[0];
static uint32_t*      g_previous_nb_steps_per_slices = g_nb_steps_per_slice[1];

//Nb total steps
static uint32_t        g_current_nb_steps         = 0;
static uint32_t        g_previous_nb_steps        = 0;

static uint32_t        g_current_nb_slices        = 0;
static uint32_t        g_previous_nb_slices       = 0;

static uint32_t        g_current_slice_start_t_ms = 0;



static float           g_flow_samples[MAX_FLOW_SAMPLES]; //  used as buffer for flow analysis and motor command computing
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
                                          float     flow_set_point,             //input:     Flow set point
                                          uint32_t  new_Tslice,                 //input:     Duration of the future slice
                                          float     *error_slice_samples,       //ouput:     error during each slice
                                          uint32_t  *t_end_of_slice_us,         //output:    Computed time of end of the slice array (Only one is updated)
                                          uint32_t  *slice_idx,                 //in/output: Current slice (will be incremented)
                                          uint32_t  *slice_start_t_ms,          //output:    Start time of the next slice
                                          uint32_t  *Tslice);                   //output:    Duration of the next slice

static void   compute_set_point_per_slice(float     flow_set_point,             //input: Global set point
                                          float*    error_slice_samples,        //input: Error per slices array
                                          uint32_t* t_end_of_slice_us,             //input: Endtime of slcie array
                                          uint32_t  nb_slices,                  //input: nb slices within arrays
                                          float*    set_point_per_slice);       //output: new setpoint per slice array


static void   apply_correction(float*    set_point_per_slice,                   //input:  set point per slice array
                               uint32_t* t_end_of_slice_us,                        //input:  t_end_of_slice of the end of the slice array
                               float*    previous_set_point_per_slice,          //input:  Previous set point per slice array
                               float*    previous_V_per_slice,                  //input:  Previous mean speed per slice array
                               uint32_t* previous_nb_steps_at_this_slice,       //input:  Previous cycle nb steps in this slice
                               uint32_t  previous_nb_steps,                     //input:  Previous nb total steps during cycle
                               float     Pcrete_previous_cycle,                 //input:  Pcrete Recorded during previous cycle
                               uint32_t  nb_slices,                             //input:  Nb slices within array
                               uint32_t  Ti,                                    //input:  Time of inhalation
                               float*    V_per_slice,                           //output: mean speed per slice array
                               uint32_t* nb_steps_per_slice,                    //output: nb steps per slice array
                               uint32_t* t_motor_step_us,                      //output: motor steps array
                               uint32_t* current_nb_steps);                     //output: nb steps within array


void compute_seed( float     flow_set_point_initial_Lmin, //input:  spec v8 20-60   L/min
                   float     set_point_volume_mL,         //input:  spec v8 100-600 mL
                   uint32_t  nb_slices,                   //input:  ex 1 à 20 (k)
                   uint32_t  Ti_us,                       //input:  temps d'insuflation moteur arrete
                   float*    flow_set_point_per_slice,    //output: flowsp per slice 
                   float*    V_per_slice,                 //output: V per slice
                   uint32_t* t_end_of_slice_us,           //output: T end of slice array
                   uint32_t* nb_steps_per_slices,         //output: Nb steps of slice array 
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
      uint32_t Ti = T*1/3;  // TODO: Check how to calculate Tinsuflation
      brth_printf("BRTH: Ti    : %ld\n", Ti);
      //Is there any slicing done ?
      if(g_current_nb_slices == 0) //NO
      {

        compute_seed( get_setting_Vmax_Lpm(), //input:  spec v8 20-60   L/min
                      get_setting_VT_mL(),    //input:  spec v8 100-600 mL
                      NB_SLICE,                      //input:  ex 1 à 20 (k)
                      (get_setting_VT_mL()*(60*1000000/1000))/get_setting_Vmax_Lpm(),       //input:  temps d'insuflation moteur arrete in us (Ti est en ms)
                      g_current_set_point_per_slice, //output: flowsp per slice 
                      g_current_V_per_slice,         //output: V per slice
                      g_t_end_of_slice_us,           //output: T end of slice array
                      g_current_nb_steps_per_slices, //output: Nb steps of slice array 
                      g_motor_steps_us,              //output: Array of steps time [MAX_MOTOR_STEPS]
                      &g_current_nb_steps);          //output: nb_steps
      }
      //YES
      //if(g_current_nb_slices > 0)
      //{
      //  compute_set_point_per_slice(get_setting_Vmax_Lpm(),
      //                              g_error_slice_samples,
      //                              g_t_end_of_slice_us,
      //                              g_current_nb_slices,
      //                              g_current_set_point_per_slice);
      //  //for(unsigned int i = 0; i < g_current_nb_slices-2; i++)
      //  //{
      //  //  printf("Set_point[%d] = %d\n", i, (int)(g_current_set_point_per_slice[i]*100.f));
      //  //}


      //  apply_correction(g_current_set_point_per_slice,           //input:  set point per slice array
      //                   g_t_end_of_slice_us,                     //input:  t_end_of_slice of the end of the slice array
      //                   g_previous_set_point_per_slice,          //input:  Previous set point per slice array
      //                   g_previous_V_per_slice,                  //input:  Previous mean speed per slice array
      //                   g_previous_nb_steps_per_slices,          //input:  Previous cycle nb steps in this slice
      //                   g_previous_nb_steps,                     //input:  Previous nb total steps during cycle
      //                   get_breathing_Pcrete_cmH2O(),            //input:  Pcrete Recorded during previous cycle
      //                   g_current_nb_slices,                     //input:  Nb slices within array
      //                   Ti,                                      //input:  Time of inhalation
      //                   g_current_V_per_slice,                   //output: mean speed per slice array
      //                   g_current_nb_steps_per_slices,           //output: nb steps per slice array
      //                   g_motor_steps_us,                        //output: motor steps array
      //                   &g_current_nb_steps);                    //output: nb steps within array

      //  //print_steps(g_motor_steps_us, MAX_MOTOR_STEPS);
      //  while(true);
      //  //for(unsigned int i = 0; i < nb_slices; i++)
      //  //{
      //  //  printf("Error %d = %d end at %"PRIu32"\n", i, (int) (100.f* error_slice_samples[i]), previous_t_end_of_slice_us[i]);
      //  //}
      //}
      //Swap set_point_per_slice, V_per_slice and nb_steps_per_slice between current and previous buffer
      float* tmp_set_point_per_slice    = g_previous_set_point_per_slice;
      float* tmp_V_per_slice            = g_previous_V_per_slice;
      uint32_t* tmp_nb_steps_per_slices = g_previous_nb_steps_per_slices;
      g_previous_set_point_per_slice    = g_current_set_point_per_slice;
      g_previous_V_per_slice            = g_current_V_per_slice;
      g_current_set_point_per_slice     = tmp_set_point_per_slice;
      g_current_V_per_slice             = tmp_V_per_slice;
      g_current_nb_steps_per_slices     = tmp_nb_steps_per_slices;
      g_previous_nb_slices              = g_current_nb_slices;
      g_current_nb_slices               = 0;

      unsigned int Ta = 80;  //Duration of the acceleration phase (computed)
      g_Tslice = Ta;
      g_current_slice_start_t_ms = get_time_ms();
      enter_state(Insufflation);

      // Start Inhalation
      valve_inhale();
      motor_press(g_motor_steps_us, g_current_nb_steps);
      g_previous_nb_steps = g_current_nb_steps;
      reset_Vol_mL();
      brth_print("BRTH: Insuflation\n");      
      while(Insufflation == g_state ) {
          if (Pmax <= read_Paw_cmH2O()) {
              brth_printf("BRTH: Paw [%ld]> Pmax --> Exhalation\n", (int32_t)(read_Paw_cmH2O()));
              enter_state(Exhalation);
              break;
          } else if (VT <= read_Vol_mL()) {
              brth_printf("BRTH: vol [%ld]>= VT --> Plateau\n", (int32_t)(read_Vol_mL()));
              enter_state(Plateau);
              break;
          } else 
          if( Ti <= (get_time_ms() - g_cycle_start_ms) ) {
              brth_printf("BRTH: dt [%lu]>= Ti\n", (get_time_ms() - g_cycle_start_ms));
              enter_state(Plateau);
              break;
          }
          //Sample flow for later adaptation.
          if(g_flow_samples_count<MAX_FLOW_SAMPLES) {
            g_flow_samples[g_flow_samples_count] = read_Pdiff_Lpm();  // in sls
            ++g_flow_samples_count;
          }

          if(g_Tslice <= (get_time_ms() - g_current_slice_start_t_ms) + PERIOD_BREATING_MS)
          {
            compute_error_slice_samples(g_flow_samples,
                                        &g_flow_samples_count,
                                        get_setting_Vmax_Lpm(),
                                        (Ti-Ta)/NB_SLICE,
                                        g_error_slice_samples,
                                        g_t_end_of_slice_us,
                                        &g_current_nb_slices,
                                        &g_current_slice_start_t_ms,
                                        &g_Tslice);
          }
          g_Pcrete_cmH2O = MAX(g_Pcrete_cmH2O, read_Paw_cmH2O());
          wait_ms(PERIOD_BREATING_MS);
      }
      motor_release(MOTOR_RELEASE_STEP_US);
      while(Plateau == g_state) {
        if(g_flow_samples_count<MAX_FLOW_SAMPLES) {
          g_flow_samples[g_flow_samples_count] = read_Pdiff_Lpm()/60.;  // in sls
          ++g_flow_samples_count;
        }

        if (Pmax <= read_Paw_cmH2O()) { 
            brth_print("BRTH: Paw > Pmax --> Exhalation\n");
            g_Pplat_cmH2O = get_Pplat_avg_cmH2O();
            enter_state(Exhalation);
        } else if ( is_command_Tpins_expired() && (Tplat <= (get_time_ms() - g_state_start_ms)) ) {
            brth_print("BRTH: Tpins expired && (dt > Tplat)\n");
            g_Pplat_cmH2O = get_Pplat_avg_cmH2O();
            enter_state(Exhalation);
        }
        sample_Pplat_cmH2O(read_Paw_cmH2O());
        g_Pcrete_cmH2O = MAX(g_Pcrete_cmH2O, read_Paw_cmH2O());
        wait_ms(PERIOD_BREATING_MS);
      }
      compute_error_slice_samples(g_flow_samples,
                                  &g_flow_samples_count,
                                  get_setting_Vmax_Lpm(),
                                  (Ti-Ta)/NB_SLICE,
                                  g_error_slice_samples,
                                  g_t_end_of_slice_us,
                                  &g_current_nb_slices,
                                  &g_current_slice_start_t_ms,
                                  &g_Tslice);
      g_VTi_mL= read_Vol_mL();
      valve_exhale();
      float VTe_start_mL=0.;
      while(Exhalation == g_state) { 
          if ( T <= (get_time_ms() - g_cycle_start_ms )) { 
              uint32_t t_ms = get_time_ms();

              g_PEP_cmH2O = get_PEP_avg_cmH2O();
              g_EoI_ratio =  (float)(t_ms-g_cycle_start_ms)/(g_state_start_ms-g_cycle_start_ms);
              g_FR_pm     = 1./(((float)(t_ms-g_cycle_start_ms))/1000/60);
              g_VTe_mL = VTe_start_mL - read_Vol_mL();
              g_VMe_Lpm   = (g_VTe_mL/1000) * g_FR_pm;

              xEventGroupSetBits(brthCycleState, BRTH_RESULT_UPDATED);

              regulation_pep();
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
  g_state= newState;
  g_state_start_ms = get_time_ms();
  EventBits_t brthState =0;
  switch(g_state) {
    case Insufflation:
      g_cycle_start_ms = get_time_ms();
      brthState =  BRTH_CYCLE_INSUFLATION;
      brth_printf("BRTH: Insuflation\n");
      break;
    case Plateau:
      brthState =  BRTH_CYCLE_PLATEAU;
      brth_printf("BRTH: Plateau\n");
      break;
    case Exhalation:
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


float get_breathing_EoI_ratio()     { return g_EoI_ratio; }
float get_breathing_FR_pm()         { return g_FR_pm; }
float get_breathing_VTe_mL()        { return g_VTe_mL; }
float get_breathing_VTi_mL()        { return g_VTi_mL; }
float get_breathing_VMe_Lpm()       { return g_VMe_Lpm; }
float get_breathing_Pcrete_cmH2O()  { return g_Pcrete_cmH2O; }
float get_Pplat_cmH20()             { return g_Pplat_cmH2O; }
float get_PEP_cmH2O()               { return g_PEP_cmH2O; }



//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

static void regulation_pep() {
  float pep_objective = get_setting_PEP_cmH2O();      // TODO: is it really what we want ? Should we use the setting retreived at the beginning of the cycle instead ?
  float current_pep = get_PEP_cmH2O();
  //Do not do regulation if the current pep is irrelevant
  if(current_pep < 0.0f)
    return;
  int relative_pep = (pep_objective*10.f - current_pep*10.f);
  if(abs(relative_pep) > 3) {
    motor_pep_move( (int) ((float)relative_pep/MOTOR_PEP_PEP_TO_MM_FACTOR));
  }
}

static void init_sample_PEP_cmH2O()
{
    //Samples PEP for a rolling average 
  g_PEP_cmH2O_samples_index = 0;
  for(int i = 0; i < MAX_PEP_SAMPLES; i++)
    g_PEP_cmH2O_samples[i] = 0;

}

static void sample_PEP_cmH2O( float Paw_cmH2O)
{
  g_PEP_cmH2O_samples[g_PEP_cmH2O_samples_index] = Paw_cmH2O;
  g_PEP_cmH2O_samples_index = (g_PEP_cmH2O_samples_index + 1) % MAX_PEP_SAMPLES; 
}

static float get_PEP_avg_cmH2O()
{
  float sum_PEP = 0;
  for(int i=0; i < MAX_PEP_SAMPLES; i++)
  {
    sum_PEP += g_PEP_cmH2O_samples[i];
  }
  return (sum_PEP / MAX_PEP_SAMPLES);
}

static void init_sample_Pplat_cmH2O()
{
    //Samples Pplat for a rolling average 
  g_Pplat_cmH2O_samples_index = 0;
  for(int i = 0; i < MAX_PPLAT_SAMPLES; i++)
    g_Pplat_cmH2O_samples[i] = 0;

}

static void sample_Pplat_cmH2O( float Paw_cmH2O)
{
  g_Pplat_cmH2O_samples[g_Pplat_cmH2O_samples_index] = Paw_cmH2O;
  g_Pplat_cmH2O_samples_index = (g_Pplat_cmH2O_samples_index + 1) % MAX_PPLAT_SAMPLES; 
}

static float get_Pplat_avg_cmH2O()
{
  float sum_Pplat = 0;
  for(int i=0; i < MAX_PPLAT_SAMPLES; i++)
  {
    sum_Pplat += g_Pplat_cmH2O_samples[i];
  }
  return (sum_Pplat / MAX_PPLAT_SAMPLES);
}

static void compute_error_slice_samples(float* flow_samples,          //input:     Samples of flow array
                                        uint32_t* flow_samples_count, //input:     size of samples array
                                        float     flow_set_point,      //input:     Flow set point
                                        uint32_t  new_Tslice,          //input:     Duration of the future slice
                                        float*    error_slice_samples, //ouput:     error during each slice
                                        uint32_t  *t_end_of_slice_us,  //output:    Computed time of end of the slice array (Only one is updated)
                                        uint32_t  *slice_idx,          //in/output: Current slice (will be incremented)
                                        uint32_t  *slice_start_t_ms,   //output:    Start time of the next slice
                                        uint32_t  *Tslice)             //output:    Duration of the next slice
{
  float error = 0.0;
  for(unsigned int i=0; i < *flow_samples_count; i++)
  {
      error += (flow_samples[i] - flow_set_point) / *flow_samples_count;
  }
  error_slice_samples[*slice_idx] = error;
  t_end_of_slice_us[*slice_idx] = get_time_ms()-g_cycle_start_ms;
  *slice_start_t_ms = get_time_ms();
  (*slice_idx)++;
  //Reset sampling per slice
  *flow_samples_count=0;
  *Tslice = new_Tslice;
}

static void compute_set_point_per_slice(float     flow_set_point,
                                        float*    error_slice_samples,
                                        uint32_t* previous_t_end_of_slice_us,
                                        uint32_t  nb_slices,
                                        float*    set_point_per_slice)
{
  float error_total = 0.0f;
  for(unsigned int i = 0; i < nb_slices+2; i++)
  {
    error_total += error_slice_samples[i];
  }
  uint32_t t_corrected = previous_t_end_of_slice_us[nb_slices] - previous_t_end_of_slice_us[0];
  float kp = 0.2;

  float new_set_point_global = flow_set_point - ( (error_total/ t_corrected) * kp);
  for(unsigned int i=1; i < nb_slices+1; i++)
  {
     set_point_per_slice[i] = new_set_point_global - error_slice_samples[i];
     printf("per slice = %d\n", (int) set_point_per_slice[i]);
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
                               float* SdMax)
{
  if(SaMax != NULL)
  {
    *SaMax = 45000;
  }
  if(SdMax != NULL)
  {
    *SdMax = 45000;
  }
}




static void apply_correction(float*    set_point_per_slice,                   //input:  set point per slice array
                             uint32_t* t_end_of_slice_us,                        //input:  t_end_of_slice_us of the end of the slice array
                             float*    previous_set_point_per_slice,          //input:  Previous set point per slice array
                             float*    previous_V_per_slice,                  //input:  Previous mean speed per slice array
                             uint32_t* previous_nb_steps_at_this_slice,       //input:  Previous cycle nb steps in this slice
                             uint32_t  previous_nb_steps,                     //input:  Previous nb total steps during cycle
                             float     Pcrete_previous_cycle,                 //input:  Pcrete Recorded during previous cycle
                             uint32_t  nb_slices,                             //input:  Nb slices within array
                             uint32_t  Ti,                                    //input:  Time of inhalation
                             float*    V_per_slice,                           //output: mean speed per slice array
                             uint32_t* nb_steps_per_slice,                    //output: nb steps per slice array
                             uint32_t* t_motor_step_us,                      //output: motor steps array
                             uint32_t* nb_steps)                      //output: nb steps within array
{
  float Sa[NB_SLICE+2];//Acceleration per slice
  uint32_t steps_offset_per_slice[NB_SLICE+2];
  float SaMax, SdMax;
  for(uint32_t k = 0; k < nb_slices+2; k++)
  {
    printf("V[%d] = %d\n", k, (int) (previous_V_per_slice[k])); 
  }

  V_per_slice[0] = previous_V_per_slice[0]; //V0 du seed
  for(int k = 1; k < NB_SLICE+1;k++)
  {
    printf("Prev V_per_slice[%d] = %d\n", k, ((int)previous_V_per_slice[k]));
    printf("set point current [%d] = %d\n", k, ((int)set_point_per_slice[k]));
    printf("prev set point current [%d] = %d\n", k, ((int)previous_set_point_per_slice[k]));
    V_per_slice[k] = previous_V_per_slice[k] * (set_point_per_slice[k]/previous_set_point_per_slice[k]) * 1.f/abaque(k);
  }

  compute_SaMaxSdMax(previous_nb_steps_at_this_slice[0], previous_nb_steps, Pcrete_previous_cycle, &Sa[0], NULL);
  for(unsigned int k = 1; k < nb_slices+1; k++)
  {
    Sa[k] = (V_per_slice[k] + V_per_slice[k-1])/2 * (t_end_of_slice_us[k] - t_end_of_slice_us[k-1]);
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
  nb_steps_per_slice[0] = 1/2*t_end_of_slice_us[0]*t_end_of_slice_us[0]/Sa[0];
  for(uint32_t j = 1; j < nb_steps_per_slice[0]+1; j++)
  {
    t_motor_step_us[j-1] = 1000000 /(sqrtf(2*j*Sa[j]));
    t_accumulated_steps+=t_motor_step_us[j-1];
    current_step++;
  }

  //Pour les autres tranches
  uint32_t k;
  for(k = 1; k < nb_slices; k++)
  {
    if(TEST_EQUALS(Sa[k], 0))
    {
      //We will apply the mean speed
      nb_steps_per_slice[k] = ((V_per_slice[k] - V_per_slice[k-1])/2) * (t_end_of_slice_us[k] - t_end_of_slice_us[k-1]);
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
        t_motor_step_us[current_step] = 1000000 / ( sqrt(2 * j * fabs(Sa[k])) );
      }
      else { //Acceleration is 0. so speed is constant for all the slice
        //Mean speed
        t_motor_step_us[current_step] = 1000000 / V_per_slice[k];
      }
      t_accumulated_steps += t_motor_step_us[current_step];
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
  uint32_t nb_step_end = 1000000/(t_motor_step_us[steps_end]*t_motor_step_us[steps_end] * 2 * Sd);
  //uint32_t t_stop = (1/(t_motor_step_us[steps_end])) * (1/Sd);
  current_step -= nb_step_end;
  uint32_t new_step_end = current_step + nb_step_end;
  for(nb_steps_per_slice; current_step < new_step_end; current_step++)
  {
    //Remove the previously computed time
    t_accumulated_steps -= t_motor_step_us[current_step];
    t_motor_step_us[current_step] = (1000000) / ( sqrtf(2*(new_step_end-current_step) * Sd) );
    t_accumulated_steps += t_motor_step_us[current_step];
  }

  //uint32_t t_acc=0;
  //k = 0;
  //for(uint32_t i=0; i < steps_end && i < 1250; i++)
  //{
  //  t_acc += t_motor_step_us[i];
  //  if (t_acc > t_end_of_slice_us[k]) k++;
  //  printf("%d;%d;%d\n", i, t_motor_step_us[i],k);
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
                   float*    V_per_slice,                 //output: V per slice
                   uint32_t* t_end_of_slice_us,           //output: T end of slice array
                   uint32_t* nb_steps_per_slices,         //output: Nb steps of slice array 
                   uint32_t* t_motor_step_us,             //output: Array of steps time [MAX_MOTOR_STEPS]
                   uint32_t* nb_steps)                    //output: nb_steps
{											
  //Constantes initiales
  const uint32_t Vmax_tour_par_min    = 250;        //Vmax_tour_par_min
  const uint32_t pas_par_tour_mot     = 1000;       //pas_par_tour_mot
  const uint32_t Sa_pas_par_s2        = 45000;      //Sa_pas_par_s²,
  
  uint32_t step_t_minQK_ns            = 380000;     //step_t_minQK_ns pour 600 l/min **** paramétrer la courbe en fonction du débit
  const uint32_t speed_down_t_ns      = 250;        //speed_down_t_ns,
  const uint32_t nb_steps_down_offset = 280;        //npas_down_offset pour 60Lmin/!\ à tester pour les autres débits
  const uint32_t speed_down_t2_ps     = 1150;       //speed_down_t2_us, /!\ pour 60mL à tester pour les autres débits
  const uint32_t Sd_pas_par_s2        = 300000;     //Sd_pas_par_s²,
  const float Ax                      = 6.14942f;
  const float Bx                      = 1/32.f;
  const float Px                      = 1.38486488f;
  const float corr_obj_vol            = 0.9f;
  const float corr_obj_flow           = 0.9f;
	
  //paraméter les paramètres conservatoires du BAVU Volume fonction de nb_steps_stop
  // a vide, mesure Adrien VmL=6,14942*(nbpas/32)^1,38486488
  
  //mesure Christophe pour C50 R05 PEP 0 = undershoot 1.37 pour 600mL (990 à 1090 pas au lieu de 874)
  //uint32_t nb_steps_stop = (uint32_t) (pow((set_point_volume_mL*corr_obj_vol/Ax),(1/Px))/Bx); //conservatoire 2x (Adrien à vide vs C50R5 et corr_obj
  uint32_t  nb_steps_stop = UINT32_MAX;
  //correction débit cible approximative
  step_t_minQK_ns = (uint32_t) (((float) step_t_minQK_ns) * flow_set_point_initial_Lmin /60.f / corr_obj_flow);
  //Estimation grossière de t0 à 95% l'accélération pour speeddown_t_ns
  //traiter les corrections sur les 3 variables de décélération TODO christophe
  
  //Calcul accélération initiale et tranches de temps et débits cibles (facile)
  //t_end_of_slice_us[0] = (1000000000.f / step_t_minQK_ns*Sa_pas_par_s2); // approximé au prorata de temps de pas mini de l'expérience
  
  t_end_of_slice_us[0] = (uint32_t) 40000;


  flow_set_point_per_slice[0] = flow_set_point_initial_Lmin;
  for (uint32_t k = 1 ; k < nb_slices + 1; k++)
  {
    t_end_of_slice_us[k] = t_end_of_slice_us[0] + ((Ti_us - t_end_of_slice_us[0])/nb_slices) *k;
    flow_set_point_per_slice[k] = flow_set_point_initial_Lmin;
  }
  t_end_of_slice_us[nb_slices] = Ti_us; //evite les erreurs d'arrondi
  flow_set_point_per_slice[nb_slices+1] = 0;

  //table des pas
  uint32_t step_total_us = 0;
  const uint32_t max_steps = MIN(nb_steps_stop, MAX_MOTOR_STEPS); //sécurité au cas ou
  unsigned int pente_acc, flow_constant;
  unsigned int Vmax_pas_par_s = (Vmax_tour_par_min / 60) * pas_par_tour_mot;
  unsigned int step_t_min_vmax_us = 1000000/ Vmax_pas_par_s;
  unsigned int npas_acc = (Vmax_pas_par_s*Vmax_pas_par_s)/(2*Sa_pas_par_s2);
  unsigned int npas_dec = (Vmax_pas_par_s*Vmax_pas_par_s)/(2*Sd_pas_par_s2);
  //variables float intermédiaires
  float flow_constant_f;
  //fin variables float intermédiaires
  
  uint32_t k = 0; 
  uint32_t total_steps_t_us = 0;
  uint32_t total_nb_steps   = 0;
  for(uint32_t step = 1; step <= max_steps && total_nb_steps < Ti_us; step++)
  {
  	//NB: Pas acceleration pas tout à fait tranche 0
  	///Initialiser k=0
  	///initialiser un t_accumulated_steps à 0
  	///initialiser un Vmoy_accumule (resetté partranche)
  	if(step <= npas_acc) {
  		pente_acc = 1000000/sqrt(2*step*Sa_pas_par_s2);
  	}
  	else {
  		pente_acc = step_t_min_vmax_us;
  	}
  
  	//Pas QK
  	if(step<nb_steps_stop)
  	{
  		float step_f = (float) step ;
  		flow_constant_f = ( (step_t_minQK_ns/1000.f)+
						    ((step_f-nb_steps_down_offset)*speed_down_t_ns/1000.f)+
						    (speed_down_t2_ps*(step_f-nb_steps_down_offset)*(step_f-nb_steps_down_offset))/1000000.f);
  		flow_constant = (unsigned int) flow_constant_f;
  	}
  	else {
  		flow_constant = step_t_min_vmax_us;
  	}

  	t_motor_step_us[step-1] = MAX(pente_acc, flow_constant);
	//If end of slice reached
	if(step_total_us < t_end_of_slice_us[k] 
			&& t_end_of_slice_us[k] <= step_total_us + t_motor_step_us[step-1])
	{
		//We've done the last slice => we have to correct it
		if(k == nb_slices) {
		  uint32_t steps_end = step-1;
		  float Sd;
		  compute_SaMaxSdMax(nb_steps_per_slices[k], step_total_us, 0.f, NULL, &Sd);


		  uint32_t tsquare = t_motor_step_us[steps_end]*t_motor_step_us[steps_end];
		  //printf("Tsqaure = %d\n", tsquare);
		  uint32_t deno = ( (tsquare/1000) * 2 * (Sd/1000) );
		  //printf("deno = %d\n", deno);

		  uint32_t nb_step_end = (uint32_t) 1000000.f/deno;
		  //printf("NB step end = %d\n", nb_step_end);
		  //uint32_t t_stop = (1/(t_motor_step_us[steps_end])) * (1/Sd);
		  uint32_t current_step = (step-1) - nb_step_end;
		  uint32_t new_step_end = current_step + nb_step_end;
		  
		  for(; current_step < new_step_end; current_step++)
		  {
		    //Remove the previously computed time
		    step_total_us -= t_motor_step_us[current_step];
		   
		    uint32_t pre_sqrt1 =  2*(new_step_end-current_step) ;
		    uint32_t pre_sqrt =   pre_sqrt1 * Sd;
		    uint32_t new_t_motor_step_us = (1000000) / ( sqrtf(pre_sqrt) );
		    //printf("%d old %d vs new %d\n", current_step, t_motor_step_us[current_step], new_t_motor_step_us);
		    t_motor_step_us[current_step] = new_t_motor_step_us;
		    step_total_us += t_motor_step_us[current_step];
		  } 
		}

		t_end_of_slice_us[k] = step_total_us;
		nb_steps_per_slices[k] = step-1 - total_nb_steps;
		V_per_slice[k] = 0;
		for(uint32_t j = total_nb_steps; j < (step-1); j++)
		{
		  V_per_slice[k] += (1000000.f/t_motor_step_us[j]) / (nb_steps_per_slices[k]);
		}
		
		total_nb_steps += nb_steps_per_slices[k];
		k++;
	}
	step_total_us += t_motor_step_us[step-1];
  }
  *nb_steps = total_nb_steps;
  k = 0;
  uint32_t t_accumulated_steps = 0;
  //for(uint32_t i=0; i < *nb_steps && i < max_steps; i++)
  //{
  //  t_accumulated_steps += t_motor_step_us[i];
  //  if (t_accumulated_steps > t_end_of_slice_us[k]) k++;
  //  printf("%d;%d;%d\n", i, t_motor_step_us[i],k);
  //}
  for(uint32_t k = 0; k < nb_slices+2; k++)
  {
    printf("0V[%d] = %d\n", k, (int) (V_per_slice[k])); 
  }
};
//il faudrait tester que step_total_us est <= Ti, sinon refaire la "fin"

//voir précédemment ou refaire le calcul de la table des pas "fin de tranche", basée sur le temps à partir de la table de pas moteur
//voir précédemment ou faire le calcul de la table des vitesse moyennes (accumuler le temps de la tranche et diviser

