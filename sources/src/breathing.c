#include "common.h"
#include "controller.h"
#include "breathing.h"
#include "platform.h"
#include "config.h"

#include <compute_motor.h>
#include <math.h>


//----------------------------------------------------------
// Private defines
//----------------------------------------------------------

#define PERIOD_BREATING_MS            (10)
#define MAX_PEP_SAMPLES               (100 / PERIOD_BREATING_MS) // moyenne glissante sur les 100ms dernieres de l'expi
#define MAX_PPLAT_SAMPLES             (50  / PERIOD_BREATING_MS) // moyenne glissante sur les 50ms dernieres de plat


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

    // _flow_samples_count=0;
    // A_calibrated= 3.577;
    // B_calibrated= -0.455;

    motor_enable(true);

    g_EoI_ratio=0;
    g_FR_pm=0;
    g_VTe_mL=0;
    g_Pcrete_cmH2O=0;
    g_Pplat_cmH2O=0;
    g_PEP_cmH2O=0;



    do  {
      brth_printf("BRTH: start cycle\n");

      enter_state(Insufflation);

      // TODO: Take into account the time to compute adaptation for the FR calculation ??!!??

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
      brth_printf("BRTH: Ti    : %lu\n", Ti);

      // adaptation(VM, _flow_samples, _flow_samples_count, 0.001*FLOW_SAMPLING_PERIOD_MS, &A_calibrated, &B_calibrated);
      // _flow_samples_count = 0;

	  uint32_t d = 300;
	  uint32_t steps = 4000;
	  //compute_constant_motor_steps(d, _steps, g_motor_steps_us);
	  compute_motor_press_christophe(350000, 2000, 65000, 20, 14, 350000, 4000, steps, g_motor_steps_us);
	  brth_printf("T_C = %ld Patmo = %ld\n", (int32_t) (read_temp_degreeC()*100), (int32_t) (read_Patmo_mbar()*100));

      // Start Inhalation
      valve_inhale();
      motor_press(g_motor_steps_us, steps);
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
          g_Pcrete_cmH2O = MAX(g_Pcrete_cmH2O, read_Paw_cmH2O());
          wait_ms(PERIOD_BREATING_MS);
      }
      motor_release(MOTOR_RELEASE_STEP_US);
      while(Plateau == g_state) {
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

