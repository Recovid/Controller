#include "recovid.h"
#include "controller.h"
#include "breathing.h"
#include "platform.h"

typedef enum { Insufflation, Plateau, Exhalation, ExhalationPause, Unknown } BreathingState;


static BreathingState _state; // TODO: expose this state in the breathing task interface
static uint32_t _state_start_ms;
static uint32_t _cycle_start_ms;

static float EoI_ratio;       // TODO: expose this data in the  breathing task interface
static float FR_pm;           // TODO: expose this data in the  breathing task interface


static float A_calibrated;
static float B_calibrated;

#define MAX_MOTOR_STEPS         (4800)

static uint16_t _motor_steps_us[MAX_MOTOR_STEPS] = {0};  // TODO: Make it configurable with a define. This represent a physical limit a the system.
static uint32_t _steps;


static float compte_motor_step_time(long step_number, float desired_flow, float A, float B, float speed) {
	float res = (0.8*A*speed*speed*step_number) + B * speed;
	res = res / desired_flow;
	if (res * 1000000 < 110) {return 110;}
	else {return res * 1000000.;}
}



static void enter_state(BreathingState newState) {
  _state= newState;
  _state_start_ms = get_time_ms();
  if (_state == Insufflation) {
      _cycle_start_ms = get_time_ms();
  }  
#ifdef DEBUG
  switch(_state) {
    case Insufflation: brth_printf("BRTH: Insuflation\n"); break;
    case Plateau: brth_printf("BRTH: Plateau\n"); break;
    case Exhalation: brth_printf("BRTH: Exhalation\n"); break;
  }
#endif
}


void breathing_run(void *args) {
  UNUSED(args);
  EventBits_t events;

  while(true) {
    brth_printf("BRTH: Standby\n");
    events= xEventGroupWaitBits(eventFlags, BREATHING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    brth_printf("BRTH: Started\n");


    A_calibrated= 3.577;
    B_calibrated= -0.455;



    do  {
      brth_printf("BRTH: start cycle\n");


      enter_state(Insufflation);

      // TODO: Take the time to compute adaptation into the FR calculation
      // Maybe we should put this caomputation at the end of the cycle.

      // Get current controller settings
      uint32_t T        = get_setting_T_ms      ();
      float    VT       = get_setting_VT_mL     ();
      float    VM       = get_setting_Vmax_Lpm  ();
      float    Pmax     = get_setting_Pmax_cmH2O();
      uint32_t Tplat    = get_setting_Tplat_ms  ();

      brth_printf("BRTH: T     : %ld\n", T);
      brth_printf("BRTH: VT    : %ld\n", (uint32_t)(VT*100));
      brth_printf("BRTH: VM    : %ld\n", (uint32_t)(VM*100));
      brth_printf("BRTH: Pmax  : %ld\n", (uint32_t)(Pmax*100));
      brth_printf("BRTH: Tplat : %ld\n", Tplat);

      float VTi=0.;
      float VTe=0.;


      // Compute adaptation based on current settings and previous collected data if any.

      uint32_t Ti = T*1/3;  // TODO: Check how to calculate Tinsuflation
      brth_printf("BRTH: Ti    : %ld\n", Ti);

      float move_T=0;
      _steps = 4000;
      for(long t=0; t<_steps; ++t) {
        float d = 200; //compte_motor_step_time(t, 1., A_calibrated, B_calibrated, 200);
        move_T += d;
        _motor_steps_us[t]= (uint32_t)d;
      }

      // Start Inhalation
      valve_inhale();
      motor_press(_motor_steps_us, _steps);
      reset_Vol_mL();
      brth_printf("BRTH: Insuflation\n");      
      while(Insufflation == _state ) {
          // TODO: record flow and volume (?) for next cycle adaptation.
          // if (Pmax <= read_Paw_cmH2O()) {
          //     light_red(On);
          //     enter_state(Exhalation);
          // } else if (VT <= read_Vol_mL()) {
          //     enter_state(Plateau);
          // } else 
          if( Ti <= get_time_ms() - _cycle_start_ms ) {
              enter_state(Plateau);
          }
          wait_ms(10);
      }
      motor_release();
      while(Plateau == _state) {
        if (Pmax <= read_Paw_cmH2O()) { 
            enter_state(Exhalation);
        } else if ( is_command_Tpins_expired() && (Tplat <= get_time_ms() - _state_start_ms) ) {
            enter_state(Exhalation);
        }
        wait_ms(10);
      }
      VTi= read_Vol_mL();
      reset_Vol_mL();        
      valve_exhale();
      while(Exhalation == _state) { 
          if ( T <= (get_time_ms() - _cycle_start_ms )) { 
              uint32_t t_ms = get_time_ms();


              EoI_ratio =  (float)(t_ms-_cycle_start_ms)/(_state_start_ms-_cycle_start_ms);
              FR_pm     = 1./(((float)(t_ms-_cycle_start_ms))/1000/60);

              // TODO regulation_pep();
              enter_state(Insufflation);
          }
        wait_ms(10);
      }
      VTe = read_Vol_mL();
      reset_Vol_mL();

      events= xEventGroupGetBits(eventFlags);
    } while ( ( events & MONITORING_RUN_FLAG ) != 0 );

    brth_printf("BRTH: Stopping\n");      

    wait_ms(200);
    xEventGroupSetBits(eventFlags, BREATHING_STOPPED_FLAG);

  }
}