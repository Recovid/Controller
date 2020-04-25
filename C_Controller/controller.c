#include "recovid.h"
#include "configuration.h"
#include "controller.h"
#include "breathing.h"
#include "monitoring.h"
#include "protocol.h"
#include "lowlevel.h"


// ------------------------------------------------------------------------------------------------
//! Settings

//! \warning Global settings MUST use types that can be atomically read/write in a threadsafe way on STM32
//! \warning Non volatile memory may be limited to uint16_t by the corresponding driver

static uint16_t setting_FR_pm; //!< \see checked_FR_pm()         to set it within the range defined by     VT, Vmax, EoI
static uint16_t setting_VT_mL; //!< \see checked_VT_mL()         to set it within the range defined by FR,     Vmax, EoI
static uint16_t setting_Vmax_Lpm; //!< \see checked_Vmax_Lpm()      to set it within the range defined by FR, VT,       EoI
static uint16_t setting_EoI_ratio_x10; //!< \see checked_EoI_ratio_x10() to set it within the range defined by FR, VT, Vmax

static uint16_t setting_PEP_cmH2O;

static uint16_t setting_Pmax_cmH2O;
static uint16_t setting_Pmin_cmH2O ;
static uint16_t setting_VTmin_mL;
static uint16_t setting_VTmax_mL;
static uint16_t setting_FRmin_pm;
static uint16_t setting_VMmin_Lpm;

const int setting_PEPmax_cmH2O = DEFAULT_setting_PEPmax_cmH2O;
const int setting_PEPmin_cmH2O = DEFAULT_setting_PEPmin_cmH2O;

uint16_t checked_FR_pm(uint16_t desired);
uint16_t checked_VT_mL(uint16_t desired);
uint16_t checked_Vmax_Lpm (uint16_t desired);
uint16_t checked_EoI_ratio_x10(uint16_t desired_x10);



static void reset_settings();
static bool load_settings();

void controller_init() {
    reset_settings();
}


void controller_run(void* args) {
  static uint32_t idx=0;
  UNUSED(args);

  while(true) {

    // TODO: Define startup process
    // For now just start breathing and monitoring based on default/stored values

    load_settings();
    printf("Settings loaded\n");

    // Start breathing and monitoring
    xEventGroupSetBits(controlFlags, ((RUN_BREATHING_FLAG | RUN_MONITORING_FLAG)));
    printf("Breathing and monitoring started\n");

    uint32_t last_report_time= get_time_ms();
    while(true) {
      // TODO Implement controller logic
      // check monitoring process
      // check breathing process
      
      
      //send_and_recv(); // TODO rework implem for a more deterministic execution time

      if(25 <= (get_time_ms()-last_report_time)) {
//        if(++idx==40) { idx=0; printf("updating hmi\n"); }
        //send_DATA(get_Paw_cmH2O(), get_Pdiff_Lpm(), 0);
        last_report_time= get_time_ms();
      }

      wait_ms(5);

    }

    // TODO: Shutdown and standby process



  }

}

static void reset_settings() {
    setting_FR_pm   = DEFAULT_setting_FR_pm;
    setting_VT_mL   = DEFAULT_setting_VT_mL;
    setting_Vmax_Lpm= DEFAULT_setting_Vmax_Lpm;
    setting_EoI_ratio_x10 = DEFAULT_setting_EoI_ratio_x10;
    setting_PEP_cmH2O = DEFAULT_setting_PEP_cmH2O;
    setting_Pmax_cmH2O= DEFAULT_setting_PEPmax_cmH2O;
    setting_Pmin_cmH2O= DEFAULT_setting_PEPmin_cmH2O;
    setting_VTmin_mL  = DEFAULT_setting_VTmin_mL;
    setting_VTmax_mL  = DEFAULT_setting_VTmax_mL;
    setting_FRmin_pm  = DEFAULT_setting_FRmin_pm;
    setting_VMmin_Lpm = DEFAULT_setting_VMmin_Lpm;
}

static bool load_settings() {
  // For now, we're not saving settings, so it's not possible to reload them :-)
  return false;
}

















//! \returns a value within the range defined by physical constraints and VT, Vmax, EoI
float set_setting_FR_pm(float desired)
{
    float max = (get_setting_Vmax_Lpm() / (get_setting_VT_mL()/1000.f/*(L)*/) /*(Ipm)*/) / (1.f+get_setting_EoI_ratio());
    setting_FR_pm= MIN(max, desired);
  return setting_FR_pm;
}
//! \returns a value within the range defined by physical constraints and FR, Vmax, EoI
float set_setting_VT_mL(float desired)
{
    float max = get_setting_Vmax_Lpm() / 60 /*(mL/ms)*/ * get_setting_Tinspi_ms();
    setting_VT_mL= MIN(max, desired);
    return setting_VT_mL;
}

//! \returns a value within the range defined by physical constraints and FR, VT, EoI
float set_setting_Vmax_Lpm (float desired)
{
    float min = 60 * get_setting_VT_mL() / get_setting_Tinspi_ms() /*(mL/ms)*/;
    setting_Vmax_Lpm= MAX(min, desired);
    return setting_Vmax_Lpm;
}
//! \returns a value within the range defined by physical constraints and FR, VT, Vmax
float set_setting_EoI_ratio_x10(float desired_x10)
{
    float max_x10 = 10*(((get_setting_Vmax_Lpm() / (get_setting_VT_mL()/1000.f/*(L)*/) /*(Ipm)*/) / get_setting_FR_pm()) - 1.f);
    setting_EoI_ratio_x10= MIN(max_x10, desired_x10);
    return setting_EoI_ratio_x10;
}

float set_setting_PEP_cmH2O(float desired) {
  setting_PEP_cmH2O= desired;
  return setting_PEP_cmH2O;
}

float set_setting_Pmax_cmH2O(float desired) {
  setting_Pmax_cmH2O= desired;
  return setting_Pmax_cmH2O;
}

float set_setting_Pmin_cmH2O(float desired) {
  setting_Pmin_cmH2O= desired;
  return setting_Pmin_cmH2O;
}

float set_setting_VTmin_mL(float desired) {
  setting_VTmin_mL= desired;
  return setting_VTmin_mL;
}

float set_setting_VTmax_mL(float desired) {
  setting_VTmax_mL= desired;
  return setting_VTmax_mL;
}

float set_setting_FRmin_pm(float desired) {
  setting_FRmin_pm= desired;
  return setting_FRmin_pm;
}

float set_setting_VMmin_Lpm(float desired) {
  setting_VMmin_Lpm= desired;
  return setting_VMmin_Lpm;
}




float get_setting_FR_pm       () { return setting_FR_pm           ; }
float get_setting_VT_mL       () { return setting_VT_mL           ; }

float get_setting_Vmax_Lpm    () { return setting_Vmax_Lpm        ; }
float get_setting_EoI_ratio   () { return setting_EoI_ratio_x10 / 10.f; }

float get_setting_PEP_cmH2O   () { return setting_PEP_cmH2O       ; }

float get_setting_Pmax_cmH2O  () { return setting_Pmax_cmH2O      ; }
float get_setting_Pmin_cmH2O  () { return setting_Pmin_cmH2O      ; }
float get_setting_VTmin_mL    () { return setting_VTmin_mL        ; }
float get_setting_VTmax_mL    () { return setting_VTmax_mL        ; }
float get_setting_FRmin_pm    () { return setting_FRmin_pm        ; }
float get_setting_VMmin_Lm    () { return setting_VMmin_Lpm       ; }

float get_setting_PEPmax_cmH2O() { return setting_PEPmax_cmH2O    ; }
float get_setting_PEPmin_cmH2O() { return setting_PEPmin_cmH2O    ; }

// ------------------------------------------------------------------------------------------------
//! Commands

//! Global settings MUST use types that can be atomically read/write in a threadsafe way on STM32

static uint32_t command_Tpins_timestamp = 0;
static uint16_t command_Tpins_ms = 0;
static uint32_t command_Tpexp_timestamp = 0;
static uint16_t command_Tpexp_ms = 0;
static uint32_t command_Tpbip_timestamp = 0;
static uint16_t command_Tpbip_ms = 0;

uint16_t get_command_Tpins_ms() { return command_Tpins_ms; }
uint16_t get_command_Tpexp_ms() { return command_Tpexp_ms; }
uint16_t get_command_Tpbip_ms() { return command_Tpbip_ms; }

uint16_t set_command_Tpins_ms(uint16_t ms) { command_Tpins_timestamp= get_time_ms(); command_Tpins_ms= ms; return ms; }
uint16_t set_command_Tpexp_ms(uint16_t ms) { command_Tpexp_timestamp= get_time_ms(); command_Tpexp_ms= ms; return ms; }
uint16_t set_command_Tpbip_ms(uint16_t ms) { command_Tpbip_timestamp= get_time_ms(); command_Tpbip_ms= ms; return ms; }

bool is_command_Tpins_expired() { return command_Tpins_ms < get_time_ms()-command_Tpins_timestamp; }
bool is_command_Tpexp_expired() { return command_Tpexp_ms < get_time_ms()-command_Tpexp_timestamp; }
bool is_command_Tpbip_expired() { return command_Tpbip_ms < get_time_ms()-command_Tpbip_timestamp; }


static bool command_soft_reset = false;
void set_command_soft_reset() { command_soft_reset= true; }


#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

#pragma GCC diagnostic ignored "-Wtype-limits"


bool PRINT(test_non_default_settings)
    setting_FR_pm         =  30;
    setting_VT_mL         = 300;
    setting_Vmax_Lpm      =  30;
    setting_EoI_ratio_x10 =  10;
    return
        TEST_FLT_EQUALS(  30.f, get_setting_FR_pm       ()) &&
        TEST_EQUALS    (2000  , get_setting_T_ms        ()) &&
        TEST_FLT_EQUALS( 300.f, get_setting_VT_mL       ()) &&
        TEST_FLT_EQUALS(  30.f, get_setting_Vmax_Lpm    ()) &&
        TEST_EQUALS    ( 600  , get_setting_Tinsu_ms    ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_IoE_ratio   ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_EoI_ratio   ()) &&
        TEST_EQUALS    (1000  , get_setting_Texp_ms     ()) &&
        TEST_EQUALS    (1000  , get_setting_Tinspi_ms   ()) &&
        TEST_EQUALS    ( 400  , get_setting_Tplat_ms    ()) &&
        true;
}

bool PRINT(test_checked_EoI)
    setting_FR_pm         =  30;
    setting_VT_mL         = 300;
    setting_Vmax_Lpm      =  30;
    setting_EoI_ratio_x10 = checked_EoI_ratio_x10(30);
    return
        TEST_FLT_EQUALS(  30.f , get_setting_FR_pm    ()) &&
        TEST_EQUALS    (2000   , get_setting_T_ms     ()) &&
        TEST_FLT_EQUALS( 300.f , get_setting_VT_mL    ()) &&
        TEST_FLT_EQUALS(  30.f , get_setting_Vmax_Lpm ()) &&
        TEST_EQUALS    ( 600   , get_setting_Tinsu_ms ()) &&
        TEST_FLT_EQUALS(   0.4f, get_setting_IoE_ratio()) &&
        TEST_FLT_EQUALS(   2.3f, get_setting_EoI_ratio()) &&
        TEST_RANGE     (1390   , get_setting_Texp_ms  (), 1400) &&
        TEST_RANGE     ( 600   , get_setting_Tinspi_ms(),  610) &&
        TEST_RANGE     (   0   , get_setting_Tplat_ms (),   10) &&
        true;
}

bool PRINT(test_checked_VM)
    setting_FR_pm         =  30;
    setting_VT_mL         = 600;
    setting_EoI_ratio_x10 =  10;
    setting_Vmax_Lpm      = checked_Vmax_Lpm(30);
    return
        TEST_FLT_EQUALS(  30.f, get_setting_FR_pm       ()) &&
        TEST_EQUALS    (2000  , get_setting_T_ms        ()) &&
        TEST_FLT_EQUALS( 600.f, get_setting_VT_mL       ()) &&
        TEST_FLT_EQUALS(  36.f, get_setting_Vmax_Lpm    ()) &&
        TEST_RANGE     ( 999  , get_setting_Tinsu_ms    (), 1000) && // due to Vmax rounding to floor
        TEST_FLT_EQUALS(   1.f, get_setting_IoE_ratio   ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_EoI_ratio   ()) &&
        TEST_EQUALS    (1000  , get_setting_Texp_ms     ()) &&
        TEST_EQUALS    (1000  , get_setting_Tinspi_ms   ()) &&
        TEST_RANGE     (   0.f, get_setting_Tplat_ms    (), 1.f ) && // due to Vmax rounding to floor
        true;
}

bool PRINT(test_checked_VT)
    setting_FR_pm         = 30;
    setting_Vmax_Lpm      = 30;
    setting_EoI_ratio_x10 = 10;
    setting_VT_mL         = checked_VT_mL(600);
    return
        TEST_FLT_EQUALS(  30.f, get_setting_FR_pm       ()) &&
        TEST_EQUALS    (2000  , get_setting_T_ms        ()) &&
        TEST_FLT_EQUALS( 500.f, get_setting_VT_mL       ()) &&
        TEST_FLT_EQUALS(  30.f, get_setting_Vmax_Lpm    ()) &&
        TEST_EQUALS    (1000  , get_setting_Tinsu_ms    ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_IoE_ratio   ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_EoI_ratio   ()) &&
        TEST_EQUALS    (1000  , get_setting_Texp_ms     ()) &&
        TEST_EQUALS    (1000  , get_setting_Tinspi_ms   ()) &&
        TEST_EQUALS    (   0  , get_setting_Tplat_ms    ()) &&
        true;
}

bool PRINT(test_checked_FR)
    setting_VT_mL         = 600;
    setting_Vmax_Lpm      =  30;
    setting_EoI_ratio_x10 =  10;
    setting_FR_pm         = checked_FR_pm(30);
    return
        TEST_RANGE     (  24.f, get_setting_FR_pm       (), 25.f) &&
        TEST_RANGE     (2400  , get_setting_T_ms        (), 2500) && // due to FR rounding
        TEST_FLT_EQUALS( 600.f, get_setting_VT_mL       ()) &&
        TEST_FLT_EQUALS(  30.f, get_setting_Vmax_Lpm    ()) &&
        TEST_EQUALS    (1200  , get_setting_Tinsu_ms    ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_IoE_ratio   ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_EoI_ratio   ()) &&
        TEST_RANGE     (1200  , get_setting_Texp_ms     (), 1250) && // due to FR rounding
        TEST_RANGE     (1200  , get_setting_Tinspi_ms   (), 1250) && // due to FR rounding
        TEST           (        get_setting_Tplat_ms    ()<= 100) && // due to FR rounding
        true;
}

#endif