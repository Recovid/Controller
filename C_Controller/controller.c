#include "controller.h"
#include "breathing.h"
#include "monitoring.h"
#include "lowlevel.h"


// ------------------------------------------------------------------------------------------------
//! Settings

//! \warning Global settings MUST use types that can be atomically read/write in a threadsafe way on STM32
//! \warning Non volatile memory may be limited to uint16_t by the corresponding driver

static uint16_t setting_FR_pm          =  18; //!< \see checked_FR_pm()         to set it within the range defined by     VT, Vmax, EoI
static uint16_t setting_VT_mL          = 300; //!< \see checked_VT_mL()         to set it within the range defined by FR,     Vmax, EoI
static uint16_t setting_Vmax_Lpm       =  60; //!< \see checked_Vmax_Lpm()      to set it within the range defined by FR, VT,       EoI
static uint16_t setting_EoI_ratio_x10  =  20; //!< \see checked_EoI_ratio_x10() to set it within the range defined by FR, VT, Vmax

static uint16_t setting_PEP_cmH2O  =   5;

static uint16_t setting_Pmax_cmH2O =  60;
static uint16_t setting_Pmin_cmH2O =  20;
static uint16_t setting_VTmin_mL   = 150;
static uint16_t setting_VTmax_mL   = 1000;
static uint16_t setting_FRmin_pm   =  10;
static uint16_t setting_VMmin_Lpm  =   3;

const int setting_PEPmax_cmH2O =  2;
const int setting_PEPmin_cmH2O = -2;

uint16_t checked_FR_pm(uint16_t desired);
uint16_t checked_VT_mL(uint16_t desired);
uint16_t checked_Vmax_Lpm (uint16_t desired);
uint16_t checked_EoI_ratio_x10(uint16_t desired_x10);


static controller_state_t state;



void controller_init() {
    state= Starting;

    // TODO Init default values.
}


void controller_run() {

  while(true) {
    while(!is_breathing_ready() || !is_monitoring_ready()) {
      // TODO: report status to HMI ?? 
      wait_ms(10);
    }

    // TODO Implement controller logic









  }


}






















//! \returns a value within the range defined by physical constraints and VT, Vmax, EoI
uint16_t checked_FR_pm(uint16_t desired)
{
    uint16_t max = (get_setting_Vmax_Lpm() / (get_setting_VT_mL()/1000.f/*(L)*/) /*(Ipm)*/) / (1.f+get_setting_EoI_ratio());
    return MIN(max, desired);
}
//! \returns a value within the range defined by physical constraints and FR, Vmax, EoI
uint16_t checked_VT_mL(uint16_t desired)
{
    uint16_t max = get_setting_Vmax_Lpm() / 60 /*(mL/ms)*/ * get_setting_Tinspi_ms();
    return MIN(max, desired);
}
//! \returns a value within the range defined by physical constraints and FR, VT, EoI
uint16_t checked_Vmax_Lpm (uint16_t desired)
{
    uint16_t min = 60 * get_setting_VT_mL() / get_setting_Tinspi_ms() /*(mL/ms)*/;
    return MAX(min, desired);
}
//! \returns a value within the range defined by physical constraints and FR, VT, Vmax
uint16_t checked_EoI_ratio_x10(uint16_t desired_x10)
{
    uint16_t max_x10 = 10*(((get_setting_Vmax_Lpm() / (get_setting_VT_mL()/1000.f/*(L)*/) /*(Ipm)*/) / get_setting_FR_pm()) - 1.f);
    return MIN(max_x10, desired_x10);
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

static uint32_t command_Tpins_ms = 0;
static uint32_t command_Tpexp_ms = 0;
static uint32_t command_Tpbip_ms = 0;

static bool command_soft_reset = false;

uint32_t get_command_Tpins_ms() { return command_Tpins_ms; }
uint32_t get_command_Tpexp_ms() { return command_Tpexp_ms; }
uint32_t get_command_Tpbip_ms() { return command_Tpbip_ms; }
