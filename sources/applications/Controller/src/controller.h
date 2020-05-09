#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "common.h"

// FreeRTOS tasks, timers and events

#define BREATHING_RUN_FLAG        (1 << 0)
#define MONITORING_RUN_FLAG       (1 << 1)
#define HMI_RUN_FLAG              (1 << 2)

#define BREATHING_STOPPED_FLAG    (1 << 4)
#define MONITORING_STOPPED_FLAG   (1 << 5)
#define HMI_STOPPED_FLAG          (1 << 6)

extern EventGroupHandle_t   g_controllerEvents;
extern TaskHandle_t         g_controllerTask;

bool controller_init();




//! set the desired setting value.
//! \returns a value within the range defined by physical constraints and VT, Vmax, EoI
float set_setting_FR_pm(float desired);

//! set the desired setting value.
//! \returns a value within the range defined by physical constraints and FR, Vmax, EoI
float set_setting_VT_mL(float desired);

//! \returns a value within the range defined by physical constraints and FR, VT, EoI
float set_setting_Vmax_Lpm (float desired);

//! \returns a value within the range defined by physical constraints and FR, VT, Vmax
float set_setting_EoI_ratio_x10(float desired_x10);

float set_setting_PEP_cmH2O(float desired);

float set_setting_Pmax_cmH2O(float desired);

float set_setting_Pmin_cmH2O(float desired);

float set_setting_VTmin_mL(float desired);

float set_setting_VTmax_mL(float desired);

float set_setting_FRmin_pm(float desired);

float set_setting_VMmin_Lpm(float desired);



//! \returns respiration frequency per minute
float get_setting_FR_pm();

//! \returns tidal volume in milliliters
float get_setting_VT_mL    ();

//! \returns Positive Exhalation Pressure in cmH2O
float get_setting_PEP_cmH2O();

//! \returns Maximum insufflated VolumeMinute in liters/minute
float get_setting_Vmax_Lpm ();

//! \returns exhalation duration relative to inspiration
float get_setting_EoI_ratio();

//! \returns inspiration duration relative to exhalation
static inline
float get_setting_IoE_ratio()
{
    return 1.f/get_setting_EoI_ratio();
}

//! \returns respiration duration based on FR
static inline
uint32_t get_setting_T_ms()
{
    return (uint32_t)(60000 / get_setting_FR_pm()); // taking floor of integer division is ok for our range and easier to test equal
}

//! \returns exhalation duration based on FR, I/E
static inline
uint32_t get_setting_Texp_ms()
{
    const float EoI = get_setting_EoI_ratio();
    return (uint32_t)(get_setting_T_ms() * (EoI/(1+EoI))); // BEWARE to not multiply ms before dividing EoI to avoid wrap-around
}

//! \returns insufflation duration based on VT, Vmax
//! \sa get_setting_Tinsp_ms()
static inline
uint32_t get_setting_Tinsu_ms()
{
    return (uint32_t)(get_setting_VT_mL() / (get_setting_Vmax_Lpm()/60.f /*(mL/ms)*/));
}

//! \returns inspiration duration based on T, Texp
//! \sa get_setting_Tinsu_ms()
static inline
uint32_t get_setting_Tinspi_ms()
{
    uint32_t  used_ms = get_setting_Texp_ms();
    uint32_t     T_ms = get_setting_T_ms();
    assert(T_ms >=used_ms);
    return T_ms - used_ms ;
}

//! \returns plateau duration based on T, Texp, Tinsu
static inline
uint32_t get_setting_Tplat_ms()
{
    uint32_t  used_ms = get_setting_Tinsu_ms()+get_setting_Texp_ms();
    uint32_t     T_ms = get_setting_T_ms();
    assert(T_ms >=used_ms);
    return T_ms - used_ms ;
}

float    get_setting_Pmax_cmH2O  ();
float    get_setting_Pmin_cmH2O  ();
float    get_setting_VTmin_mL    ();
float    get_setting_VTmax_mL    ();
float    get_setting_FRmin_pm    ();
float    get_setting_VMmin_Lm    ();
float    get_setting_PEPmax_cmH2O();
float    get_setting_PEPmin_cmH2O();


uint16_t get_Tpins_ms();
uint16_t get_Tpexp_ms();
uint16_t get_Tpbip_ms();

uint16_t set_command_Tpins_ms(uint16_t ms);
uint16_t set_command_Tpexp_ms(uint16_t ms);
uint16_t set_command_Tpbip_ms(uint16_t ms);

bool is_command_Tpins_expired();
bool is_command_Tpexp_expired();
bool is_command_Tpbip_expired();

void set_command_soft_reset();










#endif