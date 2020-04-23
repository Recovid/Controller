#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "platform.h"


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


uint32_t get_Tpins_ms();
uint32_t get_Tpexp_ms();
uint32_t get_Tpbip_ms();


typedef enum { Starting, Ready, Calibrating, Running, Stopping, Stopped } controller_state_t;

void                controller_init();
void                controller_run();
controller_state_t  get_controller_state();





#endif