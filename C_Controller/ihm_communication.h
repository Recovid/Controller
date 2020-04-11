#ifndef IHM_COMMUNICATION_H
#define IHM_COMMUNICATION_H

#include "platform.h"

// ------------------------------------------------------------------------------------------------
//! Public read-only access to settings coming from UI

// TODO Use float to avoid integer division when using these functions in computations

float    get_setting_FR_pm    ();
float    get_setting_VT_mL    ();
float    get_setting_PEP_cmH2O();
float    get_setting_Vmax_Lpm ();
float    get_setting_IoE_ratio();
float    get_setting_EoI_ratio();
uint16_t get_setting_Tplat_ms ();

float    get_setting_Pmax_cmH2O  ();
float    get_setting_Pmin_cmH2O  ();
float    get_setting_VTmin_mL    ();
float    get_setting_FRmin_pm    ();
float    get_setting_VMmin_Lm    ();
float    get_setting_PEPmax_cmH2O();
float    get_setting_PEPmin_cmH2O();

// ------------------------------------------------------------------------------------------------
//! Public read-only access to commands coming from UI

uint32_t get_command_Tpins_ms();
uint32_t get_command_Tpexp_ms();
uint32_t get_command_Tpbip_ms();

bool     is_soft_reset_asked ();


// ------------------------------------------------------------------------------------------------
//! Public interface to send event/data to the IHM and process read messages

bool send_DATA(float P, float VolM, float Vol, float Pplat, float PEP);
bool send_RESP(float IE, float FR, float VTe, float VM, float Pcrete, float Pplat, float PEP);

bool send_INIT(const char* information);

//! Send queued messages to IHM, then receive messages from IHM and process them including:
//! - update controller settings and acknowledge modified value
//! - answer to INIT
//! \returns false if soft reset received
void send_and_recv();

#endif // IHM_COMMUNICATION_H
