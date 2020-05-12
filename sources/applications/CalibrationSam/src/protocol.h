#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "common.h"

typedef struct {
    float       (*get_setting_FR_pm)(void);
    float       (*get_setting_VT_mL)(void);
    float       (*get_setting_PEP_cmH2O)(void);
    float       (*get_setting_Vmax_Lpm)(void);
    float       (*get_setting_EoI_ratio)(void);
    float       (*get_setting_Pmax_cmH2O)(void);
    float       (*get_setting_Pmin_cmH2O)(void);
    float       (*get_setting_VTmin_mL)(void);
    float       (*get_setting_VTmax_mL)(void);
    float       (*get_setting_FRmin_pm)(void);
    float       (*get_setting_VMmin_Lm)(void);

    float       (*set_setting_FR_pm)(float desired);
    float       (*set_setting_VT_mL)(float desired);
    float       (*set_setting_Vmax_Lpm) (float desired);
    float       (*set_setting_EoI_ratio)(float desired);
    float       (*set_setting_PEP_cmH2O)(float desired);
    float       (*set_setting_Pmax_cmH2O)(float desired);
    float       (*set_setting_Pmin_cmH2O)(float desired);
    float       (*set_setting_VTmin_mL)(float desired);
    float       (*set_setting_VTmax_mL)(float desired);
    float       (*set_setting_FRmin_pm)(float desired);
    float       (*set_setting_VMmin_Lpm)(float desired);

    uint16_t    (*set_command_Tpins_ms)(uint16_t ms);
    uint16_t    (*set_command_Tpexp_ms)(uint16_t ms);
    uint16_t    (*set_command_Tpbip_ms)(uint16_t ms);

    void        (*set_command_soft_reset)(void);
} protocol_handler_t;


void set_protocol_handler(protocol_handler_t* handler);

const char* get_init_str();

bool send_DATA(float P, float VolM, float Vol);
bool send_DATA_X(float P, float VolM, float Vol, float Pplat, float PEP);
bool send_RESP(float IE, float FR, float VTe, float VM, float Pcrete, float Pplat, float PEP);

bool send_INIT(const char* information);

bool send_ALRM(uint32_t alarms);

//! Send queued messages to IHM, then receive messages from IHM and process them including:
//! - update controller settings and acknowledge modified value
//! - answer to INIT
//! \returns false if soft reset received
void send_and_recv();

#ifndef NTESTS
bool TEST_PROTOCOL();
#endif

#endif // PROTOCOL
