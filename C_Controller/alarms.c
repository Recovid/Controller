#include "alarms.h"

#include <string.h>

#include "sensing.h" //  to use sensed values that depend on controller state
#include "ihm_communication.h" // to send alarms

//! Bit flags for currently active alarms
int32_t activeAlarms = 0;
int32_t activeAlarmsOld = 0;

/* NB: the bounded queues belows implemented as circular buffers are filled by
 * pushing new values to the "front" (i.e. startIdx).
 * For instance, the following shows how push a value in VTe_ml_q:
 * ```
 * VTe_startIdx = (VTe_startIdx - 1 + VTE_Q_LEN) % VTE_Q_LEN;
 * VTe_ml_q[VTe_startIdx] = new_value;
 * ```
 */

// Pmax Alarm
#define PMAX_Q_LEN 2
int pressureMax_startIdx = 0;
float pressureMax_cmH2O_q[PMAX_Q_LEN] = {0};

// Pmin Alarm
#define PCRETE_Q_LEN 8
int Pcrete_startIdx = 0;
float Pcrete_cmH2O_q[PCRETE_Q_LEN] = {0};
int Pcrete_time_ms_q[PCRETE_Q_LEN] = {0};

// VTmin Alarm
#define VTE_Q_LEN 3
int VTe_startIdx = 0;
float VTe_ml_q[3] = {0};

// FRmin Alarm
#define FR_Q_LEN 3
//int FR_startIdx = 0;
//float FR_pm_q[FR_Q_LEN] = {0};

// VMmin Alarm
#define VM_Q_LEN 3
int VM_startIdx = 0;
float VM_Lm_q[VM_Q_LEN] = {0};

// PEPmax / PEPmin Alarm
#define PEP_Q_LEN 8
int PEP_startIdx = 0;
float PEP_cmH2O_q[PEP_Q_LEN] = {0};

bool update_alarms()
{
    int Pmax_cycles = PMAX_Q_LEN;
    int Pmin_startFailing_ms = -1;
    int VTmin_cycles = VTE_Q_LEN;
    int FRmin_cycles = FR_Q_LEN;
    int VMmin_cycles = VM_Q_LEN;
    int PEPmax_cycles = PEP_Q_LEN;
    int PEPmin_cycles = PEP_Q_LEN;

    // save and reset all alarms
    activeAlarmsOld = activeAlarms;
    activeAlarms = 0;

    for (int cycle = 0; cycle < 8; ++cycle)
    {
        if (Pmax_cycles != 0)
        {
            if (pressureMax_cmH2O_q[(pressureMax_startIdx + cycle) % PMAX_Q_LEN] >= MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10))
            {
                --Pmax_cycles;
                if (Pmax_cycles == 0) {
                    // Pmax Alarm
                    activeAlarms |= PMAX_ALARM;
                }
            }
            else
                Pmax_cycles = 0; // deactivate Pmax Alarm
        }
        if (Pmin_startFailing_ms != -2)
        {
            int Pcrete_idx = (Pcrete_startIdx + cycle) % PCRETE_Q_LEN;
            if (Pcrete_cmH2O_q[Pcrete_idx] <= MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2))
            {
                if (Pmin_startFailing_ms == -1)
                    Pmin_startFailing_ms = Pcrete_time_ms_q[Pcrete_idx];
                else if (Pmin_startFailing_ms - Pcrete_time_ms_q[Pcrete_idx] > 15) {
                    // Pmin Alarm
                    activeAlarms |= PMIN_ALARM;
                }
            }
            else
                Pmin_startFailing_ms = -2; // deactivate Pmin Alarm
        }
        if (VTmin_cycles != 0)
        {
            if (VTe_ml_q[(VTe_startIdx + cycle) % VTE_Q_LEN] <= get_setting_VTmin_mL())
            {
                --VTmin_cycles;
                if (VTmin_cycles == 0) {
                    // VTmin Alarm
                    activeAlarms |= VT_MIN_ALARM;
                }
            }
            else
                VTmin_cycles = 0; // deactivate VTmin Alarm
        }
        /*
        if (FRmin_cycles != 0)
        {
            if (FR_pm_q[FR_startIdx % FR_Q_LEN] < get_setting_FRmin_pm())
            {
                --FRmin_cycles;
                if (FRmin_cycles == 0) {
                    ; // FRmin Alarm
                }
            }
            else
                FRmin_cycles = 0; // deactivate FRmin Alarm
        }
        */
        if (VMmin_cycles != 0)
        {
            if (VM_Lm_q[(VM_startIdx + cycle) % VM_Q_LEN] <= get_setting_VMmin_Lm())
            {
                --VMmin_cycles;
                if (VMmin_cycles == 0) {
                    // VMmin Alarm
                    activeAlarms |= VM_MIN_ALARM;
                }
            }
            else
                VMmin_cycles = 0; // deactivate VMmin Alarm
        }
        if (PEPmax_cycles != 0)
        {
            if (PEP_cmH2O_q[(PEP_startIdx + cycle) % PEP_Q_LEN] >= get_setting_PEP_cmH2O() + 2)
            {
                --PEPmax_cycles;
                if (PEPmax_cycles == 0) {
                    // PEPmax Alarm
                    activeAlarms |= PEP_MAX_ALARM;
                }
            }
            else
                PEPmax_cycles = 0; // deactivate PEPmax Alarm
        }
        if (PEPmin_cycles != 0)
        {
            if (PEP_cmH2O_q[(PEP_startIdx + cycle) % PEP_Q_LEN] <= get_setting_PEP_cmH2O() - 2)
            {
                --PEPmin_cycles;
                if (PEPmin_cycles == 0) {
                    // PEPmin Alarm
                    activeAlarms |= PEP_MIN_ALARM;
                }
            }
            else
                PEPmin_cycles = 0; // deactivate PEPmin Alarm
        }
    }
    return true;
}

void trigger_alarms()
{
    uint32_t newAlarms = (activeAlarms ^ activeAlarmsOld) & activeAlarms;

    // min/max alarms
    for (int i = 1; i <= 6; i++) {
        if (newAlarms & (1 << i)) {
            // TODO: trigger alarm (buzzer/led) acc. to priority
            send_ALRM(1 << i);
        }
    }
}

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

static bool PRINT(test_alarm_pmax_on0)
    pressureMax_startIdx = 1;
    pressureMax_cmH2O_q[0] = MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10);
    pressureMax_cmH2O_q[1] = MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10) + 1;
    update_alarms();
    return TEST(activeAlarms & PMAX_ALARM);
}

static bool PRINT(test_alarm_pmax_off0)
    pressureMax_startIdx = 0;
    pressureMax_cmH2O_q[0] = MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10) - 1;
    pressureMax_cmH2O_q[1] = MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10);
    update_alarms();
    return TEST(!(activeAlarms & PMAX_ALARM));
}

static bool PRINT(test_alarm_pmax_off1)
    pressureMax_startIdx = 0;
    pressureMax_cmH2O_q[0] = MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10);
    pressureMax_cmH2O_q[1] = MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10) - 1;
    update_alarms();
    return TEST(!(activeAlarms & PMAX_ALARM));
}

static bool PRINT(test_alarm_pmin_on)
    Pcrete_startIdx = 0;
    memset(Pcrete_cmH2O_q, 0, sizeof(Pcrete_cmH2O_q));
    Pcrete_cmH2O_q[0] = MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2);
    Pcrete_cmH2O_q[1] = MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2) - 1;
    Pcrete_cmH2O_q[2] = MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2) - 2;
    memset(Pcrete_time_ms_q, 0, sizeof(Pcrete_time_ms_q));
    Pcrete_time_ms_q[0] = 20;
    Pcrete_time_ms_q[1] = 10;
    Pcrete_time_ms_q[2] = 0;

    update_alarms();
    return TEST(activeAlarms & PMIN_ALARM);
}

static bool PRINT(test_alarm_pmin_on_mod)
    Pcrete_startIdx = 6;
    memset(Pcrete_cmH2O_q, 0, sizeof(Pcrete_cmH2O_q));
    Pcrete_cmH2O_q[6] = MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2);
    Pcrete_cmH2O_q[7] = MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2) - 1;
    Pcrete_cmH2O_q[0] = MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2) - 2;
    memset(Pcrete_time_ms_q, 0, sizeof(Pcrete_time_ms_q));
    Pcrete_time_ms_q[6] = 20;
    Pcrete_time_ms_q[7] = 10;
    Pcrete_time_ms_q[0] = 0;

    update_alarms();
    return TEST(activeAlarms & PMIN_ALARM);
}

static bool PRINT(test_alarm_pmin_off_too_fast)
    Pcrete_startIdx = 0;
    memset(Pcrete_cmH2O_q, 0, sizeof(Pcrete_cmH2O_q));
    Pcrete_cmH2O_q[0] = MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2);
    Pcrete_cmH2O_q[1] = MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2) - 1;
    Pcrete_cmH2O_q[2] = MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2) - 2;
    memset(Pcrete_time_ms_q, 0, sizeof(Pcrete_time_ms_q));
    Pcrete_time_ms_q[0] = 10;
    Pcrete_time_ms_q[1] = 5;
    Pcrete_time_ms_q[2] = 0;

    update_alarms();
    return TEST(!(activeAlarms & PMIN_ALARM));
}

static bool PRINT(test_alarm_pmin_off_disc)
    Pcrete_startIdx = 0;
    memset(Pcrete_cmH2O_q, 0, sizeof(Pcrete_cmH2O_q));
    Pcrete_cmH2O_q[0] = MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2);
    Pcrete_cmH2O_q[1] = MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2) + 1;
    Pcrete_cmH2O_q[2] = MAX(get_setting_Pmin_cmH2O(), get_setting_PEP_cmH2O() + 2) - 2;
    memset(Pcrete_time_ms_q, 0, sizeof(Pcrete_time_ms_q));
    Pcrete_time_ms_q[0] = 10;
    Pcrete_time_ms_q[1] = 5;
    Pcrete_time_ms_q[2] = 0;

    update_alarms();
    return TEST(!(activeAlarms & PMIN_ALARM));
}

static bool PRINT(test_alarm_vt_min_on)
    VTe_startIdx = 0;
    VTe_ml_q[0] = get_setting_VTmin_mL();
    VTe_ml_q[1] = get_setting_VTmin_mL() - 1;
    VTe_ml_q[2] = get_setting_VTmin_mL() - 2;
    update_alarms();
    return TEST(activeAlarms & VT_MIN_ALARM);
}

static bool PRINT(test_alarm_vt_min_on_mod)
    VTe_startIdx = 1;
    VTe_ml_q[1] = get_setting_VTmin_mL() - 1;
    VTe_ml_q[2] = get_setting_VTmin_mL() - 2;
    VTe_ml_q[0] = get_setting_VTmin_mL();
    update_alarms();
    return TEST(activeAlarms & VT_MIN_ALARM);
}

static bool PRINT(test_alarm_vt_min_off_disc)
    VTe_startIdx = 0;
    VTe_ml_q[0] = get_setting_VTmin_mL();
    VTe_ml_q[1] = get_setting_VTmin_mL() + 1;
    VTe_ml_q[2] = get_setting_VTmin_mL() - 2;
    update_alarms();
    return TEST(!(activeAlarms & VT_MIN_ALARM));
}

static bool PRINT(test_alarm_vm_min_on)
    VM_startIdx = 0;
    VM_Lm_q[0] = get_setting_VMmin_Lm();
    VM_Lm_q[1] = get_setting_VMmin_Lm() - 1;
    VM_Lm_q[2] = get_setting_VMmin_Lm() - 2;
    update_alarms();
    return TEST(activeAlarms & VM_MIN_ALARM);
}

static bool PRINT(test_alarm_vm_min_on_mod)
    VM_startIdx = 1;
    VM_Lm_q[1] = get_setting_VMmin_Lm() - 1;
    VM_Lm_q[2] = get_setting_VMmin_Lm() - 2;
    VM_Lm_q[0] = get_setting_VMmin_Lm();
    update_alarms();
    return TEST(activeAlarms & VM_MIN_ALARM);
}

static bool PRINT(test_alarm_vm_min_off_disc)
    VM_startIdx = 0;
    VM_Lm_q[0] = get_setting_VMmin_Lm();
    VM_Lm_q[1] = get_setting_VMmin_Lm() + 1;
    VM_Lm_q[2] = get_setting_VMmin_Lm() - 2;
    update_alarms();
    return TEST(!(activeAlarms & VM_MIN_ALARM));
}

static bool PRINT(test_alarm_pep_max_on)
    PEP_startIdx = 0;
    for (int i = 0; i < PEP_Q_LEN; ++i) {
        PEP_cmH2O_q[i] = get_setting_PEP_cmH2O() + 2 + i;
    }
    update_alarms();
    return TEST(activeAlarms & PEP_MAX_ALARM);
}

static bool PRINT(test_alarm_pep_max_on_mod)
    PEP_startIdx = 4;
    for (int i = 0; i < PEP_Q_LEN; ++i) {
        PEP_cmH2O_q[i] = get_setting_PEP_cmH2O() + 2 + i;
    }
    update_alarms();
    return TEST(activeAlarms & PEP_MAX_ALARM);
}

static bool PRINT(test_alarm_pep_max_off_disc)
    PEP_startIdx = 0;
    for (int i = 0; i < PEP_Q_LEN; ++i) {
        PEP_cmH2O_q[i] = get_setting_PEP_cmH2O() + 2 + i;
    }
    PEP_cmH2O_q[4] = get_setting_PEP_cmH2O();
    update_alarms();
    return TEST(!(activeAlarms & PEP_MAX_ALARM));
}

static bool PRINT(test_alarm_pep_min_on)
    PEP_startIdx = 0;
    for (int i = 0; i < PEP_Q_LEN; ++i) {
        PEP_cmH2O_q[i] = get_setting_PEP_cmH2O() - 2 - i;
    }
    update_alarms();
    return TEST(activeAlarms & PEP_MIN_ALARM);
}

static bool PRINT(test_alarm_pep_min_on_mod)
    PEP_startIdx = 4;
    for (int i = 0; i < PEP_Q_LEN; ++i) {
        PEP_cmH2O_q[i] = get_setting_PEP_cmH2O() - 2 - i;
    }
    update_alarms();
    return TEST(activeAlarms & PEP_MIN_ALARM);
}

static bool PRINT(test_alarm_pep_min_off_disc)
    PEP_startIdx = 0;
    for (int i = 0; i < PEP_Q_LEN; ++i) {
        PEP_cmH2O_q[i] = get_setting_PEP_cmH2O() - 2 - i;
    }
    PEP_cmH2O_q[4] = get_setting_PEP_cmH2O();
    update_alarms();
    return TEST(!(activeAlarms & PEP_MIN_ALARM));
}

bool PRINT(TEST_ALARMS)
    return
        test_alarm_pmax_on0() &&
        test_alarm_pmax_off0() &&
        test_alarm_pmax_off1() &&
        test_alarm_pmin_on() &&
        test_alarm_pmin_on_mod() &&
        test_alarm_pmin_off_too_fast() &&
        test_alarm_pmin_off_disc() &&
        test_alarm_vt_min_on() &&
        test_alarm_vt_min_on_mod() &&
        test_alarm_vt_min_off_disc() &&
        test_alarm_vm_min_on() &&
        test_alarm_vm_min_on_mod() &&
        test_alarm_vm_min_off_disc() &&
        test_alarm_pep_max_on() &&
        test_alarm_pep_max_on_mod() &&
        test_alarm_pep_max_off_disc() &&
        test_alarm_pep_min_on() &&
        test_alarm_pep_min_on_mod() &&
        test_alarm_pep_min_off_disc() &&
        true;
}

#endif
