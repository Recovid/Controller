#include "alarms.h"

#include <string.h>

#include "sensing.h" //  to use sensed values that depend on controller state
#include "ihm_communication.h" // to send alarms
#include "lowlevel/include/lowlevel.h"

//! Bit flags for currently active alarms
int32_t activeAlarms = 0;
int32_t activeAlarmsOld = 0;

static uint32_t activationTime_ms = 0;

//! Alarms levels
static const uint8_t ALARM_LEVELS[] = {
    3, // PMAX
    3, // PMIN
    2, // VT_MIN
    0, // FR
    2, // VM_MIN
    3, // PEP_MAX
    2, // PEP_MIN
    1, // BATT_A
    3, // BATT_B
    2, // BATT_C
    2, // BATT_D
    3, // FAILSAFE
    3, // CPU_LOST
    3, // P_KO
    1, // IO_MUTE
};

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

//! Save latest values into bounded queues for alarm detection
static void save_sensed_values()
{
    pressureMax_startIdx = (pressureMax_startIdx - 1 + PMAX_Q_LEN) % PMAX_Q_LEN;
    pressureMax_cmH2O_q[pressureMax_startIdx] = get_sensed_P_cmH2O();

    Pcrete_startIdx = (Pcrete_startIdx - 1 + PCRETE_Q_LEN) % PCRETE_Q_LEN;
    Pcrete_cmH2O_q[Pcrete_startIdx] = get_sensed_Pcrete_cmH2O();
    Pcrete_time_ms_q[Pcrete_startIdx] = get_last_sensed_ms();

    VTe_startIdx = (VTe_startIdx - 1 + VTE_Q_LEN) % VTE_Q_LEN;
    VTe_ml_q[VTe_startIdx] = get_sensed_VTe_mL();

    VM_startIdx = (VM_startIdx - 1 + VM_Q_LEN) % VM_Q_LEN;
    VM_Lm_q[VM_startIdx] = get_sensed_VolM_Lpm();

    PEP_startIdx = (PEP_startIdx - 1 + PEP_Q_LEN) % PEP_Q_LEN;
    PEP_cmH2O_q[PEP_startIdx] = get_sensed_PEP_cmH2O();
}

static bool monitor_sensed_values()
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
                    activeAlarms |= ALARM_PMAX;
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
                    activeAlarms |= ALARM_PMIN;
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
                    activeAlarms |= ALARM_VT_MIN;
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
                    activeAlarms |= ALARM_VM_MIN;
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
                    activeAlarms |= ALARM_PEP_MAX;
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
                    activeAlarms |= ALARM_PEP_MIN;
                }
            }
            else
                PEPmin_cycles = 0; // deactivate PEPmin Alarm
        }
    }
    return true;
}

void monitor_battery()
{
    /*
    if (is_DC_on()) {
        return;
    }

    if (is_Battery_charged()) {
        activeAlarms |= ALARM_BATT_A;
        return;
    }

    // battery is not charged (<85%)
    // TODO: alarm type (B/C/D) depends on the elpased time on battery
    activeAlarms |= ALARM_BATT_B;
    */
}

void blink_LEDs(int level)
{
    static const uint32_t CYCLE_DUR_MS = 10000;

    static bool redOn = false;
    static bool yellowOn = false;

    if (level <= 1) {
        if (redOn) {
            light_red(Off);
            redOn = false;
        }
        if (yellowOn) {
            light_yellow(Off);
            yellowOn = false;
        }
        return;
    }

    uint32_t cycleTime_ms = (get_time_ms() - activationTime_ms) % CYCLE_DUR_MS;

    if (cycleTime_ms >= CYCLE_DUR_MS / 2) {
        // only blink in the first half of a cycle
        return;
    }

    float blinkFreq_hz = (level == 3 ? 2.f : 0.5f);
    uint32_t blinkDur_ms = (uint32_t)(1000.f / blinkFreq_hz) / 2;
    bool onNew = (cycleTime_ms / blinkDur_ms) % 2 == 0;
    bool redOnNew = false;
    bool yellowOnNew = false;

    if (level == 3) {
        redOnNew = onNew;
    } else {
        yellowOnNew = onNew;
    }

    if (redOnNew != redOn) {
        light_red(redOnNew ? On : Off);
    }
    if (yellowOnNew != yellowOn) {
        light_yellow(yellowOnNew ? On : Off);
    }

    redOn = redOnNew;
    yellowOn = yellowOnNew;
}

//! Trigger and send active alarms
void trigger_alarms()
{
    // uint32_t newAlarms = (activeAlarms ^ activeAlarmsOld) & activeAlarms;

    // compute the maximum active level
    uint8_t levelMax = 0;
    uint8_t levelMaxOld = 0;
    for (int i = 0; i < ALARM_COUNT; ++i) {
        if (activeAlarms & (1 << i))
            levelMax = MAX(levelMax, ALARM_LEVELS[i]);
        if (activeAlarmsOld & (1 << i))
            levelMaxOld = MAX(levelMaxOld, ALARM_LEVELS[i]);
    }

    // reset the activation time whenever we activate a higher level alarm
    if (levelMax > levelMaxOld) {
        activationTime_ms = get_time_ms();
    }

    blink_LEDs(levelMax);

    send_ALRM(activeAlarms);
}

bool update_alarms()
{
    save_sensed_values();
    monitor_sensed_values();
    monitor_battery();
    trigger_alarms();
    return true;
}

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

static bool PRINT(test_alarm_pmax_on0)
    pressureMax_startIdx = 1;
    pressureMax_cmH2O_q[0] = MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10);
    pressureMax_cmH2O_q[1] = MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10) + 1;
    monitor_sensed_values();
    return TEST(activeAlarms & ALARM_PMAX);
}

static bool PRINT(test_alarm_pmax_off0)
    pressureMax_startIdx = 0;
    pressureMax_cmH2O_q[0] = MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10) - 1;
    pressureMax_cmH2O_q[1] = MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10);
    monitor_sensed_values();
    return TEST(!(activeAlarms & ALARM_PMAX));
}

static bool PRINT(test_alarm_pmax_off1)
    pressureMax_startIdx = 0;
    pressureMax_cmH2O_q[0] = MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10);
    pressureMax_cmH2O_q[1] = MAX(get_setting_Pmax_cmH2O(), get_setting_PEP_cmH2O() + 10) - 1;
    monitor_sensed_values();
    return TEST(!(activeAlarms & ALARM_PMAX));
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

    monitor_sensed_values();
    return TEST(activeAlarms & ALARM_PMIN);
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

    monitor_sensed_values();
    return TEST(activeAlarms & ALARM_PMIN);
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

    monitor_sensed_values();
    return TEST(!(activeAlarms & ALARM_PMIN));
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

    monitor_sensed_values();
    return TEST(!(activeAlarms & ALARM_PMIN));
}

static bool PRINT(test_alarm_vt_min_on)
    VTe_startIdx = 0;
    VTe_ml_q[0] = get_setting_VTmin_mL();
    VTe_ml_q[1] = get_setting_VTmin_mL() - 1;
    VTe_ml_q[2] = get_setting_VTmin_mL() - 2;
    monitor_sensed_values();
    return TEST(activeAlarms & ALARM_VT_MIN);
}

static bool PRINT(test_alarm_vt_min_on_mod)
    VTe_startIdx = 1;
    VTe_ml_q[1] = get_setting_VTmin_mL() - 1;
    VTe_ml_q[2] = get_setting_VTmin_mL() - 2;
    VTe_ml_q[0] = get_setting_VTmin_mL();
    monitor_sensed_values();
    return TEST(activeAlarms & ALARM_VT_MIN);
}

static bool PRINT(test_alarm_vt_min_off_disc)
    VTe_startIdx = 0;
    VTe_ml_q[0] = get_setting_VTmin_mL();
    VTe_ml_q[1] = get_setting_VTmin_mL() + 1;
    VTe_ml_q[2] = get_setting_VTmin_mL() - 2;
    monitor_sensed_values();
    return TEST(!(activeAlarms & ALARM_VT_MIN));
}

static bool PRINT(test_alarm_vm_min_on)
    VM_startIdx = 0;
    VM_Lm_q[0] = get_setting_VMmin_Lm();
    VM_Lm_q[1] = get_setting_VMmin_Lm() - 1;
    VM_Lm_q[2] = get_setting_VMmin_Lm() - 2;
    monitor_sensed_values();
    return TEST(activeAlarms & ALARM_VM_MIN);
}

static bool PRINT(test_alarm_vm_min_on_mod)
    VM_startIdx = 1;
    VM_Lm_q[1] = get_setting_VMmin_Lm() - 1;
    VM_Lm_q[2] = get_setting_VMmin_Lm() - 2;
    VM_Lm_q[0] = get_setting_VMmin_Lm();
    monitor_sensed_values();
    return TEST(activeAlarms & ALARM_VM_MIN);
}

static bool PRINT(test_alarm_vm_min_off_disc)
    VM_startIdx = 0;
    VM_Lm_q[0] = get_setting_VMmin_Lm();
    VM_Lm_q[1] = get_setting_VMmin_Lm() + 1;
    VM_Lm_q[2] = get_setting_VMmin_Lm() - 2;
    monitor_sensed_values();
    return TEST(!(activeAlarms & ALARM_VM_MIN));
}

static bool PRINT(test_alarm_pep_max_on)
    PEP_startIdx = 0;
    for (int i = 0; i < PEP_Q_LEN; ++i) {
        PEP_cmH2O_q[i] = get_setting_PEP_cmH2O() + 2 + i;
    }
    monitor_sensed_values();
    return TEST(activeAlarms & ALARM_PEP_MAX);
}

static bool PRINT(test_alarm_pep_max_on_mod)
    PEP_startIdx = 4;
    for (int i = 0; i < PEP_Q_LEN; ++i) {
        PEP_cmH2O_q[i] = get_setting_PEP_cmH2O() + 2 + i;
    }
    monitor_sensed_values();
    return TEST(activeAlarms & ALARM_PEP_MAX);
}

static bool PRINT(test_alarm_pep_max_off_disc)
    PEP_startIdx = 0;
    for (int i = 0; i < PEP_Q_LEN; ++i) {
        PEP_cmH2O_q[i] = get_setting_PEP_cmH2O() + 2 + i;
    }
    PEP_cmH2O_q[4] = get_setting_PEP_cmH2O();
    monitor_sensed_values();
    return TEST(!(activeAlarms & ALARM_PEP_MAX));
}

static bool PRINT(test_alarm_pep_min_on)
    PEP_startIdx = 0;
    for (int i = 0; i < PEP_Q_LEN; ++i) {
        PEP_cmH2O_q[i] = get_setting_PEP_cmH2O() - 2 - i;
    }
    monitor_sensed_values();
    return TEST(activeAlarms & ALARM_PEP_MIN);
}

static bool PRINT(test_alarm_pep_min_on_mod)
    PEP_startIdx = 4;
    for (int i = 0; i < PEP_Q_LEN; ++i) {
        PEP_cmH2O_q[i] = get_setting_PEP_cmH2O() - 2 - i;
    }
    monitor_sensed_values();
    return TEST(activeAlarms & ALARM_PEP_MIN);
}

static bool PRINT(test_alarm_pep_min_off_disc)
    PEP_startIdx = 0;
    for (int i = 0; i < PEP_Q_LEN; ++i) {
        PEP_cmH2O_q[i] = get_setting_PEP_cmH2O() - 2 - i;
    }
    PEP_cmH2O_q[4] = get_setting_PEP_cmH2O();
    monitor_sensed_values();
    return TEST(!(activeAlarms & ALARM_PEP_MIN));
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
