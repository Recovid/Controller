#include "sensing.h"

#include <stdlib.h>
#include <math.h>

#include "configuration.h"
#include "controller.h" // TODO isolate current_respiration_state()
#include "ihm_communication.h"

#include "lowlevel/include/lowlevel.h"

// DATA

static float VolM_Lpm     = 0.f;
static float P_cmH2O      = 0.f;
static float VTi_mL       = 0.f;
static float VTe_mL       = 0.f;

static float Pcrete_cmH2O = 0.f;
static float Pplat_cmH2O  = 0.f;
static float PEP_cmH2O    = 0.f;

static float VMe_Lpm      = 0.f;

static uint32_t last_sense_ms = 0;

static uint16_t steps_t_us[MOTOR_MAX];

float get_sensed_VTi_mL      () { return MAX(0.f,VTi_mL  ); }
float get_sensed_VTe_mL      () { return MIN(0.f,VTe_mL  ); }
float get_sensed_VolM_Lpm    () { return         VolM_Lpm ; }
float get_sensed_P_cmH2O     () { return MAX(0.f,P_cmH2O ); }

float get_sensed_Pcrete_cmH2O() { return Pcrete_cmH2O; }
float get_sensed_Pplat_cmH2O () { return Pplat_cmH2O ; }
float get_sensed_PEP_cmH2O   () { return PEP_cmH2O   ; }

float get_sensed_VMe_Lpm     () { return 0; } // TODO

float get_last_sensed_ms() { return last_sense_ms; }

// ------------------------------------------------------------------------------------------------

//! \returns estimated latency (µs) between motor motion and Pdiff readings
uint32_t compute_samples_average_and_latency_us()
{
    bool unusable_samples = true;
    uint32_t latency_us = 0;
    float sum = 0.f;
    for (uint16_t i=0 ; i<COUNT_OF(samples_Q_Lps) ; ++i) {
        if (unusable_samples) {
            if (samples_Q_Lps[i] > CALIB_UNUSABLE_PDIFF_LPS) {
                unusable_samples = false;
            }
            else {
                latency_us += SAMPLES_T_US;
            }
        }
        // Sliding average over CALIB_PDIFF_SAMPLES_MIN samples at same index
        sum += samples_Q_Lps[i];
        if (i >= CALIB_PDIFF_SAMPLES_MIN) {
            sum -= samples_Q_Lps[i-CALIB_PDIFF_SAMPLES_MIN];
        }
        const uint16_t average_index = i-CALIB_PDIFF_SAMPLES_MIN/2;
        if (i >= get_samples_Q_index_size()) {
            average_Q_Lps[average_index] = average_Q_Lps[average_index-1];
#ifndef NTESTS
            DEBUG_PRINTF("s=%d average=%f", average_index, average_Q_Lps[average_index]);
#endif
        }
        else if (i >= CALIB_PDIFF_SAMPLES_MIN) {
            average_Q_Lps[average_index] = sum / CALIB_PDIFF_SAMPLES_MIN;
#ifndef NTESTS
            DEBUG_PRINTF("s=%d average=%f", average_index, average_Q_Lps[average_index]);
#endif
        }
        else {
            average_Q_Lps[i] = 0.f;
        }
    }
    return latency_us;
}

uint16_t motor_press_constant(uint16_t step_t_us, uint16_t nb_steps)
{
    const uint16_t max_steps = MIN(nb_steps, COUNT_OF(steps_t_us));
    for(int t=0; t<max_steps; ++t) { steps_t_us[t]= step_t_us; }
    motor_press(steps_t_us, nb_steps);
    return max_steps;
}

uint16_t compute_constant_motor_steps(uint16_t step_t_us, uint16_t nb_steps)
{
    const uint16_t max_steps = MIN(nb_steps, COUNT_OF(steps_t_us));
    for(int t=0; t<max_steps; ++t) { steps_t_us[t]= step_t_us; }
    motor_press(steps_t_us, nb_steps);
    return max_steps;
}

//! \returns last steps_t_us motion to reach vol_mL
uint32_t compute_motor_steps_and_Tinsu_ms(float flow_Lps, float vol_mL)
{
    uint16_t latency_us = compute_samples_average_and_latency_us(); // removes Pdiff noise and moderates flow adjustments over cycles

    uint32_t last_step = 0;
    float Tinsu_us = 0.f;
    for (uint16_t i=0 ; i<COUNT_OF(steps_t_us) ; ++i) {
        uint16_t Q_index = (Tinsu_us + latency_us) / SAMPLES_T_US;
        const uint16_t average_Q_index = MIN(get_samples_Q_index_size()-(1+CALIB_PDIFF_SAMPLES_MIN/2),Q_index);
        const float actual_vs_desired = average_Q_Lps[average_Q_index];
        const float correction = (flow_Lps / actual_vs_desired);
        const float new_step_t_us = MAX(MOTOR_STEP_TIME_US_MIN, ((float)steps_t_us[i]) / correction);
        const float vol = Tinsu_us/1000/*ms*/ * flow_Lps;
        if (vol > 1.1f * vol_mL) { // actual Q will almost always be lower than desired
            steps_t_us[i] = UINT16_MAX; // slowest motion
        }
        else {
            Tinsu_us += new_step_t_us;
            steps_t_us[i] = new_step_t_us;
#ifndef NTESTS
            DEBUG_PRINTF("t_us=%f steps_t_us=%d vol=%f", Tinsu_us, steps_t_us[i], vol);
#endif
            last_step = i;
        }
    }
    DEBUG_PRINTF("Tinsu predicted = %d ms", (uint32_t)(Tinsu_us/1000));
    return last_step;
}

// ------------------------------------------------------------------------------------------------

void sense_and_compute(RespirationState state)
{
    static unsigned long last_state = Insufflation;
    static unsigned long sent_DATA_ms = 0;

    // TODO float Patmo_mbar = read_Patmo_mbar();
    P_cmH2O  = read_Paw_cmH2O();
    VolM_Lpm = read_Pdiff_Lpm(); // TODO Compute corrected QPatientSLM based on Patmo
    float Vol_mL = 0.f;
    if (state==Insufflation || state==Plateau) {
        if (last_state==Exhalation || last_state==ExhalationPause) {
            VTi_mL       = 0.f;
            Pcrete_cmH2O = 0.f;
            sensors_start_sampling_flow();
        }
        else {
            VTi_mL += MAX(0.f, (VolM_Lpm/60.f/*mLpms*/) * (get_time_ms()-last_sense_ms));
            Pcrete_cmH2O = MAX(Pcrete_cmH2O, P_cmH2O); // TODO check specs
        }
        if (state==Plateau) {
            if (last_state==Insufflation) {
                sensors_stop_sampling_flow();
                compute_motor_steps_and_Tinsu_ms(get_setting_Vmax_Lpm()/60.f, get_setting_VT_mL());
                Pplat_cmH2O = Pcrete_cmH2O;
            }
            else {
                Pplat_cmH2O = MIN(Pplat_cmH2O, P_cmH2O); // TODO average over Xms
            }
        }
        Vol_mL = VTi_mL; // TODO Check if we really want to ignore VTe_mL to avoid drift
    }
    else if (state==Exhalation || state==ExhalationPause) {
        if (last_state==Insufflation || last_state==Plateau) {
            VTe_mL = 0.f;
            PEP_cmH2O = 0.f;
        }
        else {
            VTe_mL += MIN(0.f, (VolM_Lpm/60.f/*mLpms*/) * (get_time_ms()-last_sense_ms));
            PEP_cmH2O = P_cmH2O; // TODO average over Xms
        }
        Vol_mL = VTi_mL+VTe_mL;
    }

    if ((sent_DATA_ms+50 < get_time_ms()) // @ 20Hz
        && send_DATA(get_sensed_P_cmH2O(), get_sensed_VolM_Lpm(), Vol_mL)) { // TODO send_DATA_X
        sent_DATA_ms = get_time_ms();
    }

    last_state    = state;
    last_sense_ms = get_time_ms();
}

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

bool PRINT(test_non_negative_sensing)
    VTi_mL   = -1.f;
    P_cmH2O  = -1.f;
    return
        TEST_FLT_EQUALS(0.f, get_sensed_VTi_mL ()) &&
        TEST_FLT_EQUALS(0.f, get_sensed_P_cmH2O()) &&
        true;
}

bool PRINT(TEST_SENSING)
    return
        test_non_negative_sensing() &&
        true;
}

#endif
