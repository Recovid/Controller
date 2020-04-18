#include "sensing.h"

#include <stdlib.h>
#include <math.h>

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

static unsigned long last_sense_ms = 0;

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

//! Compute slope of samples fetched with specified time_step_sec
//! \param slope (out)
//! \returns R if fit is ok else -1
float linear_fit(float* samples, uint16_t samples_len, float time_step_sec, float* slope)
{
    float sumx=0, sumy=0, sumxy=0, sumx2=0, sumy2=0;
    for(uint16_t i=0 ; i<samples_len ; i++) {
        sumx  = sumx  + (float)i * time_step_sec;
        sumx2 = sumx2 + (float)i * time_step_sec * (float)i * time_step_sec;
        sumy  = sumy  + (*(samples+i));
        sumy2 = sumy2 + (*(samples+i)) * (*(samples+i));
        sumxy = sumxy + (float)i*(time_step_sec)* (*(samples+i));
    }
    const float denom = (samples_len * sumx2 - (sumx * sumx));
    if (denom == 0.) {
        DEBUG_PRINT("Calibration of A is not possible");
        return -1.f;
    }
    // compute slope a
    *slope = (samples_len * sumxy  -  sumx * sumy) / denom;
    DEBUG_PRINTF("%d     ", (int32_t)(1000*((samples_len * sumxy  -  sumx * sumy) / denom)));

    // compute correlation coefficient
    return (sumxy - sumx*sumy/samples_len) / sqrtf((sumx2 - sumx*sumx/samples_len) * (sumy2 - (sumy*sumy)/samples_len));
}

//! Find the plateau of the curve by slicing it in N windows (windows_number = 10 usually).
//! The curve is describe by samples of size samples_len
//! \param low_bound (out) first index
//! \param high_bound (out) last index
//! of the samples corresponding to the plateau
//! \remark The high_bound is ALWAYS the last sample index
//!		    If no low_bound is found, low_bound = middle sample index
int32_t get_plateau(float* samples, size_t samples_len, float time_step_sec, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound)
{
    if (windows_number < 2 || 30 < windows_number) { return -1; }

    float slopes[30];
    *high_bound = samples_len-1;
    // Compute slope for time windows to detect when signal start increasing/decreasing
    for(int window=0; window<windows_number; window++) {
        const float r = linear_fit(samples+window*(samples_len/windows_number), samples_len/windows_number, time_step_sec, slopes+window);
        DEBUG_PRINTF("%d    ", (int32_t)(*(slopes+window) * 1000));
    }
    for(int window=1 ; window<windows_number ; window++) {
        float delta_slope = slopes[window-1] - slopes[window];
        if(delta_slope > 1.) {
            *low_bound = (uint32_t)((samples_len/windows_number)*(window+1));
            DEBUG_PRINTF("plateau begin at %u over %u points", *low_bound, (uint32_t)samples_len);
            return 0;
        }
    }
    *low_bound = (uint32_t)(samples_len/2);
    DEBUG_PRINT("No plateau found");
    return 1;
}

//! Compute average flow and slope to adjust A_calibrated and B_calibrated
//! A_calibrated and B_calibrated the terms of the PID
//! NB : The PID is only a P for now, so keep that in mind
//!
//! \warning not the same as "calibration(float* A, float* B, uint8_t iterations)" !
//! calibration() is for testing, compute_pid() is to be called at the end of every EXPIRATION MOVEMENT
void compute_pid(float* A, float* B, float* samples, uint32_t samples_index, float samples_time_step_sum, float desired_flow_sls)
{
    const float P_plateau_slope = 0.1;
    const float P_plateau_mean = 0.2;
    const float timeStep = samples_time_step_sum / samples_index;
    uint32_t low;
    uint32_t high;
    if (get_plateau(samples, samples_index, timeStep, 10, &low, &high) == 0) {
        DEBUG_PRINTF("plateau found from sample %u to %u", low, high);
    } else {
        DEBUG_PRINTF("plateau NOT found, considering from sample %u to %u", low, high);
    }
    float plateau_slope = linear_fit(samples+low, high-low-1, timeStep, &plateau_slope);
    float plateau_mean  = 0;
    for(uint32_t i=low ; i<high ; i++) {
        plateau_mean += samples[i];
    }
    plateau_mean = plateau_mean/(high-low);
    DEBUG_PRINTF("plateau slope : %d",(int32_t)(1000*plateau_slope));
    DEBUG_PRINTF("plateau mean  : %d",(int32_t)(1000*plateau_mean ));

    const float error_mean = plateau_mean - desired_flow_sls;

    *A += plateau_slope * P_plateau_slope;
    *B += error_mean    * P_plateau_mean ;
    DEBUG_PRINTF("A = %d", (int32_t)(1000*(*A)));
    DEBUG_PRINTF("B = %d", (int32_t)(1000*(*B)));
}

//! Compute the motor step time in us
//! \param calibration_speed is motor step time in seconds
//! \param desired_flow is in sL/s (sic)
//! \param A_calibrated is the proportional term computed from the slope
//! \param B_calibrated is the constant termb
//! \returns step time in us
float compute_motor_step_time(long step_number, float desired_flow_sls, double calibration_speed, float A_calibrated, float B_calibrated)
{
    float res = (A_calibrated*calibration_speed*calibration_speed*step_number) + B_calibrated * calibration_speed;
    res /= desired_flow_sls;
    return MAX(res * 1000000.f, 110.f); // FIXME Move magic number to configuration.h
}

uint32_t compute_motor_steps(uint32_t* motor_speed_table, long steps, float setpoint_flow_slm, float CALIBRATION_SPEED, float A_calibrated, float B_calibrated)
{
    uint32_t Tinsu_us = 0.;
    for(long t=0; t<steps; ++t) {
        float d = compute_motor_step_time(t, setpoint_flow_slm/60., CALIBRATION_SPEED, A_calibrated, B_calibrated);
        Tinsu_us += d;
        DEBUG_PRINTF("d=%d", (uint32_t)(d));
        motor_speed_table[t]= (uint32_t)d;
    }
    DEBUG_PRINTF("Tinsu predicted = %d ms\n", (uint32_t)(Tinsu_us/1000));
    return (uint32_t)(Tinsu_us/1000);
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
        }
        else {
            VTi_mL += MAX(0.f, (VolM_Lpm/60.f/*mLpms*/) * (get_time_ms()-last_sense_ms));
            Pcrete_cmH2O = MAX(Pcrete_cmH2O, P_cmH2O); // TODO check specs
        }
        if (state==Plateau) {
            if (last_state==Insufflation) {
                Pplat_cmH2O  = Pcrete_cmH2O;
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
