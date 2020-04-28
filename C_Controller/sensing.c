#include "sensing.h"

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "configuration.h"
#include "ihm_communication.h"
#include "lowlevel/include/lowlevel.h"
#include "platform.h"

#ifndef NTESTS
#include "flow_samples.h"
#endif

// DATA

static uint32_t last_sense_ms = 0;

uint16_t steps_t_us[MOTOR_MAX];
uint16_t last_step = 0;
// Updated by sensors.c

static float current_VolM_Lpm = 0.f;
static float current_P_cmH2O  = 0.f;
static float current_Vol_mL   = 0.f;

static volatile uint16_t raw_P     = 0.f;
static volatile int16_t raw_VolM  = 0.f;
static volatile uint32_t raw_dt_us = 0.f;

static volatile float    samples_Q_t_us  = 0.f;
static volatile uint16_t samples_Q_index = 0;
static volatile bool     sampling_Q      = false;


float samples_Q_Lps[SAMPLING_SIZE]; // > max Tinsu_ms
uint16_t samples_Q_Lps_dt_us[SAMPLING_SIZE]; // > max Tinsu_ms
float average_Q_Lps[SAMPLING_SIZE]; // > max Tinsu_ms

uint16_t samples_P[SAMPLING_SIZE]; // > max Tinsu_ms
uint16_t samples_P_dt_us[SAMPLING_SIZE]; // > max Tinsu_ms
static volatile uint16_t samples_P_index = 0;
static volatile bool     sampling_P      = false;
// ------------------------------------------------------------------------------------------------


//! \returns the volume (corresponding to corrected integration of pressure differences) in mLiters
float get_sensed_Vol_mL      () { return current_Vol_mL; }
void  reset_sensed_Vol_mL    () { current_Vol_mL = 0.0f; }

//! \returns the airflow corresponding to a pressure difference in Liters / minute
float get_sensed_VolM_Lpm()
{
#ifdef NTESTS
    return current_VolM_Lpm;
#else
    if (get_valve_state() == Inhale) {
        return BAVU_Q_Lpm() * EXHAL_VALVE_RATIO;
    }
    else if (get_valve_state() == Exhale) {
        return get_valve_exhale_ms()+LUNG_EXHALE_MS>get_time_ms() ?
          -(60.f*get_sensed_VTi_mL()/LUNG_EXHALE_MS)*(get_valve_exhale_ms()+LUNG_EXHALE_MS-get_time_ms())/LUNG_EXHALE_MS*2 :
            0.f; // 0 after LUNG_EXHALE_MS and VTe=-VTi
    }
    else {
        return 0.f;
    }
#endif
}

//! \returns the sensed pressure in cmH2O (1,019mbar in standard conditions)
float get_sensed_P_cmH2O()
{
#ifdef NTESTS
    return MAX(0.f, current_P_cmH2O);
#else
    const float Paw_cmH2O =
        get_setting_PEP_cmH2O() // TODO loop back with get_sensed_PEP_cmH2O()
        + (get_valve_state()==Exhale ? 0.f : (BAVU_V_ML_MAX - BAVU_V_mL()) / LUNG_COMPLIANCE)
        + fabsf(get_sensed_VolM_Lpm()) * AIRWAYS_RESISTANCE;
    assert(Paw_cmH2O >= 0);
    return Paw_cmH2O;
#endif
}

float get_last_sensed_ms() { return last_sense_ms; }

uint16_t get_samples_Q_index_size() { return samples_Q_index; }

uint16_t get_samples_P_index_size() { return samples_P_index; }

//! \returns the atmospheric pressure in mbar
//! \warning NOT IMPLEMENTED
float get_sensed_Patmo_mbar()
{
#ifndef NTESTS
    return 1013. + sinf(2*M_PI*get_time_ms()/1000/60) * PATMO_VARIATION_MBAR; // TODO test failure
#else
    return 1013.; // TODO
#endif
}

// ------------------------------------------------------------------------------------------------

bool sensors_sample_P(uint16_t read, uint16_t dt_us)
{
	UNUSED(dt_us);
    raw_P = read;
	compute_corrected_pressure();
	if (!sampling_P) return false;

	if(samples_P_index < SAMPLING_SIZE) {
		samples_P[samples_P_index] = raw_P;
		samples_P_dt_us[samples_P_index]  = dt_us;
		samples_P_index++ ;
	}
    return true;
}

void compute_corrected_pressure()
{
    current_P_cmH2O = 1.01972f/*mbar/cmH2O*/
                        * (160.f*(raw_P - 1638.f)/13107.f); // V1 Calibration
}

//! \warning TODO compute corrected QPatientSLM (Standard Liters per Minute) based on Patmo
void compute_corrected_flow_volume()
{
    const float uncorrected_flow_Lpm = - raw_VolM / 105.f; // V1 Calibration

    // V2 Calibration
    const float P = get_sensed_Pcrete_cmH2O();
    float temp_flow_Lpm;
    float error_ps;
    if (uncorrected_flow_Lpm < 0){ // polynomial error correction
        error_ps = 0.0037f * P*P - 0.5124f * P + 16.376f;
        temp_flow_Lpm = uncorrected_flow_Lpm * 0.88f;
    }
    else { // linear error correction
        error_ps = -0.0143f * P + 1.696f;
        temp_flow_Lpm = uncorrected_flow_Lpm * 0.87f;
    }

    current_VolM_Lpm = temp_flow_Lpm + uncorrected_flow_Lpm * error_ps * raw_dt_us/1000000.f/*s*/;
    current_Vol_mL  += (current_VolM_Lpm/60.f/*mLpms*/) * raw_dt_us * 1000.f;
}

static char buf[200];

bool sensors_start_sampling_flow()
{
    samples_Q_t_us = 0.f;
    samples_Q_index = 0;
    sampling_Q = true;
    samples_P_index = 0;
    sampling_P = true;

    return sampling_Q;
}



#ifdef NTESTS
bool sensors_sample_flow(int16_t read, uint16_t dt_us)
{
	raw_VolM = read;
	raw_dt_us = dt_us;
	compute_corrected_flow_volume();

	if (!sampling_Q) return false;

	if(samples_Q_index < SAMPLING_SIZE) {
		samples_Q_Lps[samples_Q_index] = read;
		samples_Q_Lps_dt_us[samples_Q_index]  = dt_us;
		samples_Q_t_us  += dt_us;
		samples_Q_index++ ;
	}
    return true;
}
#else
bool sensors_sample_flow(int16_t read, uint16_t dt_us)
{
    UNUSED(read); //TODO read values
    if (!sampling_Q) return false;

    for (uint16_t i=0 ; i<COUNT_OF(samples_Q_Lps) && i<COUNT_OF(inf_C_samples_Q_Lps) ; i++) {
        samples_Q_Lps[i] = inf_C_samples_Q_Lps[i];
        samples_Q_Lps_dt_us[i] = dt_us;
        samples_Q_t_us  += dt_us;
        samples_Q_index ++;
    }
    return true;
}

bool sensors_sample_flow_low_C()
{
    if (!sampling_Q) return false;

    for (uint16_t i=0 ; i<COUNT_OF(samples_Q_Lps) && i<COUNT_OF(low_C_samples_Q_Lps) ; i++) {
        samples_Q_Lps[i] = low_C_samples_Q_Lps[i];
        samples_Q_t_us  += SAMPLES_T_US;
        samples_Q_index ++;
    }
    return true;
}
#endif

bool sensors_stop_sampling_flow()
{
    sampling_Q = false;
    sampling_P = false;
    return !sampling_Q;
}







// ------------------------------------------------------------------------------------------------

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

bool PRINT(test_non_negative_sensing)
    current_P_cmH2O  = -1.f;
    return
        TEST_FLT_EQUALS(0.f, get_sensed_VTi_mL ()) &&
        TEST_FLT_EQUALS(0.f, get_sensed_P_cmH2O()) &&
        true;
}

bool PRINT(test_Patmo_over_time)
    float last_Patmo = 0.f;
for (uint32_t t_s=0; t_s < 60*60 ; t_s=wait_ms(60*1000/8)/1000) {
    if (!(TEST_RANGE(1013-PATMO_VARIATION_MBAR, get_sensed_Patmo_mbar(), 1013+PATMO_VARIATION_MBAR) &&
          TEST(last_Patmo != get_sensed_Patmo_mbar())))
        return false;
}
return true;
}

bool flow_samples()
{
    TEST_ASSUME(sensors_start_sampling_flow());
    TEST_ASSUME(sensors_sample_flow(0, SAMPLES_T_US));
    TEST_ASSUME(sensors_stop_sampling_flow());
    return true;
}

bool PRINT(test_compute_samples_average_and_latency_us)
    TEST_ASSUME(flow_samples());
    uint32_t latency_us = compute_samples_average_and_latency_us();
    return TEST_EQUALS(10*SAMPLES_T_US, latency_us)
       && TEST_FLT_EQUALS(2.7f, samples_Q_Lps[171]);
}

bool PRINT(test_compute_motor_steps_and_Tinsu_ms)
    TEST_ASSUME(flow_samples());
    uint32_t last_step = compute_motor_steps_and_Tinsu_ms(1.5f, 230.f, steps_t_us);
    return TEST_EQUALS(309, last_step); // TODO Check with more accurate calibration
}

bool PRINT(TEST_SENSING)
    return
        test_non_negative_sensing() &&
        test_compute_samples_average_and_latency_us() &&
        test_compute_motor_steps_and_Tinsu_ms() &&
        test_Patmo_over_time() &&
        true;
}

#endif
