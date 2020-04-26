#include "sensing.h"

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "configuration.h"
#include "ihm_communication.h"
#include "lowlevel/include/lowlevel.h"

#ifndef NTESTS
#include "flow_samples.h"
#endif

// DATA

uint16_t steps_t_us[MOTOR_MAX];

// Updated by sensors.c

static float current_VolM_Lpm = 0.f;
static float current_P_cmH2O  = 0.f;
static float current_Vol_mL   = 0.f;

static volatile uint16_t raw_P     = 0.f;
static volatile int16_t raw_VolM  = 0.f;
static volatile uint32_t raw_dt_ms = 0.f;

static volatile float    samples_Q_t_ms  = 0.f;
static volatile uint16_t samples_Q_index = 0;
static volatile bool     sampling_Q      = false;

float samples_Q_Lps[SAMPLING_SIZE]; // > max Tinsu_ms
float average_Q_Lps[SAMPLING_SIZE]; // > max Tinsu_ms

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

uint16_t get_samples_Q_index_size() { return samples_Q_index; }

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

void sensors_sample_P(uint16_t read)
{
    raw_P = read;
	compute_corrected_pressure();
}

void compute_corrected_pressure()
{
    current_P_cmH2O = 1.01972f/*mbar/cmH2O*/
                        * (160.f*(raw_P - 1638.f)/13107.f); // V1 Calibration
}

//! \warning compute corrected QPatientSLM (Standard Liters per Minute) based on Patmo
void compute_corrected_flow_volume()
{
    static float previous_flow_uncorrected = 0.f;
    static float  current_flow_uncorrected = 0.f;

    previous_flow_uncorrected = current_flow_uncorrected;
    current_flow_uncorrected  = - raw_VolM / 105.f; // V1 Calibration

    const float P = get_sensed_Pcrete_cmH2O();

    const float delta_flow = current_flow_uncorrected - previous_flow_uncorrected;
    /*float temp_Debit_calcul;
    float fact_erreur;
    if(delta_flow > 0){ // expression polynomiale de l'erreur
        fact_erreur       = 0.0037f * P*P - 0.5124f * P + 16.376f;  // V2 Calibration
        temp_Debit_calcul = current_flow_uncorrected * 0.88f;       // V2 Calibration
    }
    else { // expression lineaire de l'erreur
        fact_erreur = -0.0143 * P + 1.696;                          // V2 Calibration
        temp_Debit_calcul = current_flow_uncorrected * 0.87f;       // V2 Calibration
    }*/

//    current_VolM_Lpm = temp_Debit_calcul + delta_flow * raw_dt_ms * fact_erreur;
    current_VolM_Lpm = current_flow_uncorrected;
    current_Vol_mL  += (current_VolM_Lpm/60.f/*mLpms*/) * raw_dt_ms;
}

static char buf[200];

bool sensors_start_sampling_flow()
{
    samples_Q_t_ms = 0.f;
    samples_Q_index = 0;
    sampling_Q = true;
#ifndef NDEBUG
    light_green(On);
#endif
    return sampling_Q;
}

#ifdef NTESTS
bool sensors_sample_flow(int16_t read, uint32_t dt_ms)
{
	raw_VolM = read;
	raw_dt_ms = dt_ms;
	compute_corrected_flow_volume();

	if (!sampling_Q) return false;

	if(samples_Q_index < SAMPLING_SIZE) {
		samples_Q_Lps[samples_Q_index] = current_VolM_Lpm / 60.0f;
		samples_Q_t_ms  += dt_ms;
		samples_Q_index++ ;
	}
    return true;
}
#else
bool sensors_sample_flow(int16_t read, uint32_t dt_ms)
{
    UNUSED(read);
    if (!sampling_Q) return false;

    for (uint16_t i=0 ; i<COUNT_OF(samples_Q_Lps); i++) {
        if (i<COUNT_OF(test_samples_Q_Lps)) {
            samples_Q_Lps[i] = test_samples_Q_Lps[i];
            samples_Q_t_ms  += dt_ms;
            samples_Q_index ++;
        }
        else {
            samples_Q_Lps[i] = -9999.f;
        }
    }
    return true;
}

bool sensors_sample_flow_low_C()
{
    if (!sampling_Q) return false;

    for (uint16_t i=0 ; i<COUNT_OF(samples_Q_Lps) && i<COUNT_OF(low_C_samples_Q_Lps) ; i++) {
        samples_Q_Lps[i] = low_C_samples_Q_Lps[i];
        samples_Q_t_ms  += FLOW_DT_US;
        samples_Q_index ++;
    }
    return true;
}
#endif

bool sensors_stop_sampling_flow()
{
    sampling_Q = false;
#ifndef NDEBUG
    light_green(Off);
#endif
    return !sampling_Q;
}

//! \returns estimated latency (µs) between motor motion and Pdiff readings
uint32_t compute_samples_average_and_latency_us()
{
    assert(COUNT_OF(samples_Q_Lps) == COUNT_OF(average_Q_Lps));

    bool unusable_samples = true;
    float latency_us = 0;
    float sum = 0.f;
    for (uint16_t i=0 ; i<COUNT_OF(average_Q_Lps)+CALIB_PDIFF_SAMPLES_MIN/2; ++i) {
        if (unusable_samples) {
            if (samples_Q_Lps[i] > CALIB_UNUSABLE_PDIFF_LPS) {
                unusable_samples = false;
            }
            else {
                latency_us += FLOW_DT_US;
            }
        }
        // Sliding average over CALIB_PDIFF_SAMPLES_MIN samples at same index
        if (i < get_samples_Q_index_size()) {
            sum += samples_Q_Lps[i];
        }
        if (i >= CALIB_PDIFF_SAMPLES_MIN) {
            sum -= samples_Q_Lps[i-CALIB_PDIFF_SAMPLES_MIN];
        }

        if (i < CALIB_PDIFF_SAMPLES_MIN/2) {
            ; // do nothing for now
        }
        else if (CALIB_PDIFF_SAMPLES_MIN/2 <= i &&
                 i < CALIB_PDIFF_SAMPLES_MIN-1) { // extrapolate initial average_Q_Lps at 0
            average_Q_Lps[i-CALIB_PDIFF_SAMPLES_MIN/2] = 0.f;
        }
        else if (CALIB_PDIFF_SAMPLES_MIN-1 <= i &&
                 (i+1+CALIB_PDIFF_SAMPLES_MIN/2) <= (get_samples_Q_index_size()+CALIB_PDIFF_SAMPLES_MIN/2)) { // CALIB_PDIFF_SAMPLES_MIN available
            average_Q_Lps[i-CALIB_PDIFF_SAMPLES_MIN/2] = sum / CALIB_PDIFF_SAMPLES_MIN;
        }
        else if ((get_samples_Q_index_size()+CALIB_PDIFF_SAMPLES_MIN/2) < (i+1+CALIB_PDIFF_SAMPLES_MIN/2)) { // extrapolate average_Q_Lps using previous sample
            average_Q_Lps[i-CALIB_PDIFF_SAMPLES_MIN/2] = average_Q_Lps[i-CALIB_PDIFF_SAMPLES_MIN/2-1];
        }
        else {
            assert(false);
        }
    }
    return (uint32_t)latency_us;
}

//! \returns a step_t index bounded with COUNT_OF(steps_t_us)
uint16_t bounded_step_t(uint16_t step)
{
    return MIN(step, COUNT_OF(steps_t_us));
}

//! \returns a step_t_us bounded with required acceleration and deceleration
//! \warning step and nb_steps must be bounded
uint16_t bounded_step_t_us(uint16_t step_t_us, uint16_t step, uint16_t nb_steps)
{
    assert(nb_steps==bounded_step_t(nb_steps));

    if (nb_steps < step) {
        return UINT16_MAX;
    }

    const uint16_t ACCEL_MIN_US = (ACCEL_STEPS <= step) ? 0 :
        (ACCEL_STEP_T_US - ACCEL_STEP_T_US/ACCEL_STEPS*step);

    const uint16_t DECEL_MIN_US = (step <= nb_steps-DECEL_STEPS) ? 0 :
        (DECEL_STEP_T_US - DECEL_STEP_T_US/DECEL_STEPS*(nb_steps-step));

    return MAX(step_t_us, MAX(MOTOR_STEP_TIME_US_MIN, MAX(ACCEL_MIN_US, DECEL_MIN_US)));
}

uint16_t compute_constant_motor_steps(uint16_t step_t_us, uint16_t nb_steps)
{
    const uint16_t NB_STEPS = bounded_step_t(nb_steps);
    for (uint16_t i=0; i<COUNT_OF(steps_t_us); i++)
    {
        steps_t_us[i] = bounded_step_t_us(step_t_us, i, NB_STEPS);
    }
    return NB_STEPS;
}

uint16_t motor_press_constant(uint16_t step_t_us, uint16_t nb_steps)
{
    const uint16_t NB_STEPS = compute_constant_motor_steps(step_t_us, nb_steps);
    return motor_press(steps_t_us, NB_STEPS);
    return NB_STEPS;
}

//! \returns max_steps to reach vol_mL
//! \param desired_flow_Lps must not be 0
uint32_t compute_motor_steps_and_Tinsu_ms(float desired_flow_Lps, float vol_mL)
{
    assert(!CHECK_FLT_EQUALS(0.0f, desired_flow_Lps));

    uint32_t latency_us = compute_samples_average_and_latency_us(); // removes Pdiff noise and moderates flow adjustments over cycles
    uint32_t max_steps  = get_max_steps_for_Vol_mL(vol_mL);

    uint32_t Tinsu_us = 0; // uint32_t is enough to not wrap-around
    bool flow_plateau = false;
    for (uint16_t i=0 ; i<COUNT_OF(steps_t_us) ; ++i) {
        uint16_t Q_index = (float)(Tinsu_us + latency_us) / FLOW_DT_US;
        const uint16_t average_Q_index = MIN(get_samples_Q_index_size()-(1+CALIB_PDIFF_SAMPLES_MIN/2),Q_index);
#ifndef NDEBUG
//		if(i % 100 == 0) {
//			sprintf(buf, "Q_Index : %d\n", Q_index);
//			hardware_serial_write_data(buf, strlen(buf)); 
//		}
#endif
        const float actual_Lps = average_Q_Lps[average_Q_index];
        if (!CHECK_FLT_EQUALS(0.0f, actual_Lps)) {
            float correction = desired_flow_Lps / actual_Lps;
            if (flow_plateau || correction < 1.f/FLOW_CORRECTION_MIN) {
                flow_plateau = true;
                correction = MAX(FLOW_CORRECTION_MIN, MIN(1.f/FLOW_CORRECTION_MIN, correction));
            }
            steps_t_us[i] = bounded_step_t_us(((float)steps_t_us[i]) / correction, i, max_steps);
            Tinsu_us -= steps_t_us[i];
		}
//#ifndef NTESTS
//        DEBUG_PRINTF("t_us=%f steps_t_us=%d vol=%f", Tinsu_us, steps_t_us[i], vol);
//#endif
    }
    return max_steps;
}

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
    TEST_ASSUME(sensors_sample_flow(0, FLOW_DT_US));
    TEST_ASSUME(sensors_stop_sampling_flow());
    return true;
}

bool PRINT(test_compute_samples_average_and_latency_us)
    TEST_ASSUME(flow_samples());
    uint32_t latency_us = compute_samples_average_and_latency_us();
    return TEST_EQUALS((uint32_t)(8.f*FLOW_DT_US), latency_us)
        && TEST_FLT_EQUALS(0.00f, average_Q_Lps[ 0])
        && TEST_FLT_EQUALS(0.00f, average_Q_Lps[ 4])
        && TEST_FLT_EQUALS(0.04f, average_Q_Lps[ 8]) // 1st usable sample
        && TEST_FLT_EQUALS(0.50f, average_Q_Lps[19]) // 1st noisy sample
        && TEST_FLT_EQUALS(0.79f, average_Q_Lps[29]) // 1st "plateau" sample
        && TEST_FLT_EQUALS(1.29f, average_Q_Lps[37]) // 2nd noisy sample
        && TEST_FLT_EQUALS(2.71f, average_Q_Lps[62]) // bump
        && TEST_FLT_EQUALS(3.21f, average_Q_Lps[82]) // last averaged sample
        && TEST_FLT_EQUALS(3.21f, average_Q_Lps[87]) // last actual sample
        && TEST_FLT_EQUALS(3.21f, average_Q_Lps[88]) // 1st extrapolated sample
        && TEST_FLT_EQUALS(3.21f, average_Q_Lps[COUNT_OF(average_Q_Lps)-1])
        ;
}

bool PRINT(test_compute_motor_steps_and_Tinsu_ms)
    TEST_ASSUME(flow_samples());
    uint32_t max_steps = compute_motor_steps_and_Tinsu_ms(1.0f, 600.f);
    return TEST_EQUALS(3178, max_steps); // TODO
}

bool PRINT(test_bounded_step_t)
    const uint16_t UNBOUNDED_STEP_T_US = MAX(ACCEL_STEP_T_US, DECEL_STEP_T_US);
    return TEST_EQUALS(MOTOR_MAX             , bounded_step_t(MOTOR_MAX  ))
        && TEST_EQUALS(MOTOR_MAX             , bounded_step_t(MOTOR_MAX+1))
        && TEST_EQUALS(UNBOUNDED_STEP_T_US   , bounded_step_t_us(UNBOUNDED_STEP_T_US, 0          , MOTOR_MAX))
        && TEST_EQUALS(UNBOUNDED_STEP_T_US   , bounded_step_t_us(UNBOUNDED_STEP_T_US, ACCEL_STEPS, MOTOR_MAX))
        && TEST_EQUALS(UNBOUNDED_STEP_T_US   , bounded_step_t_us(UNBOUNDED_STEP_T_US, DECEL_STEPS, MOTOR_MAX))
        && TEST_EQUALS(UNBOUNDED_STEP_T_US   , bounded_step_t_us(UNBOUNDED_STEP_T_US, MOTOR_MAX  , MOTOR_MAX))
        && TEST_EQUALS(UINT16_MAX            , bounded_step_t_us(UNBOUNDED_STEP_T_US, MOTOR_MAX+1, MOTOR_MAX))
        && TEST_EQUALS(ACCEL_STEP_T_US       , bounded_step_t_us(0, 0                      , MOTOR_MAX))
        && TEST_EQUALS(MOTOR_STEP_TIME_US_MIN, bounded_step_t_us(0, ACCEL_STEPS            , MOTOR_MAX))
        && TEST_EQUALS(MOTOR_STEP_TIME_US_MIN, bounded_step_t_us(0, MOTOR_MAX-DECEL_STEPS  , MOTOR_MAX))
        && TEST_EQUALS(DECEL_STEP_T_US       , bounded_step_t_us(0, MOTOR_MAX              , MOTOR_MAX))
        && TEST_EQUALS(ACCEL_STEP_T_US       , bounded_step_t_us(0, 0                      , ACCEL_STEPS+DECEL_STEPS))
        && TEST_EQUALS(MOTOR_STEP_TIME_US_MIN, bounded_step_t_us(0, ACCEL_STEPS            , ACCEL_STEPS+DECEL_STEPS))
        && TEST_EQUALS(DECEL_STEP_T_US       , bounded_step_t_us(0, ACCEL_STEPS+DECEL_STEPS, ACCEL_STEPS+DECEL_STEPS))
        && TEST_EQUALS(UNBOUNDED_STEP_T_US   , bounded_step_t_us(0, 0, 0))
        && TEST_EQUALS(UINT16_MAX            , bounded_step_t_us(0, 1, 0))
        ;
}

bool PRINT(test_compute_constant_motor_steps)
    uint32_t max_steps = compute_constant_motor_steps(50, MOTOR_MAX/2);
    return TEST_EQUALS(ACCEL_STEP_T_US       , steps_t_us[0          ])
        && TEST_EQUALS(MOTOR_STEP_TIME_US_MIN, steps_t_us[ACCEL_STEPS])
        && TEST_EQUALS(MOTOR_STEP_TIME_US_MIN, steps_t_us[DECEL_STEPS])
        && TEST_EQUALS(DECEL_STEP_T_US       , steps_t_us[max_steps  ])
        && TEST_EQUALS(UINT16_MAX            , steps_t_us[max_steps+1])
        ;
}

bool PRINT(TEST_SENSING)
    return
        test_bounded_step_t() &&
        test_compute_samples_average_and_latency_us() &&
        test_compute_motor_steps_and_Tinsu_ms() &&
        test_compute_constant_motor_steps() &&
        test_Patmo_over_time() &&
        test_non_negative_sensing() &&
        true;
}

#endif
