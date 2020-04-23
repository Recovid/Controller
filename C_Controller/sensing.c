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

static uint32_t last_sense_ms = 0;

uint16_t steps_t_us[MOTOR_MAX];
uint16_t last_step = 0;
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
float reset_sensed_Vol_mL      () { current_Vol_mL = 0.0f; }

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
          -(60.f*VTi_mL/LUNG_EXHALE_MS)*(get_valve_exhale_ms()+LUNG_EXHALE_MS-get_time_ms())/LUNG_EXHALE_MS*2 :
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
	light_green(On);

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
    UNUSED(read); //TODO read values
    if (!sampling_Q) return false;

    for (uint16_t i=0 ; i<COUNT_OF(samples_Q_Lps) && i<COUNT_OF(inf_C_samples_Q_Lps) ; i++) {
        samples_Q_Lps[i] = inf_C_samples_Q_Lps[i];
        samples_Q_t_ms  += dt_ms;
        samples_Q_index ++;
    }
    return true;
}

bool sensors_sample_flow_low_C()
{
    if (!sampling_Q) return false;

    for (uint16_t i=0 ; i<COUNT_OF(samples_Q_Lps) && i<COUNT_OF(low_C_samples_Q_Lps) ; i++) {
        samples_Q_Lps[i] = low_C_samples_Q_Lps[i];
        samples_Q_t_ms  += SAMPLES_T_US;
        samples_Q_index ++;
    }
    return true;
}
#endif

bool sensors_stop_sampling_flow()
{
    sampling_Q = false;
	light_green(Off);
    return !sampling_Q;
}

//! \returns estimated latency (µs) between motor motion and Pdiff readings
uint32_t compute_samples_average_and_latency_us()
{
    bool unusable_samples = true;
    uint32_t latency_us = 0;
    float sum = 0.f;
    for (uint16_t i=0 ; i<COUNT_OF(samples_Q_Lps) && i < COUNT_OF(average_Q_Lps); ++i) {
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




        if (i >= get_samples_Q_index_size() && (i >= 1 + CALIB_PDIFF_SAMPLES_MIN/2) ) {
            average_Q_Lps[ (i-CALIB_PDIFF_SAMPLES_MIN/2) ] = average_Q_Lps[(i-CALIB_PDIFF_SAMPLES_MIN/2)- 1];
        }
        else if ((CALIB_PDIFF_SAMPLES_MIN/2 ) <= i && (i<= get_samples_Q_index_size() - (CALIB_PDIFF_SAMPLES_MIN/2)) ) {
            average_Q_Lps[(i-CALIB_PDIFF_SAMPLES_MIN/2)] = sum / CALIB_PDIFF_SAMPLES_MIN;
        }
        else {
			//if (i >= 1 + CALIB_PDIFF_SAMPLES_MIN/2) {
			//	light_green(On);
			//}
            average_Q_Lps[i] = 0.f;
        }
    }
    return latency_us;
}

uint16_t motor_press_constant(uint16_t step_t_us, uint16_t nb_steps)
{
    const uint16_t max_steps = MIN(nb_steps, COUNT_OF(steps_t_us));
    for(int i = 0; i < COUNT_OF(steps_t_us); i++)
    {
        if(i < max_steps) {
		steps_t_us[i] = MAX(step_t_us, MOTOR_STEP_TIME_INIT - (A)*i);
        }
        else {
		steps_t_us[i] = MIN(UINT16_MAX, step_t_us + (A)*(i-max_steps));
        }
    }
    motor_press(steps_t_us, max_steps);
    return max_steps;
}

uint16_t compute_constant_motor_steps(uint16_t step_t_us, uint16_t nb_steps)
{
    const uint16_t max_steps = MIN(nb_steps, COUNT_OF(steps_t_us));
    for(int t=0; t<max_steps; ++t) { steps_t_us[t]= step_t_us; }
    motor_press(steps_t_us, nb_steps);
    return max_steps;
}

float corrections[MOTOR_MAX];
//! \returns last steps_t_us motion to reach vol_mL
uint32_t compute_motor_steps_and_Tinsu_ms(float desired_flow_Lps, float vol_mL)
{
    uint32_t latency_us = compute_samples_average_and_latency_us(); // removes Pdiff noise and moderates flow adjustments over cycles
//	sprintf(buf, "latency : %d\n", latency_us);
//	hardware_serial_write_data(buf, strlen(buf)); 
//	sprintf(buf, "get_samples_Q_index_size : %d\n", get_samples_Q_index_size());
//	hardware_serial_write_data(buf, strlen(buf)); 

    uint32_t last_step = 0;
    float Tinsu_us = 0.f;
    for (uint16_t i=0 ; i<COUNT_OF(steps_t_us) ; ++i) {
        uint16_t Q_index = (Tinsu_us + latency_us) / SAMPLES_T_US;
        const uint16_t average_Q_index = MIN(get_samples_Q_index_size()-(1+CALIB_PDIFF_SAMPLES_MIN/2),Q_index);
//		if(i % 100 == 0) {
//			sprintf(buf, "Q_Index : %d\n", Q_index);
//			hardware_serial_write_data(buf, strlen(buf)); 
//		}
        const float actual_Lps = average_Q_Lps[average_Q_index];
        float correction;
		if(CHECK_FLT_EQUALS(0.0f, actual_Lps) || CHECK_FLT_EQUALS(0.0f, desired_flow_Lps))
		{
			light_red(On);
			correction = 1;
		}
		else {
			correction = MAX(0.5, MIN(2.0f, desired_flow_Lps / actual_Lps));
			light_red(Off);
		}
		corrections[i] = correction;

        const float new_step_t_us = MAX(MOTOR_STEP_TIME_US_MIN, ((float)steps_t_us[i]) / correction);
        const float vol = Tinsu_us/1000/*ms*/ * desired_flow_Lps;
		Tinsu_us += new_step_t_us;
		if(new_step_t_us < MOTOR_STEP_TIME_INIT - (A*i)) {
			steps_t_us[i] = MOTOR_STEP_TIME_INIT - (A*i);
			last_step = i;
		}
		else if (vol < 1.0f * vol_mL) { // actual Q will almost always be lower than desired TODO +10%
			steps_t_us[i] = new_step_t_us;
			last_step = i;
		}
		else {
			steps_t_us[i] = MIN(UINT16_MAX, new_step_t_us + (A)*(i-last_step));
		}
//#ifndef NTESTS
//            DEBUG_PRINTF("t_us=%f steps_t_us=%d vol=%f", Tinsu_us, steps_t_us[i], vol);
//#endif
    }
//    DEBUG_PRINTF("Tinsu predicted = %d ms", (uint32_t)(Tinsu_us/1000));
    return last_step;
}

// ------------------------------------------------------------------------------------------------

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

bool PRINT(test_non_negative_sensing)
    VTi_mL   = -1.f;
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
    TEST_ASSUME(compute_constant_motor_steps(1000, UINT16_MAX)==MOTOR_MAX);
    uint32_t last_step = compute_motor_steps_and_Tinsu_ms(1.5f, 230.f);
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
