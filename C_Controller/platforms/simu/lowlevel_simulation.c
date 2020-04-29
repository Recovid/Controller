#include "common.h"
#include "controller.h"
#ifndef WIN32
//FreeRTOS include
#include <FreeRTOS.h>
#include <task.h>
#endif

//STD include
#include <time.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

//Low-level include
#ifndef WIN32
#include "portmacro.h"
#endif

// ------------------------------------------------------------------------------------------------
//! OS simulation


//! Simulated clock for testing purposes
static uint32_t clock_ms = 0; // will not overflow before 24 simulated days

#ifdef WIN32
uint32_t get_time_ms()
{
    return clock_ms;
}

uint32_t wait_ms(uint32_t t_ms)
{
    return clock_ms += t_ms; // simulated clock for testing purposes
}
#endif

bool soft_reset()
{
    return true;
}

// ------------------------------------------------------------------------------------------------
//! IHM simulation based on stdin/stdout or Serial

FILE *in ;
FILE *out;

bool init_uart()
{
}

bool send_ihm(const char* frame)
{
}

int recv_ihm()
{
    return EOF;
}

// ------------------------------------------------------------------------------------------------
//! HW actuators

bool init_motor() { return true; }

void motor_enable(bool ena) {
}

static uint16_t motor_pos = 0;
static float    motor_speed_stepspms = 0.f;
static uint32_t motor_move_from_t_ms = 0;

//Not Yet implemented
uint16_t motor_position(float Vol_mL)
{
    //return ((uint16_t)((float)MOTOR_MAX) * (Vol_mL / BAVU_V_ML_MAX)); // TODO use a home + sin motor_position()
    return UINT16_MAX; // TODO use a home + sin motor_position()
}


//Not Yet implemented
float motor_volume_mL(float pos) // motor_Q_needs more precision than integer steps
{
    return UINT16_MAX; // TODO use inverse of motor_position()
}

//Not Yet implemented
float motor_speed_stepspms_at(uint16_t position, float VM_Lpm)
{
    UNUSED(position); // TODO use motor_position()
    // TODO simulate VM_Lpm limiting ?
    //return ((float)MOTOR_MAX/*steps*/) / (BAVU_V_ML_MAX / (VM_Lpm/60.f/*mLpms*/));
    return UINT16_MAX;
}

//Not yet implemented
float motor_Q_Lpm()
{
    //float V0 = motor_volume_mL(motor_pos);
    //float Vt = 0.f;
    //if (motor_speed_stepspms > .0f) {
    //    if (((uint16_t)motor_speed_stepspms) < (MOTOR_MAX-motor_pos)) {
    //        Vt = motor_volume_mL(((float)motor_pos) + motor_speed_stepspms);
    //    }
    //    else {
    //        Vt = motor_volume_mL(MOTOR_MAX); // max pos
    //    }
    //}
    //else {
    //    if ((0+motor_pos) > ((uint16_t)motor_speed_stepspms)) {
    //        Vt = motor_volume_mL(((float)motor_pos) - motor_speed_stepspms);
    //    }
    //    else {
    //        Vt = motor_volume_mL(0); // min pos
    //    }
    //}
    //return ((Vt-V0/*ml*/)/*/ms*/)*60.f/*mL/ms*/;
    return -1;
}


//Not yet implemented
void motor_move()
{
    //if (motor_move_from_t_ms) {
    //    uint16_t last_motor_pos = motor_pos;
    //    motor_pos = MAX(0, MIN(MOTOR_MAX,
    //        motor_pos+(motor_speed_stepspms*(get_time_ms()-motor_move_from_t_ms)))); // TODO simulate lost steps in range

    //    if (motor_pos!=last_motor_pos) {
    //        switch (motor_pos) {
    //        case 0*MOTOR_MAX/4: DEBUG_PRINT("  M: |"    ); break;
    //        case 1*MOTOR_MAX/4: DEBUG_PRINT("  M: |-"   ); break;
    //        case 2*MOTOR_MAX/4: DEBUG_PRINT("  M: |-:"  ); break;
    //        case 3*MOTOR_MAX/4: DEBUG_PRINT("  M: |-:-" ); break;
    //        case 4*MOTOR_MAX/4: DEBUG_PRINT("  M: |-:-|"); break;
    //        }
    //    }
    //}
}

//Not yet implemented
bool motor_press(uint16_t* steps_profile_us, uint16_t nb_steps)
{
    return false; // TODO
}

bool motor_press_speed(float speed)
{
    motor_move();
    motor_move_from_t_ms = get_time_ms();
    motor_speed_stepspms = speed;
    return true; // TODO simulate driver failure
}

bool motor_stop()
{
    motor_move();
    motor_move_from_t_ms = 0;
    motor_speed_stepspms = 0.f;
    return true; // TODO simulate driver failure
}

bool motor_release()
{
    motor_move();
    motor_move_from_t_ms = get_time_ms();
    motor_speed_stepspms = -MAX(20, ((float)motor_pos) / (400-motor_move_from_t_ms)); // TODO simulate moving away from home part of motor_position(Vol_mL)
    return true; // TODO simulate driver failure
}

// ------------------------------------------------------------------------------------------------

bool init_motor_pep() { return true; }

bool motor_pep_move(int relative_mm)
{
    return false; // TODO
}
bool is_motor_pep_ok() {
  return true;
}


bool motor_pep_home() { return true; }


//TODO Not yet implemented
bool is_motor_pep_home() { return true; }

//TODO Not yet implemented
bool is_motor_pep_moving() { return true; }

// ------------------------------------------------------------------------------------------------

bool init_valve() { return true; }

static enum Valve { Inhale, Exhale } valve_state = Exhale;
static uint32_t valve_exhale_ms = 0;

bool valve_exhale()
{
    if (valve_state == Exhale) return true;

    valve_state = Exhale;
    valve_exhale_ms = get_time_ms();
    return true;
}

bool valve_inhale()
{
    valve_state = Inhale;
    valve_exhale_ms = -1;
    return true;
}

// ------------------------------------------------------------------------------------------------

//! Usable BAVU volume based on motor position
//TODO Not yet implemented
float BAVU_V_mL()
{
	return -1;
    //return BAVU_V_ML_MAX * motor_pos / MOTOR_MAX; // TODO simulate BAVU perforation
}

//! Usable BAVU flow based on motor position and direction
//! \remark a valve normally ensures that Q is always positive
//TODO Not yet implemented
float BAVU_Q_Lpm()
{
	return -1;
    //const float Q_Lpm = MIN(BAVU_Q_LPM_MAX, motor_Q_Lpm()); // TODO simulate BAVU perforation
    //return Q_Lpm * (motor_speed_stepspms > 0.f ? 1. : BAVU_VALVE_RATIO); // TODO simulate BAVU valve leak
}

// ------------------------------------------------------------------------------------------------
//! HW sensors simulation

static float saved_VTi_mL;

bool init_Pdiff   () { return true; }
bool init_Paw     () { return true; }
bool init_Patmo   () { return true; }
bool sensors_start() { return true; }
bool init_sensors()  { return true;}
//TODO Not yet implemented
float read_Vol_mL()	 { return -1;}

//TODO Not yet implemented
void reset_Vol_mL()  { }

//TODO Not yet implemented
float read_Pdiff_Lpm()
{
//    if (valve_state == Inhale) {
////#ifdef NTESTS
//        saved_VTi_mL = 0;
////#endif
//        return BAVU_Q_Lpm() * EXHAL_VALVE_RATIO;
//    }
//    else if (valve_state == Exhale) {
////#ifdef NTESTS
//        if (!saved_VTi_mL) {
//            saved_VTi_mL = get_sensed_VTi_mL();
//        }
////#endif
//        return valve_exhale_ms+LUNG_EXHALE_MS>get_time_ms() ?
//            -(60.f*saved_VTi_mL/LUNG_EXHALE_MS)*(valve_exhale_ms+LUNG_EXHALE_MS-get_time_ms())/LUNG_EXHALE_MS*2 :
//            0.f; // 0 after LUNG_EXHALE_MS and VTe=-VTi
//    }
//    else {
//        return 0.;
//    }
	return -1;
}

// ------------------------------------------------------------------------------------------------

static float Paw_cmH2O = 10; // to handle exponential decrease during plateau and exhalation
static float last_Paw_change = 0.f;
static uint32_t decrease_Paw_ms = 0;

//TODO Not yet implemented
float read_Paw_cmH2O()
{
    //const float Paw_cmH2O =
    //    get_setting_PEP_cmH2O() // TODO loop back with get_sensed_PEP_cmH2O()
    //    + (valve_state==Exhale ? 0.f : (BAVU_V_ML_MAX - BAVU_V_mL()) / LUNG_COMPLIANCE)
    //    + fabsf(read_Pdiff_Lpm()) * AIRWAYS_RESISTANCE;
    //assert(Paw_cmH2O >= 0);
    //return Paw_cmH2O;
	return -1;
}

// ------------------------------------------------------------------------------------------------

//TODO Not yet implemented
float read_Patmo_mbar()
{
    //return 1013. + sinf(2*M_PI*get_time_ms()/1000/60) * PATMO_VARIATION_MBAR; // TODO test failure
	return -1;
}

// ------------------------------------------------------------------------------------------------

int read_Battery_level()
{
    return 2; // TODO simulate lower battery levels
}

// ------------------------------------------------------------------------------------------------

float    samples_Q_Lps[2000];
float    average_Q_Lps[2000];

static float    samples_Q_t_ms  = 0.f;
static uint16_t samples_Q_index = 0;
static bool     sampling_Q      = false;

bool sensors_start_sampling_flow()
{
    samples_Q_t_ms = 0.f;
    samples_Q_index = 0;
    sampling_Q = true;
    return sampling_Q;
}

bool sensors_stop_sampling_flow()
{
    sampling_Q = false;
    return !sampling_Q;
}

//TODO Not yet implemented
bool sensors_sample_flow()
{
    //if (!sampling_Q) return false;

    //for (uint16_t i=0 ; i<COUNT_OF(samples_Q_Lps) && i<COUNT_OF(inf_C_samples_Q_Lps) ; i++) {
    //    samples_Q_Lps[i] = inf_C_samples_Q_Lps[i];
    //    samples_Q_t_ms  += SAMPLES_T_US;
    //    samples_Q_index ++;
    //}
    //return true;
	return false;
}

//TODO Not yet implemented
bool sensors_sample_flow_low_C()
{
    //if (!sampling_Q) return false;

    //for (uint16_t i=0 ; i<COUNT_OF(samples_Q_Lps) && i<COUNT_OF(low_C_samples_Q_Lps) ; i++) {
    //    samples_Q_Lps[i] = low_C_samples_Q_Lps[i];
    //    samples_Q_t_ms  += SAMPLES_T_US;
    //    samples_Q_index ++;
    //}
    //return true;
	return false;
}

float sensors_samples_time_s()
{
    return get_setting_Tinsu_ms();
}

uint16_t get_samples_Q_index_size()
{
    return samples_Q_index;
}

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

//TODO Not yet implemented
bool lung_at_rest()
{
    //TEST_ASSUME(valve_exhale());
    //wait_ms(LUNG_EXHALE_MS_MAX);
    //return true;
	return false;
}

bool motor_at_home()
{
    TEST_ASSUME(motor_release());
    wait_ms(100);
    TEST_ASSUME(motor_stop());
    return true;
}

bool PRINT(test_insufflate)
    //for (float start = 120.f; start >= 0.f ; start -= 20.f) { // all range of insufflation flow
    //    for (uint32_t poll_ms=100; poll_ms >= 1 ; poll_ms/=10) { // insufflation flow is independant from polling rate
    //        TEST_ASSUME(lung_at_rest ());
    //        TEST_ASSUME(valve_inhale ());
    //        TEST_ASSUME(motor_at_home());
    //        for (uint32_t t_ms=0; t_ms<=100; t_ms+=poll_ms) {
    //            TEST_ASSUME(motor_press_speed(start));
    //            wait_ms(poll_ms);
    //            const float VMt = read_Pdiff_Lpm();
    //            const float Paw = read_Paw_cmH2O();
    //            const float PEP = get_sensed_PEP_cmH2O();
    //            const float VTi = (BAVU_V_ML_MAX-BAVU_V_mL());
    //            if (!(TEST_FLT_EQUALS(start, VMt))) return false; // unexpected VM
    //            if (!(TEST_RANGE(PEP+VTi/LUNG_COMPLIANCE_MAX, Paw,
    //                             PEP+VTi/LUNG_COMPLIANCE    + fabsf(VMt)*AIRWAYS_RESISTANCE_MAX))) return false; // unexpected Paw
    //        }
    //    }
    //}
    //return true;
    return false;
}

//TODO Not yet implemented
bool PRINT(test_plateau)
    //for (uint32_t start = 0; start <= LUNG_EXHALE_MS*2 ; start += 500) { // plateau is independant from motor_stop/release
    //    TEST_ASSUME(lung_at_rest());
    //    TEST_ASSUME(start ? motor_release() : motor_stop());
    //    TEST_ASSUME(valve_inhale());
    //    for (int t=0; t<=3 ; t++) {
    //        wait_ms(LUNG_EXHALE_MS/2);
    //        const float VMt = read_Pdiff_Lpm();
    //        const float Paw = read_Paw_cmH2O();
    //        const float PEP = get_sensed_PEP_cmH2O();
    //        const float VTi = (BAVU_V_ML_MAX-BAVU_V_mL());
    //        if (!(TEST_FLT_EQUALS(0.f, VMt))) return false; // unexpected flow
    //        if (!(TEST_RANGE(PEP+VTi/LUNG_COMPLIANCE_MAX, Paw,
    //                         PEP+VTi/LUNG_COMPLIANCE    + fabsf(VMt)*AIRWAYS_RESISTANCE_MAX))) return false; // unexpected Paw
    //    }
    //}
    //return true;
    return false;
}

//TODO Not yet implemented
bool PRINT(test_exhale)
    //for (float start = -600.f; start <= 0.f ; start += 100.f) { // decrease time is independant from VTi
    //    for (uint32_t poll_ms=100; poll_ms >= 1 ; poll_ms/=10) { // decrease is independant from polling rate
    //        TEST_ASSUME(valve_inhale());
    //        saved_VTi_mL = fabsf(start);
    //        TEST_ASSUME(valve_exhale());
    //        float VTe_mL = 0;
    //        for (uint32_t t_ms=0; t_ms<LUNG_EXHALE_MS_MAX; t_ms+=poll_ms) { // VTe takes less than 0.6s
    //            const float VM0 = read_Pdiff_Lpm();
    //            wait_ms(poll_ms);
    //            const float VMt = read_Pdiff_Lpm();
    //            const float Paw = read_Paw_cmH2O();
    //            const float PEP = get_sensed_PEP_cmH2O();
    //            if (!(TEST_RANGE(PEP, Paw,
    //                             PEP+ fabsf(VMt)*AIRWAYS_RESISTANCE_MAX))) {
    //                return false; // unexpected Paw
    //            }
    //            const float VM = (VMt+VM0)/2.f;
    //            VTe_mL += VM/60.f/*Lps*/ * poll_ms;
    //        }
    //        float last_VM = read_Pdiff_Lpm();
    //        if (!(TEST_FLT_EQUALS(0.f, last_VM) &&
    //              TEST_FLT_EQUALS(-saved_VTi_mL, VTe_mL))) {
    //            return false;
    //        }
    //    }
    //}
    //return true;
	return false;
}

//TODO Not yet implemented
bool PRINT(test_Patmo_over_time)
    //float last_Patmo = 0.f;
    //for (uint32_t t_s=0; t_s < 60*60 ; t_s=wait_ms(60*1000/8)/1000) {
    //    if (!(TEST_RANGE(1013-PATMO_VARIATION_MBAR, read_Patmo_mbar(), 1013+PATMO_VARIATION_MBAR) &&
    //          TEST(last_Patmo != read_Patmo_mbar())))
    //        return false;
    //}
    //return true;
	return false;
}

bool flow_samples()
{
    TEST_ASSUME(sensors_start_sampling_flow());
    TEST_ASSUME(sensors_sample_flow());
    TEST_ASSUME(sensors_stop_sampling_flow());
    return true;
}

//TODO Not yet implemented
bool PRINT(test_compute_samples_average_and_latency_us)
    //TEST_ASSUME(flow_samples());
    //uint32_t latency_us = compute_samples_average_and_latency_us();
    //return TEST_EQUALS(10*SAMPLES_T_US, latency_us)
    //    && TEST_FLT_EQUALS(2.7f, samples_Q_Lps[171]);
	return false;
}

//TODO Not yet implemented
bool PRINT(test_compute_motor_steps_and_Tinsu_ms)
    //TEST_ASSUME(flow_samples());
    //uint32_t last_step = compute_motor_steps_and_Tinsu_ms(1.5f, 230.f);
    //return TEST_EQUALS(309, last_step); // TODO Check with more accurate calibration
	return true;
}

bool PRINT(TEST_LOWLEVEL_SIMULATION)
    return
        test_compute_samples_average_and_latency_us() &&
        test_compute_motor_steps_and_Tinsu_ms() &&
        test_insufflate() &&
        test_plateau() &&
        test_exhale() &&
        test_Patmo_over_time() &&
        true;
}


#endif

void HardFault_Handler(void)
{
	while(1);
}

bool init_hardware()
{
	return true;
}
