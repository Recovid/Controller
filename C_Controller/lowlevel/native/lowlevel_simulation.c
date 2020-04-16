#include "lowlevel/include/lowlevel.h"

#ifndef WIN32
//FreeRTOS include
#include <FreeRTOS.h>
#include <task.h>
#endif

//STD include
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

//Recovid include
#include "sensing.h"
#include "configuration.h"
#include "ihm_communication.h"

//Low-level include
#ifndef WIN32
#include "hardware_serial.h"
#include "portmacro.h"
#endif

// ------------------------------------------------------------------------------------------------
//! OS simulation

ihm_mode_t current_ihm_mode = IHM_MODE_MAX;

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

bool init_ihm(ihm_mode_t ihm_mode, const char* pathInputFile, const char* pathOutputFile)
{
    if (ihm_mode >= IHM_MODE_MAX)
    {
        printf("Wrong ihm mode \n");
        return false;
    }
    else if (ihm_mode == IHM_MODE_FILE)
    {
        printf("Serial oppened in File Mode \n");
        // TODO Replace with HAL_UART_init, no connection per se
        if (pathInputFile)
            in  = fopen(pathInputFile, "r");
        else
            in = stdin;
        if (pathOutputFile)
            out = fopen(pathOutputFile, "w");
        else
            out = stdout;
    }
#ifndef WIN32
    else
    {
        printf("Serial opened in Serial Mode \n");
        if(!hardware_serial_init(pathInputFile))
        {
            return false;
        }
    }
#endif
    current_ihm_mode = ihm_mode;

    return true;
}

bool send_ihm(const char* frame)
{
    bool is_data_send = false;

    if (!frame || *frame=='\0') return 0;

    if(current_ihm_mode == IHM_MODE_FILE)
    {
        is_data_send = fputs(frame, out) >= 0;
    }
#ifndef WIN32
    else
    {
        is_data_send = hardware_serial_write_data(frame, strlen(frame));
    }
#endif
    return is_data_send;
}

int recv_ihm()
{
    static time_t last_blocked_s = 0;

    time_t t_s;
    time(&t_s);

    if(current_ihm_mode == IHM_MODE_FILE)
    {
        if (last_blocked_s+5 < t_s) {
            int blocking_read = fgetc(in);
            if (blocking_read == '\n') {
                last_blocked_s = t_s;
            }
            return blocking_read;
        }
    }
#ifndef WIN32
    else
    {
        if (last_blocked_s+5 < t_s) {
            char blocking_read = 0;
            hardware_serial_read_data(&blocking_read, sizeof(char));
            if (blocking_read == '\n') {
                last_blocked_s = t_s;
            }
            return blocking_read;
        }
    }
#endif
    return EOF;
}

// ------------------------------------------------------------------------------------------------
//! HW actuators

static uint16_t motor_pos = 0;
static float    motor_speed_stepspms = 0.f;
static uint32_t motor_move_from_t_ms = 0;

uint16_t motor_position(float Vol_mL)
{
    return ((uint16_t)((float)MOTOR_MAX) * (Vol_mL / BAVU_V_ML_MAX)); // TODO use a home + sin motor_position()
}

float motor_volume_mL(float pos) // motor_Q_needs more precision than integer steps
{
    return BAVU_V_ML_MAX * (pos / ((float)MOTOR_MAX)); // TODO use inverse of motor_position()
}

float motor_speed_stepspms_at(uint16_t position, float VM_Lpm)
{
    UNUSED(position) // TODO use motor_position()
    // TODO simulate VM_Lpm limiting ?
    return ((float)MOTOR_MAX/*steps*/) / (BAVU_V_ML_MAX / (VM_Lpm/60.f/*mLpms*/));
}

float motor_Q_Lpm()
{
    float V0 = motor_volume_mL(motor_pos);
    float Vt = 0.f;
    if (motor_speed_stepspms > .0f) {
        if (((uint16_t)motor_speed_stepspms) < (MOTOR_MAX-motor_pos)) {
            Vt = motor_volume_mL(((float)motor_pos) + motor_speed_stepspms);
        }
        else {
            Vt = motor_volume_mL(MOTOR_MAX); // max pos
        }
    }
    else {
        if ((0+motor_pos) > ((uint16_t)motor_speed_stepspms)) {
            Vt = motor_volume_mL(((float)motor_pos) - motor_speed_stepspms);
        }
        else {
            Vt = motor_volume_mL(0); // min pos
        }
    }
    return ((Vt-V0/*ml*/)/*/ms*/)*60.f/*mL/ms*/;
}

void motor_move()
{
    if (motor_move_from_t_ms) {
        uint16_t last_motor_pos = motor_pos;
        motor_pos = MAX(0, MIN(MOTOR_MAX,
            motor_pos+(motor_speed_stepspms*(get_time_ms()-motor_move_from_t_ms)))); // TODO simulate lost steps in range

        if (motor_pos!=last_motor_pos) {
            switch (motor_pos) {
            case 0*MOTOR_MAX/4: DEBUG_PRINT("  M: |"    ); break;
            case 1*MOTOR_MAX/4: DEBUG_PRINT("  M: |-"   ); break;
            case 2*MOTOR_MAX/4: DEBUG_PRINT("  M: |-:"  ); break;
            case 3*MOTOR_MAX/4: DEBUG_PRINT("  M: |-:-" ); break;
            case 4*MOTOR_MAX/4: DEBUG_PRINT("  M: |-:-|"); break;
            }
        }
    }
}

bool motor_press(float VM_Lpm)
{
    motor_move();
    motor_move_from_t_ms = get_time_ms();
    motor_speed_stepspms = motor_speed_stepspms_at(motor_pos, VM_Lpm);
    return true; // TODO simulate driver failure
}

bool motor_stop()
{
    motor_move();
    motor_move_from_t_ms = 0;
    motor_speed_stepspms = 0.f;
    return true; // TODO simulate driver failure
}

bool motor_release(uint32_t before_t_ms)
{
    UNUSED(before_t_ms)
    motor_move();
    motor_move_from_t_ms = get_time_ms();
    motor_speed_stepspms = -MAX(20, ((float)motor_pos) / before_t_ms-motor_move_from_t_ms); // TODO simulate moving away from home part of motor_position(Vol_mL)
    return true; // TODO simulate driver failure
}

// ------------------------------------------------------------------------------------------------

bool motor_pep_move(float relative_move_cmH2O)
{
    return false; // TODO
}

// ------------------------------------------------------------------------------------------------

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
float BAVU_V_mL()
{
    return BAVU_V_ML_MAX * motor_pos / MOTOR_MAX; // TODO simulate BAVU perforation
}

//! Usable BAVU flow based on motor position and direction
//! \remark a valve normally ensures that Q is always positive
float BAVU_Q_Lpm()
{
    const float Q_Lpm = MIN(BAVU_Q_LPM_MAX, motor_Q_Lpm()); // TODO simulate BAVU perforation
    return Q_Lpm * (motor_speed_stepspms > 0.f ? 1. : BAVU_VALVE_RATIO); // TODO simulate BAVU valve leak
}

// ------------------------------------------------------------------------------------------------
//! HW sensors simulation

static float saved_VTi_mL;

float read_Pdiff_Lpm()
{
    if (valve_state == Inhale) {
//#ifdef NTESTS
        saved_VTi_mL = 0;
//#endif
        return BAVU_Q_Lpm() * EXHAL_VALVE_RATIO;
    }
    else if (valve_state == Exhale) {
//#ifdef NTESTS
        if (!saved_VTi_mL) {
            saved_VTi_mL = get_sensed_VTi_mL();
        }
//#endif
        return valve_exhale_ms+LUNG_EXHALE_MS>get_time_ms() ?
            -(60.f*saved_VTi_mL/LUNG_EXHALE_MS)*(valve_exhale_ms+LUNG_EXHALE_MS-get_time_ms())/LUNG_EXHALE_MS*2 :
            0.f; // 0 after LUNG_EXHALE_MS and VTe=-VTi
    }
    else {
        return 0.;
    }
}

// ------------------------------------------------------------------------------------------------

static float Paw_cmH2O = 10; // to handle exponential decrease during plateau and exhalation
static float last_Paw_change = 0.f;
static uint32_t decrease_Paw_ms = 0;

float read_Paw_cmH2O()
{
    const float Paw_cmH2O =
        get_sensed_PEP_cmH2O()
        + (valve_state==Exhale ? 0.f : (BAVU_V_ML_MAX - BAVU_V_mL()) / LUNG_COMPLIANCE)
        + fabsf(read_Pdiff_Lpm()) * AIRWAYS_RESISTANCE;
    assert(Paw_cmH2O >= 0);
    return Paw_cmH2O;
}

// ------------------------------------------------------------------------------------------------

float read_Patmo_mbar()
{
    return 1013. + sinf(2*M_PI*get_time_ms()/1000/60) * PATMO_VARIATION_MBAR; // TODO test failure
}

// ------------------------------------------------------------------------------------------------

int read_Battery_level()
{
    return 2; // TODO simulate lower battery levels
}

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

bool lung_at_rest()
{
    TEST_ASSUME(valve_exhale());
    wait_ms(LUNG_EXHALE_MS_MAX);
    return true;
}
bool motor_at_home()
{
    TEST_ASSUME(motor_release(get_time_ms()+250));
    wait_ms(100);
    TEST_ASSUME(motor_stop());
    return true;
}

bool PRINT(test_insufflate)
    for (float start = 120.f; start >= 0.f ; start -= 20.f) { // all range of insufflation flow
        for (uint32_t poll_ms=100; poll_ms >= 1 ; poll_ms/=10) { // insufflation flow is independant from polling rate
            TEST_ASSUME(lung_at_rest ());
            TEST_ASSUME(valve_inhale ());
            TEST_ASSUME(motor_at_home());
            for (uint32_t t_ms=0; t_ms<=100; t_ms+=poll_ms) {
                TEST_ASSUME(motor_press(start));
                wait_ms(poll_ms);
                const float VMt = read_Pdiff_Lpm();
                const float Paw = read_Paw_cmH2O();
                const float PEP = get_sensed_PEP_cmH2O();
                const float VTi = (BAVU_V_ML_MAX-BAVU_V_mL());
                if (!(TEST_FLT_EQUALS(start, VMt))) return false; // unexpected VM
                if (!(TEST_RANGE(PEP+VTi/LUNG_COMPLIANCE_MAX, Paw,
                                 PEP+VTi/LUNG_COMPLIANCE    + fabsf(VMt)*AIRWAYS_RESISTANCE_MAX))) return false; // unexpected Paw
            }
        }
    }
    return true;
}

bool PRINT(test_plateau)
    for (uint32_t start = 0; start <= LUNG_EXHALE_MS*2 ; start += 500) { // plateau is independant from motor_stop/release
        TEST_ASSUME(lung_at_rest());
        TEST_ASSUME(start ? motor_release(get_time_ms()+start) : motor_stop());
        TEST_ASSUME(valve_inhale());
        for (int t=0; t<=3 ; t++) {
            wait_ms(LUNG_EXHALE_MS/2);
            const float VMt = read_Pdiff_Lpm();
            const float Paw = read_Paw_cmH2O();
            const float PEP = get_sensed_PEP_cmH2O();
            const float VTi = (BAVU_V_ML_MAX-BAVU_V_mL());
            if (!(TEST_FLT_EQUALS(0.f, VMt))) return false; // unexpected flow
            if (!(TEST_RANGE(PEP+VTi/LUNG_COMPLIANCE_MAX, Paw,
                             PEP+VTi/LUNG_COMPLIANCE    + fabsf(VMt)*AIRWAYS_RESISTANCE_MAX))) return false; // unexpected Paw
        }
    }
    return true;
}

bool PRINT(test_exhale)
    for (float start = -600.f; start <= 0.f ; start += 100.f) { // decrease time is independant from VTi
        for (uint32_t poll_ms=100; poll_ms >= 1 ; poll_ms/=10) { // decrease is independant from polling rate
            TEST_ASSUME(valve_inhale());
            saved_VTi_mL = fabsf(start);
            TEST_ASSUME(valve_exhale());
            float VTe_mL = 0;
            for (uint32_t t_ms=0; t_ms<LUNG_EXHALE_MS_MAX; t_ms+=poll_ms) { // VTe takes less than 0.6s
                const float VM0 = read_Pdiff_Lpm();
                wait_ms(poll_ms);
                const float VMt = read_Pdiff_Lpm();
                const float Paw = read_Paw_cmH2O();
                const float PEP = get_sensed_PEP_cmH2O();
                if (!(TEST_RANGE(PEP, Paw,
                                 PEP+ fabsf(VMt)*AIRWAYS_RESISTANCE_MAX))) {
                    return false; // unexpected Paw
                }
                const float VM = (VMt+VM0)/2.f;
                VTe_mL += VM/60.f/*Lps*/ * poll_ms;
            }
            float last_VM = read_Pdiff_Lpm();
            if (!(TEST_FLT_EQUALS(0.f, last_VM) &&
                  TEST_FLT_EQUALS(-saved_VTi_mL, VTe_mL))) {
                return false;
            }
        }
    }
    return true;
}

bool PRINT(test_Patmo_over_time)
    float last_Patmo = 0.f;
    for (uint32_t t_s=0; t_s < 60*60 ; t_s=wait_ms(60*1000/8)/1000) {
        if (!(TEST_RANGE(1013-PATMO_VARIATION_MBAR, read_Patmo_mbar(), 1013+PATMO_VARIATION_MBAR) &&
              TEST(last_Patmo != read_Patmo_mbar())))
            return false;
    }
    return true;
}

bool PRINT(TEST_LOWLEVEL_SIMULATION)
    return
        test_insufflate() &&
        test_plateau() &&
        test_exhale() &&
        test_Patmo_over_time() &&
        true;
}

#endif
