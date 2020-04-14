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

static int motor_pos = 0;
static int motor_dir = 0;
static long motor_release_ms = -1;

bool motor_press(float VM_Lpm)
{
    UNUSED(VM_Lpm)

    motor_release_ms = -1;
    motor_dir = 1; // TODO simulate Vmax_Lpm limiting by determining the approriate speed/steps
    motor_pos = MIN(MOTOR_MAX, motor_pos+motor_dir); // TODO simulate lost steps in range
    //if (motor_pos/0xF) {
    //    DEBUG_PRINTF("motor %X\n", motor_pos);
    //}
    return true; // TODO simulate driver failure
}

bool motor_stop()
{
    motor_release_ms = -1;
    motor_dir = 0;
    return true; // TODO simulate driver failure
}

bool motor_release(uint32_t before_t_ms)
{
    UNUSED(before_t_ms)

    motor_release_ms = get_time_ms();
    motor_dir = -1;
    motor_pos = MAX(0, motor_pos+motor_dir); // TODO simulate lost steps in range
    //if (motor_pos/0xF) {
    //    DEBUG_PRINTF("motor %X\n", motor_pos);
    //}
    return true; // TODO simulate driver failure
}

bool motor_pep_move(float relative_move_cmH2O)
{
    return false; // TODO
}


static enum Valve { Inhale, Exhale } valve_state = Exhale;
static long valve_exhale_ms = -1;

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

//! Usable BAVU volume based on motor position
//! \remark BAVU deformation/elasticity is simulated with a cos to easily derive Q
float BAVU_V_mL()
{
    return cosf(M_PI_2*(((float)motor_pos) /MOTOR_MAX)) * BAVU_V_ML_MAX; // TODO simulate BAVU perforation
}

//! Usable BAVU flow based on motor position and direction
//! \remark a valve normally ensures that Q is always positive
float BAVU_Q_Lpm()
{
    float piover2 = M_PI_2;
    float ratio_motor = ((float)(motor_pos)/MOTOR_MAX);
    float sinus = sinf(piover2 * ratio_motor);
    const float Q_Lpm = sinus * BAVU_Q_LPM_MAX; // TODO simulate BAVU perforation
    return Q_Lpm * (motor_dir > 0 ? 1. : BAVU_VALVE_RATIO);
}

// ------------------------------------------------------------------------------------------------
//! HW sensors simulation

static float abs_Q_Lpm = 10; // to handle exponential decrease during exhalation
static float nonzero_abs_Q_Lpm;

static float last_Pdiff_change = 0.f;
static uint32_t decrease_Pdiff_ms = 0; // to handle exponential decrease during exhalation independant from polling rate

float read_Pdiff_Lpm()
{
    if (valve_state == Inhale) {
        abs_Q_Lpm = BAVU_Q_Lpm() * EXHAL_VALVE_RATIO;
        if(abs_Q_Lpm != 0.) {
            nonzero_abs_Q_Lpm = abs_Q_Lpm;
        }
        return abs_Q_Lpm;
    }
    else if (valve_state == Exhale) {

        const float decrease = .99; // expf(- abs(get_time_ms()-valve_exhale_ms)/100.); // <1% after 500ms @ 20 FPS
        nonzero_abs_Q_Lpm *= decrease;
        return -nonzero_abs_Q_Lpm;
    }
    else {
        return 0.;
    }
}


static float Paw_cmH2O = 10; // to handle exponential decrease during plateau and exhalation
static float last_Paw_change = 0.f;
static uint32_t decrease_Paw_ms = 0;

float read_Paw_cmH2O()
{
    const float PEP_cmH2O = get_sensed_PEP_cmH2O();
    if (valve_state == Inhale) {
        if (motor_dir > 0) {
            // Pressure augments as volume decreases according to PV=k ('loi des gaz parfait')
            // Pi=P0*V0/Vi
            Paw_cmH2O = PEP_cmH2O + (4 * (BAVU_V_ML_MAX + LUNG_V_ML_MAX)/(BAVU_V_mL() + LUNG_V_ML_MAX));
        }
        else {
            // Pressure exp. decreases due to lung compliance (volume augmentation) which depends on patient (and condition)
            const float decrease = .6; // expf(- abs(get_time_ms()-motor_release_ms)/10.); // <1% after 50ms
            const float Pplat_cmH2O = PEP_cmH2O + (BAVU_V_ML_MAX - BAVU_V_mL()) / LUNG_COMPLIANCE;
            Paw_cmH2O = Pplat_cmH2O + (Paw_cmH2O-Pplat_cmH2O) * decrease;
        }
    }
    else if (valve_state == Exhale) {
        const float decrease = .9; // abs(get_time_ms()-valve_exhale_ms)/100.; // <1% after 500ms
        Paw_cmH2O = PEP_cmH2O + (Paw_cmH2O-PEP_cmH2O) * decrease;
    }
    assert(Paw_cmH2O >= 0);
    return Paw_cmH2O;
}

float read_Patmo_mbar()
{
    return 1013. + sinf(2*M_PI*get_time_ms()/1000/60) * PATMO_VARIATION_MBAR; // TODO test failure
}

int read_Battery_level()
{
    return 2; // TODO simulate lower battery levels
}

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

bool PRINT(test_Pdiff_exhale_stable)
    if (!TEST(valve_inhale())) return false; // HW failure
    abs_Q_Lpm = 0;
    if (!TEST(valve_exhale())) return false; // HW failure
    for (uint32_t t_ms=get_time_ms(); t_ms < 3000 ; t_ms=wait_ms(10)) {
        if (!(TEST_FLT_EQUALS(0.f, read_Pdiff_Lpm()))) {
            return false; // unexpected variation
        }
    }
    return true;
}

bool PRINT(test_Paw_exhale_stable)
    if (!TEST(valve_inhale())) return false; // HW failure
    abs_Q_Lpm = 0;
    if (!TEST(valve_exhale())) return false; // HW failure
    for (uint32_t t_ms=get_time_ms(); t_ms < 3000 ; t_ms=wait_ms(10)) {
        if (!(TEST_FLT_EQUALS(0.f, read_Paw_cmH2O()))) {
            return false; // unexpected variation
        }
    }
    return true;
}

bool PRINT(test_Pdiff_exhale_decrease)
    for (float start = -100.f; start <= 100.f ; start += 40.f) { // magnitude decrease is independant from start
        for (uint32_t poll_ms=1; poll_ms < 20 ; poll_ms+=1) { // magnitude decrease is independant from polling rate
            if (!TEST(valve_inhale())) return false; // HW failure
            nonzero_abs_Q_Lpm = abs_Q_Lpm = fabsf(start);
            if (!TEST(valve_exhale())) return false; // HW failure

            float last = read_Pdiff_Lpm();
            if (!(TEST_RANGE(-fabsf(start), last, 1.f-fabsf(start)))) return false;

            // TODO More meaningful test related to RCM-SW to be defined
            wait_ms(1000);
            last = read_Pdiff_Lpm();
            if (!(TEST_FLT_EQUALS(0.f, last))) {
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
        test_Pdiff_exhale_stable() &&
        test_Pdiff_exhale_decrease() &&
        test_Patmo_over_time() &&
        true;
}

#endif

