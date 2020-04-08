//FreeRTOS include
#include <FreeRTOS.h>
#include <task.h>

//STD include
#include <time.h>
#include <stdio.h>
#include <math.h>

//Recovid include
#include "hardware_simulation.h"
#include "configuration.h"
#include "controller_settings.h"
#include "portmacro.h"

// ------------------------------------------------------------------------------------------------
//! OS simulation

bool soft_reset()
{
    return true;
}

// ------------------------------------------------------------------------------------------------
//! IHM simulation based on stdin/stdout

FILE *in ;
FILE *out;

bool init_ihm(const char* pathInputFile, const char* pathOutputFile)
{
	printf("INIT IHM");
    // TODO Replace with HAL_UART_init, no connection per se
    if (pathInputFile)
        in  = fopen(pathInputFile, "r");
    else
        in = stdin;
    if (pathOutputFile)
        out = fopen(pathOutputFile, "w");
    else
        out = stdout;
    return true;
}

bool send_ihm(const char* frame)
{
    if (!frame || *frame=='\0') return 0;
    return fputs(frame, out) >= 0;
}

int recv_ihm()
{
    static time_t last_blocked_s = 0;

    time_t t_s;
    time(&t_s);
    if (last_blocked_s+5 < t_s) {
        int blocking_read = fgetc(in);
        if (blocking_read == '\n') {
            last_blocked_s = t_s;
        }
        return blocking_read;
    }
    return EOF;
}

// ------------------------------------------------------------------------------------------------
//! HW actuators

static int motor_pos = 0;
static int motor_dir = 0;
static long motor_release_ms = -1;

bool motor_press()
{
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

bool motor_release()
{
    motor_release_ms = get_time_ms();
    motor_dir = -1;
    motor_pos = MAX(0, motor_pos+motor_dir); // TODO simulate lost steps in range
    //if (motor_pos/0xF) {
    //    DEBUG_PRINTF("motor %X\n", motor_pos);
    //}
    return true; // TODO simulate driver failure
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

bool light_yellow(enum OnOff new)
{
    return true; // TODO
}

bool light_red(enum OnOff new)
{
    return true; // TODO
}

bool buzzer(enum OnOff new)
{
    return true; // TODO
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
    return Q_Lpm /** (motor_dir > 0 ? 1. : BAVU_VALVE_RATIO)*/;
}

// ------------------------------------------------------------------------------------------------
//! HW sensors simulation

float read_Pdiff_Lpm()
{
    static float abs_Q_Lpm = 10; // to handle exponential decrease during exhalation

    if (valve_state == Inhale) {
        abs_Q_Lpm = BAVU_Q_Lpm() * EXHAL_VALVE_RATIO;
        return abs_Q_Lpm;
    }
    else if (valve_state == Exhale) {
        const float decrease = .99; // expf(- abs(get_time_ms()-valve_exhale_ms)/100.); // <1% after 500ms @ 20 FPS
        abs_Q_Lpm *= decrease;
        return -abs_Q_Lpm;
    }
    else {
        return 0.;
    }
}

float read_Paw_cmH2O()
{
    static float Paw_cmH2O = 10; // to handle exponential decrease during plateau and exhalation

    if (valve_state == Inhale) {
        if (motor_dir > 0) {
            // Pressure augments as volume decreases according to PV=k ('loi des gaz parfait')
            // Pi=P0*V0/Vi
            Paw_cmH2O = PEP_cmH2O * (BAVU_V_ML_MAX + LUNG_V_ML_MAX)/(BAVU_V_mL() + LUNG_V_ML_MAX);
        }
        else {
            // Pressure exp. decreases due to lung compliance (volume augmentation) which depends on patient (and condition)
            const float decrease = .6; // expf(- abs(get_time_ms()-motor_release_ms)/10.); // <1% after 50ms
            const float Pplat_cmH2O = PEP_cmH2O + (BAVU_V_ML_MAX - BAVU_V_mL()) / LUNG_COMPLIANCE;
            Paw_cmH2O = Pplat_cmH2O
                        + (Paw_cmH2O-Pplat_cmH2O) * decrease;
        }
    }
    else if (valve_state == Exhale) {
        const float decrease = .9; // abs(get_time_ms()-valve_exhale_ms)/100.; // <1% after 500ms
        Paw_cmH2O = PEP_cmH2O + (Paw_cmH2O-PEP_cmH2O) * decrease;
    }
    return Paw_cmH2O;
}

float read_Patmo_mbar()
{
    return 1013. + sinf(2*M_PI*get_time_ms()/1000/60) * PATMO_VARIATION_MBAR; // TODO test failure
}
