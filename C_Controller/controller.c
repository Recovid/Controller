#include <math.h>

#include "controller.h"
#include "controller_settings.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#define MAX(a,b) ((a)>(b) ? (a) : (b))

#include "ihm_communication.h"

#include "hardware_simulation.h"

int FR_pm      =  18;
int VT_mL      = 300;
int PEP_cmH2O  =   5;
int Vmax_Lpm   =  60;
long Tplat_ms   = 400;

int Pmax_cmH2O =  60;
int Pmin_cmH2O =  20;
int VTmin_mL   = 400;
int FRmin_pm   =  10;
int VMmin_Lm   =   5;

long Tpins_ms   =   0;
long Tpexp_ms   =   0;
long Tpbip_ms   =   0;

const int PEPmax_cmH2O = 2;
const int PEPmin_cmH2O = 2;

// INIT

char init_str[81] = "";

// DATA

float VolM_Lpm = 0.;
float P_cmH2O  = 0.;
float Vol_mL   = 0.;

// RESP

float IE           = 0.;
float FRs_pm       = 0.;
float VTe_mL       = 0.;
float VM_Lm        = 0.;
float Pcrete_cmH2O = 0.;
float Pplat_cmH2O  = 0.;
float PEPs_cmH2O   = 0.;

void check(int* bits, int bit, bool success)
{
    if ((*bits & (1 << bit)) && !success) {
        *bits &= ~(1 << bit);
    }
}

bool sensor_test(float(*sensor)(), float min, float max, float maxstddev)
{
    // Sample sensor
    const int samples = 10;
    float value[samples], stddev = 0., sumX=0., sumX2=0., sumY=0., sumXY=0.;
    for (int i=0; i<samples; i++) {
        value[i] = (*sensor)();
        if (value[i] < min || max < value[i]) {
            return false;
        }
        sumX  += i;   //  45 for 10 samples
        sumX2 += i*i; // 285 for 10 samples
        sumY  += value[i];
        sumXY += value[i]*i;
        wait_ms(1);
    }
    // Fit a line to account for rapidly changing data such as Pdiff at start of Exhale
    float b = (samples*sumXY-sumX*sumY)/(samples*sumX2-sumX*sumX);
    float a = (sumY-b*sumX)/samples;

    // Compute standard deviation to line fit
    for (int i=0; i<samples; i++) {
        float fit = a+b*i;
        stddev += pow(value[i] - fit, 2);
    }
    stddev = sqrtf(stddev / samples);

    return maxstddev < stddev;
}

int self_tests()
{
    printf("Start self-tests\n");
    int test_bits = 0xFFFFFFFF;

    // TODO test 'Arret imminent' ?

    printf("Buzzer\n");
    check(&test_bits, 1, buzzer      (On )); wait_ms(100);
    check(&test_bits, 1, buzzer      (Off)); // start pos
    printf("Red light\n");
    check(&test_bits, 2, light_red   (On )); wait_ms(100);
    check(&test_bits, 2, light_red   (Off)); // start pos

    check(&test_bits, 3, valve_exhale());
    printf("Exhale  Pdiff  Lpm:%+.1g\n", read_Pdiff_Lpm());
    for (int i=0; i<MOTOR_MAX*1.1; i++) {
        check(&test_bits, 4, motor_release()); wait_ms(1);
    }
    printf("Release Pdiff  Lpm:%+.1g\n", read_Pdiff_Lpm());
    check(&test_bits, 3, valve_inhale());
    printf("Inhale  Pdiff  Lpm:%+.1g\n", read_Pdiff_Lpm());
    for (int i=0; i<MOTOR_MAX*0.1; i++) {
        check(&test_bits, 4, motor_press()); wait_ms(1); // start pos
    }
    printf("Press   Pdiff  Lpm:%+.1g\n", read_Pdiff_Lpm());
    check(&test_bits, 4, motor_stop());
    check(&test_bits, 3, valve_exhale()); // start pos
    printf("Exhale  Pdiff  Lpm:%+.1g\n", read_Pdiff_Lpm());

    check(&test_bits, 5, sensor_test(read_Pdiff_Lpm , -100,  100, 2)); printf("Rest    Pdiff  Lpm:%+.1g\n", read_Pdiff_Lpm ());
    check(&test_bits, 6, sensor_test(read_Paw_cmH2O ,  -20,  100, 2)); printf("Rest    Paw  cmH2O:%+.1g\n", read_Paw_cmH2O ());
    check(&test_bits, 7, sensor_test(read_Patmo_mbar,  900, 1100, 2)); printf("Rest    Patmo mbar:%+.1g\n", read_Patmo_mbar());

    // TODO check(&test_bits, 8, motor_pep ...

    printf("Yellow light\n");
    check(&test_bits,  9, light_yellow(On )); wait_ms(100);
    check(&test_bits, 10, light_yellow(Off)); // start pos

    return test_bits;
}

void sense_and_compute()
{
    static long last_sense_ms = 0;
    static long sent_DATA_ms = 0;

    P_cmH2O = read_Paw_cmH2O();
    // TODO float Patmo_mbar = read_Patmo_mbar();
    VolM_Lpm = read_Pdiff_Lpm(); // TODO Compute corrected QPatientSLM based on Patmo
    Vol_mL += (VolM_Lpm / 1000.) * abs(get_time_ms() - last_sense_ms)/1000./60.;

    Pplat_cmH2O = PEP_cmH2O = P_cmH2O; // TODO Compute average

    if ((sent_DATA_ms+50 < get_time_ms())
        && send_DATA(P_cmH2O, VolM_Lpm, Vol_mL, Pplat_cmH2O, PEP_cmH2O)) {
        sent_DATA_ms = get_time_ms();
    }


    last_sense_ms = get_time_ms();
}

enum State { Insufflation, Plateau, Exhalation, ExhalationEnd } state = Insufflation;
long respi_start_ms = -1;
long state_start_ms = -1;

enum State enter_state(enum State new)
{

    static int sent_RESP_ms = 0;
    int period_ms = 60 * 1000 / FR_pm;
    float end_insufflation = 0.5f - ((float)Tplat_ms / period_ms);
    float cycle_pos = (float)(get_time_ms() % period_ms) / period_ms; // 0 cycle start => 1 cycle end
    if (cycle_pos < end_insufflation)
    {
        float insufflation_pos = cycle_pos / end_insufflation; // 0 start insufflation => 1 end insufflation
        P_cmH2O = (int)(Pmin_cmH2O + sqrtf(insufflation_pos) * (Pmax_cmH2O - Pmin_cmH2O));
    }
    else if (cycle_pos < 0.5) // Plateau
    {
        P_cmH2O = (int)(0.9f * Pmax_cmH2O);
    }
    else
    {
        P_cmH2O = (int)(0.9f * Pmax_cmH2O - sqrtf(cycle_pos * 2.f - 1.f) * (0.9f * Pmax_cmH2O - Pmin_cmH2O));
    }
}

void cycle_respiration()
{
    if (state_start_ms==-1) state_start_ms = get_time_ms();
    if (respi_start_ms==-1) respi_start_ms = get_time_ms();

    if (Insufflation == state) {
        respi_start_ms = get_time_ms();
        valve_inhale();
        if (Pmax_cmH2O <= read_Paw_cmH2O()) {
            enter_state(Exhalation);
        }
        if (VT_mL <= Vol_mL) {
            enter_state(Plateau);
        }
        motor_press();
    }
    else if (Plateau == state) {
        valve_inhale();
        if (Pmax_cmH2O <= read_Paw_cmH2O()
            || MAX(Tplat_ms,Tpins_ms) <= get_time_ms()) { // TODO check Tpins_ms < first_pause_ms+5000
            enter_state(Exhalation);
        }
        motor_release();
    }
    else if (Exhalation == state) {
        valve_exhale();
        if (MAX(respi_start_ms+1000*60/FR_pm,Tpexp_ms) <= get_time_ms()) { // TODO check Tpexp_ms < first_pause_ms+5000
            enter_state(Insufflation);
            long t_ms = get_time_ms();

            IE = (float)((state_start_ms-respi_start_ms))/(t_ms-state_start_ms);
            FRs_pm = 1./((t_ms-respi_start_ms)/1000/60);
            VTe_mL = Vol_mL;
            // TODO ...

            send_RESP(IE, FRs_pm, VTe_mL, VM_Lm, Pcrete_cmH2O, Pplat_cmH2O, PEP_cmH2O);
        }
        motor_release();
    }
}
