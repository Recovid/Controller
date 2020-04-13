#include <math.h>

#include "controller.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "ihm_communication.h"
#include "sensing.h"
#include "lowlevel.h"
#include "simple_indicators.h"

// INIT

#define INIT_STR_SIZE 80
char init_str[INIT_STR_SIZE+1] = ""; // leaving space for off by 1 errors in code

const char *get_init_str() { return init_str; }


// RESP

float EoI_ratio    = 0.f;
float FR_pm        = 0.f;
float VTe_mL       = 0.f;
float VM_Lpm       = 0.f;
float Pcrete_cmH2O = 0.f;
float Pplat_cmH2O  = 0.f;
float PEP_cmH2O    = 0.f;

uint32_t Tpins_ms = 0;
uint32_t Tpexp_ms = 0;

bool pause_insp(int t_ms)
{
    Tpins_ms = get_time_ms()+t_ms;
    return true;
}

bool pause_exp(int t_ms)
{
    Tpexp_ms = get_time_ms()+t_ms;
    return true;
}

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
    DEBUG_PRINT("Start self tests");
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



enum State { Insufflation, Plateau, Exhalation, ExhalationEnd } state = Insufflation;
long respi_start_ms = -1;
long state_start_ms = -1;

void enter_state(enum State new)
{
    state = new;
    state_start_ms = get_time_ms();
    if(state == Insufflation) {
        respi_start_ms = get_time_ms();
    }
}

void cycle_respiration()
{

    if (respi_start_ms==-1) enter_state(Insufflation);

    if (Insufflation == state) {
        valve_inhale();
        if (get_setting_Pmax_cmH2O() <= get_sensed_P_cmH2O()) {
            Pcrete_cmH2O = get_sensed_P_cmH2O();
            enter_state(Exhalation);
        }
        if (get_setting_VT_mL() <= get_sensed_Vol_mL()) {
            Pcrete_cmH2O = get_sensed_P_cmH2O();
            enter_state(Plateau);
        }
        motor_press();
    }
    else if (Plateau == state) {
        valve_inhale();
        Pplat_cmH2O = get_sensed_P_cmH2O(); //TODO average
        //printf("Date to over %ld vs current date %ld\n", state_start_ms + MAX(get_setting_Tplat_ms(),Tpins_ms), get_time_ms());
        if (get_setting_Pmax_cmH2O() <= get_sensed_P_cmH2O()
            || (state_start_ms + MAX(get_setting_Tplat_ms(),Tpins_ms)) <= get_time_ms()) { // TODO check Tpins_ms < first_pause_ms+5000
            enter_state(Exhalation);
        }
        motor_release();
    }
    else if (Exhalation == state) {
        valve_exhale();
        PEP_cmH2O = get_sensed_P_cmH2O(); //TODO average
        //printf("respi start %d Temps cycle %ld temps actuelle %ld \n", respi_start_ms, 1000*60/F);
        if ((respi_start_ms + MAX(1000*60/get_setting_FR_pm(),Tpexp_ms)) <= get_time_ms()) { // TODO check Tpexp_ms < first_pause_ms+5000
            long t_ms = get_time_ms();

            EoI_ratio = (float)(t_ms-state_start_ms)/(state_start_ms-respi_start_ms);
            FR_pm = 1./(((float) (t_ms-respi_start_ms))/1000/60);
            VTe_mL = get_sensed_Vol_mL();
            // TODO ...

            send_RESP(EoI_ratio, FR_pm, VTe_mL, VM_Lpm, Pcrete_cmH2O, Pplat_cmH2O, PEP_cmH2O);
            enter_state(Insufflation);
        }
        motor_release();
    }
}
