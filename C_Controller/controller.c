#include "controller.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "alarms.h"
#include "configuration.h"
#include "ihm_communication.h"
#include "platform.h"
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

void check(int* bits, int bit, bool success)
{
    if ((*bits &   (1 << bit)) && !success) {
         *bits &= ~(1 << bit);
    }
}

bool sensor_test(float(*sensor)(), float min, float max, float maxstddev)
{
    // Sample sensor
    float value[10], stddev = 0., sumX=0., sumX2=0., sumY=0., sumXY=0.;
    const int samples = COUNT_OF(value);
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

    printf("Buzzer low\n");
    check(&test_bits, 1, buzzer_low      (On )); wait_ms(1000);
    check(&test_bits, 1, buzzer_low      (Off)); // start pos

    printf("Buzzer medium\n");
    check(&test_bits, 1, buzzer_medium      (On )); wait_ms(1000);
    check(&test_bits, 1, buzzer_medium      (Off)); // start pos

    printf("Buzzer high\n");
    check(&test_bits, 1, buzzer_high      (On )); wait_ms(1000);
    check(&test_bits, 1, buzzer_high      (Off)); // start pos

    printf("Red light\n");
    check(&test_bits, 2, light_red   (On )); wait_ms(1000);
    check(&test_bits, 2, light_red   (Off)); // start pos

    printf("Yellow light\n");
    check(&test_bits,  9, light_yellow(On )); wait_ms(1000);
    check(&test_bits, 10, light_yellow(Off)); // start pos

    printf("Green light\n");
    check(&test_bits,  9, light_green(On )); wait_ms(1000);
    check(&test_bits, 10, light_green(Off)); // start pos

    check(&test_bits, 5, init_sensors());
    check(&test_bits, 11, sensors_start());

    check(&test_bits, 5, sensor_test(get_sensed_VolM_Lpm  , -100,  100, 2)); printf("Rest    Pdiff  Lpm:%+.1g\n", get_sensed_VolM_Lpm  ());
    check(&test_bits, 6, sensor_test(get_sensed_P_cmH2O   ,  -20,  100, 2)); printf("Rest    Paw  cmH2O:%+.1g\n", get_sensed_P_cmH2O   ());
    check(&test_bits, 7, sensor_test(get_sensed_Patmo_mbar,  900, 1100, 2)); printf("Rest    Patmo mbar:%+.1g\n", get_sensed_Patmo_mbar());

    check(&test_bits, 3, init_valve());
    check(&test_bits, 3, valve_exhale());

    check(&test_bits, 4, init_motor());
    printf("Exhale  Pdiff  Lpm:%+.1g\n", get_sensed_VolM_Lpm());
    check(&test_bits, 4, motor_release());
    wait_ms(3000);
    check(&test_bits, 4, motor_stop());
    printf("Release Pdiff  Lpm:%+.1g\n", get_sensed_VolM_Lpm());
    check(&test_bits, 3, valve_inhale());
    printf("Inhale  Pdiff  Lpm:%+.1g\n", get_sensed_VolM_Lpm());

    motor_press_constant(400, 1000);
    wait_ms(1000);
    motor_stop();

    motor_release();
    wait_ms(3000);

	light_green(On);
	valve_inhale();
	motor_press_constant(MOTOR_STEP_TIME_US_MIN, 3800);
	wait_ms(1000);
	motor_stop();
	valve_exhale();
	motor_release();
	light_green(Off);
	wait_ms(3000);


    //printf("Press   Pdiff  Lpm:%+.1g\n", get_sensed_VolM_Lpm());
    //check(&test_bits, 4, motor_stop());
    //check(&test_bits, 3, valve_exhale()); // start pos
    //printf("Exhale  Pdiff  Lpm:%+.1g\n", get_sensed_VolM_Lpm());

    /*check(&test_bits, 8, init_motor_pep());
    motor_pep_home();
    wait_ms(1000);
    motor_pep_move(10);
    // TODO check(&test_bits, 8, motor_pep_...
    wait_ms(10000);*/

    return test_bits;
}

PEPState pep_state = Ajustement;
unsigned int pep_cycle=0;
unsigned int good_dpep_count=0;
unsigned int bad_dpep_count=0;
float bad_dpep_threshold=0.3;

bool regulation_pep()
{
    const float VTi     = get_sensed_VTi_mL         ();
    const float VTe     = get_sensed_VTe_mL         ();
    const float Pcrete  = get_sensed_Pcrete_cmH2O   ();
    const float PEP     = get_sensed_PEP_cmH2O      ();
    const float setPEP  = get_setting_PEP_cmH2O     ();

    float dpep=0;
    if(Pcrete<PEP+2 && VTe>VTi/2)
    {
        set_alarm(ALARM_SENSOR_FAIL);
    }
    else
    {
        unset_alarm(ALARM_SENSOR_FAIL);
        pep_cycle++;
        dpep = setPEP - PEP;
        if(Ajustement == pep_state)
        {
            if(fabsf(dpep)>0.1)
            {
                motor_pep_move(EXHAL_VALVE_P_RATIO*dpep/3);
            }
        }
        else if(Maintien == pep_state)
        {
            if(dpep>0.1)
            {
                motor_pep_move(EXHAL_VALVE_P_RATIO*dpep/3);
            }
        }
    }
    if(dpep<bad_dpep_threshold)
    {
        good_dpep_count++;
        bad_dpep_count=0;
    }else
    {
        good_dpep_count=0;
        bad_dpep_count++;
    }
    if(Ajustement == pep_state && good_dpep_count >= 2)
    {
        pep_state=Maintien;
    }
    else if(Maintien == pep_state && bad_dpep_count >= 10)
    {
        pep_state=Ajustement;
    }
    return true;
}
volatile RespirationState state = Unknown;

RespirationState current_respiration_state() { return state; }

uint32_t respi_start_ms = 0;
uint32_t state_start_ms = 0;

static float VTi_mL       = 0.f;
float get_sensed_VTi_mL() { return VTi_mL;}
static float VTe_mL       = 0.f;
float get_sensed_VTe_mL() { return VTe_mL;}
static float VTe_start_mL       = 0.f;
static float VTe_end_mL       = 0.f;

static float Pcrete_cmH2O = 0.f;
float get_sensed_Pcrete_cmH2O() { return Pcrete_cmH2O;}
static float Pplat_cmH2O  = 100.f;
static float PEP_cmH2O    = 100.f;
float get_sensed_PEP_cmH2O() { return PEP_cmH2O;}

static float VMe_Lpm      = 0.f;

static uint32_t last_sensed_ms = 0;

float get_last_sensed_ms() { return last_sensed_ms; }

void enter_state(RespirationState new)
{
    static uint16_t last_step = 0;

#ifndef NDEBUG
    const char* current = NULL;
    switch (state) {
    case Insufflation   : current = "Insufflation"   ; break;
    case Plateau        : current = "Plateau"        ; break;
    case Exhalation     : current = "Exhalation"     ; break;
    case ExhalationPause: current = "ExhalationPause"; break;
    default             : current = "<unknown>"      ; break;
    }
    switch (new) {
    case Insufflation   : /*hardware_serial_write_data("Insufflation \n", 15);*//*DEBUG_PRINTF(" %s -> Insufflation"   , current);*/ break;
    case Plateau        : /*hardware_serial_write_data("Plateau      \n", 15);*//*DEBUG_PRINTF(" %s -> Plateau"        , current);*/ break;
    case Exhalation     : /*hardware_serial_write_data("Exhalation   \n", 15);*//*DEBUG_PRINTF(" %s -> Exhalation"     , current);*/ break;
    case ExhalationPause: /*DEBUG_PRINTF(" %s -> ExhalationPause", current);*/ break;
    default             : /*DEBUG_PRINTF(" %s -> <unknown>"      , current);*/ break;
    }
#endif
	/*switch (new) {
		case Insufflation   : light_red(Off);light_green(On); break;
		case Plateau        : light_green(Off); light_yellow(On); break;
		case Exhalation     : light_yellow(Off); light_red(On); break;
	}*/
    state = new;
    state_start_ms = get_time_ms();
    if (state == Insufflation) {
        VTi_mL       = 0.f;
        Pcrete_cmH2O = 0.f;
        reset_sensed_Vol_mL();
        sensors_start_sampling_flow();

        valve_inhale();
        if (last_step != 0) {
            motor_press(steps_t_us, last_step);
		}
		else {
			motor_press_constant(MOTOR_STEP_TIME_US_MIN*(60.f / get_setting_Vmax_Lpm()) , 6*get_setting_VT_mL());
		}
        respi_start_ms = get_time_ms();
    }
	else if(state==Plateau) {
        sensors_stop_sampling_flow();
        //valve_inhale();
        VTe_start_mL = get_sensed_Vol_mL();
        motor_release();
	}
	else if(state==Exhalation) {
		sensors_stop_sampling_flow();
        valve_exhale();
        motor_release();
		VMe_Lpm = 0.f;
		PEP_cmH2O = 0.f;
        // TODO last_step = compute_motor_steps_and_Tinsu_ms(get_setting_Vmax_Lpm()/60.f, get_setting_VT_mL());
	}
	else if(state==ExhalationPause) {
        valve_inhale();
        motor_release();
	}
}

// TODO
//#ifndef NTESTS
//static float    T    ;
//static float    VT   ;
//static float    VM   ;
//static float    Pmax ;
//static uint32_t Tplat;
//#endif

extern float corrections[MOTOR_MAX];
extern float average_Q_Lps[SAMPLING_SIZE];

void cycle_respiration()
{
//#ifdef NTESTS
    const uint32_t T        = get_setting_T_ms      ();
    const float    VT       = get_setting_VT_mL     ();
    const float    VM       = get_setting_Vmax_Lpm  ();
    const float    Pmax     = get_setting_Pmax_cmH2O();
    const uint32_t Tplat    = get_setting_Tplat_ms  ();
    const uint32_t Tpins_ms = get_command_Tpins_ms  ();
    const uint32_t Tpexp_ms = get_command_Tpexp_ms  ();
//#endif

    if (last_sensed_ms + 25 <= get_time_ms())
    {
        float display = get_sensed_Vol_mL();
#ifndef NDEBUG
//		int current_correction_idx = COUNT_OF(steps_t_us)*(get_setting_T_ms() - respi_start_ms)/1000;
//		if (current_correction_idx < COUNT_OF(steps_t_us)) {
//			display = corrections[current_correction_idx] * 300;
//		}
//		else {
//			display = 0;
//		}
//		if (current_respiration_state() == Insufflation) {
//			int current_average_idx = (float) COUNT_OF(average_Q_Lps) * (((float) (get_setting_T_ms() - respi_start_ms))/1000.0f);
//			if (current_average_idx < COUNT_OF(average_Q_Lps)) {
//				display = current_average_idx /*average_Q_Lps[current_average_idx] * 10*/;
//			}
//		}
        send_DATA(get_sensed_P_cmH2O(), get_sensed_VolM_Lpm(), display);
#endif
        last_sensed_ms = get_time_ms();
    }

    if (Unknown == state) {
        send_INIT(get_init_str());
        enter_state(Insufflation);
    }
    else if (Insufflation == state) {
        //if (Pmax <= get_sensed_P_cmH2O()) {
        //    enter_state(Exhalation);
        //}
        /*if (VT <= get_sensed_VTi_mL()) {
            enter_state(Plateau);
        }*/
        VTi_mL = get_sensed_Vol_mL();
        Pcrete_cmH2O = MAX(Pcrete_cmH2O, get_sensed_P_cmH2O()); // TODO check specs

		if( get_time_ms()  >= ( (800U) + respi_start_ms ) ) {
	            enter_state(Plateau);
		};
    }
	else if (Plateau == state) {
        //valve_inhale();
        /*if (Pmax <= get_sensed_P_cmH2O()
            || (state_start_ms + MAX(Tplat,Tpins_ms)) <= get_time_ms()) { // TODO check Tpins_ms < first_pause_ms+5000
            enter_state(Exhalation);
        }*/ 
        VTi_mL = get_sensed_Vol_mL();
        Pplat_cmH2O = MIN(Pplat_cmH2O, get_sensed_P_cmH2O()); // TODO average over Xms
		// DO NOT FORGET PMAX !!!
		if ( (state_start_ms + MAX(Tplat,Tpins_ms)) <= get_time_ms() )  { // TODO check Tpins_ms < first_pause_ms+5000
                enter_state(Exhalation);
        }

    }
    else if (Exhalation == state ) {
        VMe_Lpm   = get_sensed_VolM_Lpm();

        PEP_cmH2O = get_sensed_P_cmH2O(); // TODO average over Xms
        if (Tpexp_ms > 0) {
			enter_state(ExhalationPause);
		}
		else if ((respi_start_ms + T) <= get_time_ms()) { // TODO check Tpexp_ms < first_pause_ms+5000
            uint32_t t_ms = get_time_ms();

            VTe_end_mL    = get_sensed_Vol_mL();
            VTe_mL        = VTe_start_mL - VTe_end_mL;

            EoI_ratio =  (float)(t_ms-state_start_ms)/(state_start_ms-respi_start_ms);
            FR_pm     = 1./(((float)(t_ms-respi_start_ms))/1000/60);

            // TODO regulation_pep();
            send_RESP(EoI_ratio, FR_pm, VTe_mL, VMe_Lpm, Pcrete_cmH2O, Pplat_cmH2O, PEP_cmH2O);
            enter_state(Insufflation);
        }
	}
    else if (ExhalationPause == state) {
        VMe_Lpm   = get_sensed_VolM_Lpm();
        PEP_cmH2O = get_sensed_P_cmH2O(); // TODO average over Xms
        if ((respi_start_ms + Tpexp_ms) <= get_time_ms()) { // TODO check Tpexp_ms < first_pause_ms+5000
            uint32_t t_ms = get_time_ms();

            VTe_end_mL    = get_sensed_Vol_mL();
            VTe_mL        = VTe_start_mL - VTe_end_mL;

            EoI_ratio =  (float)(t_ms-state_start_ms)/(state_start_ms-respi_start_ms);
            FR_pm     = 1./(((float)(t_ms-respi_start_ms))/1000/60);

            // TODO regulation_pep();
            send_RESP(EoI_ratio, FR_pm, VTe_mL, VMe_Lpm, Pcrete_cmH2O, Pplat_cmH2O, PEP_cmH2O);
            enter_state(Insufflation);
        }
	}
	
}

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

bool PRINT(test_nominal_cycle)
// TODO
//    for (T    =  2; T    <=  3; T    +=  1) {
//    for (VT   =300; VT   <=600; VT   +=150) {
//    for (VM   = 30; VM   <= 60; VT   += 15) {
//    for (Pmax = 65; Pmax >= 50; Pmax -=  5) {
//    for (Tplat=100; Tplat<=300; Tplat+=100) {
    for (uint32_t t_ms=0; t_ms<10*get_setting_T_ms(); t_ms=wait_ms(1)) { // 1kHz
        cycle_respiration();
        uint32_t t = t_ms % get_setting_T_ms();
        if (t<get_setting_Tinsu_ms()
            && !(TEST_EQUALS(Insufflation, current_respiration_state()))) {
            return false;
        }
        if (get_setting_Tinsu_ms() < t && t < get_setting_Tinsu_ms()+get_setting_Tplat_ms()
            && !(TEST_EQUALS(Plateau, current_respiration_state()))) {
            return false;
        }
        if (get_setting_Tinsu_ms()+get_setting_Tplat_ms() < t
            && !(TEST_EQUALS(Exhalation, current_respiration_state()))) {
            return false;
        }
    }
//    }}}}}
    return true;
}

bool PRINT(TEST_CONTROLLER)
    return
        test_nominal_cycle() &&
        true;
}

#endif
