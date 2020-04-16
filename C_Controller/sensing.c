#include "sensing.h"

#include <stdlib.h>
#include <math.h>

#include "controller.h" // TODO isolate current_respiration_state()
#include "ihm_communication.h"

#include "lowlevel/include/lowlevel.h"

// DATA

static float VolM_Lpm = 0.f;
static float P_cmH2O  = 0.f;
static float VTi_mL   = 0.f;
static float VTe_mL   = 0.f;

float get_sensed_VTi_mL   () { return MAX(0.f,VTi_mL  ); }
float get_sensed_VTe_mL   () { return MIN(0.f,VTe_mL  ); }
float get_sensed_VolM_Lpm () { return         VolM_Lpm ; }
float get_sensed_P_cmH2O  () { return MAX(0.f,P_cmH2O ); }

float get_sensed_Pcrete_cmH2O() { return 0; } // TODO get_sensed_P_cmH2O()
float get_sensed_PEP_cmH2O() { return 0; } // TODO get_sensed_P_cmH2O()
float get_sensed_VMe_Lpm  () { return get_sensed_P_cmH2O(); } // TODO

void sense_and_compute(RespirationState state)
{
    static unsigned long last_state = Insufflation;
    static unsigned long last_sense_ms = 0;
    static unsigned long sent_DATA_ms = 0;

    // TODO float Patmo_mbar = read_Patmo_mbar();
    P_cmH2O  = read_Paw_cmH2O();
    VolM_Lpm = read_Pdiff_Lpm(); // TODO Compute corrected QPatientSLM based on Patmo
    if (state==Insufflation || state==Plateau) {
        if (last_state==Exhalation || last_state==ExhalationPause) {
            VTi_mL = 0.f;
        }
        else {
            VTi_mL += MAX(0.f, (VolM_Lpm/60./*mLpms*/) * (get_time_ms()-last_sense_ms));
        }
    }
    if (state==Exhalation || state==ExhalationPause) {
        if (last_state==Insufflation || last_state==Plateau) {
            VTe_mL = 0.f;
        }
        else {
            VTe_mL += MIN(0.f, (VolM_Lpm/60./*mLpms*/) * (get_time_ms()-last_sense_ms));
        }
    }
    if ((sent_DATA_ms+50 < get_time_ms()) // @ 20Hz
        && send_DATA(get_sensed_P_cmH2O(), get_sensed_VolM_Lpm(), get_sensed_VTi_mL(), 0, 0)) {
        sent_DATA_ms = get_time_ms();
    }

    last_state    = state;
    last_sense_ms = get_time_ms();
}

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

bool PRINT(test_non_negative_sensing)
    VTi_mL   = -1.f;
    P_cmH2O  = -1.f;
    return
        TEST_FLT_EQUALS(0.f, get_sensed_VTi_mL ()) &&
        TEST_FLT_EQUALS(0.f, get_sensed_P_cmH2O()) &&
        true;
}

bool PRINT(TEST_SENSING)
    return
        test_non_negative_sensing() &&
        true;
}

#endif
