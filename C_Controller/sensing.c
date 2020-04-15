#include "sensing.h"

#include <stdlib.h>
#include <math.h>

#include "ihm_communication.h"

#include "lowlevel/include/lowlevel.h"

// DATA

static float VolM_Lpm = 0.f;
static float P_cmH2O  = 0.f;
static float Vol_mL   = 0.f;

float get_sensed_Vol_mL   () { return MAX(0.f,Vol_mL  ); }
float get_sensed_VolM_Lpm () { return         VolM_Lpm ; }
float get_sensed_P_cmH2O  () { return MAX(0.f,P_cmH2O ); }

float get_sensed_PEP_cmH2O() { return get_sensed_P_cmH2O(); } // TODO
float get_sensed_VMe_Lpm  () { return get_sensed_P_cmH2O(); } // TODO

void sense_and_compute()
{
    static unsigned long last_sense_ms = 0;
    static unsigned long sent_DATA_ms = 0;

    // TODO float Patmo_mbar = read_Patmo_mbar();
    P_cmH2O  = read_Paw_cmH2O();
    VolM_Lpm = read_Pdiff_Lpm(); // TODO Compute corrected QPatientSLM based on Patmo
    Vol_mL  += (VolM_Lpm * 1000.) * (get_time_ms() - last_sense_ms)/1000./60.;
    if ((sent_DATA_ms+50 < get_time_ms()) // @ 20Hz
        && send_DATA(get_sensed_P_cmH2O(), get_sensed_VolM_Lpm(), get_sensed_Vol_mL(), 0, 0)) {
        sent_DATA_ms = get_time_ms();
    }

    last_sense_ms = get_time_ms();
}

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

bool PRINT(test_non_negative_sensing)
    Vol_mL   = -1.f;
    P_cmH2O  = -1.f;
    return
        TEST_FLT_EQUALS(0.f, get_sensed_Vol_mL ()) &&
        TEST_FLT_EQUALS(0.f, get_sensed_P_cmH2O()) &&
        true;
}

bool PRINT(TEST_SENSING)
    return
        test_non_negative_sensing() &&
        true;
}

#endif
