#include "controller.h"
#include "controller_settings.h"

#include <math.h>
#define MIN(a,b) ((a)<(b) ? (a) : (b))

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

const char* init_str = "simulation";

// DATA

float VolM_Lpm = 0;
float P_cmH2O  = 0;
int   Vol_mL   = 0;

// RESP

int IE           = 0;
int FRs_pm       = 0;
int VTe_mL       = 0;
int VM_Lm        = 0;
int Pcrete_cmH2O = 0;
int Pplat_cmH2O  = 0;
int PEPs_cmH2O   = 0;

void sense_and_compute()
{
    static long last_sense_ms = 0;
    static long sent_DATA_ms = 0;

    P_cmH2O = read_Paw_cmH2O();
    // TODO float Patmo_mbar = read_Patmo_mbar();
    VolM_Lpm = read_Pdiff_Lpm(); // TODO Compute corrected QPatientSLM based on Patmo
    Vol_mL += (read_Pdiff_Lpm() / 1000) * abs(get_time_ms() - last_sense_ms)/1000/60;

    Pplat_cmH2O = PEP_cmH2O = P_cmH2O; // TODO Compute average

    if ((sent_DATA_ms+50 < get_time_ms())
        && send_DATA(P_cmH2O, VolM_Lpm, Vol_mL, Pplat_cmH2O, PEP_cmH2O)) {
        sent_DATA_ms = get_time_ms();
    }

    last_sense_ms = get_time_ms();
}

enum State { Insufflation, Plateau, Exhalation, ExhalationEnd } state = Exhalation;
long respi_start_ms = -1;
long state_start_ms = -1;
long sent_RESP_ms = 0;

enum State enter_state(enum State new)
{
    enum State old = state;
    state = new;
    state_start_ms = get_time_ms();
    return old;
}

void cycle_respiration()
{
    if (state_start_ms==-1) state_start_ms = get_time_ms();
    if (respi_start_ms==-1) respi_start_ms = get_time_ms();

    if (Insufflation == state) {
        respi_start_ms = get_time_ms();
        if (Pmax_cmH2O <= read_Paw_cmH2O()) {
            enter_state(Exhalation);
        }
        if (VT_mL <= Vol_mL) {
            enter_state(Plateau);
        }
        valve_inhale();
        motor_press();
    }
    else if (Plateau == state) {
        if (Pmax_cmH2O <= read_Paw_cmH2O()
            || MIN(Tplat_ms,Tpins_ms) <= get_time_ms()) { // TODO check Tpins_ms < first_pause_ms+5000
            enter_state(Exhalation);
        }
        valve_inhale();
        motor_release();
    }
    else if (Exhalation == state) {
        if (MIN(respi_start_ms+1000*60/FR_pm,Tpexp_ms) <= get_time_ms()) { // TODO check Tpexp_ms < first_pause_ms+5000
            enter_state(Insufflation);
        }
        valve_exhale();
        motor_release();
    }

    if ((sent_RESP_ms+2000 < get_time_ms()) // TODO after ExhalationPEP
        && send_RESP(IE, FRs_pm, VTe_mL, VM_Lm, Pcrete_cmH2O, Pplat_cmH2O, PEP_cmH2O)) {
        sent_RESP_ms = get_time_ms();
    }
}
