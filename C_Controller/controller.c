#include "controller.h"

#include "ihm_communication.h"

#include "controller_settings.h"

int FR_pm      =  18;
int VT_mL      = 300;
int PEP_cmH2O  =   5;
int Vmax_Lpm   =  60;
int Tplat_ms   = 400;

int Pmax_cmH2O =  60;
int Pmin_cmH2O =  20;
int VTmin_mL   = 400;
int FRmin_pm   =  10;
int VMmin_Lm   =   5;

const int PEPmax_cmH2O = 2;
const int PEPmin_cmH2O = 2;

enum State { Insufflation, Plateau, Exhalation, ExhalationPEP } current = Exhalation;

// INIT

const char* init_str = "simulation";

// DATA

int P_cmH2O  = 0;
int VolM_Lpm = 0;
int Vol_mL   = 0;

// RESP

int IE           = 0;
int FRs_pm       = 0;
int VTe_mL       = 0;
int VM_Lm        = 0;
int Pcrete_cmH2O = 0;
int Pplat_cmH2O  = 0;
int PEPs_cmH2O   = 0;

bool run()
{
    send_DATA(P_cmH2O, VolM_Lpm, Vol_mL, Pplat_cmH2O, PEP_cmH2O);
    send_RESP(IE, FRs_pm, VTe_mL, VM_Lm, Pcrete_cmH2O, Pplat_cmH2O, PEP_cmH2O);
    read();
}
