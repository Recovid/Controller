#include "ihm_communication.h"

#ifndef WIN32
//FreeRTOS Include
#include <FreeRTOS.h>
#include "portmacro.h"
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "controller.h"
#include "hardware_simulation.h"

// ------------------------------------------------------------------------------------------------
//! Settings

//! \warning Global settings MUST use types that can be atomically read/write in a threadsafe way on STM32
//! \warning Non volatile memory may be limited to uint16_t by the corresponding driver

static uint16_t setting_FR_pm      =  18;
static uint16_t setting_VT_mL      = 300;
static uint16_t setting_PEP_cmH2O  =   5;
static uint16_t setting_Vmax_Lpm   =  60;
static uint16_t setting_EoI_ratio  =   2;

static uint16_t setting_Pmax_cmH2O =  60;
static uint16_t setting_Pmin_cmH2O =  20;
static uint16_t setting_VTmin_mL   = 400;
static uint16_t setting_FRmin_pm   =  10;
static uint16_t setting_VMmin_Lm   =   5;

const int setting_PEPmax_cmH2O = 2;
const int setting_PEPmin_cmH2O = 2;

float get_setting_FR_pm    () { return setting_FR_pm    ; }
float get_setting_VT_mL    () { return setting_VT_mL    ; }
float get_setting_PEP_cmH2O() { return setting_PEP_cmH2O; }
float get_setting_Vmax_Lpm () { return setting_Vmax_Lpm ; }
float get_setting_EoI_ratio() { return setting_EoI_ratio; }

uint16_t get_setting_Tplat_ms  ()
{
    const float T_ms  = 60000.f / get_setting_FR_pm();
    const float Ti_ms = get_setting_VT_mL() / (get_setting_Vmax_Lpm() / 60.f); // FIXME Lpm/60=Lps and 1/Lps=s not ms
    const float Tplat_ms= (T_ms/(1.f+(1.f/get_setting_EoI_ratio()))) - Ti_ms;  // Check Tplat=(T-Te)-Ti => T-Te=T/(1+E/I) ???
    const float Te_ms = (Ti_ms+Tplat_ms) / get_setting_EoI_ratio();            // FIXME Te=(Ti+Tplat)/(E/I)=1/2 if user enters E/I=2
    return T_ms - (Ti_ms+Te_ms);
}

int get_setting_Pmax_cmH2O  () { return setting_Pmax_cmH2O  ; }
int get_setting_Pmin_cmH2O  () { return setting_Pmin_cmH2O  ; }
int get_setting_VTmin_mL    () { return setting_VTmin_mL    ; }
int get_setting_FRmin_pm    () { return setting_FRmin_pm    ; }
int get_setting_VMmin_Lm    () { return setting_VMmin_Lm    ; }

int get_setting_PEPmax_cmH2O() { return setting_PEPmax_cmH2O; }
int get_setting_PEPmin_cmH2O() { return setting_PEPmin_cmH2O; }

// ------------------------------------------------------------------------------------------------
//! Commands

//! Global settings MUST use types that can be atomically read/write in a threadsafe way on STM32

long command_Tpins_ms = -1;
long command_Tpexp_ms = -1;
long command_Tpbip_ms = -1;

bool command_soft_reset = false;

long get_command_Tpins_ms() { return command_Tpins_ms  ; }
long get_command_Tpexp_ms() { return command_Tpexp_ms  ; }
long get_command_Tpbip_ms() { return command_Tpbip_ms  ; }

bool is_soft_reset_asked () { return command_soft_reset; }

// ------------------------------------------------------------------------------------------------
//! Communication

#define MAX_FRAME 100
 
char sign(int i) { return i<0 ? '-' : '+'; }

#define CS8 "\tCS8:"
#define CS8_VALUE "%02X\n"

unsigned char checksum8(const char* s)
{
    unsigned long cs8 = 0ul;
    for (; s && *s!='\0' ; s++) {
        cs8 += *s;
    }
    return (unsigned char)(cs8 % 256);
}

bool send(const char* frame)
{
    char checked_frame[MAX_FRAME+1] = "";
    return snprintf(checked_frame, MAX_FRAME, "%s" CS8_VALUE, frame, checksum8(frame)) > 0
           && send_ihm(checked_frame)>0;
}

bool send_DATA(float P_cmH2O, float VolM_Lpm, float Vol_mL, float Pplat_cmH2O, float PEP_cmH2O)
{
    char frame[MAX_FRAME+1] = "";
    return snprintf(frame, MAX_FRAME, "DATA msec_:%06d Vol__:%04d Deb__:%c%03d Paw__:%c%03d" CS8,
			(int)(get_time_ms() % 1000000l), (int)Vol_mL, sign(VolM_Lpm), abs((int)VolM_Lpm), sign(P_cmH2O), abs((int)P_cmH2O)) > 0
		&& send(frame);
}

bool send_RESP(float EoI_ratio, float FR_pm, float VTe_mL, float VM_Lpm, float Pcrete_cmH2O, float Pplat_cmH2O, float PEP_cmH2O)
{
    char frame[MAX_FRAME+1] = "";
    return snprintf(frame, MAX_FRAME, "RESP IE___:%02d FR___:%02d VTe__:%03d PCRET:%02d VM___:%c%02d PPLAT:%02d PEP__:%02d" CS8,
                   (int)(EoI_ratio*10), (int)FR_pm, (int)VTe_mL, (int)Pcrete_cmH2O, sign(VM_Lpm), abs((int)VM_Lpm), (int)Pplat_cmH2O, (int)PEP_cmH2O) > 0
           && send(frame);
}

#define SET_  "SET_ "

#define VT___ "VT___:"
#define FR___ "FR___:"
#define PEP__ "PEP__:"
#define VMAX_ "Vmax_:"
#define EoI__ "EoI__:"
#define TPLAT "Tplat:"
#define VTMIN "VTmin:"
#define PMAX_ "Pmax_:"
#define PMIN_ "Pmin_:"
#define FRMIN "FRmin:"
#define VMMIN "VMmin:"

#define VT____FMT "%03d"
#define FR____FMT "%02d"
#define PEP___FMT "%02d"
#define VMAX__FMT "%02d"
#define EoI___FMT "%02d"
#define TPLAT_FMT "%04d"
#define VTMIN_FMT "%04d"
#define PMAX__FMT "%03d"
#define PMIN__FMT "%03d"
#define FRMIN_FMT "%02d"
#define VMMIN_FMT "%04d"

bool send_SET(const char* field, const char* fmt, int value)
{
    char frame[MAX_FRAME] = "";
    char val  [MAX_FRAME] = "";
    return snprintf(val, MAX_FRAME, fmt, value) > 0
           && snprintf(frame, MAX_FRAME, SET_ "%s%s" CS8, field, val) > 0
           && send(frame);
}

#define INIT "INIT "

bool send_INIT(const char* information)
{
    char frame[MAX_FRAME+1] = "";
    return snprintf(frame, MAX_FRAME, INIT "%s" CS8, information) > 0
           && send(frame)
           && send_SET(VT___, VT____FMT, setting_VT_mL     )
           && send_SET(FR___, FR____FMT, setting_FR_pm     )
           && send_SET(PEP__, PEP___FMT, setting_PEP_cmH2O )
           && send_SET(VMAX_, VMAX__FMT, setting_Vmax_Lpm  )
           && send_SET(EoI__, EoI___FMT, setting_EoI_ratio )
           && send_SET(TPLAT, TPLAT_FMT, get_setting_Tplat_ms())
           && send_SET(VTMIN, VTMIN_FMT, setting_VTmin_mL  )
           && send_SET(PMAX_, PMAX__FMT, setting_Pmax_cmH2O)
           && send_SET(PMIN_, PMIN__FMT, setting_Pmin_cmH2O)
           && send_SET(FRMIN, FRMIN_FMT, setting_FRmin_pm  )
           && send_SET(VMMIN, VMMIN_FMT, setting_VMmin_Lm  );
}

bool process(char const ** ppf, const char* field, const char* fmt, uint16_t* value)
{
    if (strncmp(*ppf, field, strlen(field))!=0) return false;

    *ppf += strlen(field);
    return sscanf(*ppf, fmt, value)==1 && send_SET(field, fmt, *value);
}

#define PINS "PINS "
#define PEXP "PEXP "
#define PBIP "PBIP "

#define P_FMT "%05d"

#define SRST "SRST "

const char* payload(const char* frame, const char* prefix)
{
    int prefix_length = strlen(prefix);
    return strncmp(frame, prefix, prefix_length)!=0 ? NULL : frame+prefix_length;
}

void send_and_recv()
{
    static bool initSent = true; // even if not received

    // TODO Asynchronous send

    char frame[MAX_FRAME+1] = "";
    while (true) {
        char *pf = frame;
        for (int c = EOF; (c = recv_ihm())!='\n'; pf++) { // read until \n to make sure frame starts at a new line
            if (c == EOF) {
                return;
            }
            else if (c<' ' && c!='\t' || 126<c ) { // filter out frames with C0, C1 characters but \t
                pf = frame+MAX_FRAME;
            }
            else if (pf<(frame+MAX_FRAME)) {
                *pf = c;
            }
        }
        if ((frame+MAX_FRAME)<=pf) continue;
        *(++pf)='\0';

        char* pcs8 = strstr(frame, CS8);
        if (!pcs8) continue;

        unsigned int cs8 = 0;
        if (sscanf(pcs8, CS8 CS8_VALUE, &cs8)!=1) continue;

        *(pcs8+strlen(CS8))='\0';
        unsigned char cs8computed = checksum8(frame);
        if (cs8!=cs8computed) continue;

        uint16_t ignored_Tplat_ms;
        const char *pl = NULL;
        if ((pl = payload(frame, INIT)) || !initSent) {
            initSent = send_INIT(get_init_str());
        }
        else if ((pl = payload(frame, SET_))) {
            process(&pl, VT___, VT____FMT, &setting_VT_mL     ) ||
            process(&pl, FR___, FR____FMT, &setting_FR_pm     ) ||
            process(&pl, PEP__, PEP___FMT, &setting_PEP_cmH2O ) ||
            process(&pl, VMAX_, VMAX__FMT, &setting_Vmax_Lpm  ) ||
            process(&pl, EoI__, EoI___FMT, &setting_EoI_ratio ) ||
            process(&pl, TPLAT, TPLAT_FMT, &ignored_Tplat_ms  ) ||
            process(&pl, VTMIN, VTMIN_FMT, &setting_VTmin_mL  ) ||
            process(&pl, PMAX_, PMAX__FMT, &setting_Pmax_cmH2O) ||
            process(&pl, PMIN_, PMIN__FMT, &setting_Pmin_cmH2O) ||
            process(&pl, FRMIN, FRMIN_FMT, &setting_FRmin_pm  ) ||
            process(&pl, VMMIN, VMMIN_FMT, &setting_VMmin_Lm  );
        }
        else if ((pl = payload(frame, PINS))) {
            uint16_t pause_ms = 0;
            process(&pl, "", P_FMT, &pause_ms);
            command_Tpins_ms = get_time_ms()+pause_ms;
        }
        else if ((pl = payload(frame, PEXP))) {
            uint16_t pause_ms = 0;
            process(&pl, "", P_FMT, &pause_ms);
            command_Tpexp_ms = get_time_ms()+pause_ms;
        }
        else if ((pl = payload(frame, PBIP))) {
            uint16_t pause_ms = 0;
            process(&pl, "", P_FMT, &pause_ms);
            command_Tpbip_ms = get_time_ms()+pause_ms;
        }
        else if ((pl = payload(frame, SRST))) {
            command_soft_reset = true;
        }
        else {
            DEBUG_PRINTF("%s", frame); // Unknown
        }
#ifndef WIN32
        vPortYield();
#endif
    }
}
