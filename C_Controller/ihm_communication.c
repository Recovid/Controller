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
#include "lowlevel.h"

// ------------------------------------------------------------------------------------------------
//! Settings

//! \warning Global settings MUST use types that can be atomically read/write in a threadsafe way on STM32
//! \warning Non volatile memory may be limited to uint16_t by the corresponding driver

PRIVATE uint16_t setting_FR_pm      =  18;
PRIVATE uint16_t setting_VT_mL      = 300;
PRIVATE uint16_t setting_PEP_cmH2O  =   5;
PRIVATE uint16_t setting_Vmax_Lpm   =  60;
PRIVATE uint16_t setting_EoI_ratio  =   20;

PRIVATE uint16_t setting_Pmax_cmH2O =  60;
PRIVATE uint16_t setting_Pmin_cmH2O =  20;
PRIVATE uint16_t setting_VTmin_mL   = 150;
PRIVATE uint16_t setting_FRmin_pm   =  10;
PRIVATE uint16_t setting_VMmin_Lpm  =   3;

const int setting_PEPmax_cmH2O =  2;
const int setting_PEPmin_cmH2O = -2;

float get_setting_FR_pm    () { return setting_FR_pm    ; }
float get_setting_VT_mL    () { return setting_VT_mL    ; }
float get_setting_PEP_cmH2O() { return setting_PEP_cmH2O; }
float get_setting_Vmax_Lpm () { return setting_Vmax_Lpm ; }
float get_setting_EoI_ratio() { return setting_EoI_ratio / 10.f; }
float get_setting_IoE_ratio() { return 1.f/get_setting_EoI_ratio(); }

uint16_t get_setting_Tplat_ms  ()
{
    const float T_ms  = 60000.f / get_setting_FR_pm();
    const float Ti_ms = get_setting_VT_mL() / (get_setting_Vmax_Lpm() / 60.f); // FIXME Lpm/60=Lps and 1/Lps=s not ms
    const float Tplat_ms= (T_ms/(1.f+(1.f/get_setting_EoI_ratio()))) - Ti_ms;  // Check Tplat=(T-Te)-Ti => T-Te=T/(1+E/I) ???
    const float Te_ms = (Ti_ms+Tplat_ms) / get_setting_EoI_ratio();            // FIXME Te=(Ti+Tplat)/(E/I)=1/2 if user enters E/I=2
    return T_ms - (Ti_ms+Te_ms);
}

float get_setting_Pmax_cmH2O  () { return setting_Pmax_cmH2O  ; }
float get_setting_Pmin_cmH2O  () { return setting_Pmin_cmH2O  ; }
float get_setting_VTmin_mL    () { return setting_VTmin_mL    ; }
float get_setting_FRmin_pm    () { return setting_FRmin_pm    ; }
float get_setting_VMmin_Lm    () { return setting_VMmin_Lpm   ; }

float get_setting_PEPmax_cmH2O() { return setting_PEPmax_cmH2O; }
float get_setting_PEPmin_cmH2O() { return setting_PEPmin_cmH2O; }

// ------------------------------------------------------------------------------------------------
//! Commands

//! Global settings MUST use types that can be atomically read/write in a threadsafe way on STM32

PRIVATE uint32_t command_Tpins_ms = 0;
PRIVATE uint32_t command_Tpexp_ms = 0;
PRIVATE uint32_t command_Tpbip_ms = 0;

bool command_soft_reset = false;

uint32_t get_command_Tpins_ms() { return command_Tpins_ms; }
uint32_t get_command_Tpexp_ms() { return command_Tpexp_ms; }
uint32_t get_command_Tpbip_ms() { return command_Tpbip_ms; }

bool is_soft_reset_asked () { return command_soft_reset; }

// ------------------------------------------------------------------------------------------------
//! Communication

#define MAX_FRAME 100

char sign(int i) { return i<0 ? '-' : '+'; }

#define CS8 "\tCS8:"
#define CS8_VALUE "..\n"

unsigned char checksum8(const char* s)
{
    size_t len = strlen(s) - 3; // strlen(CS8_VALUE)
    unsigned long cs8 = 0ul;
    for (size_t i = 0; i < len; ++i)
    {
        cs8 += s[i];
    }
    return (unsigned char)(cs8 % 256);
}

bool send(const char* frame)
{
    return send_ihm(frame) > 0;
}

void replace_int_with_padding(char* frame, int value, int size, int base)
{
    static const char baseChar[] = "0123456789ABCDEF";
    value = abs(value);
    char* curr = strchr(frame, '.');
    while (size > 0)
    {
        curr[size - 1] = baseChar[value % base];
        value /= base;
        --size;
    }
}

bool send_DATA(float P_cmH2O, float VolM_Lpm, float Vol_mL, float Pplat_cmH2O, float PEP_cmH2O)
{
    static const char dataFrame[] = "DATA msec_:...... Vol__:.... Deb__:.... Paw__:...." CS8 CS8_VALUE;

    char frame[sizeof(dataFrame)];
    strcpy(frame, dataFrame);

    if ( !(TEST_RANGE(    0, Vol_mL  , 10000)
        && TEST_RANGE(-1000, VolM_Lpm,  1000)
        && TEST_RANGE(-1000, P_cmH2O ,  1000))
    )
    {
        return false;
    }

    replace_int_with_padding(frame, get_time_ms() % 1000000l, 6, 10);
    replace_int_with_padding(frame, Vol_mL, 4, 10);
    *strchr(frame, '.') = sign(VolM_Lpm);
    replace_int_with_padding(frame, VolM_Lpm, 3, 10);
    *strchr(frame, '.') = sign(P_cmH2O);
    replace_int_with_padding(frame, P_cmH2O, 3, 10);

    replace_int_with_padding(frame, checksum8(frame), 2, 16);

    return send(frame);
}

bool send_RESP(float EoI_ratio, float FR_pm, float VTe_mL, float VM_Lpm, float Pcrete_cmH2O, float Pplat_cmH2O, float PEP_cmH2O)
{
    static const char respFrame[] = "RESP IE___:.. FR___:.. VTe__:... PCRET:.. VM___:... PPLAT:.. PEP__:.." CS8 CS8_VALUE;
    char tmp[8];
    char frame[sizeof(respFrame)];
    strcpy(frame, respFrame);

    if ( !(TEST_RANGE(   2, EoI_ratio  ,    6)
        && TEST_RANGE(   0, FR_pm      ,  100)
        && TEST_RANGE(   0, VTe_mL     , 1000)
        && TEST_RANGE(-100, VM_Lpm     ,  100)
        && TEST_RANGE(   0, Pplat_cmH2O,  100)
        && TEST_RANGE(   0, PEP_cmH2O  ,  100))
    )
    {
        return false;
    }
    replace_int_with_padding(frame, EoI_ratio * 10, 2, 10);
    replace_int_with_padding(frame, FR_pm, 2, 10);
    replace_int_with_padding(frame, VTe_mL, 3, 10);
    replace_int_with_padding(frame, Pcrete_cmH2O, 2, 10);
    *strchr(frame, '.') = sign(VM_Lpm);
    replace_int_with_padding(frame, VM_Lpm, 2, 10);
    replace_int_with_padding(frame, Pplat_cmH2O, 2, 10);
    replace_int_with_padding(frame, PEP_cmH2O, 2, 10);
    replace_int_with_padding(frame, checksum8(frame), 2, 16);

    return send(frame);
}

#define SET_  "SET_ "

#define VT___ "VT___:"
#define FR___ "FR___:"
#define PEP__ "PEP__:"
#define VMAX_ "FLOW_:"
#define EoI__ "IE___:"
#define TPLAT "Tplat:"
#define VTMIN "VTmin:"
#define PMAX_ "Pmax_:"
#define PMIN_ "Pmin_:"
#define FRMIN "FRmin:"
#define VMMIN "VMmin:"

#define VT____FMT 3
#define FR____FMT 2
#define PEP___FMT 2
#define VMAX__FMT 2
#define EoI___FMT 2
#define TPLAT_FMT 4
#define VTMIN_FMT 4
#define PMAX__FMT 3
#define PMIN__FMT 3
#define FRMIN_FMT 2
#define VMMIN_FMT 4

bool send_SET(const char* field, int size, int value)
{
    if (size > 4 || strlen(field) != 6)
        return false;
    char frame[sizeof(SET_) + 6 + 4 + sizeof(CS8) + sizeof(CS8_VALUE)];
    char* curr = frame;
    strncpy(curr, SET_, sizeof(SET_) - 1);
    curr += sizeof(SET_) - 1;
    for (int i = 0; i < 6 + size; ++i)
        curr[i] = '.';
    curr += 6 + size;
    strncpy(curr, CS8, sizeof(CS8) - 1);
    curr += sizeof(CS8) - 1;
    strncpy(curr, CS8_VALUE, sizeof(CS8_VALUE) - 1);
    curr += sizeof(CS8_VALUE) - 1;
    *curr = '\0';

    strncpy(strchr(frame, '.'), field, 6);
    replace_int_with_padding(frame, value, size, 10);

    replace_int_with_padding(frame, checksum8(frame), 2, 16);

    return send(frame);
}

#define INIT "INIT "

bool send_INIT(const char* information)
{
    char frame[MAX_FRAME+1] = "";
    char* curr = frame;
    strncpy(curr, INIT, sizeof(INIT) - 1);
    curr += sizeof(INIT) - 1;
    strncpy(curr, information, strlen(information));
    curr += strlen(information);
    strncpy(curr, CS8, sizeof(CS8) - 1);
    curr += sizeof(CS8) - 1;
    strncpy(curr, CS8_VALUE, sizeof(CS8_VALUE) - 1);
    curr += sizeof(CS8_VALUE) - 1;
    *curr = '\0';

    replace_int_with_padding(frame, checksum8(frame), 2, 16);

    return send(frame)
        && send_SET(VT___, VT____FMT, setting_VT_mL         )
        && send_SET(FR___, FR____FMT, setting_FR_pm         )
        && send_SET(PEP__, PEP___FMT, setting_PEP_cmH2O     )
        && send_SET(VMAX_, VMAX__FMT, setting_Vmax_Lpm      )
        && send_SET(EoI__, EoI___FMT, setting_EoI_ratio     )
        && send_SET(TPLAT, TPLAT_FMT, get_setting_Tplat_ms())
        && send_SET(VTMIN, VTMIN_FMT, setting_VTmin_mL      )
        && send_SET(PMAX_, PMAX__FMT, setting_Pmax_cmH2O    )
        && send_SET(PMIN_, PMIN__FMT, setting_Pmin_cmH2O    )
        && send_SET(FRMIN, FRMIN_FMT, setting_FRmin_pm      )
        && send_SET(VMMIN, VMMIN_FMT, setting_VMmin_Lpm     );
}

bool process(char const** ppf, const char* field, int size, uint16_t* value)
{
    if (strncmp(*ppf, field, strlen(field))!=0) return false;

    *ppf += strlen(field);
    *value = atoi(*ppf);
    return send_SET(field, size, *value);
}

#define PINS "PINS "
#define PEXP "PEXP "
#define PBIP "PBIP "

#define P_FMT 5

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
        for (int c = EOF; (c = recv_ihm()) != '\n'; pf++) { // read until \n to make sure frame starts at a new line
            if (c == EOF) {
                return;
            }
            else if ((c < ' ' && c != '\t') || (c > '~')) {
                // filter out frames with C0, C1 characters but \t
                pf = frame + MAX_FRAME;
            }
            else if (pf<(frame+MAX_FRAME)) {
                *pf = c;
            }
        }
        if ((frame+MAX_FRAME)<=pf) continue;
        *(pf++)='\n';
        *(pf++)='\0';

        char* pcs8 = strstr(frame, CS8);
        if (!pcs8) continue;

        unsigned int cs8 = 0;
        cs8 = strtol(pcs8 + sizeof(CS8) - 1, 0, 16);

        unsigned char cs8computed = checksum8(frame);
        if (cs8!=cs8computed) continue;
        *(pcs8 + strlen(CS8) - 1) = '\0';

        uint16_t ignored_Tplat_ms;
        const char *pl = NULL;
        if ((pl = payload(frame, INIT)) || !initSent) {
            initSent = send_INIT(get_init_str());
        }
        else if ((pl = payload(frame, SET_))) {
            bool processed =
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
            process(&pl, VMMIN, VMMIN_FMT, &setting_VMmin_Lpm );
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
