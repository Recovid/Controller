#include <stdio.h>
#include <string.h>

#include "controller.h"
#include "controller_settings.h"
#include "ihm_communication.h"
#include "hardware_simulation.h"

#define MAX_FRAME 1000

FILE *in ;
FILE *out;

bool connect()
{
    // TODO Replace with HAL_UART_init
    in  = stdin ;
    out = stdout;
    return true;
}

char sign(int i) { return i<0 ? '-' : '+'; }

#define CS8 "\tCS8:"
#define CS8_VALUE "%02X\n"

unsigned char checksum8(const char* s)
{
    unsigned long cs8 = 0ul;
    for (; s && *s!='\0' ; s++) {
        cs8 += *s;
    }
    return (unsigned char)(cs8 % 0xFF);
}

bool send(const char* frame)
{
    return fprintf(out, "%s" CS8_VALUE, frame, checksum8(frame)) > 0;
}

bool send_DATA(int P, int VolM, int Vol, int Pplat, int PEP)
{
    char frame[MAX_FRAME+1] = "";
    return sprintf(frame, "DATA msec_:%06d Vol__:%04d Deb__:%c%03d Paw__:%c%03d" CS8,
                   (int)(getTimeMs() % 1000000l), Vol, sign(VolM), VolM, sign(P), P) > 0
           && send(frame);
}

bool send_RESP(int IE, int FR, int VTe, int VM, int Pcrete, int Pplat, int PEP)
{
    char frame[MAX_FRAME+1] = "";
    return sprintf(frame, "RESP IE___:%02d FR___:%02d VTe__:%03d PCRET:%02d VM___:%c%02d PPLAT:%02d PEP__:%02d" CS8,
                   IE, FR, VTe, Pcrete, sign(VM), VM, Pplat, PEP) > 0
           && send(frame);
}

#define SET_  "SET_ "

#define VT___ "VT___:"
#define FR___ "FR___:"
#define PEP__ "PEP__:"
#define VMAX_ "Vmax_:"
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
#define TPLAT_FMT "%04d"
#define VTMIN_FMT "%04d"
#define PMAX__FMT "%03d"
#define PMIN__FMT "%03d"
#define FRMIN_FMT "%02d"
#define VMMIN_FMT "%04d"

bool send_SET(const char* field, const char* fmt, int value)
{
    char frame[100] = "";
    char val[100] = "";
    return sprintf(val, fmt, value) > 0
           && sprintf(frame, SET_ "%s%s" CS8, field, val) > 0
           && send(frame);
}

#define INIT "INIT "
bool send_INIT(const char* information)
{
    char frame[MAX_FRAME+1] = "";
    return sprintf(frame, INIT "%s" CS8, init_str) > 0
           && send(frame)
           && send_SET(VT___, VT____FMT, VT_mL     )
           && send_SET(FR___, FR____FMT, FR_pm     )
           && send_SET(PEP__, PEP___FMT, PEP_cmH2O )
           && send_SET(VMAX_, VMAX__FMT, Vmax_Lpm  )
           && send_SET(TPLAT, TPLAT_FMT, Tplat_ms  )
           && send_SET(VTMIN, VTMIN_FMT, VTmin_mL  )
           && send_SET(PMAX_, PMAX__FMT, Pmax_cmH2O)
           && send_SET(PMIN_, PMIN__FMT, Pmin_cmH2O)
           && send_SET(FRMIN, FRMIN_FMT, FRmin_pm  )
           && send_SET(VMMIN, VMMIN_FMT, VMmin_Lm  );
}

bool process(const char** ppf, const char* field, const char* fmt, int* value)
{
    if (strncmp(*ppf, field, strlen(field))!=0) return false;

    *ppf += strlen(field);
    return sscanf(*ppf, fmt, value)==1 && send_SET(field, fmt, *value);
}

//! Read messages from IHM and process them including:
//! - update controller settings and acknowledge modified value
//! - answer to INIT
bool read()
{
    static bool initSent = true; // even if not received

    char frame[MAX_FRAME+1] = "";
    while (true) {
        char *pf = frame;
        for (char c = EOF; (c = fgetc(in))!='\n'; pf++) { // read until \n to make sure frame starts at a new line
            if (c == EOF) {
                return connect();
            }
            else if (c<' ' && c!='\t') { // filter out frames with C0, C1 characters but \t
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

        unsigned char cs8 = 0;
        if (sscanf(pcs8, CS8 CS8_VALUE, &cs8)!=1) continue;

        *(pcs8+strlen(CS8))='\0';
        unsigned char cs8computed = checksum8(frame);
        if (cs8!=cs8computed) continue;

        if (strncmp(frame, INIT, strlen(INIT))==0 || !initSent) {
            pf = frame + strlen(INIT);
            initSent = send_INIT(init_str);
        }
        else if (strncmp(frame, SET_, strlen(SET_))==0) {
            pf = frame + strlen(SET_);
            process(&pf, VT___, VT____FMT, &VT_mL     ) ||
            process(&pf, FR___, FR____FMT, &FR_pm     ) ||
            process(&pf, PEP__, PEP___FMT, &PEP_cmH2O ) ||
            process(&pf, VMAX_, VMAX__FMT, &Vmax_Lpm  ) ||
            process(&pf, TPLAT, TPLAT_FMT, &Tplat_ms  ) ||
            process(&pf, VTMIN, VTMIN_FMT, &VTmin_mL  ) ||
            process(&pf, PMAX_, PMAX__FMT, &Pmax_cmH2O) ||
            process(&pf, PMIN_, PMIN__FMT, &Pmin_cmH2O) ||
            process(&pf, FRMIN, FRMIN_FMT, &FRmin_pm  ) ||
            process(&pf, VMMIN, VMMIN_FMT, &VMmin_Lm  );
        }

        // TODO parse other frames
    }
}
