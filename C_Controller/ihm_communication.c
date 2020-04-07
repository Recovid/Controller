#include <FreeRTOS.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "controller.h"
#include "controller_settings.h"
#include "ihm_communication.h"
#include "hardware_simulation.h"

#define MAX_FRAME 1000

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
    return sprintf(checked_frame, "%s" CS8_VALUE, frame, checksum8(frame)) > 0
           && send_ihm(checked_frame)>0;
}

bool send_DATA(float P, float VolM, float Vol, float Pplat, float PEP)
{
    char frame[MAX_FRAME+1] = "";
    return sprintf(frame, "DATA msec_:%06d Vol__:%04d Deb__:%c%03d Paw__:%c%03d" CS8,
                   (int)(get_time_ms() % 1000000l), (int)Vol, sign(VolM), abs((int)VolM), sign(P), abs((int)P)) > 0
           && send(frame);
}

bool send_RESP(float IE, float FR, float VTe, float VM, float Pcrete, float Pplat, float PEP)
{
    char frame[MAX_FRAME+1] = "";

    printf(frame, "RESP IE___:%02d FR___:%02d VTe__:%03d PCRET:%02d VM___:%c%02d PPLAT:%02d PEP__:%02d" CS8,
                   (int)(1./IE), (int)FR, (int)VTe, (int)Pcrete, sign(VM), abs((int)VM), (int)Pplat, (int)PEP);
    return sprintf(frame, "RESP IE___:%02d FR___:%02d VTe__:%03d PCRET:%02d VM___:%c%02d PPLAT:%02d PEP__:%02d" CS8,
                   (int)(1./IE), (int)FR, (int)VTe, (int)Pcrete, sign(VM), abs((int)VM), (int)Pplat, (int)PEP) > 0
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

#define PINS "PINS "
#define PEXP "PEXP "
#define PBIP "PBIP "

#define P_FMT "%05d"

#define SRST "SRST "

char* payload(char* frame, const char* prefix)
{
    int prefix_length = strlen(prefix);
    return strncmp(frame, prefix, prefix_length)!=0 ? NULL : frame+prefix_length;
}

bool send_and_recv()
{
    static bool initSent = true; // even if not received

    // TODO Asynchronous send

    char frame[MAX_FRAME+1] = "";
    while (true) {
        char *pf = frame;
        for (int c = EOF; (c = recv_ihm())!='\n'; pf++) { // read until \n to make sure frame starts at a new line
            if (c == EOF) {
                return true;
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

        unsigned char cs8 = 0;
        if (sscanf(pcs8, CS8 CS8_VALUE, &cs8)!=1) continue;

        *(pcs8+strlen(CS8))='\0';
        unsigned char cs8computed = checksum8(frame);
        if (cs8!=cs8computed) continue;

        if ((pf = payload(frame, INIT)) || !initSent) {
            initSent = send_INIT(init_str);
        }
        else if ((pf = payload(frame, SET_))) {
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
        else if ((pf = payload(frame, PINS))) {
            int pause_ms = 0;
            process(&pf, "", P_FMT, &pause_ms);
            Tpins_ms = get_time_ms()+pause_ms;
        }
        else if ((pf = payload(frame, PEXP))) {
            int pause_ms = 0;
            process(&pf, "", P_FMT, &pause_ms);
            Tpexp_ms = get_time_ms()+pause_ms;
        }
        else if ((pf = payload(frame, PBIP))) {
            int pause_ms = 0;
            process(&pf, "", P_FMT, &pause_ms);
            Tpbip_ms = get_time_ms()+pause_ms;
        }
        else if ((pf = payload(frame, SRST))
                 && soft_reset()) {
            return false;
        }
        else { // Unknown frame
            return true;
        }
		vPortYield();
    }
}
