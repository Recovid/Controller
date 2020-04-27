#include "recovid.h"
#include "protocol.h"
#include "controller.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>


#define ALARM_COUNT 18

#define MAX_FRAME 100

// TODO Not sure what this string is
#define INIT_STR_SIZE 80
char init_str[INIT_STR_SIZE+1] = ""; // leaving space for off by 1 errors in code

const char *get_init_str() { return init_str; }

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
    return uart_send(frame) > 0;
}

void replace_int_with_padding(char* frame, int value, int size, int base)
{
    static const char baseChar[] = "0123456789ABCDEF";
    value = fabs(value);
    char* curr = strchr(frame, '.');
    assert(curr);
    while (size > 0)
    {
        curr[size - 1] = baseChar[value % base];
        value /= base;
        --size;
    }
}

static const char DATA_pattern[] = "DATA msec_:...... Vol__:.... Deb__:.... Paw__:...." CS8 CS8_VALUE;
static char DATA_frame[sizeof(DATA_pattern)];

bool send_DATA(float P_cmH2O, float VolM_Lpm, float Vol_mL)
{
    strncpy(DATA_frame, DATA_pattern, sizeof(DATA_frame));

    /*if (!( CHECK_RANGE(    0, Vol_mL  , 10000)
        && CHECK_RANGE(-1000, VolM_Lpm,  1000)
        && CHECK_RANGE(-1000, P_cmH2O ,  1000)))
    {
        return false;
    }*/

    replace_int_with_padding(DATA_frame, get_time_ms() % 1000000l, 6, 10);
    replace_int_with_padding(DATA_frame, roundf(Vol_mL), 4, 10);
    *strchr(DATA_frame, '.') = sign(VolM_Lpm);
    replace_int_with_padding(DATA_frame, roundf(VolM_Lpm), 3, 10);
    *strchr(DATA_frame, '.') = sign(P_cmH2O);
    replace_int_with_padding(DATA_frame, roundf(P_cmH2O), 3, 10);

    replace_int_with_padding(DATA_frame, checksum8(DATA_frame), 2, 16);

    return send(DATA_frame);
}

static const char DATA_X_pattern[] = "DATA msec_:...... Vol__:.... Deb__:.... Paw__:.... PPLAT:.. PEP__:.." CS8 CS8_VALUE;
static char DATA_X_frame[sizeof(DATA_X_pattern)];

bool send_DATA_X(float P_cmH2O, float VolM_Lpm, float Vol_mL, float Pplat_cmH2O, float PEP_cmH2O)
{
    strncpy(DATA_X_frame, DATA_X_pattern, sizeof(DATA_X_frame));

/*   if (!( CHECK_RANGE(    0, Vol_mL     , 10000)
        && CHECK_RANGE(-1000, VolM_Lpm   ,  1000)
        && CHECK_RANGE(-1000, P_cmH2O    ,  1000)
        && CHECK_RANGE(    0, Pplat_cmH2O,   100)
        && CHECK_RANGE(    0, PEP_cmH2O  ,   100))) {
        return false;
    }*/

    replace_int_with_padding(DATA_X_frame, get_time_ms() % 1000000l, 6, 10);
    replace_int_with_padding(DATA_X_frame, roundf(Vol_mL)     , 4, 10);
    *strchr(DATA_X_frame, '.') = sign(VolM_Lpm);
    replace_int_with_padding(DATA_X_frame, roundf(VolM_Lpm)   , 3, 10);
    *strchr(DATA_X_frame, '.') = sign(P_cmH2O);
    replace_int_with_padding(DATA_X_frame, roundf(P_cmH2O)    , 3, 10);
    replace_int_with_padding(DATA_X_frame, roundf(Pplat_cmH2O), 2, 10);
    replace_int_with_padding(DATA_X_frame, roundf(PEP_cmH2O)  , 2, 10);

    replace_int_with_padding(DATA_X_frame, checksum8(DATA_X_frame), 2, 16);

    return send(DATA_X_frame);
}

static const char RESP_pattern[] = "RESP IE___:.. FR___:.. VTe__:... PCRET:.. VM___:... PPLAT:.. PEP__:.." CS8 CS8_VALUE;
static char RESP_frame[sizeof(RESP_pattern)];

bool send_RESP(float EoI_ratio, float FR_pm, float VTe_mL, float VM_Lpm, float Pcrete_cmH2O, float Pplat_cmH2O, float PEP_cmH2O)
{
    strncpy(RESP_frame, RESP_pattern, sizeof(RESP_frame));

    /*if (!( CHECK_RANGE(   0, EoI_ratio*10,  100)
        && CHECK_RANGE(   0, FR_pm       ,  100)
        && CHECK_RANGE(   0, VTe_mL      , 1000)
        && CHECK_RANGE(-100, VM_Lpm      ,  100)
        && CHECK_RANGE(   0, Pplat_cmH2O ,  100)
        && CHECK_RANGE(   0, PEP_cmH2O   ,  100)))
    {
        return false;
    }*/
    replace_int_with_padding(RESP_frame, roundf(EoI_ratio*10)    , 2, 10);
    replace_int_with_padding(RESP_frame, roundf(FR_pm)           , 2, 10);
    replace_int_with_padding(RESP_frame, roundf(VTe_mL)          , 3, 10);
    replace_int_with_padding(RESP_frame, roundf(Pcrete_cmH2O)    , 2, 10);
    *strchr(RESP_frame, '.') = sign(VM_Lpm);
    replace_int_with_padding(RESP_frame, roundf(VM_Lpm)          , 2, 10);
    replace_int_with_padding(RESP_frame, roundf(Pplat_cmH2O)     , 2, 10);
    replace_int_with_padding(RESP_frame, roundf(PEP_cmH2O)       , 2, 10);
    replace_int_with_padding(RESP_frame, checksum8(RESP_frame), 2, 16);

    return send(RESP_frame);
}

#define SET_  "SET_ "

#define VT___ "VT___:"
#define FR___ "FR___:"
#define PEP__ "PEP__:"
#define VMAX_ "FLOW_:"
#define EoI__ "IE___:"
#define TPLAT "Tplat:"
#define VTMIN "VTmin:"
#define VTMAX "VTmax:"
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
#define VTMAX_FMT 4
#define PMAX__FMT 3
#define PMIN__FMT 3
#define FRMIN_FMT 2
#define VMMIN_FMT 4

static char SET_frame[sizeof(SET_) + 6 + 4 + sizeof(CS8) + sizeof(CS8_VALUE)];

bool send_SET(const char* field, int size, int value)
{
    if (size > 4 || strlen(field) != 6)
        return false;

    char* curr = SET_frame;
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

    memcpy(strchr(SET_frame, '.'), field, 6);
    replace_int_with_padding(SET_frame, value, size, 10);

    replace_int_with_padding(SET_frame, checksum8(SET_frame), 2, 16);

    return send(SET_frame);
}

#define INIT "INIT "

static char INIT_frame[MAX_FRAME+1] = "";

bool send_INIT(const char* information)
{
    char* curr = INIT_frame;
    strncpy(curr, INIT, sizeof(INIT) - 1);
    curr += sizeof(INIT) - 1;
    memcpy (curr, information, strlen(information));
    curr += strlen(information);
    strncpy(curr, CS8, sizeof(CS8) - 1);
    curr += sizeof(CS8) - 1;
    strncpy(curr, CS8_VALUE, sizeof(CS8_VALUE) - 1);
    curr += sizeof(CS8_VALUE) - 1;
    *curr = '\0';

    replace_int_with_padding(INIT_frame, checksum8(INIT_frame), 2, 16);

    return send(INIT_frame)
        && send_SET(VT___, VT____FMT, get_setting_VT_mL()         )
        && send_SET(FR___, FR____FMT, get_setting_FR_pm()         )
        && send_SET(PEP__, PEP___FMT, get_setting_PEP_cmH2O()     )
        && send_SET(VMAX_, VMAX__FMT, get_setting_Vmax_Lpm()      )
        && send_SET(EoI__, EoI___FMT, get_setting_EoI_ratio()*10  )
        && send_SET(TPLAT, TPLAT_FMT, get_setting_Tplat_ms()      )
        && send_SET(VTMIN, VTMIN_FMT, get_setting_VTmin_mL()      )
        && send_SET(VTMAX, VTMAX_FMT, get_setting_VTmax_mL()      )
        && send_SET(PMAX_, PMAX__FMT, get_setting_Pmax_cmH2O()    )
        && send_SET(PMIN_, PMIN__FMT, get_setting_Pmin_cmH2O()    )
        && send_SET(FRMIN, FRMIN_FMT, get_setting_FRmin_pm()      )
        && send_SET(VMMIN, VMMIN_FMT, get_setting_VMmin_Lm()     );
}

#define ALRM "ALRM"

static const char* ALARM_CODES[] = {
    "PMAX",
    "PMIN",
    "VT_MIN",
    "VT_MAX",
    "FR",
    "VM_MIN",
    "PEP_MAX",
    "PEP_MIN",
    "BATT_A",
    "BATT_B",
    "BATT_C",
    "BATT_D",
    "BATT_E",
    "FAILSAFE",
    "CPU_LOST",
    "P_KO",
    "IO_MUTE",
    "SENSOR_FAIL"
};

bool send_ALRM(uint32_t alarms)
{
    char frame[MAX_FRAME+1] = "";
    char* curr = frame;
    strncpy(curr, ALRM, sizeof(ALRM) - 1);
    curr += sizeof(ALRM) - 1;

    for (int i = 0; i < ALARM_COUNT; ++i) {
        if (alarms & (1 << i)) {
            int n = strlen(ALARM_CODES[i]);
            if (curr + n + 1 > frame + MAX_FRAME - sizeof(CS8) - 1 - sizeof(CS8_VALUE) - 1) {
                // frame is full, skip remaining alarms
                break;
            }
            *curr++ = ' ';
            strncpy(curr, ALARM_CODES[i], n);
            curr += n;
        }
    }

    strncpy(curr, CS8, sizeof(CS8) - 1);
    curr += sizeof(CS8) - 1;
    strncpy(curr, CS8_VALUE, sizeof(CS8_VALUE) - 1);
    curr += sizeof(CS8_VALUE) - 1;
    *curr = '\0';

    replace_int_with_padding(frame, checksum8(frame), 2, 16);

    return send(frame);
}

bool process(char const** ppf, int index, const char* field, int size, float(*setter)(float))
{
    if (strncmp(*ppf, field, strlen(field))!=0) return false;

    *ppf += strlen(field);
    uint16_t desired = atoi(*ppf);
    uint16_t value = setter==NULL? desired: (*setter)(desired);
    return send_SET(field, size, value);
}

bool process_uint16(char const** ppf, int index, const char* field, int size, uint16_t(*setter)(uint16_t))
{
    if (strncmp(*ppf, field, strlen(field))!=0) return false;

    *ppf += strlen(field);
    uint16_t desired = atoi(*ppf);
    uint16_t value = setter==NULL? desired: (*setter)(desired);
    return send_SET(field, size, value);
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

    // TODO Asynchronous send ??

    char frame[MAX_FRAME+1] = "";
    while (true) {
        char *pf = frame;
        for (int c = EOF; (c = uart_recv()) != '\n'; pf++) { // read until \n to make sure frame starts at a new line
            if (c == EOF) {
                return;
            }
            else if ((c<' ' && c!='\t') || 126<c ) { // filter out frames with C0, C1 characters but \t
                pf = frame+MAX_FRAME;
            }
            else if (pf<(frame+MAX_FRAME)) {
                *pf = c;
            }
            dbg_printf("%2X ",c);
        }
        if ((frame+MAX_FRAME)<=pf) continue;
        *(pf++)='\n';
        *(pf++)='\0';
        dbg_printf(frame);
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
            bool process_result =
                process(&pl,  0, VT___, VT____FMT, set_setting_VT_mL) ||
                process(&pl,  1, FR___, FR____FMT, set_setting_FR_pm) ||
                process(&pl,  2, VMAX_, VMAX__FMT, set_setting_Vmax_Lpm) ||
                process(&pl,  3, EoI__, EoI___FMT, set_setting_EoI_ratio_x10) ||
                process(&pl,  4, PEP__, PEP___FMT, set_setting_PEP_cmH2O) ||
                process(&pl, -1, TPLAT, TPLAT_FMT, NULL) ||
                process(&pl,  5, VTMIN, VTMIN_FMT, set_setting_VTmin_mL) ||
                process(&pl,  6, VTMAX, VTMAX_FMT, set_setting_VTmax_mL) ||
                process(&pl,  7, PMAX_, PMAX__FMT, set_setting_Pmax_cmH2O) ||
                process(&pl,  8, PMIN_, PMIN__FMT, set_setting_Pmin_cmH2O) ||
                process(&pl,  9, FRMIN, FRMIN_FMT, set_setting_FRmin_pm) ||
                process(&pl, 10, VMMIN, VMMIN_FMT, set_setting_VMmin_Lpm);
            UNUSED(process_result)
        }
        else if ((pl = payload(frame, PINS))) {
            process_uint16(&pl, -1, "", P_FMT, set_command_Tpins_ms);
        }
        else if ((pl = payload(frame, PEXP))) {
            process_uint16(&pl, -1, "", P_FMT, set_command_Tpexp_ms);
        }
        else if ((pl = payload(frame, PBIP))) {
            process_uint16(&pl, -1, "", P_FMT, set_command_Tpbip_ms);
        }
        else if ((pl = payload(frame, SRST))) {
            set_command_soft_reset();
        }
        else {
            DEBUG_PRINTF("%s", frame); // Unknown
        }
    }
}

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

#pragma GCC diagnostic ignored "-Wtype-limits"

bool PRINT(test_non_default_settings)
    setting_FR_pm         =  30;
    setting_VT_mL         = 300;
    setting_Vmax_Lpm      =  30;
    setting_EoI_ratio_x10 =  10;
    return
        TEST_FLT_EQUALS(  30.f, get_setting_FR_pm       ()) &&
        TEST_EQUALS    (2000  , get_setting_T_ms        ()) &&
        TEST_FLT_EQUALS( 300.f, get_setting_VT_mL       ()) &&
        TEST_FLT_EQUALS(  30.f, get_setting_Vmax_Lpm    ()) &&
        TEST_EQUALS    ( 600  , get_setting_Tinsu_ms    ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_IoE_ratio   ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_EoI_ratio   ()) &&
        TEST_EQUALS    (1000  , get_setting_Texp_ms     ()) &&
        TEST_EQUALS    (1000  , get_setting_Tinspi_ms   ()) &&
        TEST_EQUALS    ( 400  , get_setting_Tplat_ms    ()) &&
        true;
}

bool PRINT(test_checked_EoI)
    setting_FR_pm         =  30;
    setting_VT_mL         = 300;
    setting_Vmax_Lpm      =  30;
    setting_EoI_ratio_x10 = checked_EoI_ratio_x10(30);
    return
        TEST_FLT_EQUALS(  30.f , get_setting_FR_pm    ()) &&
        TEST_EQUALS    (2000   , get_setting_T_ms     ()) &&
        TEST_FLT_EQUALS( 300.f , get_setting_VT_mL    ()) &&
        TEST_FLT_EQUALS(  30.f , get_setting_Vmax_Lpm ()) &&
        TEST_EQUALS    ( 600   , get_setting_Tinsu_ms ()) &&
        TEST_FLT_EQUALS(   0.4f, get_setting_IoE_ratio()) &&
        TEST_FLT_EQUALS(   2.3f, get_setting_EoI_ratio()) &&
        TEST_RANGE     (1390   , get_setting_Texp_ms  (), 1400) &&
        TEST_RANGE     ( 600   , get_setting_Tinspi_ms(),  610) &&
        TEST_RANGE     (   0   , get_setting_Tplat_ms (),   10) &&
        true;
}

bool PRINT(test_checked_VM)
    setting_FR_pm         =  30;
    setting_VT_mL         = 600;
    setting_EoI_ratio_x10 =  10;
    setting_Vmax_Lpm      = checked_Vmax_Lpm(30);
    return
        TEST_FLT_EQUALS(  30.f, get_setting_FR_pm       ()) &&
        TEST_EQUALS    (2000  , get_setting_T_ms        ()) &&
        TEST_FLT_EQUALS( 600.f, get_setting_VT_mL       ()) &&
        TEST_FLT_EQUALS(  36.f, get_setting_Vmax_Lpm    ()) &&
        TEST_RANGE     ( 999  , get_setting_Tinsu_ms    (), 1000) && // due to Vmax rounding to floor
        TEST_FLT_EQUALS(   1.f, get_setting_IoE_ratio   ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_EoI_ratio   ()) &&
        TEST_EQUALS    (1000  , get_setting_Texp_ms     ()) &&
        TEST_EQUALS    (1000  , get_setting_Tinspi_ms   ()) &&
        TEST_RANGE     (   0.f, get_setting_Tplat_ms    (), 1.f ) && // due to Vmax rounding to floor
        true;
}

bool PRINT(test_checked_VT)
    setting_FR_pm         = 30;
    setting_Vmax_Lpm      = 30;
    setting_EoI_ratio_x10 = 10;
    setting_VT_mL         = checked_VT_mL(600);
    return
        TEST_FLT_EQUALS(  30.f, get_setting_FR_pm       ()) &&
        TEST_EQUALS    (2000  , get_setting_T_ms        ()) &&
        TEST_FLT_EQUALS( 500.f, get_setting_VT_mL       ()) &&
        TEST_FLT_EQUALS(  30.f, get_setting_Vmax_Lpm    ()) &&
        TEST_EQUALS    (1000  , get_setting_Tinsu_ms    ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_IoE_ratio   ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_EoI_ratio   ()) &&
        TEST_EQUALS    (1000  , get_setting_Texp_ms     ()) &&
        TEST_EQUALS    (1000  , get_setting_Tinspi_ms   ()) &&
        TEST_EQUALS    (   0  , get_setting_Tplat_ms    ()) &&
        true;
}

bool PRINT(test_checked_FR)
    setting_VT_mL         = 600;
    setting_Vmax_Lpm      =  30;
    setting_EoI_ratio_x10 =  10;
    setting_FR_pm         = checked_FR_pm(30);
    return
        TEST_RANGE     (  24.f, get_setting_FR_pm       (), 25.f) &&
        TEST_RANGE     (2400  , get_setting_T_ms        (), 2500) && // due to FR rounding
        TEST_FLT_EQUALS( 600.f, get_setting_VT_mL       ()) &&
        TEST_FLT_EQUALS(  30.f, get_setting_Vmax_Lpm    ()) &&
        TEST_EQUALS    (1200  , get_setting_Tinsu_ms    ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_IoE_ratio   ()) &&
        TEST_FLT_EQUALS(   1.f, get_setting_EoI_ratio   ()) &&
        TEST_RANGE     (1200  , get_setting_Texp_ms     (), 1250) && // due to FR rounding
        TEST_RANGE     (1200  , get_setting_Tinspi_ms   (), 1250) && // due to FR rounding
        TEST           (        get_setting_Tplat_ms    ()<= 100) && // due to FR rounding
        true;
}

//! \warning send_DATA fails at clock_ms > 0
bool PRINT(test_send)
    TEST_ASSUME(init_uart(IHM_MODE_FILE, NULL, NULL));
    TEST_ASSUME(send_INIT(""));
    TEST_ASSUME(strcmp(INIT_frame, "INIT \tCS8:65\n")==0);
    TEST_ASSUME(send_SET(VT___, VT____FMT, 300));
    TEST_ASSUME(strcmp(SET_frame, "SET_ VT___:300\tCS8:10\n" )==0);
    TEST_ASSUME(send_SET(FR___, FR____FMT,  18));
    TEST_ASSUME(strcmp(SET_frame, "SET_ FR___:18\tCS8:D4\n"  )==0);
    TEST_ASSUME(send_SET(PEP__, PEP___FMT,   5));
    TEST_ASSUME(strcmp(SET_frame, "SET_ PEP__:05\tCS8:BE\n"  )==0);
    TEST_ASSUME(send_SET(VMAX_, VMAX__FMT,  60));
    TEST_ASSUME(strcmp(SET_frame, "SET_ FLOW_:60\tCS8:B3\n"  )==0);
    TEST_ASSUME(send_SET(EoI__, EoI___FMT,  20));
    TEST_ASSUME(strcmp(SET_frame, "SET_ IE___:20\tCS8:C3\n"  )==0);
    TEST_ASSUME(send_SET(TPLAT, TPLAT_FMT, 811));
    TEST_ASSUME(strcmp(SET_frame, "SET_ Tplat:0811\tCS8:85\n")==0);
    TEST_ASSUME(send_SET(VTMIN, VTMIN_FMT, 150));
    TEST_ASSUME(strcmp(SET_frame, "SET_ VTmin:0150\tCS8:6A\n")==0);
    TEST_ASSUME(send_SET(VTMAX, VTMAX_FMT, 1000));
    TEST_ASSUME(strcmp(SET_frame, "SET_ VTmax:1000\tCS8:6A\n")==0);
    TEST_ASSUME(send_SET(PMAX_, PMAX__FMT,  60));
    TEST_ASSUME(strcmp(SET_frame, "SET_ Pmax_:060\tCS8:41\n" )==0);
    TEST_ASSUME(send_SET(PMIN_, PMIN__FMT,  20));
    TEST_ASSUME(strcmp(SET_frame, "SET_ Pmin_:020\tCS8:3B\n" )==0);
    TEST_ASSUME(send_SET(FRMIN, FRMIN_FMT,  10));
    TEST_ASSUME(strcmp(SET_frame, "SET_ FRmin:10\tCS8:F3\n"  )==0);
    TEST_ASSUME(send_SET(VMMIN, VMMIN_FMT,   3));
    TEST_ASSUME(strcmp(SET_frame, "SET_ VMmin:0003\tCS8:60\n")==0);
    TEST_ASSUME(send_DATA(5,0,0));
    TEST_ASSUME(strcmp(DATA_frame, "DATA msec_:000000 Vol__:0000 Deb__:+000 Paw__:+005\tCS8:93\n")==0);
    TEST_ASSUME(send_DATA_X(5,0,0,19,5));
    TEST_ASSUME(strcmp(DATA_X_frame, "DATA msec_:000000 Vol__:0000 Deb__:+000 Paw__:+005 PPLAT:19 PEP__:05\tCS8:3A\n")==0);
    TEST_ASSUME(send_RESP(1.95f, 18, 300, 00, 41, 19, 5));
    TEST_ASSUME(strcmp(RESP_frame, "RESP IE___:19 FR___:18 VTe__:300 PCRET:41 VM___:+00 PPLAT:19 PEP__:05\tCS8:75\n")==0);
    return true;
}

bool PRINT(TEST_PROTOCOL)
    return
        test_send() &&
        test_non_default_settings() &&
        test_checked_EoI() &&
        test_checked_VM() &&
        test_checked_VT() &&
        test_checked_FR() &&
        true;
}

#endif
