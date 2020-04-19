#ifndef STRUCT_H
#define STRUCT_H

#include "platform.h"

#define ALARM_COUNT 18

enum alarm_type {
    ALARM_NONE = 0,
    ALARM_PMAX = 1 << 0,
    ALARM_PMIN = 1 << 1,
    ALARM_VT_MIN = 1 << 2,
    ALARM_VT_MAX = 1 << 3,
    //ALARM_FR = 1 << 3, // No longer used
    ALARM_VM_MIN = 1 << 5,
    ALARM_PEP_MAX = 1 << 6,
    ALARM_PEP_MIN = 1 << 7,
    ALARM_BATT_A = 1 << 8,
    ALARM_BATT_B = 1 << 9,
    ALARM_BATT_C = 1 << 10,
    ALARM_BATT_D = 1 << 11,
    ALARM_BATT_E = 1 << 12,
    ALARM_FAILSAFE = 1 << 13,
    ALARM_CPU_LOST = 1 << 14,
    ALARM_P_KO = 1 << 15,
    ALARM_IO_MUTE = 1 << 16,
    ALARM_SENSOR_FAIL = 1 << 17,
};

struct alarm {
    enum alarm_type type;
    bool ack_needed;
};

struct respiration_cycle {
    int t_start;    //Timestamp of the beginning of the innhalation phase
    int t_plat;     //Timestamp of the beginning of the Plateau phase
    int t_exhal;    //TImestamp of the beginning of the exhalation phase
    int Fi02;       // Fraction inspirée en di-Oxygène) : 021..100 (%)
    int Vtidal;     // (Volume tidal ou courant) : 0300..800 (mL)
    int freq_respi; //(Fréquence Respiratoire) : 10..35 (1/min)
    int PEP;        //(Pression Expiratoire Positive) : 00..15 (cmH2O)
    int PIP;        // (Pression Inspiratoire de Pointe) : 00.0..99.9 (cmH2O)
    int PPLAT;      // (Pression Plateau) : 00.0..99.9 (cmH2O)
};

struct settings {
    int fio2_settings;       //'Fi02_' (Fraction inspir�e en di-Oxyg�ne) : 021..100 (%)
    int Vtidal_settings;     //'Vt___' (Volume tidal ou courant) : 0300..1000 (mL)
    int pep_settings;        //'PEP__' (Pression Expiratoire Positive) : 00..50 (cmH2O)
    int freq_respi_settings; //'FR___' (Fr�quence Respiratoire) : 12..35 (1/min)
    int pif_settings;        //'PIF__' (D�bit de pointe) : 30..90 (1/min)
    int tplat_settings;      //'TPLAT' (Temps Plateau) : 000..200 (ms)
};


struct alarm_settings {
    int pmax_alarm;  //'PMAX_' (Pression max) : 00.0..99.9 (cmH2O)
    int pmin_alarm;  //'PMIN_' (Pression Minimum inspiration) : 00.0..99.9 (cmH2O)
    int vtmin_alarm; //'VTMIN' (Volume Minimum inspiration) : 0200..1000 (mL)
};

#define SIZE_OF_TEXT_MESSAGE 100
#define NB_MAX_MESSAGE 10
struct message {
  char text[SIZE_OF_TEXT_MESSAGE];
};

#endif //STRUCT_H
