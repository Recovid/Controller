#ifndef STRUCT_H
#define STRUCT_H

#include "platform.h"

enum alarm_type {
    ALARM_NONE = 0,
    ALARM_PMAX = 1 << 0, // Paw > max(Pmax, PEPs+10)
    ALARM_PMIN = 1 << 1, // Paw < max(Pmin, PEPs+2)
    ALARM_VT_MIN = 1 << 2, // VTe > VTeMini
    //ALARM_FR = 1 << 3, // Freq non correct ??? -> No longer used
    ALARM_VM_MIN = 1 << 4, // VMe <= VMin
    ALARM_PEP_MAX = 1 << 5, // PEP > PEPs - 2
    ALARM_PEP_MIN = 1 << 6, // PEP < PEPs + 2
    ALARM_LOW_BATTERY = 1 << 7
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
