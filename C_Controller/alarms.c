#include "alarms.h"

#include "controller_settings.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))

// Pmax Alarm
int pressureMax_startIdx = 0;
float pressureMax_cmH2O_q[2] = {0, 0};

// Pmin Alarm
int Pcrete_startIdx = 0;
float Pcrete_cmH2O_q[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int Pcrete_time_ms_q[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// VTmin Alarm
int VTe_startIdx = 0;
float VTe_ml_q[3] = {0, 0, 0};

// FRmin Alarm
int FR_startIdx = 0;
float FR_pm_q[3] = {0, 0, 0};

// VMmin Alarm
int VM_startIdx = 0;
float VM_Lm_q[3] = {0, 0, 0};

// PEPmax / PEPmin Alarm
int PEP_startIdx = 0;
float PEP_cmH2O_q[8] = {0, 0, 0, 0, 0, 0, 0, 0};

bool update_alarms()
{
    int Pmax_cycles = 2;
    int Pmin_startFailing_ms = -1;
    int VTmin_cycles = 3;
    int FRmin_cycles = 3;
    int VMmin_cycles = 3;
    int PEPmax_cycles = 8;
    int PEPmin_cycles = 8;

    for (int cycle = 0; cycle < 8; ++cycle)
    {
        if (Pmax_cycles != 0)
        {
            if (pressureMax_cmH2O_q[(pressureMax_startIdx + cycle) % 2] >= MAX(Pmax_cmH2O, PEP_cmH2O + 10))
            {
                --Pmax_cycles;
                if (Pmax_cycles == 0)
                    ; // Pmax Alarm
            }
            else
                Pmax_cycles = 0; // deactivate Pmax Alarm
        }
        if (Pmin_startFailing_ms != 0)
        {
            int Pcrete_idx = (Pcrete_startIdx + cycle) % 8;
            if (Pcrete_cmH2O_q[Pcrete_idx] <= MAX(Pmin_cmH2O, PEP_cmH2O + 2))
            {
                if (Pmin_startFailing_ms == -1)
                    Pmin_startFailing_ms = Pcrete_time_ms_q[Pcrete_idx];
                else if (Pmin_startFailing_ms - Pcrete_time_ms_q[Pcrete_idx] > 15)
                    ; // Pmin Alarm
            }
            else
                Pmin_startFailing_ms = 0; // deactivate Pmin Alarm
        }
        if (VTmin_cycles != 0)
        {
            if (VTe_ml_q[VTe_startIdx % 3] < VTmin_mL)
            {
                --VTmin_cycles;
                if (VTmin_cycles == 0)
                    ; // VTmin Alarm
            }
            else
                VTmin_cycles = 0; // deactivate VTmin Alarm
        }
        if (FRmin_cycles != 0)
        {
            if (FR_pm_q[VTe_startIdx % 3] < FRmin_pm)
            {
                --FRmin_cycles;
                if (FRmin_cycles == 0)
                    ; // FRmin Alarm
            }
            else
                FRmin_cycles = 0; // deactivate FRmin Alarm
        }
        if (VMmin_cycles != 0)
        {
            if (VM_Lm_q[VTe_startIdx % 3] < VMmin_Lm)
            {
                --VMmin_cycles;
                if (VMmin_cycles == 0)
                    ; // VMmin Alarm
            }
            else
                VMmin_cycles = 0; // deactivate VMmin Alarm
        }
        if (PEPmax_cycles != 0)
        {
            if (PEP_cmH2O_q[PEP_startIdx] >= PEP_cmH2O + 2)
            {
                --PEPmax_cycles;
                if (PEPmax_cycles == 0)
                    ; // PEPmax Alarm
            }
            else
                PEPmax_cycles = 0; // deactivate PEPmax Alarm
        }
        if (PEPmin_cycles != 0)
        {
            if (PEP_cmH2O_q[PEP_startIdx] <= PEP_cmH2O - 2)
            {
                --PEPmin_cycles;
                if (PEPmin_cycles == 0)
                    ; // PEPmin Alarm
            }
            else
                PEPmin_cycles = 0; // deactivate PEPmin Alarm
        }
    }
    return false; // TODO
}
