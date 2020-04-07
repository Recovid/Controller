#include "alarms.h"

#include "controller_settings.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))

// Pmax Alarm
int pressureMax_startIdx = 0;
float pressureMax_cmH2O[2] = {0, 0};

// Pmin Alarm
int Pcrete_startIdx = 0;
float Pcrete_cmH2O[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int Pcrete_time_ms[8] = {0, 0, 0, 0, 0, 0, 0, 0};

bool update_alarms()
{
    int Pmax_cycles = 2;
    int Pmin_startFailing_ms = -1;

    for (int cycle = 0; cycle < 8; ++cycle)
    {
        if (Pmax_cycles != 0)
        {
            if (pressureMax_cmH2O[(pressureMax_startIdx + cycle) % 2] >= MAX(Pmax_cmH2O, PEP_cmH2O + 10))
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
            if (Pcrete_cmH2O[Pcrete_idx] <= MAX(Pmin_cmH2O, PEP_cmH2O + 2))
            {
                if (Pmin_startFailing_ms == -1)
                    Pmin_startFailing_ms = Pcrete_time_ms[Pcrete_idx];
                else if (Pmin_startFailing_ms - Pcrete_time_ms[Pcrete_idx] > 15)
                    ; // Pmin Alarm
            }
            else
                Pmin_startFailing_ms = 0; // deactivate Pmin Alarm
        }
    }

    return false; // TODO
}
