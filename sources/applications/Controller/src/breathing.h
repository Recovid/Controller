#ifndef __BREATHING_H__
#define __BREATHING_H__

#include "common.h"

// Breathing FreeRTOS stuff

#define BRTH_CYCLE_INSUFLATION      (1 << 0)
#define BRTH_CYCLE_PLATEAU          (1 << 1)
#define BRTH_CYCLE_EXHALATION       (1 << 2)

#define BRTH_CYCLE_PINS             (1 << 4)
#define BRTH_CYCLE_PEXP             (1 << 5)

#define BRTH_CYCLE_UPDATED          (1 << 7)      // To inform Controller that the breathing info have been updated

extern EventGroupHandle_t   g_breathingEvents;
extern TaskHandle_t         g_breathingTask;
extern TimerHandle_t        g_samplingTimer;
bool breathing_init();



// Breathing public interface

float get_cycle_EoI_ratio();
float get_cycle_FR_pm();
float get_cycle_VTe_mL();
float get_cycle_VTi_mL();
float get_cycle_VMe_Lpm();
float get_cycle_Pcrete_cmH2O();
float get_cycle_Pplat_cmH2O();
float get_cycle_PEP_cmH2O();




#endif

