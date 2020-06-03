#ifndef __BREATHING_H__
#define __BREATHING_H__

#include "common.h"


//********************************************************************************
// Public defines and types
//********************************************************************************

// Sampling timer period
#define SAMPLING_PERIOD_MS      (5)

// Max insuflation data samples (2 seconds of data)
#define MAX_INSUFLATION_SAMPLES        (2000 / SAMPLING_PERIOD_MS)

// Breathing FreeRTOS stuff
#define BRTH_CYCLE_INSUFLATION      (1 << 0)
#define BRTH_CYCLE_PLATEAU          (1 << 1)
#define BRTH_CYCLE_EXHALATION       (1 << 2)

#define BRTH_CYCLE_PINHA            (1 << 4)
#define BRTH_CYCLE_PEXHA            (1 << 5)

#define BRTH_CYCLE_UPDATED          (1 << 7)      // To inform Controller that the breathing info have been updated


typedef struct {
   uint32_t T_ms;
   float    VT_mL;
   float    Vmax_Lpm;
   float    Pmax_cmH2O;
   uint32_t Tinha_ms;
   uint32_t Tinsu_ms;
   uint32_t Texha_ms;
} settings_t;

typedef struct {
    float EoI_ratio;
    float FR_pm;
    float VTi_mL;
    float VTe_mL;
    float VMe_Lpm;
    float Pcrete_cmH2O;
    float Pplat_cmH2O;
    float PEP_cmH2O;
} cycle_data_t;

typedef struct {
    uint32_t count;
    uint32_t T_ms;
    float    Pdiff_Lpm[MAX_INSUFLATION_SAMPLES];
    float    Paw_cmH2O[MAX_INSUFLATION_SAMPLES];
    uint32_t motor_step_idx[MAX_INSUFLATION_SAMPLES];
} insuflation_samples_t;



//********************************************************************************
// Public variables
//********************************************************************************
extern EventGroupHandle_t   g_breathingEvents;
extern TaskHandle_t         g_breathingTask;
extern TimerHandle_t        g_samplingTimer;


//********************************************************************************
// Public functions
//********************************************************************************

bool breathing_init();

// Breathing public interface

float get_cycle_EoI_ratio();
float get_cycle_FR_pm();
float get_cycle_VTi_mL();
float get_cycle_VTe_mL();
float get_cycle_VMe_Lpm();
float get_cycle_Pcrete_cmH2O();
float get_cycle_Pplat_cmH2O();
float get_cycle_PEP_cmH2O();




#endif

