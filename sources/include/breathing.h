#ifndef __BREATHING_H__
#define __BREATHING_H__

#include "common.h"

float get_breathing_EoI_ratio();
float get_breathing_FR_pm();
float get_breathing_VTe_mL();
float get_breathing_VMe_Lpm();
float get_breathing_Pcrete_cmH2O();
float get_Pplat_cmH20();
float get_PEP_cmH2O();


void breathing_run(void*);

#endif