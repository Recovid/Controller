#ifndef SENSING_H
#define SENSING_H

#include "platform.h"

float get_sensed_P_cmH2O ();
float get_sensed_VolM_Lpm();
float get_sensed_VTi_mL  ();
float get_sensed_VTe_mL  ();

//! \returns max sensed Paw during Insufflation
float get_sensed_Pcrete_cmH2O();
//! \returns sensed Paw at end of Plateau
float get_sensed_Pplat_cmH2O ();
//! \returns sensed Paw at end of Exhale
float get_sensed_PEP_cmH2O   ();

float get_sensed_VMe_Lpm     ();

void sense_and_compute();

#ifndef NTESTS
bool TEST_SENSING();
#endif

#endif // SENSING_H
