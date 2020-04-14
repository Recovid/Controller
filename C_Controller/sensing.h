#ifndef SENSING_H
#define SENSING_H

#include "platform.h"

float get_sensed_P_cmH2O ();
float get_sensed_VolM_Lpm();
float get_sensed_Vol_mL  ();

//! \returns sensed Paw at end of Exhale
float get_sensed_PEP_cmH2O();

void sense_and_compute();

#ifndef NTESTS
bool TEST_SENSING();
#endif

#endif // SENSING_H
