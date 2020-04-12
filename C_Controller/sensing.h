#ifndef SENSING_H
#define SENSING_H

#include "platform.h"

float get_sensed_P_cmH2O ();
float get_sensed_VolM_Lpm();
float get_sensed_Vol_mL  ();

void sense_and_compute();

#ifdef TESTS
bool test_sensing();
#endif

#endif // SENSING_H
