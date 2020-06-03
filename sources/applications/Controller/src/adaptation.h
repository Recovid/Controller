#ifndef __ADAPTATION_H__
#define __ADAPTATION_H__
#include "common.h"
#include "platform.h"
#include "breathing.h"

// Initialize the adaptation engine.
// Called before the recovid starts the breathing cycles.
void adaptation_prepare();

// Compute the motor step table based on targeted Vmax, VT, and previous flow samples.
// fills the motor step table and return the number of steps.
uint32_t adaptation(
    const settings_t* p_settings, 
    const insuflation_samples_t* p_samples, 
    uint32_t motor_max_steps, 
    uint32_t* motor_steps_us);


#endif

