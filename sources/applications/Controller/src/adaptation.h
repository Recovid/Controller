#ifndef __ADAPTATION_H__
#define __ADAPTATION_H__
#include "common.h"
#include "platform.h"

#define CALIBRATION_STEP_US    (MOTOR_MIN_STEP_US*2)


// Initialize the adaptation engine.
// Called before the recovid starts the breathing cycles.
void adaptation_init(float a, float b);

// Compute the motor step table based on targeted Vmax, VT, and previous flow samples.
// fills the motor step table and return the number of steps.
uint32_t adaptation(
    float       target_VT_mL,
    float       target_Pdiff_Lpm,
    uint32_t    flow_samples_period_ms, 
    uint32_t    flow_samples_count, 
    float*      flow_samples_Lpm, 
    uint32_t    motor_max_steps, 
    uint32_t*   motor_steps_us);



float    linear_fit(float* samples, uint32_t samples_len, float* a, float* b);


#endif
