/*
 * utils.h
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32f3xx_hal.h"


void compute_pid(float* A, float* B, uint32_t log_index, float log_time_step_sum, float flow_setpoint_slm, float* inhalation_flow);
int32_t calibration(float* A, float* B, uint8_t iterations);
float linear_fit(float* samples, size_t samples_len, float time_step_sec, float* slope);
int32_t get_plateau(float* samples, size_t samples_len, float time_step_sec, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound);
float compte_motor_step_time(long step_number, float desired_flow, double calibration_speed, float A_calibrated,float B_calibrated);

#endif /* INC_PID_H_ */
