/*
 * pid.c
 */
#include <math.h>
#include <stdio.h>
#include "pid.h"

// Compute average flow and slope to adjust A_calibrated and B_calibrated
// A_calibrated and B_calibrated the terms of the PID
// NB : The PID is only a P for now, so keep that in mind

// Careful, not the same as "calibration(float* A, float* B, uint8_t iterations)" !
// calibration() is for testing, compute_pid() is to be called at the end of every EXPIRATION MOVEMENT
void compute_pid(float* A, float* B, uint32_t log_index, float log_time_step_sum, float flow_setpoint_slm, float* inhalation_flow){
		float P_plateau_slope = 0.1;
		float P_plateau_mean = 0.2;
		float timeStep = log_time_step_sum/ log_index;
		uint32_t low;
		uint32_t high;
		if(get_plateau(inhalation_flow, log_index, timeStep, 10, &low, &high) == 0) {
			printf("plateau found from sample %lu to %lu\n", low, high);
		} else {
			printf("plateau NOT found, considering from sample %lu to %lu\n", low, high);
		}
		float plateau_slope = linear_fit(inhalation_flow+low, high-low-1, timeStep, &plateau_slope);
		float plateau_mean = 0;
		for(int i=low; i<high; i++) {
			plateau_mean += inhalation_flow[i];
		}
		plateau_mean = plateau_mean/(high-low);
		printf("plateau slope : %ld\n",(int32_t)(1000*plateau_slope));
		printf("plateau mean : %ld\n",(int32_t)(1000*plateau_mean));

		float error_mean = plateau_mean - (flow_setpoint_slm/60.);

		*A += plateau_slope * P_plateau_slope;
		*B += error_mean * P_plateau_mean;
		//printf("A = %ld\n", (int32_t)(1000*A));
		//printf("B = %ld\n", (int32_t)(1000*B));
}



/*
// Calibration function if need be to find a fitting A and B
// In this current version, the calibration of B doesn't seems to work well enough
int32_t calibration(float* A, float* B, uint8_t iterations) {
	float slope = 0; // slope of flow(t) curve
	float originFlow = 0; // origin flow of flow(t) curve
	uint32_t steps;

	printf("Press button to calibrate.\n");
	wait_btn_clicked();

	// Calibrate slope
	printf("---------- Calibrating slope ---------------\n");
	for(int iter=0; iter<iterations; ++iter) {
		sensors_reset_volume();
		// HIGH PEEP
		HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_HIGH);
		steps= (uint32_t) (STEPS_PER_REVOLUTION)*(360/360.0);
		double speed = CALIBRATION_SPEED*1000000;
//		steps= (uint32_t) (STEPS_PER_REVOLUTION)*(20/360.0);
//		motor_move(COMPRESS, speed, steps);
//		while(!motor_is_done());
//		steps= (uint32_t) (STEPS_PER_REVOLUTION)*(180/360.0);
		motor_move(&bavu_motor, COMPRESS, speed, steps);
		HAL_Delay(200);
		// Start logging flow
		_logging_index = 0;
		_logging_time_step_sum = 0.;
		sensors_set_flow_callback(flow_callback);

//		reporting_start(100);
		while(motor_is_moving(&bavu_motor));
		// Stop logging flow
		sensors_set_flow_callback(NULL);

//		reporting_stop();
		motor_stop(&bavu_motor);
		HAL_Delay(500);
		float volumeIT = sensors_get_volume();
		printf("volume = %lu ml\n", (uint32_t)(1000*volumeIT));
//		// LOW PEEP
		HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_LOW);
		_is_home = false;
		motor_run(&bavu_motor, RELEASE, HOME_SPEED);
		while(!bavu_motor_is_home());
		motor_stop(&bavu_motor);
		HAL_Delay(2000);

		float a = 0;
		float r = linear_fit(_inhalation_flow, _logging_index, _logging_time_step_sum/_logging_index, &a);
		printf("a=%lu\n", (uint32_t)(1000.*a));
		printf("r=%lu\n", (uint32_t)(1000.*r));
		slope += a / (float)iterations;
	}
	*A = slope;
	printf("A=%lu\n", (uint32_t)(1000.* *A));


	// Calibrate originFlow
	printf("---------- Calibrating B ---------------\n");
	for(int iter=0; iter<iterations; ++iter) {
		// HIGH PEEP
		HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_HIGH);
		steps= (uint32_t) (STEPS_PER_REVOLUTION)*(360/360.0);
		sensors_reset_volume();
		motor_move(&bavu_motor, COMPRESS, CALIBRATION_SPEED*1000000., steps);
		while(motor_is_moving(&bavu_motor));
		motor_stop(&bavu_motor);
		HAL_Delay(2000);
		float volumeIT = sensors_get_volume();
		// LOW PEEP
		HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_LOW);
		_is_home=false;
		motor_run(&bavu_motor, RELEASE, HOME_SPEED);
		while(!bavu_motor_is_home());
		motor_stop(&bavu_motor);
		printf("volume = %luml\n", (uint32_t)(volumeIT*1000));
		float b = volumeIT/((CALIBRATION_SPEED) * steps) - (*A * (CALIBRATION_SPEED)*steps / 2.);
		printf("b=%ld\n", (int32_t)(1000*b));
		// Add values for averaging over iterations
		originFlow += b/(float)iterations;
	}
	*B = 1.5; //The cheat since the B calibration seems flawed
//	*B = 1.3;
	printf("B=%ld\n", (int32_t)(1000*(*B)));
	printf("Calibration...DONE\n");
	return 0;
}
*/

// Compute slope of samples fetched with specified time_step
// Returns 	R  if fit is ok
// 			-1 if fit is not possible
float linear_fit(float* samples, size_t samples_len, float time_step_sec, float* slope){
	float sumx=0,sumy=0,sumxy=0,sumx2=0, sumy2=0;
	for(int i=0;i<samples_len;i++) {
		sumx  = sumx + (float)i * time_step_sec;
		sumx2 = sumx2 + (float)i*time_step_sec*(float)i*time_step_sec;
		sumy  = sumy + *(samples+i);
		sumy2 = sumy2 + (*(samples+i)) * (*(samples+i));
		sumxy = sumxy + (float)i*(time_step_sec)* (*(samples+i));
	}
	float denom = (samples_len * sumx2 - (sumx * sumx));
	if(denom == 0.) {
		//printf("Calibration of A is not possible\n");
		return 1;
	}
	// compute slope a
	*slope = (samples_len * sumxy  -  sumx * sumy) / denom;
//	printf("%ld     ", (int32_t)(1000*((samples_len * sumxy  -  sumx * sumy) / denom)));

	// compute correlation coefficient
	return (sumxy - sumx * sumy / samples_len) / sqrtf((sumx2 - (sumx*sumx)/samples_len) * (sumy2 - (sumy*sumy)/samples_len));
}



// Find the plateau of the curve by slicing it in N windows (windows_number = 10 usually).
// The curve is describe by samples of size samples_len
// This function return the first and last indexes(low_bound and high_bound respectively)
// of the samples corresponding to the plateau
// NB : The high_bound is ALWAYS the last sample index
//		If no low_bound is found, low_bound = middle sample index
int32_t get_plateau(float* samples, size_t samples_len, float time_step_sec, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound){
	if(windows_number < 2 || windows_number > 30) {return -1;}
	float slopes[30];
	*high_bound = samples_len-1;
	// Compute slope for time windows to detect when signal start increasing/decreasing
	for(int window=0; window<windows_number; window++) {
		float r = linear_fit(samples+window*(samples_len/windows_number), samples_len/windows_number, time_step_sec, slopes+window);
		printf("%ld    ", (int32_t)(*(slopes+window) * 1000));
	}
	printf("\n");
	for(int window=1; window<windows_number; window++) {
		float delta_slope = slopes[window-1] - slopes[window];
		if(delta_slope > 1.) {
			*low_bound = (uint32_t)((samples_len/windows_number)*(window+1));
			printf("plateau begin at %lu over %lu points\n", *low_bound, (uint32_t)samples_len);
			return 0;
		}
	}
	*low_bound = (uint32_t)(samples_len/2);
	printf("No plateau found\n");
	return 1;
}


// Compute the motor step time in us
// Inputs : calibration_speed is motor step time in seconds
// 			desired_flow is in sL/s
//          A_calibrated is the proportional term computed from the slope (previously a somewhat global var, now an input)
//          B_calibrated is the constant term (previously a somewhat global var, now an input)
// Returns step time in us
float compte_motor_step_time(long step_number, float desired_flow, double calibration_speed, float A_calibrated,float B_calibrated) {
	float res = (A_calibrated*calibration_speed*calibration_speed*step_number) + B_calibrated * calibration_speed;
	res = res / desired_flow;
	if (res * 1000000 < 110) {return 110;}
	else {return res * 1000000.;}
}







