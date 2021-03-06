#include "common.h"
#include "config.h"
#include "adaptation.h"

// Other dependencies
#include <math.h>

//----------------------------------------------------------
// Private defines
//----------------------------------------------------------

//----------------------------------------------------------
// Private typedefs
//----------------------------------------------------------

//----------------------------------------------------------
// Private variables
//----------------------------------------------------------
static float A;
static float B;

//----------------------------------------------------------
// Private functions prototypes
//----------------------------------------------------------

static float    compte_motor_step_time(uint32_t step_number, float desired_flow_Ls, float A, float B, float speed);
static void     pid(float target_flow_Lpm, float flow_samples_period_s, uint32_t flow_samples_count, float* flow_samples,  float* A, float* B);
static float    linear_fit(float* samples, uint32_t samples_len, float flow_samples_period_s, float* slope);
static int32_t  get_plateau(float* samples, uint32_t samples_len, float flow_samples_period_s, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound);


//----------------------------------------------------------
// Public variables
//----------------------------------------------------------

//----------------------------------------------------------
// Public functions
//----------------------------------------------------------

// Initialize the adaptation engine.
// Called before the recovid starts the breathing cycles.
void adaptation_init() {
    A= 3.577;
    B= -0.455;
}

// Compute the motor step table based on targeted Vmax, VT, and previous flow samples.
// fills the motor step table and return the number of steps.
uint32_t adaptation(
    float       target_VT_mL,
    float       target_Flow_Lpm,
    uint32_t    flow_samples_period_ms, 
    uint32_t    flow_samples_count, 
    float*      flow_samples_Lpm, 
    uint32_t    motor_max_steps, 
    uint32_t*   motor_steps_us) 
{

    pid(target_Flow_Lpm, 0.001*flow_samples_period_ms, flow_samples_count, flow_samples_Lpm, &A, &B);

    for(uint32_t t=0; t<motor_max_steps; ++t) {
        motor_steps_us[t]= (uint32_t) compte_motor_step_time(t, target_Flow_Lpm/60.0, A, B, 1.0/1600);
    }
    return motor_max_steps;
}

//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

static float compte_motor_step_time(uint32_t step_number, float desired_flow_Ls, float A, float B, float speed) {
	float res = (0.8*A*speed*speed*step_number) + B * speed;
	res = 1000000* res / desired_flow_Ls;
	if (res  < 600) { 
        res = 600;
    }
	return res;
}


static void pid(float target_flow_Lpm, float flow_samples_period_s, uint32_t flow_samples_count, float* flow_samples,  float* A, float* B) {
  if(flow_samples_count==0) return;
//************************************************* PID ZONE ********************************************//
		// Compute average flow and slope to adjust A and B coefficients
		float P_plateau_slope = 0.1;
		float P_plateau_mean = 0.2;
		uint32_t low;
		uint32_t high;
		if(get_plateau(flow_samples, flow_samples_count, flow_samples_period_s, 10, &low, &high) == 0) {
//			brth_printf("plateau found from sample %lu to %lu\n", low, high);
		} else {
//			brth_printf("plateau NOT found, considering from sample %lu to %lu\n", low, high);
		}
		float plateau_slope = linear_fit(flow_samples+low, high-low-1, flow_samples_period_s, &plateau_slope);
		float plateau_mean = 0;
		for(uint32_t i=low; i<high; i++) {
			plateau_mean += flow_samples[i];
		}
		plateau_mean = plateau_mean/(high-low);
//		brth_printf("plateau slope : %ld\n",(int32_t)(1000*plateau_slope));
//		brth_printf("plateau mean : %ld\n",(int32_t)(1000*plateau_mean));

		float error_mean = plateau_mean - (target_flow_Lpm/60.);

		*A += plateau_slope * P_plateau_slope;
		*B += error_mean * P_plateau_mean;
//		brth_printf("A = %ld\n", (int32_t)(1000*(*A)));
//		brth_printf("B = %ld\n", (int32_t)(1000*(*B)));

}

// Compute slope of samples fetched with specified time_step
// Returns 	R  if fit is ok
// 			-1 if fit is not possible
static float linear_fit(float* samples, uint32_t samples_len, float flow_samples_period_s, float* slope){
	float sumx=0,sumy=0,sumxy=0,sumx2=0, sumy2=0;
	for(uint32_t i=0;i<samples_len;i++) {
		sumx  = sumx + (float)i * flow_samples_period_s;
		sumx2 = sumx2 + (float)i*flow_samples_period_s*(float)i*flow_samples_period_s;
		sumy  = sumy + (*(samples+i)/60.0);
		sumy2 = sumy2 + ((*(samples+i))/60.0) * ((*(samples+i)/60.0));
		sumxy = sumxy + (float)i*(flow_samples_period_s)* (*(samples+i)/60.0);
	}
	float denom = (samples_len * sumx2 - (sumx * sumx));
	if(denom == 0.) {
//		brth_printf("Calibration of A is not possible\n");
		return 1;
	}
	// compute slope a
	*slope = (samples_len * sumxy  -  sumx * sumy) / denom;

	// compute correlation coefficient
	return (sumxy - sumx * sumy / samples_len) / sqrtf((sumx2 - (sumx*sumx)/samples_len) * (sumy2 - (sumy*sumy)/samples_len));
}

static int32_t get_plateau(float* samples, uint32_t samples_len, float flow_samples_period_s, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound){
	if(windows_number < 2 || windows_number > 30) {return -1;}
	float slopes[30];
	*high_bound = samples_len-1;
	// Compute slope for time windows to detect when signal start increasing/decreasing
	for(uint32_t window=0; window<windows_number; window++) {
		float r = linear_fit(samples+window*(samples_len/windows_number), samples_len/windows_number, flow_samples_period_s, slopes+window);
//		brth_printf("%ld    ", (int32_t)(*(slopes+window) * 1000));
	}
//	brth_printf("\n");
	for(uint32_t window=1; window<windows_number; window++) {
		float delta_slope = slopes[window-1] - slopes[window];
		if(delta_slope > 1.) {
			*low_bound = (uint32_t)((samples_len/windows_number)*(window+1));
//			brth_printf("plateau begin at %lu over %lu points\n", *low_bound, (uint32_t)samples_len);
			return 0;
		}
	}
	*low_bound = (uint32_t)(samples_len/2);
//	brth_printf("No plateau found\n");
	return 1;
}
