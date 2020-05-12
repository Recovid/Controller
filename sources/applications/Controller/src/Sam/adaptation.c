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
static bool     get_plateau(float* samples, uint32_t samples_len, float flow_samples_period_s, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound);
static bool     get_plateau2(float* samples, uint32_t samples_len, float flow_samples_period_s, uint32_t* low_bound, uint32_t* high_bound);
static bool     get_plateau3(float* samples, uint32_t samples_len, uint32_t* low_bound, uint32_t* high_bound);
static float    linear_fit(float* samples, uint32_t samples_len, float* a, float* b);

//----------------------------------------------------------
// Public variables
//----------------------------------------------------------

//----------------------------------------------------------
// Public functions
//----------------------------------------------------------

// Initialize the adaptation engine.
// Called before the recovid starts the breathing cycles.
void adaptation_init(float a, float b) {
    A= 0.585; //3.577;
    B= -0.074; //-0.455;
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
    // if(flow_samples_count>0)
    // {
    //     uint32_t low, high;
    //     get_plateau3(flow_samples_Lpm, flow_samples_count, &low, &high);
    // }
    
    pid(target_Flow_Lpm, 0.001*flow_samples_period_ms, flow_samples_count, flow_samples_Lpm, &A, &B);

    for(uint32_t t=0; t<motor_max_steps; ++t) {
        motor_steps_us[t]= (uint32_t) compte_motor_step_time(t, target_Flow_Lpm, A, B, (MOTOR_MIN_STEP_US*2)*0.000001);
    }
    return motor_max_steps;
}

//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

static float compte_motor_step_time(uint32_t step_number, float desired_flow_Lm, float A, float B, float speed) {
	float res = (0.8*A*speed*speed*step_number) + B * speed;
	res = 1000000* res / desired_flow_Lm;
	if (res  < 0.8*MOTOR_MIN_STEP_US) { 
        res = 0.8*MOTOR_MIN_STEP_US;
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

		get_plateau3(flow_samples, flow_samples_count, &low, &high);

        float origin;
		float plateau_slope;
        plateau_slope= linear_fit(flow_samples+low, high-low-1, &plateau_slope, &origin);
		float plateau_mean = 0;
		for(uint32_t i=low; i<high; i++) {
			plateau_mean += flow_samples[i];
		}
		plateau_mean = plateau_mean/(high-low);
		brth_printf("plateau: slope=%ld origin=%ld mean=%ld\n",(int32_t)(1000*plateau_slope),(int32_t)(1000*origin),(int32_t)(1000*plateau_mean));

		float error_mean = plateau_mean - target_flow_Lpm;

		*A += plateau_slope * P_plateau_slope;
		*B += error_mean * P_plateau_mean;
		brth_printf("A = %ld\n", (int32_t)(1000*(*A)));
		brth_printf("B = %ld\n", (int32_t)(1000*(*B)));

}

// Compute slope of samples fetched with specified time_step
// Returns 	R  if fit is ok
// 			-1 if fit is not possible
float linear_fit(float* samples, uint32_t samples_len, float* a, float *b){
	float sumx=0,sumy=0,sumxy=0,sumx2=0, sumy2=0;
    float x,y;
	for(uint32_t i=00;i<samples_len;i++) {
        x= (float)i;
        y= samples[i];
		sumx  = sumx + x;
		sumx2 = sumx2 + (x*x);
		sumy  = sumy + y;
		sumy2 = sumy2 + (y*y);
		sumxy = sumxy + (x*y);
	}
	float denom = (samples_len * sumx2 - (sumx * sumx));
	if(denom == 0.) {
		brth_printf("no fit\n");
		return -1;
	}
	// compute slope a
	*a = (samples_len * sumxy  -  sumx * sumy) / denom;
    *b = (sumy - (*a)*sumx)/samples_len;

	// compute correlation coefficient
	return (sumxy - sumx * sumy / samples_len) / sqrtf((sumx2 - (sumx*sumx)/samples_len) * (sumy2 - (sumy*sumy)/samples_len));
}

static bool get_plateau(float* samples, uint32_t samples_len, float flow_samples_period_s, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound)
{
// 	if(windows_number < 2 || windows_number > 30) {return -1;}
// 	float slopes[30];
// 	*high_bound = samples_len-1;
// 	// Compute slope for time windows to detect when signal start increasing/decreasing
// 	for(uint32_t window=0; window<windows_number; window++) 
//     {
// 		float r = linear_fit(&samples[window*(samples_len/windows_number)], samples_len/windows_number, flow_samples_period_s, slopes+window);
// //		brth_printf("%ld    ", (int32_t)(*(slopes+window) * 1000));
// 	}
// //	brth_printf("\n");
// 	for(uint32_t window=1; window<windows_number; window++) {
// 		float delta_slope = slopes[window-1] - slopes[window];
// 		if(delta_slope > 1.) {
// 			*low_bound = (uint32_t)((samples_len/windows_number)*(window+1));
// //			brth_printf("plateau begin at %lu over %lu points\n", *low_bound, (uint32_t)samples_len);
// 			return 0;
// 		}
// 	}
// 	*low_bound = (uint32_t)(samples_len/2);
// //	brth_printf("No plateau found\n");
	return true;
}

static bool get_plateau2(float* samples, uint32_t samples_len, float flow_samples_period_s, uint32_t* low_bound, uint32_t* high_bound)
{
    // First we have to dismiss the first "zero flux" points
    uint32_t firstNonZeroIndex = 0;
    while(firstNonZeroIndex< samples_len && samples[firstNonZeroIndex++] < 0.001);

    // Now we try to find the best midpoint to fit two lines to flow data
    // TODO : dismiss the first and last N points
    uint32_t N = 3;
    int32_t samples_len_no_zeros = samples_len - firstNonZeroIndex;
    // TODO Check samples_len_no_zeros !!

    float bestRScore = 0.;
    float bestSlopes[2];
    uint32_t bestMidpointIndex = 0;
    // For each midpoint index
    uint32_t midpointIndex;
    for(midpointIndex=samples_len*1/3; midpointIndex<samples_len; ++midpointIndex) 
    {
        // Compute the two slopes
        float slope1, origin1;
        float slope2, origin2;
        float r1 = linear_fit(&samples[firstNonZeroIndex], midpointIndex-firstNonZeroIndex+1, &slope1, &origin1);
        float r2 = linear_fit(&samples[midpointIndex], samples_len-midpointIndex, &slope2, &origin2);
        // Compute score based on fit RSQ and points number
//        float score = (r1 * (float)(midpointIndex-firstNonZeroIndex+1)/(float)(samples_len_no_zeros)* 2.0/3.0) + (r2 * (float)(samples_len-midpointIndex)/(float)(samples_len_no_zeros) *1.0/3.0);
        float score = (r1 * 2.0/3.0) + (r2 * 1.0/3.0);
        //brth_printf("plateau: r1=%lu, r2=%lu, score=%lu\n", (uint32_t)(r1*1000),(uint32_t)(r2*1000),(uint32_t)(score*1000) )
        if(score > bestRScore) 
        {
            bestRScore = score;
            bestMidpointIndex = midpointIndex;
            bestSlopes[0] = slope1;
            bestSlopes[1] = slope2;
        }
    }
    *high_bound = samples_len-1;
    *low_bound = (uint32_t)(bestMidpointIndex);

    brth_printf("plateau begin at %lu and ends at %lu with score %lu\n", *low_bound, *high_bound, (uint32_t)(1000*bestRScore));
    return true;
}

static bool get_plateau3(float* samples, uint32_t samples_len, uint32_t* low_bound, uint32_t* high_bound)
{
    // First we have to dismiss the first "zero flux" points
    uint32_t firstNonZeroIndex = 0;
    while(firstNonZeroIndex< samples_len && samples[firstNonZeroIndex++] < 0.001);

    if(firstNonZeroIndex==samples_len) return false;
    
    // Now we try to find the best midpoint to fit two lines to flow data
    // TODO : dismiss the first and last N points
    uint32_t N = 3;
    int32_t samples_len_no_zeros = samples_len - firstNonZeroIndex;
    // TODO Check samples_len_no_zeros !!

    float slope1, origin1;
    float slope2, origin2;
    float r1 = linear_fit(&samples[firstNonZeroIndex], 20, &slope1, &origin1);
    float r2 = linear_fit(&samples[samples_len-20], 20, &slope2, &origin2);

    if(slope1==slope2) 
    {
        *low_bound = samples_len/2;
    }
    else
    {
        origin1-= slope1*firstNonZeroIndex;
        origin2-= slope2*(samples_len-20);

        float intersection = ((origin2-origin1)/(slope1-slope2));
        if(intersection<firstNonZeroIndex || intersection>samples_len-20)
        {
            *low_bound = samples_len-20;
        }
        else
        {
            *low_bound = (uint32_t) intersection;
        }
    }
    
    *high_bound = samples_len-1;

    for(uint32_t t=0; t<samples_len; ++t) 
    {
        brth_printf("%ld, ", (int32_t)(slope1*t+origin1)*1000);
    }
    brth_printf("\n");

    for(uint32_t t=0; t<samples_len; ++t) 
    {
        brth_printf("%ld, ", (int32_t)(slope2*t+origin2)*1000);
    }
    brth_printf("\n");




    brth_printf("plateau [%lu - %lu] | [%ld - %ld] [%ld - %ld]\n", *low_bound, *high_bound, (int32_t)(slope1*1000), (int32_t)(origin1*1000), (int32_t)(slope2*1000), (int32_t)(origin2*1000));
    return true;
}