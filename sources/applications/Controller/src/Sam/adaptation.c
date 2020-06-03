#include "common.h"
#include "config.h"
#include "adaptation.h"
// Other dependencies
#include <math.h>

//----------------------------------------------------------
// Private defines
//----------------------------------------------------------

#define CALIBRATION_STEP_US    (MOTOR_MIN_STEP_US*2)

//----------------------------------------------------------
// Private typedefs
//----------------------------------------------------------

//----------------------------------------------------------
// Private variables
//----------------------------------------------------------
// Coeff with quadratic coeff !!
// #define A_COEF      (3.15)
// #define A_ORIGIN    (169.31)
// #define B_COEF      (0.57)
// #define B_ORIGIN    (-59.72)


#define A_COEF      (2.14)
#define A_ORIGIN    (238.35)
#define B_COEF      (-0.47)
#define B_ORIGIN    (-4.48)


static float A;
static float B;

static float g_target_flow_Lpm;

static uint32_t g_Ni; // index motor step corresponding to the beginning of the plateau
static float    g_Ts; // time in second corresponding to the beginning of the plateau

static uint32_t     g_plateau_start_ms; // time in milisecond corresponding to the beginning of the plateau

static const float g_calibration_step_s =  CALIBRATION_STEP_US*0.000001;

//----------------------------------------------------------
// Private functions prototypes
//----------------------------------------------------------

static float    compte_motor_step_time(uint32_t step_number, float desired_flow_Ls, float A, float B);
static bool     pid(float target_flow_Lpm, float flow_samples_period_ms, uint32_t flow_samples_count, const float* flow_samples, uint32_t plateau_start_ms, float* A, float* B);
// static bool     get_plateau(float* samples, uint32_t samples_len, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound);
// static bool     get_plateau2(float* samples, uint32_t samples_len, uint32_t* low_bound, uint32_t* high_bound);
// static bool     get_plateau3(float* samples, uint32_t samples_len, uint32_t* low_bound, uint32_t* high_bound);
// static bool     get_plateau4(float* samples, uint32_t samples_len, uint32_t* low_bound, uint32_t* high_bound);
static float    linear_fit(const float* samples, uint32_t samples_len, float* a, float* b);


static float    get_A_guess(float flow_setpoint_lpm);
static float    get_B_guess(float flow_setpoint_lpm);


//----------------------------------------------------------
// Public variables
//----------------------------------------------------------

//----------------------------------------------------------
// Public functions
//----------------------------------------------------------

// Initialize the adaptation engine.
// Called before the recovid starts the breathing cycles.
void adaptation_prepare() {
    A= 103.689; //a; //1.5; //a; //3.577; //3.577;
    B= -33.674; //b; //-0.05; //b; //-(MOTOR_MIN_STEP_US*2)*0.000001 +  MOTOR_MIN_STEP_US * 0.000001;

    g_target_flow_Lpm=0;
    g_Ni = 0;
}

// Compute the motor step table based on targeted Vmax, VT, and previous flow samples.
// fills the motor step table and return the number of steps.
uint32_t adaptation(
    const settings_t* p_settings, 
    const insuflation_samples_t* p_samples, 
    uint32_t motor_max_steps, 
    uint32_t* motor_steps_us)
{
#ifdef DEBUG_ADAPTATION
            for(uint32_t t=0; t<p_samples->count; ++t) {
                if(t) adpt_printf(",");
                adpt_printf("%ld",(int32_t)(p_samples->Pdiff_Lpm[t]*1000));
            }
            adpt_printf("\n");
#endif
    
    if(g_target_flow_Lpm!=p_settings->Vmax_Lpm) 
    {
        g_target_flow_Lpm= p_settings->Vmax_Lpm;
        A= get_A_guess(g_target_flow_Lpm);
        B= get_B_guess(g_target_flow_Lpm);

        A = 496.417; //342.080;
        B = -68.140; //-25.016;

        g_Ni=0;
        g_Ts=0;
    } else {
        if(!pid(g_target_flow_Lpm, p_samples->T_ms, p_samples->count, p_samples->Pdiff_Lpm, g_plateau_start_ms, &A, &B))
        {
            g_Ni=0;
        }
    }

    // Compute ideal motor steps.
    for(uint32_t step=0; step<motor_max_steps; ++step) {
        float step_us= compte_motor_step_time(step, g_target_flow_Lpm, A, B);
        if(step_us<MOTOR_MIN_STEP_US) 
        {
            // pad to max speed
            step_us= MOTOR_MIN_STEP_US;
        } 
        motor_steps_us[step]= (uint32_t) (step_us);
    }
    // Add acceleration and find plateau start time
    g_plateau_start_ms = 0;
    float T_ms=0;
    #define ACCEL_US  4000
    float step_us= MOTOR_MIN_STEP_US*2.5;
    for(uint32_t step=0; step<motor_max_steps; ++step) 
    {
        if(motor_steps_us[step]>=(uint32_t)step_us) 
        {
            T_ms+= motor_steps_us[step]*0.001;
            if(motor_steps_us[step]> MOTOR_MIN_STEP_US)
            {
                g_plateau_start_ms= T_ms;
                break;
            }
            continue;
        }
        T_ms+= step_us*0.001;
        motor_steps_us[step]= (uint32_t) step_us;
        step_us-= (step_us*0.000001)*ACCEL_US;
        if(step_us<MOTOR_MIN_STEP_US) 
        {
            // limit to max speed
            step_us= MOTOR_MIN_STEP_US;
        }
    }

#ifdef DEBUG_ADAPTATION
    for(uint32_t t=0; t<motor_max_steps; t+=12) {
        if(t!=0) adpt_printf(",");
        adpt_printf("%lu", motor_steps_us[t]);
    }
    adpt_printf("\n");
#endif

    return motor_max_steps;
}

//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

static float compte_motor_step_time(uint32_t step_number, float desired_flow_Lpm, float A, float B) {
	float step_us = /* (0.000001*0.001*step_number*step_number ) */ + (A*g_calibration_step_s*g_calibration_step_s*step_number) + B * g_calibration_step_s;
  	step_us = 1000000 * step_us / desired_flow_Lpm;
	return step_us;
}


static bool pid(float target_flow_Lpm, float flow_samples_period_ms, uint32_t flow_samples_count, const float* flow_samples_Lpm, uint32_t plateau_start_ms, float* A, float* B) {
  if(flow_samples_count==0) 
  {
      return false;
  }
//************************************************* PID ZONE ********************************************//
    // Compute average flow and slope to adjust A and B coefficients
    float P_plateau_slope = 0.1;
    float P_plateau_mean = 0.1;

    uint32_t low;
    uint32_t high;

    //get_plateau(flow_samples_Lpm, flow_samples_count, 10, &low, &high);
    // get_plateau2(flow_samples_Lpm, flow_samples_count, &low, &high);
    // if(!get_plateau3(flow_samples_Lpm, flow_samples_count, &low, &high))
    // {
    //     // No plateau found !!
    //     // Skip adaptation
    //     return false;
    // }


    high= flow_samples_count-1;
    low= plateau_start_ms==0? flow_samples_count/2: plateau_start_ms/flow_samples_period_ms;
    if(low>=high)
    {
        low= high/2;
    }


    float origin;
    float plateau_slope;
    linear_fit(&flow_samples_Lpm[low], high-low+1, &plateau_slope, &origin);

    // scale slope from sample_idx to seconds
    plateau_slope= plateau_slope*1000/flow_samples_period_ms;

    float plateau_mean = 0;
    for(uint32_t i=low; i<high; i++) {
        plateau_mean += flow_samples_Lpm[i];
    }
    plateau_mean = plateau_mean/(high-low);
    float error_mean = plateau_mean - (target_flow_Lpm);
    adpt_printf("plateau [%lu / %lu]\n",low, high);
    adpt_printf("slope : %ld\n",(int32_t)(1000*plateau_slope));
    adpt_printf("mean  : %ld\n",(int32_t)(1000*plateau_mean));
    adpt_printf("error : %ld\n",(int32_t)(1000*error_mean));
    
    float previous_A = *A;
    *A += plateau_slope * P_plateau_slope;
    
    // // EXPERIMENTAL : used to compensate the average speed when modifying A
    //  adpt_printf("Adapting B with Ni= %lu\n",Ni);
    //  *B += (previous_A -*A)*g_calibration_step_s*Ni;

    *B += error_mean * P_plateau_mean;

    // // Compute the new Ts (plateau start time)
    // *Ts= low*flow_samples_period_s;

    adpt_printf("A = %ld\n", (int32_t)(1000*(*A)));
    adpt_printf("B = %ld\n", (int32_t)(1000*(*B)));

    return true;
}

// Compute slope of samples fetched with specified time_step
// Returns 	R  if fit is ok
// 			-1 if fit is not possible
float linear_fit(const float* samples, uint32_t samples_len, float* a, float *b){
	float sumx=0,sumy=0,sumxy=0,sumx2=0, sumy2=0;
    float x,y;
	for(uint32_t i=00;i<samples_len;i++) {
        x= i;
        y= samples[i];
		sumx  = sumx + x;
		sumx2 = sumx2 + (x*x);
		sumy  = sumy + y;
		sumy2 = sumy2 + (y*y);
		sumxy = sumxy + (x*y);
	}
	float denom = (samples_len * sumx2 - (sumx * sumx));
	if(denom == 0.) {
		adpt_printf("no fit\n");
		return -1;
	}
	// compute slope a
	*a = (samples_len * sumxy  -  sumx * sumy) / denom;
    *b = (sumy - (*a)*sumx)/samples_len;

	// compute correlation coefficient
	return (sumxy - sumx * sumy / samples_len) / sqrtf((sumx2 - (sumx*sumx)/samples_len) * (sumy2 - (sumy*sumy)/samples_len));
}

// static bool get_plateau(float* samples, uint32_t samples_len, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound)
// {
// 	if(windows_number < 2 || windows_number > 30) {return -1;}
// 	float slopes[30];
// 	*high_bound = samples_len-1;
// 	// Compute slope for time windows to detect when signal start increasing/decreasing
// 	for(uint32_t window=0; window<windows_number; window++) 
//     {
//         float a,b;
// 		float r = linear_fit(&samples[window*(samples_len/windows_number)], samples_len/windows_number, &a, &b);
//         slopes[window]=a;
// 		adpt_printf("%ld    ", (int32_t)(slopes[window] * 1000));
// 	}
// 	adpt_printf("\n");
// 	for(uint32_t window=1; window<windows_number; window++) {
// 		float delta_slope = slopes[window-1] - slopes[window];
// 		if(delta_slope > 60.) {
// 			*low_bound = (uint32_t)((samples_len/windows_number)*(window+1));
// 			adpt_printf("plateau begin at %lu over %lu points\n", *low_bound, (uint32_t)samples_len);
// 			return true;
// 		}
// 	}
// 	*low_bound = (uint32_t)(samples_len/2);
// 	adpt_printf("No plateau found\n");
// 	return false;
// }

// static bool get_plateau2(float* samples, uint32_t samples_len, uint32_t* low_bound, uint32_t* high_bound)
// {
//     // First we have to dismiss the first "zero flux" points
//     uint32_t firstNonZeroIndex = 0;
//     while(firstNonZeroIndex< samples_len && samples[firstNonZeroIndex++] < 0.001);

//     // Now we try to find the best midpoint to fit two lines to flow data
//     // TODO : dismiss the first and last N points
//     uint32_t N = 10;
//     int32_t samples_len_no_zeros = samples_len - firstNonZeroIndex;
//     // TODO Check samples_len_no_zeros !!

//     float bestRScore = 0.;
//     float bestSlopes[2];
//     uint32_t bestMidpointIndex = 0;
//     // For each midpoint index
//     uint32_t midpointIndex;
//     for(midpointIndex=samples_len*1/3; midpointIndex<samples_len-N; ++midpointIndex) 
//     {
//         // Compute the two slopes
//         float slope1, origin1;
//         float slope2, origin2;
//         float r1 = linear_fit(&samples[firstNonZeroIndex], midpointIndex-firstNonZeroIndex+1, &slope1, &origin1);
//         float r2 = linear_fit(&samples[midpointIndex], samples_len-midpointIndex, &slope2, &origin2);
//         // Compute score based on fit RSQ and points number
//         float score = (r1 * (float)(midpointIndex-firstNonZeroIndex+1)/(float)(samples_len_no_zeros)) + (r2 * (float)(samples_len-midpointIndex)/(float)(samples_len_no_zeros) );
// //        float score = (r1 * 2.0/3.0) + (r2 * 1.0/3.0);
//         //adpt_printf("plateau: r1=%lu, r2=%lu, score=%lu\n", (uint32_t)(r1*1000),(uint32_t)(r2*1000),(uint32_t)(score*1000) )
//         if(score > bestRScore) 
//         {
//             bestRScore = score;
//             bestMidpointIndex = midpointIndex;
//             bestSlopes[0] = slope1;
//             bestSlopes[1] = slope2;
//         }
//     }
//     *high_bound = samples_len-1;
//     *low_bound = (uint32_t)(bestMidpointIndex);

//     adpt_printf("plateau begin at %lu and ends at %lu with score %lu\n", *low_bound, *high_bound, (uint32_t)(1000*bestRScore));
//     return true;
// }

// static bool get_plateau3(float* samples, uint32_t samples_len, uint32_t* low_bound, uint32_t* high_bound)
// {
//     // First we have to dismiss the first "zero flux" points
//     uint32_t firstNonZeroIndex = 0;
//     while(firstNonZeroIndex< samples_len && samples[firstNonZeroIndex++] < 0.001);

//     if(samples_len-firstNonZeroIndex<3) 
//     {
//         adpt_printf("Too few samples !!\n");
//         *low_bound=0;
//         *high_bound=0;
//         return false;
//     }
//     // Now we try to find the best midpoint to fit two lines to flow data
//     // TODO : dismiss the first and last N points
//     uint32_t N;
//     int32_t samples_len_no_zeros = samples_len - firstNonZeroIndex;
//     // TODO Check samples_len_no_zeros !!

//     if(samples_len_no_zeros<40)
//     {
//         adpt_printf("Very few samples !!\n");
//         N= samples_len_no_zeros/2;
//     } 
//     else 
//     {
//         N=20;        
//     }
//     adpt_printf("considering %lu first and last samples to find plateau !!\n", N);

//     float slope1, origin1;
//     float slope2, origin2;
//     float r1 = linear_fit(&samples[firstNonZeroIndex], N, &slope1, &origin1);
//     float r2 = linear_fit(&samples[samples_len-N], N, &slope2, &origin2);

//     if(slope1==slope2) 
//     {
//         *low_bound = samples_len/2;
//     }
//     else
//     {
//         origin1-= slope1*firstNonZeroIndex;
//         origin2-= slope2*(samples_len-N);

//         float intersection = ((origin2-origin1)/(slope1-slope2));
//         if(intersection<firstNonZeroIndex || intersection>samples_len-N)
//         {
//             *low_bound = samples_len-N;
//         }
//         else
//         {
//             *low_bound = (uint32_t) intersection;
//         }
//     }
    
//     *high_bound = samples_len-1;

// #ifdef DEBUG_ADAPTATION
//     for(int t=0; t<samples_len-1; ++t) {
//         adpt_printf("%ld, ", (int32_t)(slope1*t+origin1)*1000);
//     }
//     adpt_printf("\n");
//     for(int t=0; t<samples_len-1; ++t) {
//         adpt_printf("%ld, ", (int32_t)(slope2*t+origin2)*1000);
//     }
//     adpt_printf("\n");

//     float slope,origin;
//     linear_fit(&samples[*low_bound], (*high_bound)-(*low_bound), &slope,&origin);
//     origin-= slope*(*low_bound);
//     for(int t=0; t<samples_len-1; ++t) {
//         adpt_printf("%ld, ", (int32_t)(slope*t+origin)*1000);
//     }
//     adpt_printf("\n");

//     adpt_printf("plateau [%lu / %lu] | [%ld / %ld] [%ld / %ld]\n", *low_bound, *high_bound, (int32_t)(slope1*1000), (int32_t)(origin1*1000), (int32_t)(slope2*1000), (int32_t)(origin2*1000));
// #endif

//     return true;
// }

// static bool get_plateau4(float* samples, uint32_t samples_len, uint32_t* low_bound, uint32_t* high_bound)
// {
//     // First we have to dismiss the first "zero flux" points
//     uint32_t firstNonZeroIndex = 0;
//     while(firstNonZeroIndex< samples_len && samples[firstNonZeroIndex++] < 1);

//     if(samples_len-firstNonZeroIndex<15) 
//     {
//         adpt_printf("Too few samples !!\n");
//         *low_bound=0;
//         *high_bound=0;
//         return false;
//     }
//     // Now we try to find the best midpoint to fit two lines to flow data
//     // TODO : dismiss the first and last N points
//     uint32_t N1,N2;
//     int32_t samples_len_no_zeros = samples_len - firstNonZeroIndex;
//     // TODO Check samples_len_no_zeros !!

//     if(samples_len_no_zeros<45)
//     {
//         adpt_printf("Very few samples !!\n");
//         N1 = N2 = samples_len_no_zeros/3;
//     } 
//     else 
//     {
//         N1=10;
//         N2=samples_len_no_zeros*1/2;
//     }

//     float slope1, origin1;
//     float slope2, origin2;
//     float r1 = linear_fit(&samples[firstNonZeroIndex], N1, &slope1, &origin1);
//     float r2 = linear_fit(&samples[samples_len-N2], N2, &slope2, &origin2);
//     origin1-= slope1*firstNonZeroIndex;
//     origin2-= slope2*(samples_len-N2);

//     // if(slope1==slope2) 
//     // {
//     //     *low_bound = samples_len/2;
//     // }
//     // else
//     // {
//     //     origin1-= slope1*firstNonZeroIndex;
//     //     origin2-= slope2*(samples_len-N2);

//     //     float intersection = ((origin2-origin1)/(slope1-slope2));
//     //     if(intersection<firstNonZeroIndex || intersection>samples_len-N2)
//     //     {
//     //         *low_bound = samples_len-N2;
//     //     }
//     //     else
//     //     {
//     //         *low_bound = (uint32_t) intersection;
//     //     }
//     // }
    
//     *low_bound =  samples_len-N2;
//     *high_bound = samples_len;

// #ifdef DEBUG_ADAPTATION
//     for(int t=0; t<samples_len-1; ++t) {
//         if(t) adpt_printf(",");
//         adpt_printf("%ld", (int32_t)(slope1*t+origin1)*1000);
//     }
//     adpt_printf("\n");
//     for(int t=0; t<samples_len-1; ++t) {
//         if(t) adpt_printf(",");
//         adpt_printf("%ld", (int32_t)(slope2*t+origin2)*1000);
//     }
//     adpt_printf("\n");
// #endif
//     // float slope,origin;
//     // linear_fit(&samples[*low_bound], (*high_bound)-(*low_bound), &slope,&origin);
//     // origin-= slope*(*low_bound);

//     // for(int t=0; t<samples_len-1; ++t) {
//     //     if(t) adpt_printf(",");
//     //     adpt_printf("%ld", (int32_t)(slope*t+origin)*1000);
//     // }
//     // adpt_printf("\n");

//     adpt_printf("considering %lu first and %lu last samples to find plateau !!\n", N1, N2);


//     adpt_printf("plateau [%lu / %lu] | [%ld / %ld] [%ld / %ld]\n", *low_bound, *high_bound, (int32_t)(slope1*1000), (int32_t)(origin1*1000), (int32_t)(slope2*1000), (int32_t)(origin2*1000));
//     return true;
// }


static float get_A_guess(float flow_setpoint_lpm){
    return A_COEF*flow_setpoint_lpm + A_ORIGIN;
}

static float get_B_guess(float flow_setpoint_lpm){
    return B_COEF*flow_setpoint_lpm + B_ORIGIN;
}