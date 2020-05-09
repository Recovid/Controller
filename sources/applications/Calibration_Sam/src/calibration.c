#include "common.h"
#include "config.h"
#include "platform.h"
#include "calibration.h"

#include <math.h>
#include <string.h>

//----------------------------------------------------------
// Private defines
//----------------------------------------------------------


#define SAMPLING_PERIOD_MS      (5)
#define PEP_SAMPLES_COUNT       (100 / SAMPLING_PERIOD_MS)          // moyenne glissante sur les 100ms dernieres de l'expi
#define PPLAT_SAMPLES_COUNT     (50  / SAMPLING_PERIOD_MS)          // moyenne glissante sur les 50ms dernieres de plat
#define MAX_PAW_SAMPLES         MAX(PEP_SAMPLES_COUNT, PPLAT_SAMPLES_COUNT)
#define MAX_PDIFF_SAMPLES       (2000 / SAMPLING_PERIOD_MS)

#define CALIBRATION_STEP_US    (MOTOR_MIN_STEP_US*2)


//----------------------------------------------------------
// Private typedefs
//----------------------------------------------------------

//----------------------------------------------------------
// Private variables
//----------------------------------------------------------



static float                g_Paw_cmH2O_samples[MAX_PAW_SAMPLES];
static volatile uint16_t    g_Paw_cmH2O_sample_count;
static volatile uint16_t    g_Paw_cmH2O_sample_idx;

static float                g_Pdiff_Lpm_samples[MAX_PDIFF_SAMPLES];
static volatile uint16_t    g_Pdiff_Lpm_sample_count;

static uint32_t g_motor_steps_us[MOTOR_MAX_STEPS] ; 


//----------------------------------------------------------
// Private functions prototypes
//----------------------------------------------------------

static float    compte_motor_step_time(uint32_t step_number, float desired_flow_Ls, float A, float B, float speed);
static void     pid(float target_flow_Lpm, float flow_samples_period_s, uint32_t flow_samples_count, float* flow_samples,  float* A, float* B);
static int32_t  get_plateau(float* samples, uint32_t samples_len, float flow_samples_period_s, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound);
static float    linear_fit(float* samples, uint32_t samples_len, float flow_samples_period_s, float* slope);



static void init_Paw_cmH2O_sampling();
static void init_Pdiff_Lpm_sampling();
static float get_avg_Paw_cmH2O(uint16_t count);


static void samplingCallback(TimerHandle_t xTimer);
static void calibration_run(void *args);

//----------------------------------------------------------
// Public variables
//----------------------------------------------------------
static TaskHandle_t        g_calibrationTask;
static TimerHandle_t       g_samplingTimer;

#ifdef DEBUG
SemaphoreHandle_t dbgMutex;
#endif

//----------------------------------------------------------
// Public functions
//----------------------------------------------------------

void application_main()
{

#ifdef DEBUG
    printf("Starting Calibration\n");

    dbgMutex = xSemaphoreCreateBinary();
    if(NULL == dbgMutex) {
        printf("Unable to create dbgMutex\n");
        return;
    }
    xSemaphoreGive(dbgMutex);
#endif

    if(calibration_init() == false) 
    {
        return;
    }

#ifdef DEBUG
    printf("Starting scheduler\n");
#endif

    // start scheduler
    vTaskStartScheduler();

    // We should never get here
}



bool calibration_init() {
#ifdef DEBUG
    printf("Initializing\n");
#endif

    g_samplingTimer = xTimerCreate("SamplingTimer", SAMPLING_PERIOD_MS / portTICK_PERIOD_MS, pdTRUE, 0, samplingCallback);
    if ( NULL == g_samplingTimer)
    {
#ifdef DEBUG
        printf("Unable to create samplingTimer\n");
#endif
        return false;
    }

    if (xTaskCreate(calibration_run, "Calibration", CALIBRATION_TASK_STACK_SIZE, NULL, CALIBRATION_TASK_PRIORITY, &g_calibrationTask) != pdTRUE)
    {
#ifdef DEBUG
        printf("Unable to create calibrationTask\n");
#endif
        return false;
    }

#ifdef DEBUG
    printf("Initialized\n");
#endif
    return true;
}


//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

static void calibration_run(void *args)
{
    UNUSED(args);

    init_indicators();
    init_valve();
    init_sensors();
    init_motor();

    while (true)
    {
        dbg_printf("Waiting for failsafe signal\n");
        while (is_Failsafe_Enabled())
        {
            wait_ms(200);
        }


        dbg_printf("Calibration...\n");
        int16_t     iterations = 3;
        float       A,B;
        float       slope = 0; // slope of flow(t) cruve
        float       originFlow = 0; // origin flow of flow(t) curve
        uint32_t    steps;

        steps= (uint32_t) (MOTOR_MAX_STEPS*80/100);
        for(uint32_t t=0; t<steps; ++t) 
        {
            g_motor_steps_us[t]= (uint32_t)CALIBRATION_STEP_US;
        }


        // Calibrate slope
        dbg_printf("---------- Calibrating slope ---------------\n");
        for(int iter=0; iter<iterations; ++iter) {
            init_Paw_cmH2O_sampling();
            init_Pdiff_Lpm_sampling();
            
            // HIGH PEEP
            valve_inhale();

            reset_Vol_mL();
            motor_press(g_motor_steps_us, steps);
            xTimerReset(g_samplingTimer, 10/portTICK_PERIOD_MS);
            wait_ms(200); // skip first 200ms
            while(is_motor_moving()) wait_ms(5);
            xTimerStop(g_samplingTimer, 10/portTICK_PERIOD_MS);
            wait_ms(500);
            float volumeIT = read_Vol_mL();
            dbg_printf("volume = %lu ml\n", (uint32_t)(volumeIT));
            // LOW PEEP
            valve_exhale();
            motor_release(MOTOR_RELEASE_STEP_US);
            while(!is_motor_home());
            wait_ms(2000);

            float a = 0;
            float r = linear_fit(g_Pdiff_Lpm_samples, g_Pdiff_Lpm_sample_count, SAMPLING_PERIOD_MS*0.001, &a);
            dbg_printf("a=%lu\n", (uint32_t)(1000.*a));
            dbg_printf("r=%lu\n", (uint32_t)(1000.*r));
            slope += a / (float)iterations;
        }
        A = slope;
        dbg_printf("A=%lu\n", (uint32_t)(1000*A));


        // Calibrate originFlow
        dbg_printf("---------- Calibrating B ---------------\n");
        for(int iter=0; iter<iterations; ++iter) {
            // HIGH PEEP
            valve_inhale();
            reset_Vol_mL();
            motor_press(g_motor_steps_us, steps);
            while(is_motor_moving()) wait_ms(5);
            wait_ms(1000);
            float volumeIT = read_Vol_mL();
            // LOW PEEP
            valve_exhale();
            motor_release(MOTOR_RELEASE_STEP_US);
            while(!is_motor_home());
            brth_printf("volume = %luml\n", (uint32_t)(volumeIT));
            float b = volumeIT*0.001/((CALIBRATION_STEP_US*0.000001) * steps) - (A * (CALIBRATION_STEP_US*0.000001)*steps / 2.);
            brth_printf("b=%ld\n", (int32_t)(1000*b));
            // Add values for averaging over iterations
            originFlow += b/(float)iterations;
        }
        B = originFlow;
        dbg_printf("B=%ld\n", (int32_t)(1000*B));
        dbg_printf("Calibration...DONE\n");        
    }
}


static void init_Paw_cmH2O_sampling()
{
    taskENTER_CRITICAL();
    memset((void*)g_Paw_cmH2O_samples, 0, sizeof(g_Paw_cmH2O_samples));
    g_Paw_cmH2O_sample_count = 0;
    g_Paw_cmH2O_sample_idx = 0;
    taskEXIT_CRITICAL();
}

static void init_Pdiff_Lpm_sampling()
{
    taskENTER_CRITICAL();
    memset((void*)g_Pdiff_Lpm_samples, 0, sizeof(g_Pdiff_Lpm_samples));
    g_Pdiff_Lpm_sample_count= 0;
    taskEXIT_CRITICAL();
}

static float get_avg_Paw_cmH2O(uint16_t count)
{
    taskENTER_CRITICAL();
    float sum = 0.;
    count = MIN(count, g_Paw_cmH2O_sample_count);
    for (uint16_t i = 1; i <= count; i++)
    {
        sum += g_Paw_cmH2O_samples[ (MAX_PAW_SAMPLES + g_Paw_cmH2O_sample_idx + i - count ) % MAX_PAW_SAMPLES ];
    }
    taskEXIT_CRITICAL();
    return count == 0 ? 0 : sum/count;
}

static void samplingCallback(TimerHandle_t timer) {
    taskENTER_CRITICAL();
    // sample Paw
    g_Paw_cmH2O_samples[g_Paw_cmH2O_sample_idx]= read_Paw_cmH2O();
    g_Paw_cmH2O_sample_idx = (g_Paw_cmH2O_sample_idx + 1) % MAX_PAW_SAMPLES;
    if(g_Paw_cmH2O_sample_count<MAX_PAW_SAMPLES) 
    {
        ++g_Paw_cmH2O_sample_count;
    }

    // sample Pdiff 
    if(g_Pdiff_Lpm_sample_count<MAX_PDIFF_SAMPLES)
    {
        g_Pdiff_Lpm_samples[g_Pdiff_Lpm_sample_count++]= read_Pdiff_Lpm();
    }

    taskEXIT_CRITICAL();
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
    float A,B;

    pid(target_Flow_Lpm, 0.001*flow_samples_period_ms, flow_samples_count, flow_samples_Lpm, &A, &B);

    for(uint32_t t=0; t<motor_max_steps; ++t) {
        motor_steps_us[t]= (uint32_t) compte_motor_step_time(t, target_Flow_Lpm/60.0, A, B, (MOTOR_MIN_STEP_US*2)*0.000001);
    }
    return motor_max_steps;
}

//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

static float compte_motor_step_time(uint32_t step_number, float desired_flow_Ls, float A, float B, float speed) {
	float res = (0.8*A*speed*speed*step_number) + B * speed;
	res = 1000000* res / desired_flow_Ls;
	if (res  < MOTOR_MIN_STEP_US) { 
        res = MOTOR_MIN_STEP_US;
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
			plateau_mean += flow_samples[i]/60;
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
float linear_fit(float* samples, uint32_t samples_len, float flow_samples_period_s, float* slope){
	float sumx=0,sumy=0,sumxy=0,sumx2=0, sumy2=0;
    float x,y;
	for(uint32_t i=00;i<samples_len;i++) {
        x= (float)i * flow_samples_period_s;
        y= samples[i]/60.0;
		sumx  = sumx + x;
		sumx2 = sumx2 + (x*x);
		sumy  = sumy + y;
		sumy2 = sumy2 + (y*y);
		sumxy = sumxy + (x*y);
	}
	float denom = (samples_len * sumx2 - (sumx * sumx));
	if(denom == 0.) {
		brth_printf("Calibration of A is not possible\n");
		return -1;
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
