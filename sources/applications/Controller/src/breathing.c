#include "common.h"
#include "config.h"
#include "defaults.h"
#include "controller.h"
#include "breathing.h"
#include "platform.h"

#include <adaptation.h>
#include <math.h>
#include <string.h>

//----------------------------------------------------------
// Private defines
//----------------------------------------------------------

#define INSUFLATION_PROCESSING_PERIOD_MS    (10)
#define PLATEAU_PROCESSING_PERIOD_MS        (10)
#define EXHALATION_PROCESSING_PERIOD_MS     (10)


#define SAMPLING_PERIOD_MS      (5)
#define PEP_SAMPLES_COUNT       (100 / SAMPLING_PERIOD_MS)          // moyenne glissante sur les 100ms dernieres de l'expi
#define PPLAT_SAMPLES_COUNT     (50  / SAMPLING_PERIOD_MS)          // moyenne glissante sur les 50ms dernieres de plat
#define MAX_PAW_SAMPLES         MAX(PEP_SAMPLES_COUNT, PPLAT_SAMPLES_COUNT)
#define MAX_PDIFF_SAMPLES       (2000 / SAMPLING_PERIOD_MS)


//----------------------------------------------------------
// Private typedefs
//----------------------------------------------------------

typedef enum
{
    None,
    Insuflation,
    Plateau,
    Exhalation
} BreathingState;

//----------------------------------------------------------
// Private variables
//----------------------------------------------------------

static float g_cycle_EoI_ratio;
static float g_cycle_FR_pm;
static float g_cycle_VTe_mL;
static float g_cycle_VTi_mL;
static float g_cycle_VMe_Lpm;
static float g_cycle_Pcrete_cmH2O;
static float g_cycle_Pplat_cmH2O;
static float g_cycle_PEP_cmH2O;

static uint32_t g_setting_T;
static float g_setting_VT;
static float g_setting_VM;
static float g_setting_Pmax;
static uint32_t g_setting_Tinspi_ms;   // Computed based on cycle period T and I/E ratio
static uint32_t g_setting_Tinsu_ms;    // Théoritical Insuflation's time based on volume and theoritical flow
static uint32_t g_setting_Texp_ms;     // Computed based on cycle period T and I/E ratio


static float                g_Paw_cmH2O_samples[MAX_PAW_SAMPLES];
static volatile uint16_t    g_Paw_cmH2O_sample_count;
static volatile uint16_t    g_Paw_cmH2O_sample_idx;

static float                g_Pdiff_Lpm_samples[MAX_PDIFF_SAMPLES];
static volatile uint16_t    g_Pdiff_Lpm_sample_count;

static volatile BreathingState g_state;


static uint32_t g_motor_steps_us[MOTOR_MAX_STEPS] ; 

static volatile bool g_Pdiff_sampling;


//----------------------------------------------------------
// Private functions prototypes
//----------------------------------------------------------
static void regulation_pep();

static void init_Paw_cmH2O_sampling();
static void init_Pdiff_Lpm_sampling();
static float get_avg_Paw_cmH2O(uint16_t count);


static void signal_state(BreathingState newState);
static void signal_pins(bool pause);
static void signal_pexp(bool pause);

static void samplingCallback(TimerHandle_t xTimer);
static void breathing_run(void *args);

static void calibration(float* A, float* B, uint8_t iterations, uint32_t calibration_step_us);

//----------------------------------------------------------
// Public variables
//----------------------------------------------------------
EventGroupHandle_t  g_breathingEvents;
TaskHandle_t        g_breathingTask;
TimerHandle_t       g_samplingTimer;

//----------------------------------------------------------
// Public functions
//----------------------------------------------------------

bool breathing_init() {
#ifdef DEBUG
    printf("BRTH: Initializing\n");
#endif
    g_breathingEvents = xEventGroupCreate();
    if ( NULL == g_breathingEvents)
    {
#ifdef DEBUG
        printf("BRTH: Unable to create breathingEvents\n");
#endif
        return false;
    }

    g_samplingTimer = xTimerCreate("SamplingTimer", SAMPLING_PERIOD_MS / portTICK_PERIOD_MS, pdTRUE, 0, samplingCallback);
    if ( NULL == g_samplingTimer)
    {
#ifdef DEBUG
        printf("BRTH: Unable to create samplingTimer\n");
#endif
        return false;
    }

    if (xTaskCreate(breathing_run, "Breathing", BREATHING_TASK_STACK_SIZE, NULL, BREATHING_TASK_PRIORITY, &g_breathingTask) != pdTRUE)
    {
#ifdef DEBUG
        printf("BRTH: Unable to create breathingTask\n");
#endif
        return false;
    }


    signal_state(None);

#ifdef DEBUG
    printf("BRTH: Initialized\n");
#endif
    return true;
}


float get_cycle_EoI_ratio() { return g_cycle_EoI_ratio; }
float get_cycle_FR_pm() { return g_cycle_FR_pm; }
float get_cycle_VTe_mL() { return g_cycle_VTe_mL; }
float get_cycle_VTi_mL() { return g_cycle_VTi_mL; }
float get_cycle_VMe_Lpm() { return g_cycle_VMe_Lpm; }
float get_cycle_Pcrete_cmH2O() { return g_cycle_Pcrete_cmH2O; }
float get_cycle_Pplat_cmH2O() { return g_cycle_Pplat_cmH2O; }
float get_cycle_PEP_cmH2O() { return g_cycle_PEP_cmH2O; }


//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

static void breathing_run(void *args)
{
    UNUSED(args);
    EventBits_t events;

    while (true)
    {
        signal_state(None);
        brth_printf("BRTH: Standby\n");
        events = xEventGroupWaitBits(g_controllerEvents, BREATHING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY);
        brth_printf("BRTH: Started\n");

        motor_enable(true);

        float a=0,b=0;
#if 0
        brth_printf("BRTH: Calibration...\n");
        calibration(&a, &b, 3, CALIBRATION_STEP_US);
        wait_ms(2000);
#endif
        adaptation_init(a,b);

        // Clear cycle data
        g_cycle_EoI_ratio = 0;
        g_cycle_FR_pm = 0;
        g_cycle_VTe_mL = 0;
        g_cycle_Pcrete_cmH2O = 0;
        g_cycle_Pplat_cmH2O = 0;
        g_cycle_PEP_cmH2O = 0;


        // Init Paw samples
        init_Paw_cmH2O_sampling();
        // Init Pdiff sampling
        init_Pdiff_Lpm_sampling();
        // Start timer
        if( xTimerReset(g_samplingTimer, 20/portTICK_PERIOD_MS) != pdTRUE ) 
        {
            // TODO : What should we do? Raise an alarm ??
            brth_printf("Error timer !!\n");
        }

        // Init state machine
        BreathingState next_state = Insuflation;
        uint32_t inhalation_start_ms;
        uint32_t inhalation_pause_t_ms;

        uint32_t exhalation_start_ms;
        uint32_t exhalation_pause_t_ms;
        do
        {
            brth_printf("BRTH: start cycle\n");

            //--------------------------------------------------------------
            // Inhalation (macro) state
            //--------------------------------------------------------------
            inhalation_start_ms   = get_time_ms();

            //--------------------------------------------------------------
            // Insuflation state: onEntry
            //--------------------------------------------------------------
            signal_state(Insuflation);

            // Get current controller settings
            taskENTER_CRITICAL();
            g_setting_T = get_setting_T_ms();
            g_setting_VT = get_setting_VT_mL();
            g_setting_VM = get_setting_Vmax_Lpm();
            g_setting_Pmax = get_setting_Pmax_cmH2O();
            g_setting_Tinspi_ms = get_setting_Tinspi_ms(); // Computed based on cycle period T and I/E ratio
            g_setting_Tinsu_ms = get_setting_Tinsu_ms();   // Théoritical Insuflation's time based on volume and theoritical flow
            g_setting_Texp_ms = get_setting_Texp_ms();     // Computed based on cycle period T and I/E ratio
            taskEXIT_CRITICAL();

            brth_printf("BRTH: T     : %lu\n", g_setting_T);
            brth_printf("BRTH: VT    : %lu\n", (uint32_t)(g_setting_VT ));
            brth_printf("BRTH: VM    : %lu\n", (uint32_t)(g_setting_VM ));
            brth_printf("BRTH: Pmax  : %lu\n", (uint32_t)(g_setting_Pmax ));
            brth_printf("BRTH: Tinsu : %lu\n", g_setting_Tinsu_ms);
            brth_printf("BRTH: Tinspi: %lu\n", g_setting_Tinspi_ms);
            brth_printf("BRTH: samples: %u\n", g_Pdiff_Lpm_sample_count);
#ifdef DEBUG_BREATHING            
            for(uint32_t t=0; t<g_Pdiff_Lpm_sample_count; ++t) {
                dbg_printf("%lu, ",(uint32_t)g_Pdiff_Lpm_samples[t]*1000);
            }
            dbg_printf("\n");
#endif

            uint32_t nb_steps= adaptation(g_setting_VT, g_setting_VM, SAMPLING_PERIOD_MS, g_Pdiff_Lpm_sample_count, g_Pdiff_Lpm_samples, MOTOR_MAX_STEPS, g_motor_steps_us);

            // Init Pdiff samples
            init_Pdiff_Lpm_sampling();

            valve_inhale();
            reset_Vol_mL();

            // start Pdiff Sampling 
            g_Pdiff_sampling=true;
            motor_press(g_motor_steps_us, nb_steps);

            float cycle_Pcrete_cmH2O= 0;

            // Insuflation state: do
            // Pdiff and Paw sampling will be done by the samplingTimer in background
            do
            {
                wait_ms(INSUFLATION_PROCESSING_PERIOD_MS);
                cycle_Pcrete_cmH2O = MAX(cycle_Pcrete_cmH2O, read_Paw_cmH2O());
                if ( g_setting_Pmax <= cycle_Pcrete_cmH2O)
                {
                    brth_printf("BRTH: Paw [%ld]> Pmax --> Exhalation\n", (int32_t)(g_cycle_Pcrete_cmH2O));
                    // Clear sampled data
                    g_Pdiff_sampling=false;
                    init_Pdiff_Lpm_sampling();
                    next_state = Exhalation;
                }
                else 
                if (g_setting_VT <= read_Vol_mL())
                {
                    brth_printf("BRTH: vol [%ld]>= VT --> Plateau\n", (int32_t)(read_Vol_mL()));
                    next_state = Plateau;
                }
                // else 
                // if (g_setting_Tinsu_ms * 1.2 <= (get_time_ms() - inhalation_start_ms))   // TODO: see how it fits with the adaptation
                // {
                //     brth_printf("BRTH: dt [%lu]>= Ti\n", (get_time_ms() - inhalation_start_ms));
                //     next_state = Plateau;
                // }
            } while (Insuflation == next_state);
            
            // Insuflation state: onExit
            g_Pdiff_sampling=false;
            motor_stop();
            bool released=false;
            if (Plateau == next_state)
            {
                // Plateau state : onEntry
                signal_state(Plateau);
                inhalation_pause_t_ms = 0;
                float release_Pdiff= read_Pdiff_Lpm()*0.25;
                // Plateau state: do
                do {
                    wait_ms(PLATEAU_PROCESSING_PERIOD_MS);
                    if(!released && read_Pdiff_Lpm()<release_Pdiff) {
                        motor_release(MOTOR_RELEASE_STEP_US);
                        released=true;
                    }
                    if(is_command_Tpins_expired())
                    {
                        signal_pins(false);                  
                    }
                    else
                    {   
                        // Update cycle Pplat with current average
                        g_cycle_Pplat_cmH2O = get_avg_Paw_cmH2O(PPLAT_SAMPLES_COUNT);
                        // Count pause time
                        inhalation_pause_t_ms+= PLATEAU_PROCESSING_PERIOD_MS;                  
                        signal_pins(true);
                    }
                    
                    cycle_Pcrete_cmH2O = MAX(cycle_Pcrete_cmH2O, read_Paw_cmH2O());

                    if (g_setting_Pmax <= cycle_Pcrete_cmH2O)
                    {
                        brth_printf("BRTH: Paw [%ld]> Pmax --> Exhalation\n", (int32_t)(cycle_Pcrete_cmH2O));
                        next_state= Exhalation;
                    }
                    else if ((g_setting_Tinspi_ms + DEFAULT_Tpins_max_ms) <= (get_time_ms() - inhalation_start_ms) )
                    {
                        brth_print("BRTH: MAX Tpins exceeded\n");
                        next_state= Exhalation;
                    }
                    else if ( (g_setting_Tinspi_ms + inhalation_pause_t_ms) <= (get_time_ms() - inhalation_start_ms) )
                    {
                        brth_print("BRTH: dt > Tinspi\n");
                        next_state= Exhalation;
                    }
                } while (Plateau == next_state);

                // Plateau state: onExit
                // get the average Paw over the last Pplat samples count
                g_cycle_Pplat_cmH2O = get_avg_Paw_cmH2O(PPLAT_SAMPLES_COUNT);
                // Reset Inhalation pause signal
                signal_pins(false);
            }

            // End of (macro) state Inhalation            
            g_cycle_VTi_mL = read_Vol_mL();
            if(!released) motor_release(MOTOR_RELEASE_STEP_US);

            // Exhalation state: onEntry
            signal_state(Exhalation);
            valve_exhale();
            float VTe_start_mL = read_Vol_mL();
            exhalation_start_ms = get_time_ms();
            exhalation_pause_t_ms = 0;
            // Exhalation state: do
            do {
                wait_ms(EXHALATION_PROCESSING_PERIOD_MS);
                if (is_command_Tpexp_expired())
                {
                    // Release valve
                    valve_exhale();
                    signal_pexp(false);
                }
                else
                {
                    // Block valve
                    valve_inhale();
                    // Update cycle PEP with current average
                    g_cycle_PEP_cmH2O = get_avg_Paw_cmH2O(PEP_SAMPLES_COUNT);
                    exhalation_pause_t_ms+=EXHALATION_PROCESSING_PERIOD_MS;
                    signal_pexp(true);
                }
                
                if ((g_setting_Texp_ms + DEFAULT_Tpexp_max_ms) <= (get_time_ms() - exhalation_start_ms)) 
                {
                    next_state= Insuflation;
                }
                else if (g_setting_Texp_ms + exhalation_pause_t_ms <= (get_time_ms() - exhalation_start_ms))
                {
                    brth_print("BRTH: Tpexp expired && (T <= dt)\n");
                    next_state= Insuflation;
                }
            } while (Exhalation == next_state);            
            
            // Exhalation state: onExit
            // Compute cycle data
            uint32_t t_ms = get_time_ms();
            g_cycle_PEP_cmH2O = get_avg_Paw_cmH2O(PEP_SAMPLES_COUNT);
            g_cycle_Pcrete_cmH2O= cycle_Pcrete_cmH2O;
            g_cycle_EoI_ratio = (float)(t_ms - exhalation_start_ms - exhalation_pause_t_ms) / (exhalation_start_ms - inhalation_start_ms - inhalation_pause_t_ms);
            g_cycle_FR_pm = 1. / (((float)(t_ms - inhalation_start_ms - inhalation_pause_t_ms - exhalation_pause_t_ms)) / 1000 / 60);
            g_cycle_VTe_mL = VTe_start_mL - read_Vol_mL();
            g_cycle_VMe_Lpm = (g_cycle_VTe_mL / 1000) * g_cycle_FR_pm;

            // Notify system that new cycle data is available.
            xEventGroupSetBits(g_breathingEvents, BRTH_CYCLE_UPDATED);

            // Proceed to the PEP regulation
            //regulation_pep();

            // Check if controller asked us to stop.
            events = xEventGroupGetBits(g_controllerEvents);
        } while ((events & BREATHING_RUN_FLAG) != 0);

        // stop Sampling (Paw and Pdiff)
        if( xTimerStop(g_samplingTimer, 20/portTICK_PERIOD_MS) != pdTRUE ) 
        {
            // TODO : What should we do? Raise an alarm ??
        }

        signal_state(None);
        brth_printf("BRTH: Stopping\n");

        wait_ms(200);
        xEventGroupSetBits(g_controllerEvents, BREATHING_STOPPED_FLAG);
    }
}


static void signal_pins(bool pause) {
    if(pause) 
    {
        xEventGroupSetBits(g_breathingEvents, BRTH_CYCLE_PINS);
    }
    else
    {
        xEventGroupClearBits(g_breathingEvents, BRTH_CYCLE_PINS);
    }
}

static void signal_pexp(bool pause) {
    if(pause) 
    {
        xEventGroupSetBits(g_breathingEvents, BRTH_CYCLE_PEXP);
    }
    else
    {
        xEventGroupClearBits(g_breathingEvents, BRTH_CYCLE_PEXP);
    }
}

static void signal_state(BreathingState state)
{
    EventBits_t brthState = 0;
    switch (state)
    {
    case None:
        brthState = 0;
        brth_printf("BRTH: None\n");
        break;
    case Insuflation:
        brthState = BRTH_CYCLE_INSUFLATION;
        brth_printf("BRTH: Insuflation\n");
        break;
    case Plateau:
        brthState = BRTH_CYCLE_PLATEAU;
        brth_printf("BRTH: Plateau\n");
        break;
    case Exhalation:
        brthState = BRTH_CYCLE_EXHALATION;
        brth_printf("BRTH: Exhalation\n");
        break;
    }
    // Inform system about current state
    g_state= state;
    xEventGroupClearBits(g_breathingEvents, (BRTH_CYCLE_INSUFLATION | BRTH_CYCLE_PLATEAU | BRTH_CYCLE_EXHALATION | BRTH_CYCLE_PINS | BRTH_CYCLE_PEXP));
    xEventGroupSetBits(g_breathingEvents, brthState);
}


static void regulation_pep()
{
    float pep_objective = get_setting_PEP_cmH2O(); // TODO: is it really what we want ? Should we use the setting retreived at the beginning of the cycle instead ?
    float current_pep = get_cycle_PEP_cmH2O();
    int relative_pep_mmH2O = (int) (pep_objective * 10 - current_pep * 10);
    if (abs(relative_pep_mmH2O) > 3)
    {
        brth_printf(">>>>>>>>>>>>>>>>>>>> Adjusting PEP: %ld\n", (int)(SIGN(relative_pep_mmH2O)*MIN(5,relative_pep_mmH2O)));
        motor_pep_stop();
        motor_pep_move(SIGN(relative_pep_mmH2O)*MIN(5,relative_pep_mmH2O));
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
    g_Pdiff_sampling= false;
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

    // sample Pdiff only during Insuflation state
    if(g_Pdiff_sampling==true) 
    {
        if(g_Pdiff_Lpm_sample_count<MAX_PDIFF_SAMPLES)
        {
            g_Pdiff_Lpm_samples[g_Pdiff_Lpm_sample_count]= read_Pdiff_Lpm();
            ++g_Pdiff_Lpm_sample_count;
        }
    }

    taskEXIT_CRITICAL();
}


//Flow(t) = A*t + B
static void calibration(float* A, float* B, uint8_t iterations, uint32_t calibration_step_us) {
	float slope = 0; // slope of flow(t) cruve
	float originFlow = 0; // origin flow of flow(t) curve
	uint32_t steps;

    steps= (uint32_t) (MOTOR_MAX_STEPS*66/100);
    for(uint32_t t=0; t<steps; ++t) 
    {
        g_motor_steps_us[t]= (uint32_t)calibration_step_us;
    }


	// Calibrate slope
	brth_printf("---------- Calibrating slope ---------------\n");
	for(int iter=0; iter<iterations; ++iter) {
        init_Paw_cmH2O_sampling();
        init_Pdiff_Lpm_sampling();
    	
        // HIGH PEEP
        valve_inhale();

        reset_Vol_mL();
        xTimerReset(g_samplingTimer, 10/portTICK_PERIOD_MS);
		motor_press(g_motor_steps_us, steps);
        wait_ms(200); // skip first 200ms
        g_Pdiff_sampling= true;
		while(is_motor_moving()) wait_ms(5);
        g_Pdiff_sampling= false;
        xTimerStop(g_samplingTimer, 10/portTICK_PERIOD_MS);
        g_state= None;
		wait_ms(500);
		float volumeIT = read_Vol_mL();
		brth_printf("volume = %lu ml\n", (uint32_t)(volumeIT));
        // LOW PEEP
        valve_exhale();
        motor_release(MOTOR_RELEASE_STEP_US);
		while(!is_motor_home());
        wait_ms(2000);

		float a = 0, b = 0;
		float r = linear_fit(g_Pdiff_Lpm_samples, g_Pdiff_Lpm_sample_count, &a, &b);

        // scale slope from sample idx to to seconds
        a= a/(0.001*SAMPLING_PERIOD_MS);

        for(int t=0; t<g_Pdiff_Lpm_sample_count; ++t ) 
        {
            brth_printf("%ld, ", (int32_t)(g_Pdiff_Lpm_samples[t]));
        }
        brth_printf("\n");
        for(int t=0; t<g_Pdiff_Lpm_sample_count; ++t ) 
        {
            brth_printf("%ld, ", (int32_t)(a*t+b));
        }
        brth_printf("\n");

		brth_printf("a=%ld b=%ld\n", (int32_t)(1000.*a), (int32_t)(1000.*b));
		brth_printf("r=%lu\n", (uint32_t)(1000.*r));
		slope += a / (float)iterations;
	}
	*A = slope;
	brth_printf("A=%lu\n", (uint32_t)(1000.* *A));


	// Calibrate originFlow
	brth_printf("---------- Calibrating B ---------------\n");
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
		float b = volumeIT*0.001/((CALIBRATION_STEP_US*0.000001) * steps) - (*A * (CALIBRATION_STEP_US*0.000001)*steps / 2.);
		brth_printf("b=%ld\n", (int32_t)(1000*b));
		// Add values for averaging over iterations
		originFlow += b/(float)iterations;
	}
	*B = originFlow;
	brth_printf("B=%ld\n", (int32_t)(1000*(*B)));

    // Reset Paw samples
    init_Paw_cmH2O_sampling();

    // Reset Pdiff samples
    init_Pdiff_Lpm_sampling();


	brth_printf("Calibration...DONE\n");
}

