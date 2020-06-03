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

#define PEP_SAMPLES_COUNT       (100 / SAMPLING_PERIOD_MS)          // moyenne glissante sur les 100ms dernieres de l'expi
#define PPLAT_SAMPLES_COUNT     (50  / SAMPLING_PERIOD_MS)          // moyenne glissante sur les 50ms dernieres de plat
#define MAX_PAW_SAMPLES         MAX(PEP_SAMPLES_COUNT, PPLAT_SAMPLES_COUNT)

#define AVG_CYCLE_PEP_SAMPLES       (1)
#define MAX_PEP_ADJUSTEMENT_mmH2O   (10)  
#define PEP_ADJUSTMENT_KP           (0.5)
#define PEP_ADJUSTMENT_KD           (0.5)

#define AVG_PDIF_SAMPLES            (4)
#define MOTOR_PID_KP                (0.8)
#define MOTOR_PID_KD                (0.5)
#define MOTOR_PID_KI                (0)

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

static cycle_data_t g_cycle;

// static float g_cycle_EoI_ratio;
// static float g_cycle_FR_pm;
// static float g_cycle_VTe_mL;
// static float g_cycle_VTi_mL;
// static float g_cycle_VMe_Lpm;
// static float g_cycle_Pcrete_cmH2O;
// static float g_cycle_Pplat_cmH2O;
// static float g_cycle_PEP_cmH2O;

static settings_t g_settings;

// static uint32_t g_setting_T;
// static float g_setting_VT;
// static float g_setting_VM;
// static float g_setting_Pmax;
// static uint32_t g_setting_Tinspi_ms;   // Computed based on cycle period T and I/E ratio
// static uint32_t g_setting_Tinsu_ms;    // Théoritical Insuflation's time based on volume and theoritical flow
// static uint32_t g_setting_Texp_ms;     // Computed based on cycle period T and I/E ratio


static float                g_Paw_cmH2O_samples[MAX_PAW_SAMPLES];
static volatile uint16_t    g_Paw_cmH2O_sample_count;
static volatile uint16_t    g_Paw_cmH2O_sample_idx;

static insuflation_samples_t    g_insuflation_samples;
static volatile bool            g_insuflation_sampling;

// static float                g_Pdiff_Lpm_samples[MAX_PDIFF_SAMPLES];
// static uint32_t             g_motor_step_samples[MAX_PDIFF_SAMPLES];
// static volatile uint16_t    g_Pdiff_Lpm_sample_count;

static float                g_cycle_PEP_samples[AVG_CYCLE_PEP_SAMPLES];
static uint32_t             g_cycle_PEP_sample_idx;
static uint32_t             g_cycle_PEP_sample_cnt;
static float                g_cycle_PEP_adjustment_error;


static BreathingState g_state;


static uint32_t             g_motor_steps_us[MOTOR_MAX_STEPS] ; 
static volatile uint32_t    g_motor_step_idx;
static volatile uint32_t    g_motor_step_count;
static volatile float       g_motor_pid_error;
static volatile float       g_motor_pid_integral;






//----------------------------------------------------------
// Private functions prototypes
//----------------------------------------------------------
static void regulation_pep(float target_PEP_cmH2O, float cycle_PEP_cmH2O);

static void  init_cycle_PEP_adjustment();
static float get_cycle_PEP_average(float cycle_PEP_cmH2O);
static float get_cycle_PEP_adjustment(float target_PEP_cmH2O, float cycle_PEP_cmH2O);


static void clear_insuflation_samples();
static void init_Paw_cmH2O_sampling();
static float get_avg_Paw_cmH2O(uint16_t count);
static void start_timer();
static void stop_timer();

static void signal_state(BreathingState newState);
static void signal_pinha(bool pause);
static void signal_pexha(bool pause);

static void samplingCallback(TimerHandle_t xTimer);
static void breathing_run(void *args);

//static uint32_t motor_step_callback(uint32_t elapsed_step_us);


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


float get_cycle_EoI_ratio() { return g_cycle.EoI_ratio; }
float get_cycle_FR_pm() { return g_cycle.FR_pm; }
float get_cycle_VTe_mL() { return g_cycle.VTe_mL; }
float get_cycle_VTi_mL() { return g_cycle.VTi_mL; }
float get_cycle_VMe_Lpm() { return g_cycle.VMe_Lpm; }
float get_cycle_Pcrete_cmH2O() { return g_cycle.Pcrete_cmH2O; }
float get_cycle_Pplat_cmH2O() { return g_cycle.Pplat_cmH2O; }
float get_cycle_PEP_cmH2O() { return g_cycle.PEP_cmH2O; }


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

        motor_init(MOTOR_HOME_STEP_US);

        motor_pep_home();
        while (!motor_pep_is_home()) 
        {
            wait_ms(10);
        }
        motor_pep_move(get_setting_PEP_cmH2O()*10);
        while (motor_pep_is_moving()) 
        {
            wait_ms(10);
        }

        adaptation_prepare();

        // Clear cycle data
        g_cycle.EoI_ratio = 0;
        g_cycle.FR_pm = 0;
        g_cycle.VTe_mL = 0;
        g_cycle.VTi_mL = 0;
        g_cycle.VMe_Lpm = 0;
        g_cycle.Pcrete_cmH2O = 0;
        g_cycle.Pplat_cmH2O = 0;
        g_cycle.PEP_cmH2O = 0;

        // Init cycle PEP adjustment PID
        init_cycle_PEP_adjustment();
        // Init Paw samples
        init_Paw_cmH2O_sampling();
        // Init Pdiff sampling
        clear_insuflation_samples();
        // Start timer (200Hz)
        start_timer();

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
            g_settings.T_ms = get_setting_T_ms();
            g_settings.VT_mL = get_setting_VT_mL();
            g_settings.Vmax_Lpm = get_setting_Vmax_Lpm();
            g_settings.Pmax_cmH2O = get_setting_Pmax_cmH2O();
            g_settings.Tinha_ms = get_setting_Tinha_ms();   // Computed based on cycle period T and I/E ratio
            g_settings.Tinsu_ms = get_setting_Tinsu_ms();   // Théoritical Insuflation's time based on volume and theoritical flow
            g_settings.Texha_ms = get_setting_Texha_ms();   // Computed based on cycle period T and I/E ratio
            taskEXIT_CRITICAL();

            brth_printf("BRTH: T     : %lu\n", g_settings.T_ms);
            brth_printf("BRTH: VT    : %lu\n", (uint32_t)(g_settings.VT_mL ));
            brth_printf("BRTH: VLpm  : %lu\n", (uint32_t)(g_settings.Vmax_Lpm ));
            brth_printf("BRTH: Pmax  : %lu\n", (uint32_t)(g_settings.Pmax_cmH2O ));
            brth_printf("BRTH: Tinha : %lu\n", g_settings.Tinha_ms);
            brth_printf("BRTH: Tinsu : %lu\n", g_settings.Tinsu_ms);
            brth_printf("BRTH: Texha : %lu\n", g_settings.Texha_ms);



            valve_inhale();
            reset_Vol_mL();


            // g_motor_steps_us[0]= MOTOR_MIN_STEP_US*2;
            // g_motor_step_idx=1;
            // g_motor_step_count=MOTOR_MAX_STEPS*80/100;
            //motor_move(MOTOR_PRESS, g_motor_steps_us[0], motor_step_callback);
            // g_motor_pid_error = 0;
            // g_motor_pid_integral = 0;

            uint32_t nb_steps= adaptation((const settings_t*)&g_settings, (const insuflation_samples_t *) &g_insuflation_samples, MOTOR_MAX_STEPS, g_motor_steps_us);
            
            // Clear previous insuflation's samples
            clear_insuflation_samples();

            // start insuflation's sampling 
            g_insuflation_sampling=true;
            // Wait for first sample.
            volatile uint32_t count;
            do 
            {
                count = g_insuflation_samples.count;
            }
            while(count==0);
            motor_press(g_motor_steps_us, nb_steps);

            float cycle_Pcrete_cmH2O= 0;

            // Insuflation state: do
            // insuflation's data and Paw sampling will be done by the samplingTimer in background
            do
            {
                wait_ms(INSUFLATION_PROCESSING_PERIOD_MS);
                cycle_Pcrete_cmH2O = MAX(cycle_Pcrete_cmH2O, read_Paw_cmH2O());
                if ( g_settings.Pmax_cmH2O <= cycle_Pcrete_cmH2O)
                {
                    brth_printf("BRTH: Paw [%ld]> Pmax --> Exhalation\n", (int32_t)(cycle_Pcrete_cmH2O));
                    // Stop sampling and clear insuflation's data
                    g_insuflation_sampling=false;
                    clear_insuflation_samples();
                    next_state = Exhalation;
                }
                else 
                if (g_settings.VT_mL <= read_Vol_mL())
                {
                    brth_printf("BRTH: vol [%ld]>= VT --> Plateau\n", (int32_t)(read_Vol_mL()));
                    next_state = Plateau;
                }
                else 
                if (g_settings.Tinsu_ms * 3 <= (get_time_ms() - inhalation_start_ms))   // TODO: see how it fits with the adaptation
                {
                    brth_printf("BRTH: dt [%lu]>= Ti\n", (get_time_ms() - inhalation_start_ms));
                    next_state = Plateau;
                }
            } while (Insuflation == next_state);
            
            // Insuflation state: onExit
            g_insuflation_sampling=false;
            motor_stop();
            float release_Pdiff= read_Pdiff_Lpm()*0.5;
            bool released=false;
            if (Plateau == next_state)
            {
                // Plateau state : onEntry
                signal_state(Plateau);
                // Plateau state: do
                do {
                    wait_ms(PLATEAU_PROCESSING_PERIOD_MS);
                     
                    cycle_Pcrete_cmH2O = MAX(cycle_Pcrete_cmH2O, read_Paw_cmH2O());

                    if (g_settings.Pmax_cmH2O <= cycle_Pcrete_cmH2O)
                    {
                        brth_printf("BRTH: Paw [%ld]> Pmax --> Exhalation\n", (int32_t)(cycle_Pcrete_cmH2O));
                        next_state= Exhalation;
                    }
                    else if ( g_settings.Tinha_ms <= (get_time_ms() - inhalation_start_ms) )
                    {
                        brth_print("BRTH: dt > Tinha\n");
                        next_state= Exhalation;
                    }
                    if(!released && read_Pdiff_Lpm()<release_Pdiff) {
                        motor_release(MOTOR_RELEASE_STEP_US);
                        released=true;
                    }                    
                } while (Plateau == next_state);

                // --------------------------------------
                // Start of inhalation pause processing if requested
                inhalation_pause_t_ms = 0;

                while(g_settings.Pmax_cmH2O > cycle_Pcrete_cmH2O && inhalation_pause_t_ms<DEFAULT_Tpinha_max_ms && !is_command_Tpinha_expired()) 
                {
                    if(inhalation_pause_t_ms==0) 
                    {
                        brth_printf("BRTH: Inhalation pause start\n");
                    }
                    wait_ms(PLATEAU_PROCESSING_PERIOD_MS);
                    inhalation_pause_t_ms+= PLATEAU_PROCESSING_PERIOD_MS;                  

                    // Update current Pcrete
                    cycle_Pcrete_cmH2O = MAX(cycle_Pcrete_cmH2O, read_Paw_cmH2O());

                    // Update cycle Pplat with current average for HMI reporting
                    g_cycle.Pplat_cmH2O = get_avg_Paw_cmH2O(PPLAT_SAMPLES_COUNT);
                    signal_pinha(true);
                }
                if(inhalation_pause_t_ms>0)  
                {
                    brth_printf("BRTH: Inhalation pause end: %lu\n", inhalation_pause_t_ms);
                }
                // End of inhalation pause processing
                // --------------------------------------



                // Plateau state: onExit
                // get the average Paw over the last Pplat samples count
                g_cycle.Pplat_cmH2O = get_avg_Paw_cmH2O(PPLAT_SAMPLES_COUNT);
                // Reset Inhalation pause signal
                signal_pinha(false);
            }

            // End of (macro) state Inhalation            
            g_cycle.VTi_mL = read_Vol_mL();
            if(!released) motor_release(MOTOR_RELEASE_STEP_US);

            // Exhalation state: onEntry
            signal_state(Exhalation);
            valve_exhale();
            float VTe_start_mL = read_Vol_mL();
            exhalation_start_ms = get_time_ms();
            // Exhalation state: do
            do {
                wait_ms(EXHALATION_PROCESSING_PERIOD_MS);
                
                if (g_settings.Texha_ms <= (get_time_ms() - exhalation_start_ms))
                {
                    brth_print("BRTH:  (Texha <= dt)\n");
                    next_state= Insuflation;
                }
            } while (Exhalation == next_state);            

            // --------------------------------------
            // Start of exhalation pause processing if requested
            exhalation_pause_t_ms = 0;

            while(exhalation_pause_t_ms<DEFAULT_Tpexha_max_ms && !is_command_Tpexha_expired()) 
            {
                if(exhalation_pause_t_ms==0) 
                {
                    brth_printf("BRTH: Exhalation pause start\n");
                }
                wait_ms(EXHALATION_PROCESSING_PERIOD_MS);
                exhalation_pause_t_ms+= EXHALATION_PROCESSING_PERIOD_MS;                  

                // Update cycle PEP with current average
                g_cycle.PEP_cmH2O = get_avg_Paw_cmH2O(PEP_SAMPLES_COUNT);
                signal_pexha(true);
            }
            if(exhalation_pause_t_ms>0) 
            {
                brth_printf("BRTH: Exhalation pause end: %lu\n", exhalation_pause_t_ms);
            }
            // End of exhalation pause processing
            // --------------------------------------

            // Exhalation state: onExit
            // Compute cycle data
            uint32_t t_ms = get_time_ms();
            g_cycle.PEP_cmH2O = get_avg_Paw_cmH2O(PEP_SAMPLES_COUNT);
            g_cycle.Pcrete_cmH2O= cycle_Pcrete_cmH2O;
            g_cycle.EoI_ratio = (float)(t_ms - exhalation_start_ms - exhalation_pause_t_ms) / (exhalation_start_ms - inhalation_start_ms - inhalation_pause_t_ms);
            g_cycle.FR_pm = 1. / (((float)(t_ms - inhalation_start_ms - inhalation_pause_t_ms - exhalation_pause_t_ms)) / 1000 / 60);
            g_cycle.VTe_mL = VTe_start_mL - read_Vol_mL();
            g_cycle.VMe_Lpm = (g_cycle.VTe_mL / 1000) * g_cycle.FR_pm;

            // Notify system that new cycle data is available.
            xEventGroupSetBits(g_breathingEvents, BRTH_CYCLE_UPDATED);

            // Proceed to the PEP regulation
            regulation_pep(get_setting_PEP_cmH2O(),  g_cycle.PEP_cmH2O);

            // Check if controller asked us to stop.
            events = xEventGroupGetBits(g_controllerEvents);
        } while ((events & BREATHING_RUN_FLAG) != 0);

        // stop Sampling (Paw and Pdiff)
        stop_timer();

        signal_state(None);
        brth_printf("BRTH: Stopping\n");

        wait_ms(200);
        xEventGroupSetBits(g_controllerEvents, BREATHING_STOPPED_FLAG);
    }
}

static void start_timer() 
{
    if( xTimerReset(g_samplingTimer, 20/portTICK_PERIOD_MS) != pdTRUE ) 
    {
        // TODO : What should we do? Raise an alarm ??
        brth_printf("Error timer !!\n");
    }    
}

static void stop_timer()
{
    if( xTimerStop(g_samplingTimer, 20/portTICK_PERIOD_MS) != pdTRUE ) 
    {
        // TODO : What should we do? Raise an alarm ??
    }
}


// static uint32_t motor_step_callback(uint32_t elapsed_step_us) {
//     if(g_motor_step_idx>=g_motor_step_count) return 0;

//     float Pdiff=0;
//     if(g_Pdiff_Lpm_sample_count>0)
//     {
//         uint nb_samples= MIN(g_Pdiff_Lpm_sample_count,4);
//         for(int t=0; t<nb_samples; ++t)
//         {
//             Pdiff+= g_Pdiff_Lpm_samples[g_Pdiff_Lpm_sample_count-nb_samples+t];
//         }
//         Pdiff/=nb_samples;
//     }

//     float last_error= g_motor_pid_error;
//     g_motor_pid_error= g_setting_VM-Pdiff;
//     g_motor_pid_integral+= g_motor_pid_error;

//     float step_us = elapsed_step_us - (  (MOTOR_PID_KP * g_cycle_PEP_adjustment_error) + (MOTOR_PID_KD * (g_cycle_PEP_adjustment_error - last_error)) + (MOTOR_PID_KI * g_motor_pid_integral));    

//     if( step_us < (1+MOTOR_MAX_ACCEL_USPUS) * elapsed_step_us)
//     {
//         step_us = (1+MOTOR_MAX_ACCEL_USPUS) * elapsed_step_us;
//     }
//     if( step_us < MOTOR_MIN_STEP_US)
//     {
//         step_us= MOTOR_MIN_STEP_US;
//     }

//     g_motor_steps_us[g_motor_step_idx++]= (uint32_t) step_us;

//     return step_us;
// }


static void signal_pinha(bool pause) {
    if(pause) 
    {
        xEventGroupSetBits(g_breathingEvents, BRTH_CYCLE_PINHA);
    }
    else
    {
        xEventGroupClearBits(g_breathingEvents, BRTH_CYCLE_PINHA);
    }
}

static void signal_pexha(bool pause) {
    if(pause) 
    {
        xEventGroupSetBits(g_breathingEvents, BRTH_CYCLE_PEXHA);
    }
    else
    {
        xEventGroupClearBits(g_breathingEvents, BRTH_CYCLE_PEXHA);
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
    xEventGroupClearBits(g_breathingEvents, (BRTH_CYCLE_INSUFLATION | BRTH_CYCLE_PLATEAU | BRTH_CYCLE_EXHALATION | BRTH_CYCLE_PINHA | BRTH_CYCLE_PEXHA));
    xEventGroupSetBits(g_breathingEvents, brthState);
}


static void regulation_pep(float target_PEP_cmH2O, float cycle_PEP_cmH2O)
{
    float pep_adjustment_cmH2O  = get_cycle_PEP_adjustment(target_PEP_cmH2O, cycle_PEP_cmH2O);

    int32_t pep_adjustment_mmH2O = (int32_t) (10.0*pep_adjustment_cmH2O);
    brth_printf("BRTH: >>>>>>>>>>>>>>>> PEP adjustment mmH2O: %ld\n", pep_adjustment_mmH2O);
    motor_pep_stop();
//    int32_t pep_adjustment_mmH2O= (int32_t) (SIGN(pep_adjustment_mmH2O)*MIN(MAX_PEP_ADJUSTEMENT_mmH2O,abs(pep_adjustment_mmH2O)));
    // if(abs(pep_adjustment_mmH2O)>1)
    // {
        motor_pep_move(pep_adjustment_mmH2O);
    // }
}

static void init_cycle_PEP_adjustment() {
    memset((void*)g_cycle_PEP_samples, 0, sizeof(g_cycle_PEP_samples));
    g_cycle_PEP_sample_idx = 0;
    g_cycle_PEP_sample_cnt = 0;
    g_cycle_PEP_adjustment_error=0;
}

static float get_cycle_PEP_average(float cycle_PEP_cmH2O) 
{
    g_cycle_PEP_samples[g_cycle_PEP_sample_idx]= cycle_PEP_cmH2O;
    g_cycle_PEP_sample_idx= (g_cycle_PEP_sample_idx +1)% AVG_CYCLE_PEP_SAMPLES;
    g_cycle_PEP_sample_cnt+= (g_cycle_PEP_sample_cnt<AVG_CYCLE_PEP_SAMPLES) ? 1 : 0;

    float avg_PEP=0;
    for(uint16_t i=0; i<g_cycle_PEP_sample_cnt; ++i) 
    {
        avg_PEP+= g_cycle_PEP_samples[ (AVG_CYCLE_PEP_SAMPLES + g_cycle_PEP_sample_idx + i - g_cycle_PEP_sample_cnt) % AVG_CYCLE_PEP_SAMPLES ];
    }
    avg_PEP/= g_cycle_PEP_sample_cnt;

    return avg_PEP;
}

static float get_cycle_PEP_adjustment(float target_PEP_cmH2O, float cycle_PEP_cmH2O) {
    
    float avg_PEP= get_cycle_PEP_average(cycle_PEP_cmH2O);

    float last_error= g_cycle_PEP_adjustment_error;
    g_cycle_PEP_adjustment_error = target_PEP_cmH2O - avg_PEP;

    float adjustment = (PEP_ADJUSTMENT_KP * g_cycle_PEP_adjustment_error) + (PEP_ADJUSTMENT_KD * (g_cycle_PEP_adjustment_error - last_error));
    return adjustment;
}

static void init_Paw_cmH2O_sampling()
{
    taskENTER_CRITICAL();
    memset((void*)g_Paw_cmH2O_samples, 0, sizeof(g_Paw_cmH2O_samples));
    g_Paw_cmH2O_sample_count = 0;
    g_Paw_cmH2O_sample_idx = 0;
    taskEXIT_CRITICAL();
}

static float get_avg_Paw_cmH2O(uint16_t count)
{
    taskENTER_CRITICAL();
    float sum = 0.;
    count = MIN(count, g_Paw_cmH2O_sample_count);
    for (uint16_t i = 0; i < count; i++)
    {
        sum += g_Paw_cmH2O_samples[ (MAX_PAW_SAMPLES + g_Paw_cmH2O_sample_idx + i - count ) % MAX_PAW_SAMPLES ];
    }
    taskEXIT_CRITICAL();
    return count == 0 ? 0 : sum/count;
}

static void clear_insuflation_samples()
{
    taskENTER_CRITICAL();
    memset((void*)&g_insuflation_samples, 0, sizeof(g_insuflation_samples));
    g_insuflation_samples.T_ms= SAMPLING_PERIOD_MS;
    g_insuflation_sampling= false;
    taskEXIT_CRITICAL();
}

static void samplingCallback(TimerHandle_t timer) {
    taskENTER_CRITICAL();
    // sample Paw
    g_Paw_cmH2O_samples[g_Paw_cmH2O_sample_idx]= read_Paw_cmH2O();
    g_Paw_cmH2O_sample_idx = (g_Paw_cmH2O_sample_idx +1) % MAX_PAW_SAMPLES;
    if(g_Paw_cmH2O_sample_count<MAX_PAW_SAMPLES) 
    {
        ++g_Paw_cmH2O_sample_count;
    }

    // sample insuflation data only when requested
    if(g_insuflation_sampling==true) 
    {
        if(g_insuflation_samples.count<MAX_INSUFLATION_SAMPLES)
        {
            g_insuflation_samples.Pdiff_Lpm[g_insuflation_samples.count]= read_Pdiff_Lpm();
            g_insuflation_samples.Paw_cmH2O[g_insuflation_samples.count]= read_Paw_cmH2O();
            g_insuflation_samples.motor_step_idx[g_insuflation_samples.count] = motor_press_get_current_step();
            ++g_insuflation_samples.count;
        }
    }

    taskEXIT_CRITICAL();
}


// //Flow(t) = A*t + B
// static void calibration(float* A, float* B, uint8_t iterations, uint32_t calibration_step_us) {
// 	float slope = 0; // slope of flow(t) cruve
// 	float originFlow = 0; // origin flow of flow(t) curve
// 	uint32_t steps;

//     steps= (uint32_t) (MOTOR_MAX_STEPS*66/100);
//     for(uint32_t t=0; t<steps; ++t) 
//     {
//         g_motor_steps_us[t]= (uint32_t)calibration_step_us;
//     }


// 	// Calibrate slope
// 	brth_printf("---------- Calibrating slope ---------------\n");
// 	for(int iter=0; iter<iterations; ++iter) {
//         init_Paw_cmH2O_sampling();
//         init_Pdiff_Lpm_sampling();
    	
//         // HIGH PEEP
//         valve_inhale();

//         reset_Vol_mL();
//         xTimerReset(g_samplingTimer, 10/portTICK_PERIOD_MS);
// 		motor_press(g_motor_steps_us, steps);
//         wait_ms(200); // skip first 200ms
//         g_Pdiff_sampling= true;
// 		while(motor_is_moving()) wait_ms(5);
//         g_Pdiff_sampling= false;
//         xTimerStop(g_samplingTimer, 10/portTICK_PERIOD_MS);
//         g_state= None;
// 		wait_ms(500);
// 		float volumeIT = read_Vol_mL();
// 		brth_printf("volume = %lu ml\n", (uint32_t)(volumeIT));
//         // LOW PEEP
//         valve_exhale();
//         motor_release(MOTOR_RELEASE_STEP_US);
// 		while(!motor_is_home());
//         wait_ms(2000);

// 		float a = 0, b = 0;
// 		float r = linear_fit(g_Pdiff_Lpm_samples, g_Pdiff_Lpm_sample_count, &a, &b);

//         // scale slope from sample idx to to seconds
//         a= a/(0.001*SAMPLING_PERIOD_MS);

//         for(int t=0; t<g_Pdiff_Lpm_sample_count; ++t ) 
//         {
//             brth_printf("%ld, ", (int32_t)(g_Pdiff_Lpm_samples[t]));
//         }
//         brth_printf("\n");
//         for(int t=0; t<g_Pdiff_Lpm_sample_count; ++t ) 
//         {
//             brth_printf("%ld, ", (int32_t)(a*t+b));
//         }
//         brth_printf("\n");

// 		brth_printf("a=%ld b=%ld\n", (int32_t)(1000.*a), (int32_t)(1000.*b));
// 		brth_printf("r=%lu\n", (uint32_t)(1000.*r));
// 		slope += a / (float)iterations;
// 	}
// 	*A = slope;
// 	brth_printf("A=%lu\n", (uint32_t)(1000.* *A));


// 	// Calibrate originFlow
// 	brth_printf("---------- Calibrating B ---------------\n");
// 	for(int iter=0; iter<iterations; ++iter) {
// 		// HIGH PEEP
//         valve_inhale();
//         reset_Vol_mL();
// 		motor_press(g_motor_steps_us, steps);
// 		while(motor_is_moving()) wait_ms(5);
// 		wait_ms(1000);
// 		float volumeIT = read_Vol_mL();
// 		// LOW PEEP
//         valve_exhale();
//         motor_release(MOTOR_RELEASE_STEP_US);
// 		while(!motor_is_home());
// 		brth_printf("volume = %luml\n", (uint32_t)(volumeIT));
// 		float b = volumeIT*0.001/((CALIBRATION_STEP_US*0.000001) * steps) - (*A * (CALIBRATION_STEP_US*0.000001)*steps / 2.);
// 		brth_printf("b=%ld\n", (int32_t)(1000*b));
// 		// Add values for averaging over iterations
// 		originFlow += b/(float)iterations;
// 	}
// 	*B = originFlow;
// 	brth_printf("B=%ld\n", (int32_t)(1000*(*B)));

//     // Reset Paw samples
//     init_Paw_cmH2O_sampling();

//     // Reset Pdiff samples
//     init_Pdiff_Lpm_sampling();


// 	brth_printf("Calibration...DONE\n");
// }

