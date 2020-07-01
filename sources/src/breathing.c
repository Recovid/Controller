#include "common.h"
#include "compute_motor.h"
#include "config.h"
#include "defaults.h"
#include "controller.h"
#include "breathing.h"
#include "platform.h"

#include <adaptation.h>
#include <math.h>
#include <string.h>

#include"adrien/const_calibs.h"


//----------------------------------------------------------
// Private defines
//----------------------------------------------------------

#define INSUFLATION_PROCESSING_PERIOD_MS    (2) /// a faire plus souvent pr protection Pmax et surtout precision du volume VTi ---> voir motor_stop() qui controle 
#define PLATEAU_PROCESSING_PERIOD_MS        	(5) /// a faire plus souvent pr protection Pmax
#define EXHALATION_PROCESSING_PERIOD_MS     (10)


// #define SAMPLING_PERIOD_MS      (5)
#define PEP_SAMPLES_COUNT       (100 / SAMPLING_PERIOD_MS)          // moyenne glissante sur les 100ms dernieres de l'expi
#define PPLAT_SAMPLES_COUNT     (50  / SAMPLING_PERIOD_MS)          // moyenne glissante sur les 50ms dernieres de plat
#define MAX_PAW_SAMPLES         MAX(PEP_SAMPLES_COUNT, PPLAT_SAMPLES_COUNT)
#define MAX_PDIFF_SAMPLES       (6000 / SAMPLING_PERIOD_MS)


//----------------------------------------------------------
// Private typedefs
//----------------------------------------------------------

typedef enum
{
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
static uint32_t g_cycle_insuflation_duration;
static volatile bool g_sampling_flow;

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




static uint32_t g_motor_steps_us[MOTOR_MAX_STEPS] ; 




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
uint32_t get_cycle_insuflation_duration() { return g_cycle_insuflation_duration; }


//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

static void breathing_run(void *args)
{
    UNUSED(args);
    EventBits_t events;

    while (true)
    {
        brth_printf("BRTH: Standby\n");
        events = xEventGroupWaitBits(g_controllerEvents, BREATHING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY);
        brth_printf("BRTH: Started\n");

        motor_enable(true);

        // Clear cycle data
        g_cycle_EoI_ratio = 0;
        g_cycle_FR_pm = 0;
        g_cycle_VTe_mL = 0;
        g_cycle_Pcrete_cmH2O = 0;
        g_cycle_Pplat_cmH2O = 0;
        g_cycle_PEP_cmH2O = 0;
        g_cycle_insuflation_duration = 0;
		GLOBALE_pep = 0;
		
		float cycle_Pcrete_cmH2O = 0.0;
		float cycle_Debit_insu_max = -10;
		float cycle_Debit_exsu_max = 10;
		
        // Init state machine
        BreathingState next_state = Insuflation;
        uint32_t inhalation_start_ms;
        uint32_t inhalation_pause_t_ms;
        uint32_t insuflation_stop_ms;

        uint32_t exhalation_start_ms;
        uint32_t exhalation_pause_t_ms;
		uint32_t nb_steps;
		
		
        do
        {
            #if		NIVEAU_VERBOSE_DEBUG		>=		3
			brth_printf("BRTH: start cycle\n");
			#endif


            // Get current controller settings
            taskENTER_CRITICAL();
            g_setting_T = get_setting_T_ms();
            g_setting_VT = get_setting_VT_mL();
            g_setting_VM = get_setting_Vmax_Lpm();
            g_setting_Pmax = get_setting_Pmax_cmH2O();
            g_setting_Tinspi_ms = get_setting_Tinspi_ms(); // Computed based on cycle period T and I/E ratio
            g_setting_Tinsu_ms = get_setting_Tinsu_ms();   // Théoritical Insuflation's time based on volume and theoritical flow
            g_setting_Texp_ms = get_setting_Texp_ms();     // Computed based on cycle period T and I/E ratio
           
			brth_printf("BRTH: T     : %lu\n", g_setting_T);
			brth_printf("BRTH: VT    : %lu\n", (uint32_t)(g_setting_VT ));
			brth_printf("BRTH: VM    : %lu\n", (uint32_t)(g_setting_VM ));
			brth_printf("BRTH: Pmax  : %lu\n", (uint32_t)(g_setting_Pmax ));
			brth_printf("BRTH: Tinsu : %lu\n", g_setting_Tinsu_ms);
			brth_printf("BRTH: Tinspi: %lu\n", g_setting_Tinspi_ms);
			brth_printf("BRTH: samples: %u\n", g_Pdiff_Lpm_sample_count);
				
				
			#if	1
				static uint32_t	LAST_g_setting_T = 0; /// 					= g_setting_T = 0;
				static float		LAST_g_setting_VT = 0; ///  				= g_setting_VT = 0;
				static float		LAST_g_setting_VM = 0; ///  				= g_setting_VM = 0;
				static float		LAST_g_setting_Pmax = 0; ///  			= g_setting_Pmax = 0;
				static uint32_t	LAST_g_setting_Tinspi_ms = 0; ///  	= g_setting_Tinspi_ms = 0;
				static uint32_t	LAST_g_setting_Tinsu_ms = 0; ///  	= g_setting_Tinsu_ms = 0;
				static uint32_t	LAST_g_setting_Texp_ms = 0; ///  	= g_setting_Texp_ms;
				
				static	uint16_t		compteur_print = 0;
				if(
						LAST_g_setting_T 				!= g_setting_T
					||	LAST_g_setting_VT 				!= g_setting_VT
					||	LAST_g_setting_VM 				!= g_setting_VM
					||	LAST_g_setting_Pmax 			!= g_setting_Pmax
					||	LAST_g_setting_Tinspi_ms 	!= g_setting_Tinspi_ms
					||	LAST_g_setting_Tinsu_ms 	!= g_setting_Tinsu_ms
					||	LAST_g_setting_Texp_ms 	!= g_setting_Texp_ms
				){
					
					init_variables_PID( g_setting_VM ); /// + reinit_Modele_erreur_Pneumo(); /// notamment reinit modele erreur pneumo pr compute_corrected_flow() /// CHANGEMENTS_POST_LYON_0
					// reset_Vol_error_phase_accel(); /// deja fait ds : init_variables_PID()
					
					g_Pdiff_Lpm_sample_count = 0; /// pr thread acquisition i2c
					compteur_print = 0;
					
					#if			ENABLE_ADAPTATION_DURING_EXPIRATION			==			1 /// pas top pour timing general et I/E : decalle tout
					// Compute adaptation based on current settings and previous collected data if any.
					nb_steps = adaptation(
																					g_setting_VT,
																					g_setting_VM,
																					SAMPLING_PERIOD_MS,
																					
																					g_Pdiff_Lpm_sample_count,
																					// GLOB_index_sample_GO_motor_stop, /// g_Pdiff_Lpm_sample_count,
																					
																					g_Pdiff_Lpm_samples,
																					MOTOR_MAX_STEPS,
																					g_motor_steps_us
																			);
					#endif
					
					

				}
				
				LAST_g_setting_T 				= g_setting_T;
				LAST_g_setting_VT 				= g_setting_VT;
				LAST_g_setting_VM 				= g_setting_VM;
				LAST_g_setting_Pmax 			= g_setting_Pmax;
				LAST_g_setting_Tinspi_ms 	= g_setting_Tinspi_ms;
				LAST_g_setting_Tinsu_ms 	= g_setting_Tinsu_ms;
				LAST_g_setting_Texp_ms 	= g_setting_Texp_ms;
				
				if( ( compteur_print++ % 12 ) == 0 ){
					brth_printf("BRTH: T     : %lu\n", g_setting_T);
					brth_printf("BRTH: VT    : %lu\n", (uint32_t)(g_setting_VT ));
					brth_printf("BRTH: VM    : %lu\n", (uint32_t)(g_setting_VM ));
					brth_printf("BRTH: Pmax  : %lu\n", (uint32_t)(g_setting_Pmax ));
					brth_printf("BRTH: Tinsu : %lu\n", g_setting_Tinsu_ms);
					brth_printf("BRTH: Tinspi: %lu\n", g_setting_Tinspi_ms);
					brth_printf("BRTH: samples: %u\n", g_Pdiff_Lpm_sample_count);
				}
            #endif
			 taskEXIT_CRITICAL();
			 
			 
			#if			ENABLE_ADAPTATION_DURING_EXPIRATION			==			0 /// pas top pour timing general et I/E : decalle tout
            // Compute adaptation based on current settings and previous collected data if any.
            nb_steps = adaptation(
																			g_setting_VT,
																			g_setting_VM,
																			SAMPLING_PERIOD_MS,
																			
																			g_Pdiff_Lpm_sample_count,
																			// GLOB_index_sample_GO_motor_stop, /// g_Pdiff_Lpm_sample_count,
																			
																			g_Pdiff_Lpm_samples,
																			MOTOR_MAX_STEPS,
																			g_motor_steps_us
																	);
			#endif
																	
            //--------------------------------------------------------------
            // Inhalation (macro) state
            //--------------------------------------------------------------
            inhalation_start_ms   = get_time_ms(); /// CHANGEMENTS_POST_LYON_0

            //--------------------------------------------------------------
            // Insuflation state: onEntry
            //--------------------------------------------------------------
            signal_state(Insuflation);
			
			
			
			
			cycle_Debit_insu_max = -10;
			cycle_Debit_exsu_max = 10;
			cycle_Pcrete_cmH2O = 0; 
            // Init Paw samples
            init_Paw_cmH2O_sampling();

            // Init Pdiff samples
            init_Pdiff_Lpm_sampling();

            valve_inhale();
            reset_Vol_mL();
			#if	ENABLE_CORRECTION_FABRICE_GERMAIN		==		1
			// reinit_Modele_erreur_Pneumo(); /// notamment reinit modele erreur pneumo pr compute_corrected_flow() /// CHANGEMENTS_POST_LYON_0
			reset_volumes_Modele_erreur_Pneumo();  /// anti_divergence volume pr modele erreur Fabrice : uniqt  		MODELE_ERR_PNEUMO_volume_pneumo = 0
			#endif

			uint32_t	debut_motor = get_time_ms();
			g_sampling_flow = true; ///-----------> on DEMARRE le sampling
			
			/// reinit pour calcul de volume de phase d'accel
			GLOB_Volume_erreur_phase_accel = -12345.0f; /// pr calcul 			GLOB_debit_to_compensate_ERROR_accel			
			

            motor_press(g_motor_steps_us, nb_steps);
			
			
			
			
            // print_steps(g_motor_steps_us, nb_steps);
            // start Sampling (Paw and Pdiff)
            if( xTimerReset(g_samplingTimer, 20/portTICK_PERIOD_MS) != pdTRUE ) 
            {
                // TODO : What should we do? Raise an alarm ??
            }
			
			
			
			
			
			/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Insuflation state: do
            // Pdiff and Paw sampling will be done by the samplingTimer in background
			bool		MUST_printf_arret_motor	= false;
            do
            {
                
                cycle_Pcrete_cmH2O 		= MAX(cycle_Pcrete_cmH2O, read_Paw_cmH2O());
				cycle_Debit_insu_max 		= MAX( cycle_Debit_insu_max, read_Pdiff_Lpm() );
				
				wait_ms(INSUFLATION_PROCESSING_PERIOD_MS);
				
				/// on coupe moteur car PROTECTION PMax
				/// on coupe moteur car PROTECTION PMax
				/// on coupe moteur car PROTECTION PMax
				if ( g_setting_Pmax <= g_cycle_Pcrete_cmH2O) 
                {
                    g_sampling_flow = false; ///-----------> on ARRETE le sampling
					// GLOB_index_sample_GO_motor_stop = g_Pdiff_Lpm_sample_count; /// on se remember l'index echantillon
					
					/// PAS de printf ici !!!
					/// PAS de printf ici !!!
					/// PAS de printf ici !!!
					 // valve_exhale(); /// prio absolue
					// brth_printf("\n\nBRTH: Paw [%ld]> Pmax --> Exhalation\n\n", (int32_t)(g_cycle_Pcrete_cmH2O));
					/// PAS de printf ici !!!
					/// PAS de printf ici !!!
					/// PAS de printf ici !!!
					
					init_variables_PID(get_setting_Vmax_Lpm());
                    next_state = Exhalation;
                }
				
				
				
				#if	1
				
					#define				LIMITE_CUT_MOTOR					5 /// mL : petite marge secu car motor_stop n'est pas immediat
					/// on coupe moteur car VOLUME ATTEINDS : g_setting_VT
					/// on coupe moteur car VOLUME ATTEINDS : g_setting_VT
					/// on coupe moteur car VOLUME ATTEINDS : g_setting_VT
					else if (g_setting_VT <= 
															read_Vol_mL()
														+ 	GLOB_TOTAL_Volume_insu_AFTER_motor_stop /// GLOB_Volume_insu_AFTER_motor_stop
														+	LIMITE_CUT_MOTOR
					) 
					// else if (g_setting_VT <= 600)
					{
						 
						 g_sampling_flow = false; ///-----------> on ARRETE le sampling
						 // GLOB_index_sample_GO_motor_stop = g_Pdiff_Lpm_sample_count; /// on se remember l'index echantillon
						 
						MUST_printf_arret_motor = true; /// debug only
						next_state = Plateau;
					}
					
				#endif
				
				
				
                // else if (g_setting_Tinsu_ms <= (get_time_ms() - inhalation_start_ms))   // TODO: see how it fits with the adaptation
                // {
                //     brth_printf("BRTH: dt [%lu]>= Ti\n", (get_time_ms() - inhalation_start_ms));
                //     next_state = Plateau;
                // }
            } while (Insuflation == next_state);
            // Insuflation state: onExit
			

			
			
			/// TIMECODE necessaire pour PID
			GLOB_index_sample_GO_motor_stop = g_Pdiff_Lpm_sample_count; /// on se remember l'index echantillon
			GLOB_Volume_BEFORE_motor_stop = read_Vol_mL();
					
					motor_stop( ); /// nouvelle version avec decceleration adrien on the fly DMA MAJ + gestion GLOB_TOTAL_Volume_insu_AFTER_motor_stop ---> attention avec augmentation gain PID
					/// todo : gerer un 		facteur_evolution_gain_PID_last_segment 		pour faire evoluer 		GLOB_TOTAL_Volume_insu_AFTER_motor_stop :
					/// ds adapation() ------> facteur_evolution_gain_PID_last_segment = fabs( new_gain_PID_dernier_segment / last_gain_PID_dernier_segment );
					/// last_gain_PID_dernier_segment = new_gain_PID_dernier_segment; /// MAJ
					/// GLOB_TOTAL_Volume_insu_AFTER_motor_stop *= facteur_evolution_gain_PID_last_segment;
			
			g_sampling_flow = false; /// on arrete le sampling
			

			
			/// TIMECODE necessaire pour PID
			GLOB_index_sample_motor_stop_DONE = g_Pdiff_Lpm_sample_count; /// on se remember l'index echantillon
			GLOB_Volume_AFTER_motor_stop = read_Vol_mL();
			
			
			
			/// VERIFICATION de Ti
			/// VERIFICATION de Ti
			/// VERIFICATION de Ti
			GLOB_Verif_Tinsu = get_time_ms() - debut_motor;
			/// VERIFICATION de Ti
			/// VERIFICATION de Ti
			/// VERIFICATION de Ti
			
			
            // if(Plateau == next_state) 
            // {
                // wait_ms(MOTOR_INERTIA_STOP_MS);
            // }
            motor_release(MOTOR_RELEASE_STEP_US);
			
			
			
			/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            if (Plateau == next_state)
            {
                // Plateau state : onEntry
                signal_state(Plateau);
                inhalation_pause_t_ms = 0;

                // Plateau state: do
                do {
                    wait_ms(PLATEAU_PROCESSING_PERIOD_MS);
					
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
                    
                    g_cycle_Pcrete_cmH2O = MAX(g_cycle_Pcrete_cmH2O, read_Paw_cmH2O());

                    if (g_setting_Pmax <= g_cycle_Pcrete_cmH2O)
                    {
                        /// PAS de printf ici !!!
						/// PAS de printf ici !!!
						/// PAS de printf ici !!!
						 // valve_exhale(); /// prio absolue
						// brth_printf("\n\nBRTH: Paw [%ld]> Pmax --> Exhalation\n\n\n", (int32_t)(g_cycle_Pcrete_cmH2O));
						/// PAS de printf ici !!!
						/// PAS de printf ici !!!
						/// PAS de printf ici !!!
                        next_state= Exhalation;
                    }
                    else if ((g_setting_Tinspi_ms + DEFAULT_Tpins_max_ms) <= (get_time_ms() - inhalation_start_ms) )
                    {
                        brth_print("BRTH: MAX Tpins exceeded\n");
                        next_state= Exhalation;
                    }
                    else if ( (g_setting_Tinspi_ms + inhalation_pause_t_ms) <= (get_time_ms() - inhalation_start_ms) )
                    {
                        #if		NIVEAU_VERBOSE_DEBUG		>=		2
						brth_print("BRTH: dt > Tinspi\n");
						#endif
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
			GLOB_Volume_AFTER_Plateau = g_cycle_VTi_mL; /// read_Vol_mL();
			
			// g_sampling_flow = false; ///-----------> on ARRETE le sampling
			
			
			/// volume parasite pr calcul : USE_PID_VENTILATION_Verreur_accel
			/// volume parasite pr calcul : USE_PID_VENTILATION_Verreur_accel			
			/// volume parasite pr calcul : USE_PID_VENTILATION_Verreur_accel
			GLOB_Volume_insu_AFTER_motor_stop 					= GLOB_Volume_AFTER_motor_stop - GLOB_Volume_BEFORE_motor_stop;
			GLOB_TOTAL_Volume_insu_AFTER_motor_stop 		= GLOB_Volume_AFTER_Plateau - GLOB_Volume_BEFORE_motor_stop;
			/// volume parasite pr calcul : USE_PID_VENTILATION_Verreur_accel
			/// volume parasite pr calcul : USE_PID_VENTILATION_Verreur_accel
			/// volume parasite pr calcul : USE_PID_VENTILATION_Verreur_accel
			
			#if	1
			if( MUST_printf_arret_motor == true ){
				
				// get_setting_Tplat_ms() ==  g_setting_Tinspi_ms - g_setting_Tinsu_ms : a verifier ???  /// CHANGEMENTS_POST_LYON_0
				
                     printf( "\n-----> declenche at %i + %i = %i mL --> %i != %i ms < %i\n\n", 
																						(int)( ( GLOB_Volume_BEFORE_motor_stop ) * 1 ),
																						(int)( GLOB_TOTAL_Volume_insu_AFTER_motor_stop * 1 ),
																						(int)( GLOB_Volume_BEFORE_motor_stop + GLOB_TOTAL_Volume_insu_AFTER_motor_stop * 1 ),
																						
																						(int)( g_setting_Tinsu_ms ),
																						(int)( GLOB_Verif_Tinsu ),
																						(int)( g_setting_Tinspi_ms )
																					);
			
			}
			#endif
			
			
			/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			
            // Exhalation state: onEntry
            signal_state(Exhalation);
            valve_exhale();
            float VTe_start_mL = read_Vol_mL();
            exhalation_start_ms = get_time_ms();
            exhalation_pause_t_ms = 0;
			
			#if			ENABLE_ADAPTATION_DURING_EXPIRATION			==			1 /// mieux pr timing general bien propre I/E etc...
				#define			MS_TEMPS_CALCUL_DEVOLU_ADAPTATION						250 /// 65 ms max mesured /// ms : todo : a mesurer CHANGEMENTS_POST_LYON_0
			#else
				#define			MS_TEMPS_CALCUL_DEVOLU_ADAPTATION						0 /// 150 /// ms : todo : a mesurer CHANGEMENTS_POST_LYON_0
			#endif
			
            // Exhalation state: do
            do {
                cycle_Debit_exsu_max 		= MIN( cycle_Debit_exsu_max, read_Pdiff_Lpm() );
				
				wait_ms(EXHALATION_PROCESSING_PERIOD_MS);
				
         /*        if (is_command_Tpexp_expired())
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
                } */
                
                if ((g_setting_Texp_ms - MS_TEMPS_CALCUL_DEVOLU_ADAPTATION + DEFAULT_Tpexp_max_ms) <= (get_time_ms() - exhalation_start_ms))  /// CHANGEMENTS_POST_LYON_0
                {
                    next_state= Insuflation;
                }
                else if (g_setting_Texp_ms - MS_TEMPS_CALCUL_DEVOLU_ADAPTATION + exhalation_pause_t_ms <= (get_time_ms() - exhalation_start_ms)) /// CHANGEMENTS_POST_LYON_0
                {
                    #if		NIVEAU_VERBOSE_DEBUG		>=		2
					brth_print("BRTH: Tpexp expired && (T <= dt)\n");
					#endif
                    next_state= Insuflation;
                }
            } while (Exhalation == next_state);            
            




			#if		1 ///			ENABLE_ADAPTATION_DURING_EXPIRATION			==			1 /// mieux pr timing general bien propre I/E etc...
				
				bool			overrun_cpu = true;
				uint32_t 		Duree_cpu_pr_adaptation = get_time_ms();
				
				
					
				/// faire adaptation() ici ! /// CHANGEMENTS_POST_LYON_0
				/// faire adaptation() ici ! /// CHANGEMENTS_POST_LYON_0
				/// faire adaptation() ici ! /// CHANGEMENTS_POST_LYON_0
				
				#if			ENABLE_ADAPTATION_DURING_EXPIRATION			==			1 /// mieux pr timing general bien propre I/E etc...
				// Compute adaptation based on current settings and previous collected data if any.
				nb_steps = adaptation(
																				g_setting_VT,
																				g_setting_VM,
																				SAMPLING_PERIOD_MS,
																				
																				g_Pdiff_Lpm_sample_count,
																				// GLOB_index_sample_GO_motor_stop, /// g_Pdiff_Lpm_sample_count,
																				
																				g_Pdiff_Lpm_samples,
																				MOTOR_MAX_STEPS,
																				g_motor_steps_us
																		);
				#endif
				Duree_cpu_pr_adaptation = get_time_ms() - Duree_cpu_pr_adaptation;
				/// faire adaptation() ici ! /// CHANGEMENTS_POST_LYON_0
				/// faire adaptation() ici ! /// CHANGEMENTS_POST_LYON_0
				/// faire adaptation() ici ! /// CHANGEMENTS_POST_LYON_0
				
				printf( "---tps_calcul %u << %u\n", Duree_cpu_pr_adaptation, MS_TEMPS_CALCUL_DEVOLU_ADAPTATION );
				
				while( 1 ){ /// wait 		MS_TEMPS_CALCUL_DEVOLU_ADAPTATION
					
					if ((g_setting_Texp_ms + DEFAULT_Tpexp_max_ms) <= (get_time_ms() - exhalation_start_ms))  /// CHANGEMENTS_POST_LYON_0
					{
						next_state= Insuflation;
						break;
					}
					else if (g_setting_Texp_ms + exhalation_pause_t_ms <= (get_time_ms() - exhalation_start_ms)) /// CHANGEMENTS_POST_LYON_0
					{
						next_state= Insuflation;
						break;
					}
					
					overrun_cpu = false;
					wait_ms( 1 );
				}
				
				// printf( "---debug 2\n" );
				
				if( overrun_cpu == true ){
					/// alarme !! CPU
					printf(" alarm CPU %u < %u ms insuffisant !!!!\n", MS_TEMPS_CALCUL_DEVOLU_ADAPTATION, Duree_cpu_pr_adaptation );
				}
			#endif
			
			/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// PAUSE EXPI only 
            while( is_command_Tpexp_expired() == false ){
				
                wait_ms(EXHALATION_PROCESSING_PERIOD_MS);
                if (is_command_Tpexp_expired())
                {
                    // Release valve
                    valve_exhale();
                    signal_pexp(false);
					break;
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
			
			}
			
			
			
            // Exhalation state: onExit
			// printf( "--------------------------> g_cycle_PEP_cmH2O %i\n", (int)(g_cycle_PEP_cmH2O * 1 ) );
			GLOBALE_pep = g_cycle_PEP_cmH2O;
			
            g_cycle_PEP_cmH2O = get_avg_Paw_cmH2O(PEP_SAMPLES_COUNT);
			

            // stop Sampling (Paw and Pdiff)
            if( xTimerStop(g_samplingTimer, 20/portTICK_PERIOD_MS) != pdTRUE ) 
            {
                // TODO : What should we do? Raise an alarm ??
            }

            // Compute cycle data
            uint32_t t_ms = get_time_ms();
            g_cycle_EoI_ratio = (float)(t_ms - exhalation_start_ms - exhalation_pause_t_ms) / (exhalation_start_ms - inhalation_start_ms - inhalation_pause_t_ms);
            g_cycle_FR_pm = 1. / (((float)(t_ms - inhalation_start_ms - inhalation_pause_t_ms - exhalation_pause_t_ms)) / 1000 / 60);
            g_cycle_VTe_mL = VTe_start_mL - read_Vol_mL();
            g_cycle_VMe_Lpm = (g_cycle_VTe_mL / 1000) * g_cycle_FR_pm;
			g_cycle_Pcrete_cmH2O = cycle_Pcrete_cmH2O;
            // Notify system that new cycle data is available.
            xEventGroupSetBits(g_breathingEvents, BRTH_CYCLE_UPDATED);

            // Proceed to the PEP regulation
			#if					ENABLE_ASSERVISSEMENT_PEP						==				1
				#define			DETECTION_DECONNEXION_PCrete										(15)	/// 5 PEEP min
				#define			DETECTION_DECONNEXION_DEBIT_MIN_INSU						5.0f		/// lpm
				#define			DETECTION_DECONNEXION_DEBIT_MIN_EXSU					-2.5f		/// lpm
				if( 
						/// poumon_effectivement_detected == true
							cycle_Pcrete_cmH2O		>		DETECTION_DECONNEXION_PCrete
					&&	cycle_Debit_insu_max		>		DETECTION_DECONNEXION_DEBIT_MIN_INSU
					&&	cycle_Debit_exsu_max		<		DETECTION_DECONNEXION_DEBIT_MIN_EXSU
					/// todo : analyse ecart à moyenne cycles précédents : + fin
					
				){ /// todo : attention alarme debranchement circuit sinon elle se plante ds le fond... CHANGEMENTS_POST_LYON_0
					regulation_pep();
					
				} else {
					/// ALARME deconnexion !!!
					brth_printf("\n\n\nALARME DECONNEXION !!! insu %i   exsu %i lpm : Paw %i\n\n\n",
																																	(int)(cycle_Debit_insu_max),
																																	(int)(cycle_Debit_exsu_max),
																																	(int)(cycle_Pcrete_cmH2O)
																															);
				}
			#endif
			
            // Check if controller asked us to stop.
            events = xEventGroupGetBits(g_controllerEvents);
        } while ((events & BREATHING_RUN_FLAG) != 0);

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
    case Insuflation:
        brthState = BRTH_CYCLE_INSUFLATION;
        #if		NIVEAU_VERBOSE_DEBUG		>=		3
		brth_printf("BRTH: Insuflation\n");
		#endif
        break;
    case Plateau:
        brthState = BRTH_CYCLE_PLATEAU;
        #if		NIVEAU_VERBOSE_DEBUG		>=		3
		brth_printf("BRTH: Plateau\n");
		#endif
        break;
    case Exhalation:
        brthState = BRTH_CYCLE_EXHALATION;
        #if		NIVEAU_VERBOSE_DEBUG		>=		3
		brth_printf("BRTH: Exhalation\n");
		#endif
        break;
    }
    // Inform system about current state
    xEventGroupClearBits(g_breathingEvents, (BRTH_CYCLE_INSUFLATION | BRTH_CYCLE_PLATEAU | BRTH_CYCLE_EXHALATION | BRTH_CYCLE_PINS | BRTH_CYCLE_PEXP));
    xEventGroupSetBits(g_breathingEvents, brthState);
}


static void regulation_pep()
{
    float pep_objective = get_setting_PEP_cmH2O(); // TODO: is it really what we want ? Should we use the setting retreived at the beginning of the cycle instead ?
    float current_pep = get_cycle_PEP_cmH2O();
    int relative_pep = (pep_objective * 10.f - current_pep * 10.f);
    if (abs(relative_pep) > 3)
    {
        motor_pep_move((int)((float)relative_pep / MOTOR_PEP_PEP_TO_MM_FACTOR));
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

    // sample Pdiff only during Insuflation state
    if(g_sampling_flow == true) 
    {
        if(g_Pdiff_Lpm_sample_count<MAX_PDIFF_SAMPLES)
        {
			g_Pdiff_Lpm_samples[g_Pdiff_Lpm_sample_count++]= read_Pdiff_Lpm(); /// PASS_MESURES_DEBIT
        }
    }

	#if		USE_PID_VENTILATION_Verreur_accel		==		1
	/// remember errur liee a phase accel
	if( 
				// get_time_ms() > GLOB_TIMER_fin_accel_moteur_ms
		// &&	
				GLOB_Volume_erreur_phase_accel == -12345.0f /// MAJ needed
		&&	GLOB_timecode_ms_full_speed != 0
		&&	( g_Pdiff_Lpm_sample_count * SAMPLING_PERIOD_MS ) >= GLOB_timecode_ms_full_speed
	){ /// GLOB_timecode_ms_full_speed ){
		
		float	current_volume = read_Vol_mL();
		
		float	Volume_attendu_mL = 			( 
																				( 
																						g_setting_VM /// consigne debit
																					// + 	GLOB_debit_to_compensate_ERROR_accel
																				) /// get_setting_Vmax_Lpm()  // debit_consigne_slm
																		/ 		60.0f /// min to sec : MODIF_POST_COMMIT
																	)
																	
																	
															/// a peu pres equivalent
															// * 	GLOB_timecode_ms_full_speed
															*	( g_Pdiff_Lpm_sample_count * SAMPLING_PERIOD_MS )
														;
														
		GLOB_Volume_erreur_phase_accel = Volume_attendu_mL - current_volume;
		
		float		Volume_a_compenser = GLOB_Volume_erreur_phase_accel - GLOB_Volume_insu_AFTER_motor_stop;
		
		float		proportion_erreur = current_volume / Volume_attendu_mL;
		#define		ATTENUATION_OSCILLATION_COMP_VOLUME_ERR			0.85f
		GLOB_debit_to_compensate_ERROR_accel = 
																							ATTENUATION_OSCILLATION_COMP_VOLUME_ERR
																					* 		60.0f * Volume_a_compenser / ( 1000 - GLOB_timecode_ms_full_speed );
		
		
		#if	0
			printf( "\n\n\n--cons %i mL but %i mL -> deb comp %i full speed %i ms : period %i / index %i\n\n\n\n", 
																											(int)( Volume_attendu_mL ),
																											(int)( current_volume ),
																											(int)( GLOB_debit_to_compensate_ERROR_accel * 1000 ),
																											(int)( GLOB_timecode_ms_full_speed ),
																											(int)( SAMPLING_PERIOD_MS ),
																											(int)( g_Pdiff_Lpm_sample_count )
																									);
		#endif
	}
	#endif


    taskEXIT_CRITICAL();
}
