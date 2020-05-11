#include "common.h"
#include "controller.h"
#include "breathing.h"
#include "platform.h"
#include <inttypes.h>
#include "../platforms/recovid_revB/HAL/CMSIS/Include/cmsis_gcc.h"
#include "config.h"

#include <compute_motor.h>
#include <math.h>



//----------------------------------------------------------
// Private defines
//----------------------------------------------------------

#define PERIOD_BREATING_MS (10)
#define MAX_PEP_SAMPLES (100 / PERIOD_BREATING_MS)  // moyenne glissante sur les 100ms dernieres de l'expi
#define MAX_PPLAT_SAMPLES (50 / PERIOD_BREATING_MS) // moyenne glissante sur les 50ms dernieres de plat

//----------------------------------------------------------
// Private typedefs
//----------------------------------------------------------

typedef enum
{
  Insufflation,
  Plateau,
  Exhalation,
  Finished
} BreathingState;

//----------------------------------------------------------
// Private variables
//----------------------------------------------------------

static float g_EoI_ratio;
static float g_FR_pm;
static float g_VTe_mL;
static float g_VTi_mL;
static float g_VMe_Lpm;
static float g_Pcrete_cmH2O;
static float g_Pplat_cmH2O;
static float g_PEP_cmH2O;
static float g_PEP_cmH2O_samples[MAX_PEP_SAMPLES];
static uint32_t g_PEP_cmH2O_samples_index;
static float g_Pplat_cmH2O_samples[MAX_PPLAT_SAMPLES];
static uint32_t g_Pplat_cmH2O_samples_index;

static BreathingState g_state;

static uint32_t g_state_start_ms;
static uint32_t g_cycle_start_ms;



//----------------------------------------------------------
// Private functions prototypes
//----------------------------------------------------------
static void regulation_pep();
static void init_sample_PEP_cmH2O();
static void sample_PEP_cmH2O(float Paw_cmH2O);
static float get_PEP_avg_cmH2O();
static void init_sample_Pplat_cmH2O();
static void sample_Pplat_cmH2O(float Paw_cmH2O);
static float get_Pplat_avg_cmH2O();
static void enterg_state(BreathingState newState);

//----------------------------------------------------------
// Public functions
//----------------------------------------------------------
static float EoI_ratio;       
static float FR_pm;           
static float VTe_mL; 
static float Pcrete_cmH2O; 
static float Pplat_cmH2O; 
static float PEP_cmH2O; 
static float VTe_start=0.;



void breathing_run(void *args)
{
  UNUSED(args);
  EventBits_t events;

  while (true)
  {
    brth_printf("BRTH: Standby\n");
    events = xEventGroupWaitBits(ctrlEventFlags, BREATHING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY);
    brth_printf("BRTH: Started\n");

    // _flow_samples_count=0;
    // A_calibrated= 3.577;
    // B_calibrated= -0.455;

    motor_enable(true);

    g_EoI_ratio = 0;
    g_FR_pm = 0;
    g_VTe_mL = 0;
    g_Pcrete_cmH2O = 0;
    g_Pplat_cmH2O = 0;
    g_PEP_cmH2O = 0;

	#if	1 /// test adrien
	
		
		
		unsigned int _steps = 4000/MOTOR_CORRECTION_USTEPS;
		
		wait_ms( 2500 );
		
		
		while(1){
			
			brth_printf("#################################    : %u\n", get_time_ms() );
			brth_printf("#################################    : %u\n", get_time_ms() );
			brth_printf("#################################    : %u\n", get_time_ms() );
			brth_printf("#################################    : %u\n", get_time_ms() );
			brth_printf("#################################    : %u\n", get_time_ms() );
			brth_printf("----DEMARRAGE test calib     : %ld\n", get_time_ms()  );
			
			
			/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			///
			/// on teste immediatement sur des cycles à vide : on pourrait constituer les tab de const_calibs.h automatiquement + ecriture en flash (easy mais pas prio) WIP

			/// ///////////////////////////////////////////////////////////////////////////////////////
			/// ///////////////////////////////////////////////////////////////////////////////////////
			float				debit_consigne_slm = get_setting_Vmax_Lpm(); /// DEBIT_CIBLE_SLM; /// slm
			
									init_variables_PID(
																			debit_consigne_slm
																		);
			
			
			
			#if			USE_PID_VENTILATION_Verreur_accel		==		1
			// float				GLOB_debit_from_error_slm = 0; /// provenant essentiellement de la phase d'acceleration qu'il va falloir compenser en augmentation de dit (Ti restant fixed)
			// float				GLOB_Volume_erreur_phase_accel = 0;
			// uint32_t			Remember_duree_phase_ACCEL = 0;
			#endif
			
			/// ///////////////////////////////////////////////////////////////////////////////////////
			/// ///////////////////////////////////////////////////////////////////////////////////////
					
					
			#if	1 /// Algo Adrien PID
				

				while ( ( events & BREATHING_RUN_FLAG ) != 0 ){ /// CONTROLE GENERAL IHM
					
					
					if( GLOB_NBR_de_cycles_before_auto_crash_test > 0 ){
						if( --GLOB_NBR_de_cycles_before_auto_crash_test == 0 ){
							hardfault_CRASH_ME();				 /// gasp ;)
						}
					}
						
					
					// enterg_state(Insufflation);
					// TODO: Take into account the time to compute adaptation for the FR calculation ??!!??

					// Get current controller settings
					uint32_t T        		= get_setting_T_ms      (); /// CONTROLE GENERAL IHM
					float    VT       		= get_setting_VT_mL     ();
					float    VM       		= get_setting_Vmax_Lpm  ();
					float    Pmax     		= get_setting_Pmax_cmH2O();
					uint32_t Tplat    	= get_setting_Tplat_ms  ();

					float VTi=0.;
					float VTe=0.;
					
					
					static uint8_t	compteur = 0;
					debit_consigne_slm = VM;
					
					#if	0
					if( something_changed == true ){
							/// on repart de zero et basta... mais on pourraity aussi laisser intouched si augmentation volume : atester et valider Nicolas
							init_variables_PID(
																	debit_consigne_slm
																);
							compteur = 0;
					}
					#endif
					
					// Compute adaptation based on current settings and previous collected data if any.
					uint32_t Ti = T*1/3;  // TODO: Check how to calculate Tinsuflation
					
					
					if( ( compteur++ % 20 ) == 0 ){
						brth_printf("BRTH: T     : %ld\n", T);
						brth_printf("BRTH: VT    : %ld\n", (uint32_t)(VT*100));
						brth_printf("BRTH: VM    : %ld\n", (uint32_t)(VM*100));
						brth_printf("BRTH: Pmax  : %ld\n", (uint32_t)(Pmax*100));
						brth_printf("BRTH: Tplat : %ld\n", Tplat);
						
						brth_printf("BRTH: Ti    : %ld\n", Ti);
					}


					// adaptation(VM, _flow_samples, _flow_samples_count, 0.001*FLOW_SAMPLING_PERIOD_MS, &A_calibrated, &B_calibrated);
					// _flow_samples_count = 0;

					uint32_t d = 300*MOTOR_CORRECTION_USTEPS;
					unsigned int _steps = 4000/MOTOR_CORRECTION_USTEPS;
							
					
					
					
				#if			USE_PID_ON_AUTO_CALIB		==		1
					
					if( 
								compute_PID_factors(
																			// (float)debit_consigne_slm,
																			(float)debit_consigne_slm, /// GLOB_debit_from_error_slm 		sera ajouted ds corps function, mais a passer en argument
																			(uint32_t )Ti /// ms
																		) == false
					){
						brth_printf( "\n\r\n\r\n\r---hardfault_CRASH_ME : compute_PID_factors() error\n\r\n\r\n\r" );
						motor_release(MOTOR_RELEASE_STEP_US);
						wait_ms( 3000 );
						hardfault_CRASH_ME(); /// ALARM
					}
					
					/// OUPUT ETAPE 1 Analyse error : dans l'ordre
					// TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit 
					// TAB_volume_slm_calib[ index_segt ] .MODE_Panique
					// GLOB_MOYENNE_erreur
					// GLOB_MOYENNE_facteur_I_PID
					// GLOB_FACTEUR_linearite_plateau_inspi
					// TAB_volume_slm_calib[ index_segt ] .PID_i_factor 
					// TAB_volume_slm_calib[ index_SEGMENT_max_error_en_cours ] .max_error_checked /// foireux a virer
					// GLOB_debit_from_error_slm

					/// OUTPUT ETAPE 2 commande moteur :
					// VOLUME_segment_corriged /// todo a passer ds TAB_volume_slm_calib
					// TAB_volume_slm_calib[ index_segt ].CALIB_US_avt_motor_full_speed
					// GLOB_index_sgt_full_speed
					// GLOB_timecode_ms_full_speed
					// GLOB_index_premier_segment_FULL_SPEED
					// TAB_volume_slm_calib[ index_segt ].PID_tunable_sgt 
					// g_motor_steps_us
					// GLOB_index_pas_stepper
					// TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT 
					// TAB_volume_slm_calib[ index_segt ].is_vitesse_max_SEGMENT
					// TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US
					// GLOB_duree_TOTAL_theorique_US
					// GLOB_is_first_guess_from_abaques

					/// reinit
					// GLOB_Volume_erreur_phase_accel 
					
					
					
				#endif /// USE_PID_ON_AUTO_CALIB
				
					/// WIP : todo : code MMM a integrer
					valve_inhale();
					enterg_state(Insufflation);
					
					reset_Vol_mL();
					reset_Vol_error_phase_accel(); /// pr compensation debit :  /// GLOB_Volume_erreur_phase_accel = -12345.0f; /// sera remesured ds interrupt i2c

					// GLOB_Volume_erreur_phase_accel = -12345.0f; /// sera remesured ds interrupt i2c
					/// on demarre le sampling en IT
					/// on demarre le sampling en IT ds TAB_dp_raw_temps_moteur			WIP : integrer sampling continu Eric
					uint32_t		maintenant_sampling = get_time_ms();
					start_sampling_temps_moteur( maintenant_sampling );
					/// on demarre le sampling en IT
					/// on demarre le sampling en IT
					

					
					/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					
					
					
				#define			ESSAI_PORTAGE_OLD_COD					0
					
				#if					ESSAI_PORTAGE_OLD_COD 		==		0  /// bricolage adrien le temps du dev
						
						motor_press( g_motor_steps_us, GLOB_index_pas_stepper ); /// GLOB_index_pas_stepper en remplacement de _steps);
						
						
				#else
					
						// motor_press(g_motor_steps_us, _steps);
						motor_press(g_motor_steps_us, GLOB_index_pas_stepper); /// GLOB_index_pas_stepper en remplacement de _steps);
						
						#if			NIVEAU_VERBOSE_DEBUG		>=				3
						// brth_print("BRTH: Insuflation\n");      
						#endif
						while(Insufflation == g_state ) { /// CONTROLE GENERAL IHM
						  if (Pmax <= read_Paw_cmH2O()) {
							  brth_printf("BRTH: Paw [%ld]> Pmax --> Exhalation\n", (int32_t)(read_Paw_cmH2O()));
							  enterg_state(Exhalation);
							  break;
						  } else if (VT <= read_Vol_mL()) {
							  brth_printf("BRTH: vol [%ld]>= VT --> Plateau\n", (int32_t)(read_Vol_mL()));
							  enterg_state(Plateau);
							  break;
						  } else 
						  if( Ti <= (get_time_ms() - g_cycle_start_ms) ) {
							  brth_printf("BRTH: dt [%lu]>= Ti\n", (get_time_ms() - g_cycle_start_ms));
							  enterg_state(Plateau);
							  break;
						  }
						  // Sample flow for later adaptation.
						  // if(_flow_samples_count<MAX_FLOW_SAMPLES) {
						  //   _flow_samples[_flow_samples_count] = read_Pdiff_Lpm()/60.;  // in sls
						  //   ++_flow_samples_count;          
						  // }
						  // wait_ms(FLOW_SAMPLING_PERIOD_MS);
						  wait_ms(10);
						}
					  
					#endif
	  
					/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					
				#if		ESSAI_PORTAGE_OLD_COD 		==		0 /// bricolage adrien le temps du dev
					/// A VIRER !!!
					/// A VIRER !!!
					/// A VIRER !!!
					
					uint32_t			temps_restant_sur_phase_moteur = get_time_ms() - maintenant_sampling + GLOB_duree_TOTAL_theorique_US / 1000;
											wait_ms( temps_restant_sur_phase_moteur ); /// GLOB_duree_TOTAL_theorique_US / 1000 );
					
					
					/// retour moteur HOME
					/// retour moteur HOME
					/// retour moteur HOME
					motor_release(MOTOR_RELEASE_STEP_US);
					/// retour moteur HOME
					/// retour moteur HOME
					/// retour moteur HOME
					
					#if			NIVEAU_VERBOSE_DEBUG			>=		1
						if( GLOB_index_TAB_dp_raw_temps_moteur == 0 ){ /// detection des capteurs i2c plantés...
							brth_printf( "---fin sampling %i < %i\n\r", GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED, NBR_VALEURS_TAB_debits_temps_moteur );
						}
					#endif
					


					/// on attends la fin du sampling DEBIT
					/// on attends la fin du sampling DEBIT
					uint32_t	maintenant = get_time_ms() ;
					while( maintenant < GLOB_TIMER_ms_fin_sampling_temps_moteur ){
						wait_ms( 1 );
						maintenant = get_time_ms() ;
					}
					
					/// normalement le sampling est deja termined :
					if( maintenant >= GLOB_TIMER_ms_fin_sampling_temps_moteur ){
						// GLOB_index_TAB_dp_raw_temps_moteur = 0;
						/// impossible de sampler ds l'IT avec ces valeurs :
						// GLOB_TIMER_ms_debut_sampling_temps_moteur = 1;
						// GLOB_TIMER_ms_fin_sampling_temps_moteur = 0;
						GLOB_is_running_sampling_temps_moteur = false;
						
						if(
							GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED >= NBR_VALEURS_TAB_debits_temps_moteur
						){
								brth_printf( "\n\r\n\r\n\r---hardfault_CRASH_ME : fin sampling %i < %i\n\r\n\r\n\r", GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED, NBR_VALEURS_TAB_debits_temps_moteur );
								motor_release(MOTOR_RELEASE_STEP_US);
								wait_ms( 3000 );
								hardfault_CRASH_ME();
						}
						
					}
					
				#else
						
						motor_release(MOTOR_RELEASE_STEP_US);

						/// on arrete le sampling en interrupt ds TAB_dp_raw_temps_moteur
						/// on arrete le sampling en interrupt ds TAB_dp_raw_temps_moteur
						uint32_t	maintenant = get_time_ms() ;
						if( maintenant >= GLOB_TIMER_ms_fin_sampling_temps_moteur ){
							// GLOB_index_TAB_dp_raw_temps_moteur = 0;
							/// impossible de sampler ds l'IT avec ces valeurs :
							// GLOB_TIMER_ms_debut_sampling_temps_moteur = 1;
							// GLOB_TIMER_ms_fin_sampling_temps_moteur = 0;
							GLOB_is_running_sampling_temps_moteur = false;
							
							if(
								GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED >= NBR_VALEURS_TAB_debits_temps_moteur
							){
									brth_printf( "\n\r\n\r\n\r---hardfault_CRASH_ME : fin sampling %i < %i\n\r\n\r\n\r", GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED, NBR_VALEURS_TAB_debits_temps_moteur );
									hardfault_CRASH_ME();
							}
							
						}

						
				#endif
					
					
					
					/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// LOG : paleocode
					
					#if	0 /// PASS_Function_PID
					for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
					
							
							brth_printf( "---%i vol %i DEB %i slm TUNABLE %u | ERR %i PID_I %i: TIMING ACCEL %i pct = %u ms on %u ms \n\r", 
																																													index_segt,
																																													(int)( TAB_volume_slm_calib[ index_segt ] .CALIB_result_volume_segment * 1000 ), /// volume sgt
																																													(int)( TAB_volume_slm_calib[ index_segt ] .debit_slm_en_cours * 1000 ), /// volume sgt
																																													TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt,
																																													
																																													(int)( TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit * 1000 ), /// erreur debit
																																													
																																													(int)( TAB_volume_slm_calib[ index_segt ] .PID_i_factor * 1000  ),
																																													
																																													
																																													(int)( (float)GLOB_timecode_ms_full_speed/( DUREE_Totale_cycle /1000 ) * 100 ),
																																													GLOB_timecode_ms_full_speed,
																																													DUREE_Totale_cycle / 1000
																																													
																																											);
							
					
					}
					#endif
					/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					
						#if					ESSAI_PORTAGE_OLD_COD 		==		0 /// bricolage adrien le temps du dev
						
							wait_ms( 750 ); /// plateau
							
							VTi = read_Vol_mL();
							VTe_start= VTi; /// read_Vol_mL();

							
							enterg_state(Exhalation);
							valve_exhale();
							
							wait_ms( 4500 ); /// exhale 
							
							float	V_must_be_zero = read_Vol_mL();
							VTe = VTe_start - V_must_be_zero; /// CONTROLE GENERAL IHM
							
							
							
							
									#if			NIVEAU_VERBOSE_DEBUG		>=				1
									
										 #if			NIVEAU_VERBOSE_DEBUG		>=				2
											#define			NBR_PAR_LIGNE_DEBUG			8
											for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
												
												// brth_printf( "%i %u%u~ %i\t",
																							// (int)( TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit * 10 ) ,
																							// TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT,
																							// TAB_volume_slm_calib[ index_segt ].is_vitesse_max_SEGMENT,
																							// (int)( TAB_volume_slm_calib[ index_segt ] .PID_i_factor * 1000 ) 
																					// );
												
												brth_printf( "%i\t",
																							(int)( TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit * 10 ) 
																					);
												
												if( ( index_segt % NBR_PAR_LIGNE_DEBUG ) == NBR_PAR_LIGNE_DEBUG-1 ) brth_printf("\n\r" );
											}
																				
																				
											
										#endif
										
											#if		0
												brth_printf( "---- ERR  \t%i\t%i\tLin  ACCEL %ims \tif max  \t%i < %i  \t-> PID P %i  +  \tmoy PID I :  \t%i\n\r", 
																																																												(int)( GLOB_MOYENNE_erreur * 1000 ),
																																																												(int)( GLOB_FACTEUR_linearite_plateau_inspi * 1000 ),
																																																												(int)GLOB_timecode_ms_full_speed,
																																																												
																																																												(int)( MAX_erreur * 1000 ),
																																																												(int)( LIMITE_BASSE_DESACTIVATION_COMPOSANTE_P_PID * 1000 ),
																																																												(int)( composante_PID_P_factor_SEGMENT_complet * 1000 ),
																																																												(int)( GLOB_MOYENNE_facteur_I_PID * 1000 * 1000 )
																																																												
																																																										);
											#else																																									
												float	pourcentage_erreur = ( V_must_be_zero ) / VTe;
												brth_printf( "---- ERR  \t%i\t%i\tLin  ACCEL %ims \tVTi %i VTe %i  Verr_Acc %i : pmill %i \tmoy PID I :  \t%i\n\r", 
																																																												(int)( GLOB_MOYENNE_erreur * 1000 ),
																																																												(int)( GLOB_FACTEUR_linearite_plateau_inspi * 1000 ),
																																																												(int)GLOB_timecode_ms_full_speed,
																																																												
																																																												(int)( VTi * 1000 ),
																																																												(int)( VTe * 1000 ),
																																																												(int)( GLOB_Volume_erreur_phase_accel * 1000 ),
																																																												
																																																												(int)( pourcentage_erreur * 1000 ),
																																																												
																																																												(int)( GLOB_MOYENNE_facteur_I_PID * 1000 * 1000 )
																																																												
																																																										);
											#endif
											
									#endif
							
							
							
							
										#if		LIMITE_CYCLES_INSPI_TEST_DEBUG		>		0
										static	int	compteur = LIMITE_CYCLES_INSPI_TEST_DEBUG;
										if( compteur-- == 0 ){
											while(1);
										}
										#endif
										
							// prgr_moteur( go_home );
							while( is_motor_home() == false );
							
						#else /// WIP : a integrer !!!
							
							while(Plateau == g_state) { /// CONTROLE GENERAL IHM
							if (Pmax <= read_Paw_cmH2O()) { 
								brth_print("BRTH: Paw > Pmax --> Exhalation\n");
								enterg_state(Exhalation);
							} else if ( is_command_Tpins_expired() && (Tplat <= (get_time_ms() - g_state_start_ms)) ) {
								brth_print("BRTH: Tpins expired && (dt > Tplat)\n");
								enterg_state(Exhalation);
							}
							wait_ms(10);
							}
							VTi= read_Vol_mL();
							valve_exhale();
							
							
							while(Exhalation == g_state) { /// CONTROLE GENERAL IHM
							  if ( T <= (get_time_ms() - g_cycle_start_ms )) { 
								  uint32_t t_ms = get_time_ms();


								  EoI_ratio =  (float)(t_ms-g_cycle_start_ms)/(g_state_start_ms-g_cycle_start_ms);
								  FR_pm     = 1./(((float)(t_ms-g_cycle_start_ms))/1000/60);

								  xEventGroupSetBits(brthCycleState, BRTH_RESULT_UPDATED);

								  // TODO regulation_pep();
								  enterg_state(Finished);
							  }
							wait_ms(10);
							}
							VTe = VTe_start - read_Vol_mL(); /// CONTROLE GENERAL IHM
							
						#endif
						
						events= xEventGroupGetBits(ctrlEventFlags); /// CONTROLE GENERAL IHM
					}
					

					brth_printf("BRTH: Stopping\n");  /// CONTROLE GENERAL IHM
					
					wait_ms(200);
					xEventGroupSetBits(ctrlEventFlags, BREATHING_STOPPED_FLAG); /// CONTROLE GENERAL IHM
			
					#endif
					/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					while( 1 );
					
				
				}
				
			#endif




	/// version MMM
    do  {
		brth_printf("BRTH: start cycle\n");

		enterg_state(Insufflation);

      // Get current controller settings
      uint32_t T = get_setting_T_ms();
      float VT = get_setting_VT_mL();
      float VM = get_setting_Vmax_Lpm();
      float Pmax = get_setting_Pmax_cmH2O();
      uint32_t Tplat = get_setting_Tplat_ms();

      init_sample_PEP_cmH2O();
      init_sample_Pplat_cmH2O();

      brth_printf("BRTH: T     : %lu\n", T);
      brth_printf("BRTH: VT    : %lu\n", (uint32_t)(VT * 100));
      brth_printf("BRTH: VM    : %lu\n", (uint32_t)(VM * 100));
      brth_printf("BRTH: Pmax  : %lu\n", (uint32_t)(Pmax * 100));
      brth_printf("BRTH: Tplat : %lu\n", Tplat);

      // Compute adaptation based on current settings and previous collected data if any.
      uint32_t Ti = T * 1 / 3; // TODO: Check how to calculate Tinsuflation
      brth_printf("BRTH: Ti    : %lu\n", Ti);

      // adaptation(VM, _flow_samples, _flow_samples_count, 0.001*FLOW_SAMPLING_PERIOD_MS, &A_calibrated, &B_calibrated);
      // _flow_samples_count = 0;

      uint32_t d = 300;
      uint32_t steps = 4000;
      //compute_constant_motor_steps(d, _steps, g_motor_steps_us);
      compute_motor_press_christophe(350000, 2000, 65000, 20, 14, 350000, 4000, steps, g_motor_steps_us);
      brth_printf("T_C = %ld Patmo = %ld\n", (int32_t)(read_temp_degreeC() * 100), (int32_t)(read_Patmo_mbar() * 100));

      // Start Inhalation
      valve_inhale();
      motor_press(g_motor_steps_us, steps);
      reset_Vol_mL();
      brth_print("BRTH: Insuflation\n");
      while (Insufflation == g_state)
      {
        wait_ms(PERIOD_BREATING_MS);
        g_Pcrete_cmH2O = MAX(g_Pcrete_cmH2O, read_Paw_cmH2O());
        if (Pmax <= read_Paw_cmH2O())
        {
          brth_printf("BRTH: Paw [%ld]> Pmax --> Exhalation\n", (int32_t)(read_Paw_cmH2O()));
          enterg_state(Exhalation);
          break;
        }
        else if (VT <= read_Vol_mL())
        {
          brth_printf("BRTH: vol [%ld]>= VT --> Plateau\n", (int32_t)(read_Vol_mL()));
          enterg_state(Plateau);
          break;
        }
        else if (Ti <= (get_time_ms() - g_cycle_start_ms))
        {
          brth_printf("BRTH: dt [%lu]>= Ti\n", (get_time_ms() - g_cycle_start_ms));
          enterg_state(Plateau);
          break;
        }
      }
      motor_release(MOTOR_RELEASE_STEP_US);
      while (Plateau == g_state)
      {
        wait_ms(PERIOD_BREATING_MS);
        sample_Pplat_cmH2O(read_Paw_cmH2O());
        g_Pcrete_cmH2O = MAX(g_Pcrete_cmH2O, read_Paw_cmH2O());
        if (Pmax <= read_Paw_cmH2O())
        {
          brth_print("BRTH: Paw > Pmax --> Exhalation\n");
          g_Pplat_cmH2O = get_Pplat_avg_cmH2O();
          enterg_state(Exhalation);
        }
        else if (is_command_Tpins_expired() && (Tplat <= (get_time_ms() - g_state_start_ms)))
        {
          brth_print("BRTH: Tpins expired && (dt > Tplat)\n");
          g_Pplat_cmH2O = get_Pplat_avg_cmH2O();
          enterg_state(Exhalation);
        }
      }
      g_VTi_mL = read_Vol_mL();
      valve_exhale();
      float VTe_start_mL = 0.;
      while (Exhalation == g_state)
      {
        wait_ms(PERIOD_BREATING_MS);
        sample_PEP_cmH2O(read_Paw_cmH2O());
        if( !is_command_Tpexp_expired() ) {
          valve_inhale();
        } 
        else 
        {
          valve_exhale();
          if (T <= (get_time_ms() - g_cycle_start_ms))
          {
            brth_print("BRTH: Tpexp expired && (T <= dt)\n");
            uint32_t t_ms = get_time_ms();

            g_PEP_cmH2O = get_PEP_avg_cmH2O();
            g_EoI_ratio = (float)(t_ms - g_cycle_start_ms) / (g_state_start_ms - g_cycle_start_ms);
            g_FR_pm = 1. / (((float)(t_ms - g_cycle_start_ms)) / 1000 / 60);
            g_VTe_mL = VTe_start_mL - read_Vol_mL();
            g_VMe_Lpm = (g_VTe_mL / 1000) * g_FR_pm;

            xEventGroupSetBits(brthCycleState, BRTH_RESULT_UPDATED);

            regulation_pep();
            enterg_state(Finished);
          }
        }
      }

      events = xEventGroupGetBits(ctrlEventFlags);
    } while ((events & BREATHING_RUN_FLAG) != 0);

    brth_printf("BRTH: Stopping\n");

    wait_ms(200);
    xEventGroupSetBits(ctrlEventFlags, BREATHING_STOPPED_FLAG);
  }
}

static void enterg_state(BreathingState newState) {
  g_state= newState;
  g_state_start_ms = get_time_ms();
  EventBits_t brthState =0;
  switch(g_state) {
    case Insufflation:
      g_cycle_start_ms = get_time_ms();
      brthState =  BRTH_CYCLE_INSUFLATION;
	  #if			NIVEAU_VERBOSE_DEBUG		>=				3
      brth_printf("BRTH: Insuflation...\n");
	  #endif
      break;
    case Plateau:
      brthState =  BRTH_CYCLE_PLATEAU;
	  #if			NIVEAU_VERBOSE_DEBUG		>=				3
      brth_printf("BRTH: Plateau...\n");
	  #endif
      break;
    case Exhalation:
      VTe_start = read_Vol_mL();
      brthState =  BRTH_CYCLE_EXHALATION;
	  #if			NIVEAU_VERBOSE_DEBUG		>=				3
      brth_printf("BRTH: Exhalation...\n");
	  #endif
      break;
    case Finished:
      brthState =  BRTH_CYCLE_FINISHED;
	   #if			NIVEAU_VERBOSE_DEBUG		>=				3
      brth_printf("BRTH: Finished...\n");
	  #endif
      break;
  }
  // Inform system about current state
  xEventGroupClearBits(brthCycleState, (BRTH_CYCLE_INSUFLATION | BRTH_CYCLE_PLATEAU | BRTH_CYCLE_EXHALATION | BRTH_CYCLE_FINISHED));
  xEventGroupSetBits(brthCycleState, brthState);
}

float get_breathing_EoI_ratio() { return g_EoI_ratio; }
float get_breathing_FR_pm() { return g_FR_pm; }
float get_breathing_VTe_mL() { return g_VTe_mL; }
float get_breathing_VTi_mL() { return g_VTi_mL; }
float get_breathing_VMe_Lpm() { return g_VMe_Lpm; }
float get_breathing_Pcrete_cmH2O() { return g_Pcrete_cmH2O; }
float get_Pplat_cmH20() { return g_Pplat_cmH2O; }
float get_PEP_cmH2O() { return g_PEP_cmH2O; }

//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

static void regulation_pep()
{
  float pep_objective = get_setting_PEP_cmH2O(); // TODO: is it really what we want ? Should we use the setting retreived at the beginning of the cycle instead ?
  float current_pep = get_PEP_cmH2O();
  int relative_pep = (pep_objective * 10.f - current_pep * 10.f);
  if (abs(relative_pep) > 3)
  {
    motor_pep_move((int)((float)relative_pep / MOTOR_PEP_PEP_TO_MM_FACTOR));
  }
}

static void init_sample_PEP_cmH2O()
{
  //Samples PEP for a rolling average
  g_PEP_cmH2O_samples_index = 0;
  for (int i = 0; i < MAX_PEP_SAMPLES; i++)
    g_PEP_cmH2O_samples[i] = 0;
}

static void sample_PEP_cmH2O(float Paw_cmH2O)
{
  g_PEP_cmH2O_samples[g_PEP_cmH2O_samples_index] = Paw_cmH2O;
  g_PEP_cmH2O_samples_index = (g_PEP_cmH2O_samples_index + 1) % MAX_PEP_SAMPLES;
}

static float get_PEP_avg_cmH2O()
{
  float sum_PEP = 0;
  for (int i = 0; i < MAX_PEP_SAMPLES; i++)
  {
    sum_PEP += g_PEP_cmH2O_samples[i];
  }
  return (sum_PEP / MAX_PEP_SAMPLES);
}

static void init_sample_Pplat_cmH2O()
{
  //Samples Pplat for a rolling average
  g_Pplat_cmH2O_samples_index = 0;
  for (int i = 0; i < MAX_PPLAT_SAMPLES; i++)
    g_Pplat_cmH2O_samples[i] = 0;
}

static void sample_Pplat_cmH2O(float Paw_cmH2O)
{
  g_Pplat_cmH2O_samples[g_Pplat_cmH2O_samples_index] = Paw_cmH2O;
  g_Pplat_cmH2O_samples_index = (g_Pplat_cmH2O_samples_index + 1) % MAX_PPLAT_SAMPLES;
}

static float get_Pplat_avg_cmH2O()
{
  float sum_Pplat = 0;
  for (int i = 0; i < MAX_PPLAT_SAMPLES; i++)
  {
    sum_Pplat += g_Pplat_cmH2O_samples[i];
  }
  return (sum_Pplat / MAX_PPLAT_SAMPLES);
}
