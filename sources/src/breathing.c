#include "controller.h"
#include "breathing.h"
#include "platform.h"
#include <inttypes.h>

#include "../platforms/recovid_revB/HAL/CMSIS/Include/cmsis_gcc.h"

#include <compute_motor.h>
#include <math.h>


static float EoI_ratio;       
static float FR_pm;           
static float VTe_mL; 
static float Pcrete_cmH2O; 
static float Pplat_cmH2O; 
static float PEP_cmH2O; 
static float VTe_start=0.;



typedef enum { Insufflation, Plateau, Exhalation, Finished } BreathingState;

static BreathingState _state; 

static uint32_t       _state_start_ms; 
static uint32_t       _cycle_start_ms;



static uint32_t _motor_steps_us[MAX_MOTOR_STEPS] = {0};  // TODO: Make it configurable with a define. This represent a physical limit a the system.







// static float compte_motor_step_time(uint32_t step_number, float desired_flow_Ls, float A, float B, float speed);
// static void adaptation(float target_flow_Lm, float* flow_samples, uint32_t nb_samples, float time_step_sec, float* A, float* B);
// static float linear_fit(float* samples, uint32_t samples_len, float time_step_sec, float* slope);
// static int32_t get_plateau(float* samples, uint32_t samples_len, float time_step_sec, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound);


static void enter_state(BreathingState newState);

// extern static volatile bool _home;

#include 		"const_calibs.h"
bool		load_default_calib_from_debit(	float debit_consigne_slm ){
	
	if( debit_consigne_slm < 20 ){ /// on prends 		CONST_calib_10_lpm
		float		proportion_TAB_du_bas = ( 20.0f - debit_consigne_slm ) / 10;
		// proportion_TAB_du_bas = 1.0f; /// debug only : par le bas
		for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
			TAB_volume_slm_calib[ index_segt ].CALIB_result_volume_segment = 
																																		( (float)CONST_calib_10_lpm[ index_segt ] / (1000*1000) ) * proportion_TAB_du_bas
																																	+	( (float)CONST_calib_20_lpm[ index_segt ] / (1000*1000) ) * ( 1.0f - proportion_TAB_du_bas )
																																;
			TAB_volume_slm_calib[ index_segt ].CALIB_nbr_echant = 1;
		}
		return true;
	} else
	if( debit_consigne_slm < 30 ){ /// on prends 		CONST_calib_20_lpm
		float		proportion_TAB_du_bas = ( 30.0f - debit_consigne_slm ) / 10;
		// proportion_TAB_du_bas = 1.0f; /// debug only : par le bas
		for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
			TAB_volume_slm_calib[ index_segt ].CALIB_result_volume_segment = 
																																		( (float)CONST_calib_20_lpm[ index_segt ] / (1000*1000) ) * proportion_TAB_du_bas
																																	+	( (float)CONST_calib_30_lpm[ index_segt ] / (1000*1000) ) * ( 1.0f - proportion_TAB_du_bas )
																																;
			TAB_volume_slm_calib[ index_segt ].CALIB_nbr_echant = 1;
		}
		return true;
	} else
	if( debit_consigne_slm < 40 ){ /// on prends 		CONST_calib_30_lpm
		float		proportion_TAB_du_bas = ( 40.0f - debit_consigne_slm ) / 10;
		// proportion_TAB_du_bas = 1.0f; /// debug only : par le bas
		for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
			TAB_volume_slm_calib[ index_segt ].CALIB_result_volume_segment = 
																																		( (float)CONST_calib_30_lpm[ index_segt ] / (1000*1000) ) * proportion_TAB_du_bas
																																	+	( (float)CONST_calib_40_lpm[ index_segt ] / (1000*1000) ) * ( 1.0f - proportion_TAB_du_bas )
																																;
			TAB_volume_slm_calib[ index_segt ].CALIB_nbr_echant = 1;
		}
		return true;
	} else
	if( debit_consigne_slm < 50 ){ /// on prends 		CONST_calib_40_lpm
		float		proportion_TAB_du_bas = ( 50.0f - debit_consigne_slm ) / 10;
		// proportion_TAB_du_bas = 1.0f; /// debug only : par le bas
		for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
			TAB_volume_slm_calib[ index_segt ].CALIB_result_volume_segment = 
																																		( (float)CONST_calib_40_lpm[ index_segt ] / (1000*1000) ) * proportion_TAB_du_bas
																																	+	( (float)CONST_calib_50_lpm[ index_segt ] / (1000*1000) ) * ( 1.0f - proportion_TAB_du_bas )
																																;
			TAB_volume_slm_calib[ index_segt ].CALIB_nbr_echant = 1;
		}
		return true;
	} else
	if( debit_consigne_slm < 60 ){ /// on prends 		CONST_calib_50_lpm
		float		proportion_TAB_du_bas = ( 60.0f - debit_consigne_slm ) / 10;
		// proportion_TAB_du_bas = 1.0f; /// debug only : par le bas
		for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
			TAB_volume_slm_calib[ index_segt ].CALIB_result_volume_segment = 
																																		( (float)CONST_calib_50_lpm[ index_segt ] / (1000*1000) ) * proportion_TAB_du_bas
																																	+	( (float)CONST_calib_60_lpm[ index_segt ] / (1000*1000) ) * ( 1.0f - proportion_TAB_du_bas )
																																;
			TAB_volume_slm_calib[ index_segt ].CALIB_nbr_echant = 1;
		}
		return true;
	} else					
	if( debit_consigne_slm < 70 ){ /// on prends 		CONST_calib_60_lpm
		float		proportion_TAB_du_bas = ( 70.0f - debit_consigne_slm ) / 10;
		// proportion_TAB_du_bas = 1.0f; /// debug only : par le bas
		for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
			TAB_volume_slm_calib[ index_segt ].CALIB_result_volume_segment = 
																																		( (float)CONST_calib_60_lpm[ index_segt ] / (1000*1000) ) * proportion_TAB_du_bas
																																	+	( (float)CONST_calib_70_lpm[ index_segt ] / (1000*1000) ) * ( 1.0f - proportion_TAB_du_bas )
																																;
			TAB_volume_slm_calib[ index_segt ].CALIB_nbr_echant = 1;
		}
		return true;
	}
	else
	if( debit_consigne_slm < 80 ){ /// on prends 		CONST_calib_70_lpm
		float		proportion_TAB_du_bas = ( 80.0f - debit_consigne_slm ) / 10;
		// float			proportion_TAB_du_bas = 1.0f; /// debug only : par le bas
		for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
			TAB_volume_slm_calib[ index_segt ].CALIB_result_volume_segment = 
																																		( (float)CONST_calib_70_lpm[ index_segt ] / (1000*1000) ) * proportion_TAB_du_bas
																																	+	( (float)CONST_calib_80_lpm[ index_segt ] / (1000*1000) ) * ( 1.0f - proportion_TAB_du_bas )
																																;
			TAB_volume_slm_calib[ index_segt ].CALIB_nbr_echant = 1;
		}
		return true;
	} 
	else
	if( debit_consigne_slm < 90 ){ /// on prends 		CONST_calib_80_lpm
		float		proportion_TAB_du_bas = ( 90.0f - debit_consigne_slm ) / 10;
		// proportion_TAB_du_bas = 1.0f; /// debug only : par le bas
		for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
			TAB_volume_slm_calib[ index_segt ].CALIB_result_volume_segment = 
																																		( (float)CONST_calib_80_lpm[ index_segt ] / (1000*1000) ) * proportion_TAB_du_bas
																																	+	( (float)CONST_calib_90_lpm[ index_segt ] / (1000*1000) ) * ( 1.0f - proportion_TAB_du_bas )
																																;
			TAB_volume_slm_calib[ index_segt ].CALIB_nbr_echant = 1;
		}
		return true;
	} else
	{
	// if( debit_consigne_slm < 100 ){ /// on prends 		CONST_calib_90_lpm
		// float		proportion_TAB_du_bas = ( 100.0f - debit_consigne_slm ) / 10;
		// proportion_TAB_du_bas = 1.0f; /// debug only : par le bas
		for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
			TAB_volume_slm_calib[ index_segt ].CALIB_result_volume_segment = 
																																		( (float)CONST_calib_90_lpm[ index_segt ] / (1000*1000) )
																																;
			TAB_volume_slm_calib[ index_segt ].CALIB_nbr_echant = 1;
		}
		return true;
	}
	
	return false;
}




float		Segment_error_time_sliced(
															/// INPUT : TAB_dp_raw_temps_moteur
															int				index_segt,
															uint32_t		TIMER_US_debut_segment, 						/// INPUT : TIMER_US_debut_segment
															uint32_t		TIMER_US_fin_segment, 								//// INPUT : TIMER_US_fin_segment
															uint32_t		TIME_SLICING_ERROR_debut, 					/// INPUT : TIME_SLICING_ERROR_debut
															uint32_t		TIME_SLICING_ERROR_fin, 						/// INPUT : TIME_SLICING_ERROR_fin
															/// important que ces 2 variables soient JUSTES et mise à jour !!!																				WIP : a verifier
															uint64_t		TIMER_fin_accel_moteur_ms, 								// TIMER_fin_accel_moteur_ms = 1;
															uint64_t		TIMER_fin_sampling_temps_moteur 			// TIMER_fin_sampling_temps_moteur = 0;
){
							
		int64_t			Debit_moyen_dp_raw_SEGMENT = 0;
		// uint64_t		Timecode_moyen_Debit_SEGMENT = 0;
		int				denom = 0;
		
		if(
				TIMER_US_fin_segment / 1000 + TIME_SLICING_ERROR_fin <= TIMER_fin_accel_moteur_ms
		){
			/// segment inutilisable car accel pure
			return 0.0f;
		}
		
		/// d'abord on cherche la Paw du segment
		uint64_t		timecode_milieu_segment_MS = TIMER_US_debut_segment + (TIMER_US_fin_segment - TIMER_US_debut_segment)  / 2;
							timecode_milieu_segment_MS /= 1000;
							
		uint8_t		Paw_milieu_segment = 0;
		for( int index_TAB_sampl_DEBITS = 0;  index_TAB_sampl_DEBITS < index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED; index_TAB_sampl_DEBITS++ ){
			
			if( timecode_milieu_segment_MS >= TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS ){
				Paw_milieu_segment = TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].Paw;
				break;
			}
		}
		
		#if		1 
		int		Time_slicing_DEBUT 	= TIME_SLICING_ERROR_debut;
		int		Time_slicing___FIN 		= TIME_SLICING_ERROR_fin;
		#else /// WIP : il y a une relation fine a Paw certainement : en tout cas a compliance faible, dc Paw elevee, on observe que la fin du segment oscille un peu
		int		Time_slicing_DEBUT 	= TIME_SLICING_ERROR_debut 	- Paw_milieu_segment / 5; if( Time_slicing_DEBUT < 0 ) Time_slicing_DEBUT = 0;
		int		Time_slicing___FIN 		= TIME_SLICING_ERROR_fin 		- Paw_milieu_segment / 5; if( Time_slicing___FIN < 10 ) Time_slicing___FIN = 10;
		#endif
		
		for( int index_TAB_sampl_DEBITS = 0;  index_TAB_sampl_DEBITS < index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED; index_TAB_sampl_DEBITS++ ){
		
			if(
							TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS  >= TIMER_US_debut_segment / 1000 + Time_slicing_DEBUT
					&&	TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS >= TIMER_fin_accel_moteur_ms
				// &&	TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS  <= TIMER_US_fin_segment
			){
				
				if( 
						TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS  > TIMER_US_fin_segment / 1000 + Time_slicing___FIN
					||	TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS  > TIMER_fin_sampling_temps_moteur
				) break; /// cdtion de sortie

				Debit_moyen_dp_raw_SEGMENT 					+= TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].dp_raw;
				// Timecode_moyen_Debit_SEGMENT 	+= TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS;
				
				denom++;
			}
		} /// fin du for 
		
		
		if(
					index_segt 		< NBR_SEGMENTS_CALIBRATION
			&&	index_segt 		>= NBR_SEGMENTS_CALIBRATION - 3
			&&	denom 			<= 0
		){
			
			uint32_t		timecode_le_plus_proche = 0;
			
			/// TIME SLICING impossible....
			for( int index_TAB_sampl_DEBITS = ( index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED ) - 1;  index_TAB_sampl_DEBITS >= 0; index_TAB_sampl_DEBITS-- ){
			
				if(
							TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS  <= TIMER_US_fin_segment / 1000 
				){

					Debit_moyen_dp_raw_SEGMENT 					= TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].dp_raw;
					denom = 1;
					break;
					/// on prends le tout dernier echantillon sur le temps moteur effectif
				}
			} /// fin du for 
		}
			
		if( denom <= 0 ){
			TAB_volume_slm_calib[ index_segt ] .MODE_Panique = true;
			brth_printf( "\n\r\n\r\n\r\n\r\n\r--- PANIQUE SGT %i : pas de sample debit within %i !!! %i sample at time %u %u : Paw %u\n\r\n\r\n\r\n\r\n\r\n\r", 
																																																			index_segt,
																																																			index_TAB_dp_raw_temps_moteur,
																																																			denom,
																																																			(uint32_t)TIMER_US_debut_segment,
																																																			(uint32_t)TIMER_US_fin_segment,
																																																			Paw_milieu_segment
																																																		);
																																																		
			
			#if	1 /// ON SAMPLE L'ECHANTILLON LE PLUS PROCHE : la frequence d'echantillonage n'est pas stable...
			
				uint32_t	meilleur_ecart = 1000 * 1000;
				int			meilleur_index_trouved = ( index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED ) / 2; /// milieu par defaut
				
				for( int index_TAB_sampl_DEBITS = 0;  index_TAB_sampl_DEBITS < index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED; index_TAB_sampl_DEBITS++ ){
					
					uint32_t 		ecart;
					if( timecode_milieu_segment_MS > TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS ){
						ecart = timecode_milieu_segment_MS - TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS;
					} else {
						ecart = TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS - timecode_milieu_segment_MS;
					}
					
					if( ecart < meilleur_ecart ){
						meilleur_ecart = ecart;
						meilleur_index_trouved = index_TAB_sampl_DEBITS;
					}
					
				}
				/// meilleur_index_trouved
				
				Debit_moyen_dp_raw_SEGMENT = TAB_dp_raw_temps_moteur[ meilleur_index_trouved ].dp_raw;
				denom = 1;
			
			#else
				hardfault_CRASH_ME();				 /// gasp ;)
			#endif
		}
		
	Debit_moyen_dp_raw_SEGMENT /= denom;
	
	return -(float)( Debit_moyen_dp_raw_SEGMENT / 105.0f );
}
/// OUPUT : Debit_slm_segment
						

void breathing_run(void *args) {
  UNUSED(args);
  EventBits_t events;

  while(true) {
    brth_printf("BRTH: Standby\n");
    events= xEventGroupWaitBits(ctrlEventFlags, BREATHING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    brth_printf("BRTH: Started\n");

    // _flow_samples_count=0;
    // A_calibrated= 3.577;
    // B_calibrated= -0.455;

    motor_enable(true);

    EoI_ratio=0;
    FR_pm=0;
    VTe_mL=0;
    Pcrete_cmH2O=0;
    Pplat_cmH2O=0;
    PEP_cmH2O=0;

	#if	1 /// test adrien
	

		/// ne demarrera pas le sampling par defaut ds TAB_dp_raw_temps_moteur
		__disable_irq();
		
				// TIMER_debut_sampling_temps_moteur = 1;
				// TIMER_fin_sampling_temps_moteur = 0;
				is_running_sampling_temps_moteur = false;
				index_TAB_dp_raw_temps_moteur = 0;
				denom_TAB_dp_raw_temps_moteur = 0;
				accumule_TAB_dp_raw_temps_moteur = 0;
				accumule_TAB_TIMECODE_temps_moteur = 0;
				
		__enable_irq();
		
		/// reinit tab
		/// reinit tab
		/// reinit tab
		for( int	index_TAB = 0; index_TAB < NBR_VALEURS_TAB_debits_temps_moteur; index_TAB++ ){
			TAB_dp_raw_temps_moteur[ index_TAB ].dp_raw = 0;
			TAB_dp_raw_temps_moteur[ index_TAB ].timecode_sample_MS = 0;
		}
		
		for( int i = 0; i < NBR_SEGMENTS_CALIBRATION; i++ ){
			TAB_volume_slm_calib[ i ] .CALIB_nbr_echant 								= 0;
			TAB_volume_slm_calib[ i ] .CALIB_duree_sgt_US 							= 0;
			TAB_volume_slm_calib[ i ] .CALIB_result_volume_segment 			= 0;
			TAB_volume_slm_calib[ i ] .CALIB_US_avt_motor_full_speed 		= 0;
			TAB_volume_slm_calib[ i ] .debit_slm_en_cours								= 0;
			
			
			#if			USE_PID_ON_AUTO_CALIB		==		1
				TAB_volume_slm_calib[ i ] .PID_ERREUR_Debit 						= 0.0f;
				// TAB_volume_slm_calib[ i ] .PID_Comp_Volume_Debit 				= 0.0f;
				// TAB_volume_slm_calib[ i ] .PID_p_factor 									= 0.0f;
				TAB_volume_slm_calib[ i ] .PID_i_factor 										= 0.0f;
				
				TAB_volume_slm_calib[ i ] .max_error_checked 						= false;
				TAB_volume_slm_calib[ i ] .PID_tunable_sgt 								= true;
				TAB_volume_slm_calib[ i ] .MODE_Panique								= false;
				TAB_volume_slm_calib[ i ] .is_vitesse_max_SEGMENT			= false;
				TAB_volume_slm_calib[ i ] .is_accel_max_SEGMENT 				= false;
				// TAB_volume_slm_calib[ i ] .is_TUNABLE_segment					= true;
			#endif
		}
		/// reinit tab
		/// reinit tab
		/// reinit tab
		
		unsigned int _steps = 4000/MOTOR_CORRECTION_USTEPS;
		
		wait_ms( 2500 );
		
		
		while(1){
			
			brth_printf("#################################    : %u\n", get_time_ms() );
			brth_printf("#################################    : %u\n", get_time_ms() );
			brth_printf("#################################    : %u\n", get_time_ms() );
			brth_printf("#################################    : %u\n", get_time_ms() );
			brth_printf("#################################    : %u\n", get_time_ms() );
			brth_printf("----DEMARRAGE test calib     : %ld\n", get_time_ms()  );
			
			
			/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la consigne de debit est 			DEBIT_CIBLE_SLM
			/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la consigne de debit est 			DEBIT_CIBLE_SLM
			/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la consigne de debit est 			DEBIT_CIBLE_SLM
			#define					NBR_PAS_MOTEUR_MAX													MAX_MOTOR_STEPS  /// 4800 //4480
			// #define					NBR_SEGMENTS_CALIBRATION										64
			
			#define					NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE				( (float)NBR_PAS_MOTEUR_MAX / NBR_SEGMENTS_CALIBRATION )
			
			#define					VITESSE_DEMARRAGE_CALIB											( 2048 ) // us pas moteur * MOTOR_CORRECTION_USTEPS
			
			int32_t					vitesse_cste_moteur_US 															= VITESSE_DEMARRAGE_CALIB;
			
			#define					NBR_ITERATION_CALIB														3
			int							nbr_tours_calib = NBR_ITERATION_CALIB;
			
			#define					MAX_ACCEL_MOTEUR_POSITIF										( 40 ) /// 40
			#define					MAX_ACCEL_MOTEUR_NEGATIF									( 45 ) /// 45
			#define					DUREE_STEP_MIN_MOTOR												380 /// cad vitesse max
			#define					DUREE_ARRET_MOTEUR_PR_ACCEL							2048
			
			/// P meilleur que I ???
			#define					USE_PID_ON_AUTO_CALIB												1
			#define							FACTEUR_P_PID_CONSIGNE_DEBIT													0.20f /// 0.33f max
			#define									LIMITE_BASSE_DESACTIVATION_COMPOSANTE_P_PID					(-1500.0f/1000) /// on desactivera la 		composante_PID_P_factor_SEGMENT_complet		a l'approche de la convergence, pour ne laisser que la composante Integral
			#define							FACTEUR_I_PID_CONSIGNE_DEBIT_ERR_NEGATIVE					0.0003f /// mode pas de panik : 0.00005f
			#define							FACTEUR_P_PID_CONSIGNE_DEBIT_ERR_POSITIVE						0.50f  /// en cas de debranchement patient par ex : on diminue en mode vlan
			#define					MAX_INTEGRALE_ACCUMULEE_PID														(40.0f /1000) /// 2000 : on empeche les tres grosses accumulations /// todo_lyon : 
			
			#define				PROPORTION_FACTEUR_LINEARITE_PLATEAU							7.5f /// diminuer pour ajouter de la resolution
			#define						MINI_FACTEUR_LINEARITE_PLATEAU												0.05f
			#define						MAXI_FACTEUR_LINEARITE_PLATEAU											0.80f
			
			/// 10 - 40 marche tres bien pour les compliances basses : il va falloir tenir compte de la Paw : voir 		Segment_error_time_sliced()		WIP
			#define							TIME_SLICING_ERROR_debut										10 /// ms
			#define							TIME_SLICING_ERROR_fin												40 /// ms
			
			
			
			#define					ENABLE_LISSAGE_COURBE_FACT_INTEGRALE			0 /// NE PAS UTILISER c'est foireux...
			#define							FACTEUR_LISSAGE_NM1													0.08f
			#define							FACTEUR_LISSAGE_NP1													0.08f
			
			
			/// sampling debit IT pendant PHASE MOTEUR inspi
			#define					DEBIT_OVERSAMPLING_TIME											0 /// ms apres arret moteur (toujours un peu de débit) : mais on en tient pas compte finalement pour la pid
			
						
			#define					USE_PID_VENTILATION_Verreur_accel							0 /// WIP : pas encore tested, pas meme compiled
			#define							FACTEUR_I_PID_VENTILATION_ERREUR_PHASE_ACCEL				0.0002f
			
			
			#define					DEBIT_CIBLE_SLM																60 /// slm
			// #if							DEBIT_CIBLE_SLM		< 		10
				// #error 		"DEBIT_CIBLE_SLM trop bas..."
			// #endif
			
			#define					DEBUG_PRINTF_CYCLE_PID											0 /// ---DEBUT sampling data
			#define					DEBUG_PRINTF_PR_CALIB_A_VIDE								0
			
			#define					ENABLE_PID_REGULATION												1
			
			
			#define					NIVEAU_VERBOSE_DEBUG												1
			/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la consigne de debit est 			DEBIT_CIBLE_SLM
			/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la consigne de debit est 			DEBIT_CIBLE_SLM
			/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la consigne de debit est 			DEBIT_CIBLE_SLM
			


			/// Proces AUTO CALIBRATION des volumes pour chaque segment du ballon ambu
			/// Proces AUTO CALIBRATION des volumes pour chaque segment du ballon ambu
			/// Proces AUTO CALIBRATION des volumes pour chaque segment du ballon ambu
			
			#if	0 /// Algo Adrien : auto calibration volumes pour chaque segment
			
					#define		DEBUG_wait_duree				0
					
					int	NBR_pas_moteur_par_segment = round( NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE );
					
					
					while( nbr_tours_calib-- ){
						
						
						brth_printf("------------------------NBR_PAS_MOTEUR_MAX \t\t\t%u\n", NBR_PAS_MOTEUR_MAX);
						brth_printf("------------------------NBR_SEGMENTS_CALIBRATION \t\t%u\n", NBR_SEGMENTS_CALIBRATION);
						brth_printf("------------------------NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE \t%i\n", (int)( round( NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE ) ));
						brth_printf("------------------------VITESSE_DEMARRAGE_CALIB \t\t%u\n", VITESSE_DEMARRAGE_CALIB);
						brth_printf("------------------------vitesse_cste_moteur_US \t\t\t%u\n", vitesse_cste_moteur_US);
						
						/// chgt de vitesse :
						for( int i = 0; i < NBR_SEGMENTS_CALIBRATION; i++ ){
							TAB_volume_slm_calib[ i ] .CALIB_US_avt_motor_full_speed 		= 0;
						}
						
						
						enter_state(Insufflation);
						valve_inhale();
						
						/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						///
						/// DEBUT CALIBRATION
					
						
						float		duree_segment_analyse_US = vitesse_cste_moteur_US * NBR_pas_moteur_par_segment;
						// EN REMPLACEMENT DE : compute_constant_motor_steps(d, _steps, _motor_steps_us);
						
						uint64_t	duree_TOTAL_theorique_US = 0;
						uint32_t	duree_SEGMENT_theorique_US = 0;
						
						int32_t	Last_duree_step_moteur_en_cours = DUREE_ARRET_MOTEUR_PR_ACCEL; /// on considere moteur quasi a l'arret;
						
						/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						///
						/// on consitute la MAP DMA de controle moteur :
						/// on consitute la MAP DMA de controle moteur :
						/// on consitute la MAP DMA de controle moteur :
						int	index_segt = 0;
						
						for( int i_step = 0; i_step < NBR_PAS_MOTEUR_MAX; i_step++ ){
							
							/// gestion acceleration
							uint32_t  	duree_step_US;
							int32_t		Accel_moteur = Last_duree_step_moteur_en_cours - vitesse_cste_moteur_US;
							if(
											-Accel_moteur > MAX_ACCEL_MOTEUR_NEGATIF
							){
								/// phase d'acceleration intiale
								duree_step_US = Last_duree_step_moteur_en_cours + MAX_ACCEL_MOTEUR_NEGATIF;
								TAB_volume_slm_calib[ index_segt ].CALIB_US_avt_motor_full_speed += duree_step_US;
							} else if(
											Accel_moteur > MAX_ACCEL_MOTEUR_POSITIF
							){
								/// phase d'acceleration intiale
								duree_step_US = Last_duree_step_moteur_en_cours - MAX_ACCEL_MOTEUR_POSITIF;
								TAB_volume_slm_calib[ index_segt ].CALIB_US_avt_motor_full_speed += duree_step_US;
							} else {
								/// phase FULL SPEED
								duree_step_US = vitesse_cste_moteur_US;
							}
							Last_duree_step_moteur_en_cours = duree_step_US;
							
							
							/// on save pr le DMA
							_motor_steps_us[ i_step ] = duree_step_US;
							
							
							duree_SEGMENT_theorique_US += duree_step_US;
							if( 
										i_step > 0
								&&	( i_step % NBR_pas_moteur_par_segment ) == 0
							){
								
								#if			DEBUG_wait_duree		==		1
										brth_printf("----todo Segt %i : duree : %u\n", index_segt, duree_SEGMENT_theorique_US );
								#endif
								
								TAB_volume_slm_calib[ index_segt++ ].CALIB_duree_sgt_US = duree_SEGMENT_theorique_US;
								
								/// MAJ
								duree_SEGMENT_theorique_US = 0;
							}
							
							duree_TOTAL_theorique_US += duree_step_US;
							
						} /// fin du for
						/// on consitute la MAP DMA de controle moteur :
						/// on consitute la MAP DMA de controle moteur :
						/// on consitute la MAP DMA de controle moteur :
						
						
						
						/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						///
						/// ETAPE_calib 1 : on demarre le programme DMA moteur
						
						#if		0
						for( int i_step = 0; i_step < NBR_PAS_MOTEUR_MAX; i_step++ ){
							brth_printf( "-- %"PRIu32" %i\n\r", _motor_steps_us[ i_step ], i_step  );
						}
						#endif
						
						motor_press( _motor_steps_us, NBR_PAS_MOTEUR_MAX );
						
						/// on demarre le sampling en IT
						/// on demarre le sampling en IT ds TAB_dp_raw_temps_moteur
						
						uint32_t	maintenant = get_time_ms();
						
						__disable_irq();
						
							index_TAB_dp_raw_temps_moteur = 0;
							denom_TAB_dp_raw_temps_moteur = 0;
							accumule_TAB_dp_raw_temps_moteur = 0;
							accumule_TAB_TIMECODE_temps_moteur = 0;
							
							TIMER_debut_sampling_temps_moteur 	= maintenant;
							TIMER_fin_sampling_temps_moteur 		= maintenant + duree_TOTAL_theorique_US / 1000 + DEBIT_OVERSAMPLING_TIME;
							is_running_sampling_temps_moteur 		= true; /// on demarre l'acquisition en IT
							
						__enable_irq();
						
						brth_printf( "---sampling %u\n\r", TIMER_debut_sampling_temps_moteur - TIMER_debut_sampling_temps_moteur );
						/// on demarre le sampling en IT
						/// on demarre le sampling en IT
						
						
						uint32_t			Remember_timer_demarrage_moteur_MS	 	= get_time_ms(); /// get_timer_us();
						uint32_t			Remember_timer_last_echant 							= Remember_timer_demarrage_moteur_MS;
						uint64_t		 	timecode_us_next_echant 								= Remember_timer_demarrage_moteur_MS * 1000;
						
						/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						///
						/// ETAPE 1 MOYENNE : on remplit le tableau avec les valeurs
						
						
						#if			DEBUG_wait_duree		==		1
						uint32_t	DEBUG_wait_duree_MS[ NBR_SEGMENTS_CALIBRATION ];
						for( int i_segt = 0; i_segt < NBR_SEGMENTS_CALIBRATION; i_segt++ ){
							DEBUG_wait_duree_MS[ i_segt ]  = 0;
							// brth_printf("----done Segt %i : duree : %u\n", index_segt, DEBUG_wait_duree_MS[ i_segt ] );
						}
						#endif
						
						for( int i_segt = 0; i_segt < NBR_SEGMENTS_CALIBRATION; i_segt++ ){
							
							timecode_us_next_echant += TAB_volume_slm_calib[ i_segt ].CALIB_duree_sgt_US;
							
							uint32_t		timecode_ms_next_echant = timecode_us_next_echant / 1000;
							
							
							uint32_t	duree_wait = timecode_ms_next_echant - get_time_ms() ;
							
							/// on fait une moyenne sur le segment : attention ce serait + juste avec 5 ms d'ecart (200 Hz)
							float		moyenne_debit = 0;
							int		denom = 0;
							
							/// on attends que le moteur ait atteins la FULL SPEED
							if( TAB_volume_slm_calib[ i_segt ].CALIB_US_avt_motor_full_speed >= 1000 ){
								wait_ms( (uint32_t)TAB_volume_slm_calib[ i_segt ].CALIB_US_avt_motor_full_speed / 1000 );
							}
							uint32_t	maintenant = get_time_ms();
							
							while(  maintenant < timecode_ms_next_echant ){
								
								wait_ms( 1 );
								
								moyenne_debit += read_Pdiff_Lpm();
								denom++;
								
								maintenant = get_time_ms();
							}
							float 			debit_slm_en_cours = (float)moyenne_debit / denom;
							// brth_printf( "--%i", (int)debit_slm_en_cours*1000 );
						
							#if			DEBUG_wait_duree		==		1
							DEBUG_wait_duree_MS[ i_segt ] = duree_wait;
							// DEBUG_wait_duree_MS[ i_segt ] = timecode_ms_next_echant;
							#endif
							
								
							
							/// verifier timing avec toggle pin !!!!!!!!!
							/// verifier timing avec toggle pin !!!!!!!!!
							
							uint32_t	timer_mesure_en_cours 	= maintenant;
							
							/// verifier timing avec toggle pin !!!!!!!!!
							/// verifier timing avec toggle pin !!!!!!!!!
							
							#define			MIN_VAL_LECTURE_SIGNIFICATIVE_DEBIT			0.0001 /// slm
							// #define			MAX_VAL_LECTURE_SIGNIFICATIVE_DEBIT			255 /// slm
							if(
									fabs( debit_slm_en_cours ) > MIN_VAL_LECTURE_SIGNIFICATIVE_DEBIT
								// &&	debit_slm_en_cours < MAX_VAL_LECTURE_SIGNIFICATIVE_DEBIT
							){
								uint32_t	duree_mesure_en_cours = timer_mesure_en_cours - Remember_timer_last_echant;
								
								TAB_volume_slm_calib[ i_segt ].CALIB_nbr_echant++;
								TAB_volume_slm_calib[ i_segt ].CALIB_result_volume_segment += debit_slm_en_cours * ( (float)duree_mesure_en_cours/(60*1000) ); /// todo : checker unité L ? mL ? uS ou S, min...
								
							}
							
							// brth_printf( "--%u", duree_wait );
							
							Remember_timer_last_echant = timer_mesure_en_cours; /// MAJ
							
						} /// fin du for

						
						/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
							
						enter_state(Exhalation);
						// valve_exhale();
						
						/// retour moteur HOME
						motor_release();
						
						#if			DEBUG_wait_duree		==		1
						for( int i_segt = 0; i_segt < NBR_SEGMENTS_CALIBRATION; i_segt++ ){
							brth_printf("----done Segt %i : duree : %u\n", i_segt, DEBUG_wait_duree_MS[ i_segt ] );
						}
						#endif
						
						while( is_motor_home() == false );
						
						/// on passe a vitesse superieure pr tour suivant
						vitesse_cste_moteur_US -= 512;
					

						/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						
					} /// fin du while
					
					
					
					
					/// ETAPE 2 MOYENNE : denom
					for( int i_segt = 0; i_segt < NBR_SEGMENTS_CALIBRATION; i_segt++ ){
						
						if( TAB_volume_slm_calib[ i_segt ].CALIB_nbr_echant == 0 ){
							
							brth_printf( "___error___ @%i pas de debit significatif : augmenter vitesse depart\n\r", i_segt );
							if( i_segt > 0 ){ /// bug dernier segment
								TAB_volume_slm_calib[ i_segt ].CALIB_result_volume_segment 	= TAB_volume_slm_calib[ i_segt - 1 ].CALIB_result_volume_segment;
								TAB_volume_slm_calib[ i_segt ].CALIB_nbr_echant 						= TAB_volume_slm_calib[ i_segt - 1 ].CALIB_nbr_echant;
							}
							
						} else {
							TAB_volume_slm_calib[ i_segt ].CALIB_result_volume_segment = TAB_volume_slm_calib[ i_segt ].CALIB_result_volume_segment  / TAB_volume_slm_calib[ i_segt ].CALIB_nbr_echant;
							
							#if	0
								brth_printf( "TAB_volume_slm_calib[ %i ].CALIB_result_volume_segment = (float)%i/1000000;\n\r", 
																	i_segt,
																	(int)round( (float)TAB_volume_slm_calib[ i_segt ].CALIB_result_volume_segment * 1000000 )
								);
							#else
								brth_printf( "%i\n\r",
																	(int)round( (float)TAB_volume_slm_calib[ i_segt ].CALIB_result_volume_segment * 1000000 )
								);
							#endif
							
						}
					}
					
					
			#endif
			/// Proces AUTO CALIBRATION des volumes pour chaque segment du ballon ambu
			/// Proces AUTO CALIBRATION des volumes pour chaque segment du ballon ambu
			/// Proces AUTO CALIBRATION des volumes pour chaque segment du ballon ambu	
	
	
	
	
			
			
			
			
			
			
			
			/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			///
			/// on teste immediatement sur des cycles à vide : on pourrait constituer les tab de const_calibs.h automatiquement + ecriture en flash (easy mais pas prio) WIP

			/// ///////////////////////////////////////////////////////////////////////////////////////
			/// ///////////////////////////////////////////////////////////////////////////////////////
			float				debit_consigne_slm = DEBIT_CIBLE_SLM; /// slm
			#if			USE_PID_VENTILATION_Verreur_accel		==		1
			float				debit_from_error_slm = 0; /// provenant essentiellement de la phase d'acceleration qu'il va falloir compenser en augmentation de dit (Ti restant fixed)
			float				Volume_erreur_phase_accel = 0;
			uint32_t			Remember_duree_phase_ACCEL = 0;
			#endif
			
			/// charge un programme de 		TAB_volume_slm_calib[ index_segt ] .CALIB_result_volume_segment 		par defaut, ok a vide, la PID fera le reste
			if( load_default_calib_from_debit( debit_consigne_slm ) == false ) hardfault_CRASH_ME();				 /// gasp ;)
			/// ///////////////////////////////////////////////////////////////////////////////////////
			/// ///////////////////////////////////////////////////////////////////////////////////////
					
					
			#if	1 /// Algo Adrien PID
					
				/// on repasse tout a zero avant de commencer
				for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
					// TAB_volume_slm_calib[ index_segt ] .CALIB_nbr_echant 								= 0;
					// TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US 							= 0;
					// TAB_volume_slm_calib[ index_segt ] .CALIB_result_volume_segment 			= 0;
					TAB_volume_slm_calib[ index_segt ] .CALIB_US_avt_motor_full_speed 		= 0;
					
					#if			USE_PID_ON_AUTO_CALIB		==		1
						TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit 						= 0.0f;
						// TAB_volume_slm_calib[ index_segt ] .PID_Comp_Volume_Debit				= 0.0f;
						// TAB_volume_slm_calib[ index_segt ] .PID_p_factor 									= 0.0f;
						TAB_volume_slm_calib[ index_segt ] .PID_i_factor 									= 0.0f;
						TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt 								= false;
						TAB_volume_slm_calib[ index_segt ] .max_error_checked						= false;
						
					#endif
				}
				
				bool			is_first_guess_from_abaques 						= true;
				uint32_t		index_premier_segment_FULL_SPEED 	= 0;
				uint32_t		NBR_de_cycles_before_auto_crash_test 	= 0; /// 0 pr desactiver le test crash
				
				
				while ( ( events & BREATHING_RUN_FLAG ) != 0 ){ /// CONTROLE GENERAL IHM
					
					
					if( NBR_de_cycles_before_auto_crash_test > 0 ){
						if( --NBR_de_cycles_before_auto_crash_test == 0 ){
							hardfault_CRASH_ME();				 /// gasp ;)
						}
					}
						
					
					// enter_state(Insufflation);
					// TODO: Take into account the time to compute adaptation for the FR calculation ??!!??

					// Get current controller settings
					uint32_t T        		= get_setting_T_ms      (); /// CONTROLE GENERAL IHM
					float    VT       		= get_setting_VT_mL     ();
					float    VM       		= get_setting_Vmax_Lpm  ();
					float    Pmax     	= get_setting_Pmax_cmH2O();
					uint32_t Tplat    	= get_setting_Tplat_ms  ();

					float VTi=0.;
					float VTe=0.;
					
					// Compute adaptation based on current settings and previous collected data if any.
					uint32_t Ti = T*1/3;  // TODO: Check how to calculate Tinsuflation
					
					static uint8_t	compteur = 0;
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
							
					float				duree_ACCEL = 0;
					float				duree_FULL_SPEED = 0;
					
							
					/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					///
					/// on prepare les corrections PID 
					
					#define			LIMITE_CYCLES_INSPI_TEST_DEBUG			0 /// 0 pr boucle infinie
					
					/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					///
					/// calculs PID
						
					if( index_TAB_dp_raw_temps_moteur > 0 ){ /// on a des datas dispo
							
							#if				DEBUG_PRINTF_CYCLE_PID			==			1 /// ---DEBUT sampling data
							brth_printf( "---DEBUT sampling data %i\n\r", index_TAB_dp_raw_temps_moteur /  DIVISEUR_NBR_VALEURS_SAMPLED );
							
								uint32_t			maintenant = get_time_ms() ;
								
								for( int	index_TAB = 0; index_TAB < index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED; index_TAB++ ){
									brth_printf( "%i \t\t%i\t%u\t%u\n\r", 
																											// index_TAB, 
																											// TAB_dp_raw_temps_moteur[ index_TAB ]
																											(int)( (float)compute_corrected_flow( TAB_dp_raw_temps_moteur[ index_TAB ].dp_raw ) ),
																											(int)( (float)maintenant - (float)TAB_dp_raw_temps_moteur[ index_TAB ].timecode_sample_MS ),
																											TAB_dp_raw_temps_moteur[ index_TAB ].timecode_sample_MS
																										);
								}
								
							#endif
							
							/// Nouvel algo PID
							/// Nouvel algo PID
							/// Nouvel algo PID
							/// Nouvel algo PID
							// bool			MODE_Panique = false;
							uint32_t		index_step_moteur_en_cours = 0; /// pr analyse ds 	_motor_steps_us[ ]	
							uint32_t		TIMER_US_debut_segment 	= TIMER_debut_sampling_temps_moteur * 1000;
							uint32_t		TIMER_US_fin_segment 		= TIMER_US_debut_segment + TAB_volume_slm_calib[ 0 ] .CALIB_duree_sgt_US;
							
							/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
							/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
							/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
							for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){ /// ds 		TAB_volume_slm_calib
							
								/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								///
								/// ETAPE 1 : on recupere l'ERREUR sur le debit comme future base de la PID
								
								/// il faut faire un time slicing car on a un temps de reponse entre la commande moteur et son effet sur le débits
								/// todo reecrire en prenant en compte le tableau de debit du debut à la fin du temps moteur : beaucoup + precis
								
								/// ON FAIT LA MOYENNE : 	ATTENTION  INSTABLE CAR FREQUENCE ECHANTILLONAGE CAPTEUR PAS FIXE...
								float		Debit_slm_segment = Segment_error_time_sliced(
																																		/// INPUT : TAB_dp_raw_temps_moteur
																																		(int)index_segt,
																																		(uint32_t)TIMER_US_debut_segment, 							/// INPUT : TIMER_US_debut_segment
																																		(uint32_t)TIMER_US_fin_segment, 									//// INPUT : TIMER_US_fin_segment
																																		(uint32_t)TIME_SLICING_ERROR_debut, 					/// INPUT : TIME_SLICING_ERROR_debut
																																		(uint32_t)TIME_SLICING_ERROR_fin, 							/// INPUT : TIME_SLICING_ERROR_fin
																																		/// important que ces 2 variables soient JUSTES et mise à jour !!!
																																		(uint64_t)TIMER_fin_accel_moteur_ms, 						// TIMER_fin_accel_moteur_ms = 1;
																																		(uint64_t)TIMER_fin_sampling_temps_moteur				// TIMER_fin_sampling_temps_moteur = 0;
																																);

								
								
								/// MAJ 		ERREUR 
								/// MAJ 		ERREUR 
								/// MAJ 		ERREUR 
								TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit = 
																																				Debit_slm_segment
																																			-	(
																																					debit_consigne_slm
																																				#if			USE_PID_VENTILATION_Verreur_accel		==		1
																																				+	debit_from_error_slm
																																				#endif
																																				)
																																		;
								/// MAJ 		ERREUR 
								/// MAJ 		ERREUR 
								/// MAJ 		ERREUR 
																																		
								// brth_printf( "---deb mes %i Err %i\n\r", (int)( Debit_slm * 1000 ), (int)( TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit * 1000 ) );
								
								
								/// on recupere l'erreur du SEGMENT
								float		ERREUR_debit_segment 		= TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit;
								float		DUREE_segment_US			= TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US;
								
								/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								///
								/// ETAPE 2 : on check l'état de la commande MOTEUR		+		on applique la PID sur l'erreur du segment
								
								/// on va tout recalculer :
								TAB_volume_slm_calib[ index_segt ] .is_vitesse_max_SEGMENT						= false;
								TAB_volume_slm_calib[ index_segt ] .is_accel_max_SEGMENT 						= false;
								TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt											= true;
								TAB_volume_slm_calib[ index_segt ] .max_error_checked									= false;
								/// on commence par récupérer l'erreur sur le segment
								if( 
										0 /// WIP
										// ERREUR_debit_segment > 5000.0f / 1000
								){
									/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
									/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
									/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
									///
									///				MODE PANIQUE ;)
									///
									/// que le segment soit tunable ou pas ne change rien : on a OVERSHOOT !!!!!
									
									/// on repart de zero : mais il fadrait deplacer ca : WIP
									if( load_default_calib_from_debit( debit_consigne_slm ) == false ) hardfault_CRASH_ME();				 /// gasp ;)
									
									/// ALARME : debranchement patient
								
									/// MAJ indispensable 
									/// MAJ indispensable 
									index_step_moteur_en_cours += NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE; /// pr analyse sgt suivant
									/// MAJ indispensable 
									/// MAJ indispensable 
									
									TAB_volume_slm_calib[ index_segt ] .MODE_Panique = true;
									
								} else if(
										0 /// WIP : marche pas trop mal, mais peut etre deplacer ca en mode global : pas segment par segment
										// ERREUR_debit_segment > 1500.0f / 1000
								){
									TAB_volume_slm_calib[ index_segt ] .PID_i_factor 					*= 	FACTEUR_P_PID_CONSIGNE_DEBIT_ERR_POSITIVE; 									/// et vlan
									// TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US		*=	1.0f  + ERREUR_debit_segment / ( debit_consigne_slm + debit_consigne_slm );	/// et vlan
									
									brth_printf( "---SGT %i : OVERSHOT !!! ERR %i\n\r", index_segt, (int)( ERREUR_debit_segment * 1000 ) );
									
									/// MAJ indispensable 
									/// MAJ indispensable 
									index_step_moteur_en_cours += NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE; /// pr analyse sgt suivant
									/// MAJ indispensable 
									/// MAJ indispensable 
									
									TAB_volume_slm_calib[ index_segt ] .MODE_Panique = true;
								}
								/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								///
								else { /// l'erreur est negative			PAS DE PANIQUE
								
									/// on verifie les commandes moteurs du cycle precedant		--->  		on passe		is_vitesse_max_SEGMENT		et			is_accel_max_SEGMENT
									if(
											DUREE_segment_US		< NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE * DUREE_STEP_MIN_MOTOR /// cad SEGMENT a vitesse max
									){
										brth_printf( "---ERREUR !!! duree %u < duree max %u\n\r", DUREE_segment_US, NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE * DUREE_STEP_MIN_MOTOR  );
										/// ALARME et PANIQUE : WIP
										
										TAB_volume_slm_calib[ index_segt ] .MODE_Panique = true;
										TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt = false;
										TAB_volume_slm_calib[ index_segt ] .is_vitesse_max_SEGMENT = true;
										
										/// MAJ indispensable 
										/// MAJ indispensable 
										index_step_moteur_en_cours += NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE; /// pr analyse sgt suivant
										/// MAJ indispensable 
										/// MAJ indispensable 
										
									} else if(
											DUREE_segment_US		== NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE * DUREE_STEP_MIN_MOTOR /// cad SEGMENT a vitesse max
									){
										/// SEGMENT non tunable
										TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt = false;
										TAB_volume_slm_calib[ index_segt ] .is_vitesse_max_SEGMENT = true;
										
										/// MAJ indispensable 
										/// MAJ indispensable 
										index_step_moteur_en_cours += NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE; /// pr analyse sgt suivant
										/// MAJ indispensable 
										/// MAJ indispensable 
										
									} else {
										
										/// on cherche si on a atteinds ACCEL_max
										TAB_volume_slm_calib[ index_segt ] .is_accel_max_SEGMENT = true; /// par defaut
										
										for( int		index_STEP_moteur = 0; index_STEP_moteur < NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE; index_STEP_moteur++ ){
											// index_step_moteur_en_cours += NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE;
											/// MAJ indispensable 
											/// MAJ indispensable 
											index_step_moteur_en_cours++;
											/// MAJ indispensable 
											/// MAJ indispensable 
											
											int64_t			duree_step_NM1 	= _motor_steps_us[ index_step_moteur_en_cours - 1 ] ;
											int64_t			duree_step_N 		= _motor_steps_us[ index_step_moteur_en_cours      ] ;
											if(
													(int)duree_step_NM1 - (int)duree_step_N < -MAX_ACCEL_MOTEUR_NEGATIF
												||	(int)duree_step_NM1 - (int)duree_step_N > MAX_ACCEL_MOTEUR_POSITIF
											){
												TAB_volume_slm_calib[ index_segt ] .is_accel_max_SEGMENT = false;
											}
											
											/// no break allowed !!!
											
										} /// fin du for NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE
										
									}
									/// else : TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt == true implicite
						
								} /// fin de l'erreur is negative			PAS DE PANIQUE
								
								/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								
								#if				DEBUG_PRINTF_CYCLE_PID			==			1 /// ---DEBUT sampling data
								/// RESUME : 
								brth_printf( "---SGT %i  : Panik %u || TUNABLE %u : %u %u || Dur_step %u || ERR %i || PID %i\n\r", 
																														index_segt, 
																														TAB_volume_slm_calib[ index_segt ] .MODE_Panique,
																														TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt,
																														TAB_volume_slm_calib[ index_segt ] .is_vitesse_max_SEGMENT,
																														TAB_volume_slm_calib[ index_segt ] .is_accel_max_SEGMENT,
																														
																														TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US,
																														(int)(  ERREUR_debit_segment * 1000 ), /// TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit
																														// Debit_moyen_dp_raw_SEGMENT,
																														(int)( TAB_volume_slm_calib[ index_segt ] .PID_i_factor * 1000  * 1000 )
																														
																													);
																													
								#endif
								
								/// MAJ
								TIMER_US_debut_segment 	+= TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US;
								TIMER_US_fin_segment 		+= TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US;
								
							} /// fin du for		NBR_SEGMENTS_CALIBRATION
							/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
							/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
							/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
							///
							/// ETAPE 2 : analyse linearite erreur : WIP une fois que periodicite capteurs  : a ameliorer
							
							/// on calcule un facteur P sur les GROSSES ERREURS UNIQUEMENT, pour faire converger + rapidement : sera accumuled ds		PID_i_factor 					WIP
							///		-	la moyenne de l'erreur
							///		-	le max de l'erreur lorsque celui ci est UNDERSHOOTED

							float			composante_PID_P_factor_SEGMENT_complet = 0.0f;
							float			FACTEUR_linearite_plateau_inspi;
						
							float			MOYENNE_facteur_I_PID 	= 0;
							float			MOYENNE_erreur 				= 0;
							float			MAX_erreur 							= -123456789;
							float			MIN_erreur 							= 123456789;
							int				denom 									= 0;
							
							for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){ /// ds 		TAB_volume_slm_calib
								
								 #if			NIVEAU_VERBOSE_DEBUG		>=				2
									#define			NBR_PAR_LIGNE_DEBUG			15
									brth_printf( "%i\t", (int)( TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit * 1000 ) );
									if( ( index_segt % NBR_PAR_LIGNE_DEBUG ) == NBR_PAR_LIGNE_DEBUG-1 ) brth_printf("\n\r" );
								#endif
								
								if(
									TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == false /// on ne fait la moyenne que de ce qui n'est pas phase accel
								){
									
									MOYENNE_erreur 				+= TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit;
									MOYENNE_facteur_I_PID 	+= TAB_volume_slm_calib[ index_segt ] .PID_i_factor;
									denom++;
									
									if(
											TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit > MAX_erreur
									){
										MAX_erreur = TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit;
									}
									
									if(
											TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit < MIN_erreur
									){
										MIN_erreur = TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit;
									}
									
								
								}
							
							} /// fin du for
							MOYENNE_erreur 				/= denom;
							MOYENNE_facteur_I_PID 	/= denom;
							

							FACTEUR_linearite_plateau_inspi = fabs( MOYENNE_erreur - MAX_erreur ) / PROPORTION_FACTEUR_LINEARITE_PLATEAU; /// ajustement fin		PID_i_factor
							if( FACTEUR_linearite_plateau_inspi < MINI_FACTEUR_LINEARITE_PLATEAU ) 	FACTEUR_linearite_plateau_inspi = MINI_FACTEUR_LINEARITE_PLATEAU;
							if( FACTEUR_linearite_plateau_inspi > MAXI_FACTEUR_LINEARITE_PLATEAU ) 	FACTEUR_linearite_plateau_inspi = MAXI_FACTEUR_LINEARITE_PLATEAU;
							/// OUTPUT : MOYENNE_erreur
							/// OUTPUT : MAX_erreur
							/// OUTPUT : FACTEUR_linearite_plateau_inspi
							
							
							/// on applique tout ca :
							/// on applique tout ca :
							/// on applique tout ca :
							for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){ /// ds 		TAB_volume_slm_calib
							
								if( TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt == false ){
									
								} else { /// cad TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt == true
									/// on calcule la PID
									
									TAB_volume_slm_calib[ index_segt ] .PID_i_factor  	+= 	
																																		// 1.0f
																																		(
																																			/// todo : chercher si depedant de Paw :
																																			// ERREUR_debit_Tn 		* PCT_ERR_DEBIT_from_N
																																		// +	ERREUR_debit_Tnp1	* PCT_ERR_DEBIT_from_Np1
																																		// +	ERREUR_debit_Tnp2 	* PCT_ERR_DEBIT_from_Np2
																																		TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit
																																			// /	debit_consigne_slm
																																		)
																																	* 	FACTEUR_I_PID_CONSIGNE_DEBIT_ERR_NEGATIVE
																																	
																																	* 	FACTEUR_linearite_plateau_inspi /// va diminuer l'impact de l'integrale a mesure que le plateau apparait
																																; 
									
									/// on ajoute un gros facteur P pour accelerer la convergence
									/// on ajoute un gros facteur P pour accelerer la convergence		WIP		--->			le facteur P est + pertinent applique en global comme ca que par segment	:		marche pas trop mal, a tuner
									/// on ajoute un gros facteur P pour accelerer la convergence
									#if	0
									if(
												FACTEUR_P_PID_CONSIGNE_DEBIT > 0.0f
										&&	MAX_erreur 	< 		- LIMITE_BASSE_DESACTIVATION_COMPOSANTE_P_PID
									){
										composante_PID_P_factor_SEGMENT_complet = ( FACTEUR_P_PID_CONSIGNE_DEBIT * MAX_erreur );
										
										/// on ajoute directement ça à la composante Integrale
										// for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
											TAB_volume_slm_calib[ index_segt ] .PID_i_factor += composante_PID_P_factor_SEGMENT_complet;
										// }
										
									}
									#endif
									/// on ajoute un gros facteur P pour accelerer la convergence
									/// on ajoute un gros facteur P pour accelerer la convergence
									/// on ajoute un gros facteur P pour accelerer la convergence
							
									
								}
								
							/// on applique tout ca :
							/// on applique tout ca :
							/// on applique tout ca :
							
						}
						/// Nouvel algo PID
						/// Nouvel algo PID
						/// Nouvel algo PID
						/// Nouvel algo PID
						
						#if			NIVEAU_VERBOSE_DEBUG		>=				1
						brth_printf( "---- ERR  \t%i\t%i\tLin  ACCEL %ims \tif max  \t%i < %i  \t-> PID P %i  +  \tmoy PID I :  \t%i\n\r", 
																													(int)( MOYENNE_erreur * 1000 ),
																													(int)( FACTEUR_linearite_plateau_inspi * 1000 ),
																													(int)timecode_ms_full_speed,
																													
																													(int)( MAX_erreur * 1000 ),
																													(int)( LIMITE_BASSE_DESACTIVATION_COMPOSANTE_P_PID * 1000 ),
																													(int)( composante_PID_P_factor_SEGMENT_complet * 1000 ),
																													(int)( MOYENNE_facteur_I_PID * 1000 * 1000 )
																													
																											);
						#endif
						
						/// on limite les facteurs de PID PID_i_factor : pas plus de 20000 !!!
						/// on limite les facteurs de PID PID_i_factor : pas plus de 20000 !!!		WIP	:		retuner avec nouveaux facteurs define : imperatif securite patient : tube bouched...
						for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
							
							if( 
										TAB_volume_slm_calib[ index_segt ] .PID_i_factor > MAX_INTEGRALE_ACCUMULEE_PID
								// &&	TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == false
							){
								if(
										// TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == true
										index_segt < index_premier_segment_FULL_SPEED
								){
									TAB_volume_slm_calib[ index_segt ] .PID_i_factor = 0; /// par defaut, on garde ceinture brettelles
								} else {
									TAB_volume_slm_calib[ index_segt ] .PID_i_factor = MAX_INTEGRALE_ACCUMULEE_PID;
									brth_printf( "---limitation++ %i : %i\n\r", index_segt, (int)( TAB_volume_slm_calib[ index_segt ] .PID_i_factor * 1000 ) );
								}
								
							}
							else 
							if( 
										TAB_volume_slm_calib[ index_segt ] .PID_i_factor < -MAX_INTEGRALE_ACCUMULEE_PID
								// &&	TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == false
							){
								
								if(
										// TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == true
										index_segt < index_premier_segment_FULL_SPEED
								){
									TAB_volume_slm_calib[ index_segt ] .PID_i_factor = 0; /// par defaut, on garde ceinture brettelles
								} else {
									TAB_volume_slm_calib[ index_segt ] .PID_i_factor = -MAX_INTEGRALE_ACCUMULEE_PID;
									#if			NIVEAU_VERBOSE_DEBUG		>=				2
										brth_printf( "---limitation-- %i : %i\n\r", index_segt, (int)( TAB_volume_slm_calib[ index_segt ] .PID_i_factor * 1000 ) );
									#endif
								}
								
							}
						}
						/// on limite les facteurs de PID PID_i_factor : pas plus de 20000 !!!
						/// on limite les facteurs de PID PID_i_factor : pas plus de 20000 !!!
						
						
						/// LISSAGE param PID
						/// LISSAGE param PID
						/// LISSAGE param PID
						if( ENABLE_LISSAGE_COURBE_FACT_INTEGRALE  == 1 ){ /// WIP : pas terrible : ne pas utiliser, pour memoire : a virer
							
							uint8_t		TAB_tri_index_error[ NBR_SEGMENTS_CALIBRATION ];
							
							for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){ /// ds 		TAB_volume_slm_calib
								float		MAX_erreur = 0.0f;
								
								/// on cherche l'err max suivante :
								int		index_SEGMENT_max_error_en_cours = -1;
								for( int index_segt_look_for_err= 0; index_segt_look_for_err < NBR_SEGMENTS_CALIBRATION; index_segt_look_for_err++ ){ /// ds 		TAB_volume_slm_calib
									
									if(
												fabs( TAB_volume_slm_calib[ index_segt_look_for_err ] .PID_ERREUR_Debit	)			> 		MAX_erreur
										&&	TAB_volume_slm_calib[ index_segt_look_for_err ] .max_error_checked 							== 	false /// pas deja retenu : WIP : foireux, a virer
									){
										index_SEGMENT_max_error_en_cours = index_segt_look_for_err;
										MAX_erreur = fabs( TAB_volume_slm_calib[ index_segt_look_for_err ] .PID_ERREUR_Debit );
									}
								}
							
								if( index_SEGMENT_max_error_en_cours == -1 ) break; /// on a fini
								/// else
								
								// brth_printf( "-------------------------------------------- %i Max error %i\n\r", index_segt, (int)( TAB_volume_slm_calib[ index_SEGMENT_max_error_en_cours ] .PID_ERREUR_Debit * 1000 ) );
								
								/// on ventile PID_i_factor dans les segments adjacents
								if( index_SEGMENT_max_error_en_cours > 0 ){ /// gestion nm1
									TAB_volume_slm_calib[ index_SEGMENT_max_error_en_cours - 1 ] .PID_i_factor = 
																																														TAB_volume_slm_calib[ index_SEGMENT_max_error_en_cours - 1 ] .PID_i_factor * ( 1.0f - FACTEUR_LISSAGE_NM1 )
																																													+	TAB_volume_slm_calib[ index_SEGMENT_max_error_en_cours      ] .PID_i_factor * FACTEUR_LISSAGE_NM1
																																												;
								}
								
								if( index_SEGMENT_max_error_en_cours < NBR_SEGMENTS_CALIBRATION - 1 ){ /// gestion nm1
									TAB_volume_slm_calib[ index_SEGMENT_max_error_en_cours + 1 ] .PID_i_factor = 
																																														TAB_volume_slm_calib[ index_SEGMENT_max_error_en_cours + 1 ] .PID_i_factor * ( 1.0f - FACTEUR_LISSAGE_NP1 )
																																													+	TAB_volume_slm_calib[ index_SEGMENT_max_error_en_cours      ] .PID_i_factor * FACTEUR_LISSAGE_NP1
																																												;
								}
							
								/// MAJ
								TAB_volume_slm_calib[ index_SEGMENT_max_error_en_cours ] .max_error_checked	= true;
								
							} /// fin du for
							
						}
						
						/// LISSAGE param PID
						/// LISSAGE param PID
						/// LISSAGE param PID
						
						#if				DEBUG_PRINTF_CYCLE_PID			==			1 /// ---DEBUT sampling data
						brth_printf( "---FIN   sampling data %i\n\r", index_TAB_dp_raw_temps_moteur /  DIVISEUR_NBR_VALEURS_SAMPLED );
						#endif
					}
					else {
						
						brth_printf( "-- No data for PID %i on %i\n\r", index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED, NBR_VALEURS_TAB_debits_temps_moteur * DIVISEUR_NBR_VALEURS_SAMPLED );
						/// Alarm(); /// WIP 
					}
						

							
					
							
					#if	USE_PID_VENTILATION_Verreur_accel		==		1 /// WIP
					for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
						
						wait_ms( 1 );
						
								float		Volume_erreur_segment = 0;
								
								if(
									TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt == true
								){
									if( TAB_volume_slm_calib[ index_segt ] .CALIB_US_avt_motor_full_speed == 0 ){
										/// phase FULL SPEED
										duree_FULL_SPEED		+= TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US;
									} else {
										/// phase ACCEL
										duree_ACCEL 				+= TAB_volume_slm_calib[ index_segt ] .CALIB_US_avt_motor_full_speed;
										duree_FULL_SPEED		+= TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US;
										
										/// attention c'est faux ça :
										Volume_erreur_segment = 		TAB_volume_slm_calib[ index_segt ] .CALIB_result_volume_segment 
																						* 	( //// proportion du volume en erreur
																									(float)TAB_volume_slm_calib[ index_segt ] .CALIB_US_avt_motor_full_speed
																								/ 	TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US
																							);
									}
									
								} else {
									duree_FULL_SPEED 	+= TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US;
								}
								Volume_erreur_phase_accel += Volume_erreur_segment;
					}
					
					/// MAJ : WIP
					debit_from_error_slm = Volume_erreur_phase_accel / duree_FULL_SPEED;
					
					if( 
								Remember_duree_phase_ACCEL > 0
						// &&	Volume_erreur_phase_accel != 0
					){
						/// on va considerer l'erreur faite pendant l'ACCEL
						debit_from_error_slm += 
																	(	
																		Volume_erreur_phase_accel
																	) /// debit
																	
																	* FACTEUR_I_PID_VENTILATION_ERREUR_PHASE_ACCEL
																	; /// facteur INTEGRAL PID
						
					}
					#else
					wait_ms( 4 ); /// on laisse respirer le CPU
					#endif
					
							
					/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
							
				#endif /// USE_PID_ON_AUTO_CALIB
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					uint32_t	index_pas_stepper = 0;
					uint32_t	duree_TOTAL_theorique_US = 0;
					int32_t	Last_duree_step_moteur_en_cours = DUREE_ARRET_MOTEUR_PR_ACCEL; /// pr gestion accel uniqt
					uint32_t	timecode_depuis_demarrage_moteur = 0;
					bool		is_still_ACCEL_phase = true; /// phase ACCEL au demarrage, avant FULL SPEED
					uint32_t 	DUREE_Totale_cycle = 0;
					
						
											
					for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){ /// pour chaque segment, on calcule la duree pour assurer le debit consigne (CALIB_result_volume_segment -> debit)
						
						wait_ms( 1 ); /// on laisse du temps CPU pour la COM IHM
						

						float			VOLUME_segment_corriged = /// PID	I
																				TAB_volume_slm_calib[ index_segt ].CALIB_result_volume_segment /// from		TAB_volume_slm_calib[ index_segt ].CALIB_result_volume_segment 		cad		const_calibs.h
																				
																				#if		USE_PID_ON_AUTO_CALIB		==		1
																				
																					#if		ENABLE_PID_REGULATION		==		1
																					+ (
																								(
																									TAB_volume_slm_calib[ index_segt ] .PID_i_factor 
																								)
																					)
																					#endif
																					
																				#endif
																		;
						/// vol / debit = US
						uint32_t	duree_segment_US = (uint32_t)( 
																										round( 
																														(float)(
																																				(
																																					VOLUME_segment_corriged /// PID
																																				)
																																		/ 
																																				(
																																						debit_consigne_slm 
																																						
																																					#if			USE_PID_VENTILATION_Verreur_accel		==		1
																																					+	debit_from_error_slm /// todo : on accumule le VOL d'erreur pdt accel : A VENTILER SUR debit_consigne_slm
																																					#endif
																																					
																																				) * 60 * 1000 * 1000
																																)
																																

																													)
																							);
																							
						#if		DEBUG_PRINTF_PR_CALIB_A_VIDE		==		1
							// brth_printf( "%i %i %i\n\r", index_segt, duree_segment_US, (int)( debit_consigne_slm ) );
							brth_printf( "%i\n\r", (int)( VOLUME_corriged *  1000 * 1000 ) ); /// pratique pr creer les const_calibs.h		WIP	: a refaire avec nouveaux mors
						#endif
						
						uint32_t	duree_step_ideale_US = round( (float)duree_segment_US / NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE ); /// linearised : todo : faire varier le nombre de pas POUR EVITER DE TOUT DEPLACER
						
						
						/// on s'apprete a recalculer :
						TAB_volume_slm_calib[ index_segt ].CALIB_US_avt_motor_full_speed  = 0;
						if( is_first_guess_from_abaques == true ){ /// uniquement au premier tour : WIP : A VERIFIER, attention paleocode !!!!
							TAB_volume_slm_calib[ index_segt ].PID_tunable_sgt  = false;
						}
						
						/// on recalcule le futur programme moteur en tenant compte des accelerations et vit max : WIP : a passer en fonction
						/// on recalcule le futur programme moteur en tenant compte des accelerations et vit max : WIP : a passer en fonction
						/// on recalcule le futur programme moteur en tenant compte des accelerations et vit max : WIP : a passer en fonction
						
						/// INPUT : _motor_steps_us
						/// INPUT : duree_step_ideale_US
						/// INPUT : Last_duree_step_moteur_en_cours en ref a MAJ
						/// INPUT : timecode_depuis_demarrage_moteur en ref a MAJ
						/// INPUT : duree_segment_US en ref a MAJ
						
						{ /// a passer en fonction : WIP : pas mal de globales a proprifier			commande_steps_moteur_from_duree_segment_US_ideal()
							duree_segment_US = 0;
							uint32_t	duree_step_US;
							for( int nbr_pas_segment = 0; nbr_pas_segment < NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE; nbr_pas_segment++ ){
								
									/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
									/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
									///
									/// on consitute la MAP DMA de controle moteur :
									/// gestion acceleration
									
										int32_t 		Accel_moteur = (int32_t)Last_duree_step_moteur_en_cours - duree_step_ideale_US;
										bool				is_full_speed = true;
										
										if( duree_step_ideale_US < DUREE_STEP_MIN_MOTOR ){
											/// vitesse impossible
											duree_step_ideale_US = DUREE_STEP_MIN_MOTOR;
											is_full_speed = false; /// flag ds struct ? WIP
										}
										
										
										
										if(
														-Accel_moteur > MAX_ACCEL_MOTEUR_NEGATIF
										){
											/// phase d'acceleration intiale
											duree_step_US = Last_duree_step_moteur_en_cours + MAX_ACCEL_MOTEUR_NEGATIF;
											TAB_volume_slm_calib[ index_segt ].CALIB_US_avt_motor_full_speed += duree_step_US;
											is_full_speed = false; /// flag ds struct ? WIP
										} else if(
														Accel_moteur > MAX_ACCEL_MOTEUR_POSITIF
										){
											/// phase d'acceleration intiale
											duree_step_US = Last_duree_step_moteur_en_cours - MAX_ACCEL_MOTEUR_POSITIF;
											TAB_volume_slm_calib[ index_segt ].CALIB_US_avt_motor_full_speed += duree_step_US;
											is_full_speed = false; /// flag ds struct ? WIP
										} else  {
											/// phase FULL SPEED
											duree_step_US = duree_step_ideale_US;
											
											if( is_first_guess_from_abaques == true ){
												TAB_volume_slm_calib[ index_segt ].PID_tunable_sgt = true; /// par defaut, tous les autres segment restent tunables par la PID : WIP
											}
										}
										
										if( 
													is_still_ACCEL_phase == true
											&&	is_full_speed == true
										){ /// on fixe le point de fin d'accel
											// brth_printf( "--------------------->  timecode_ms_full_speed %u SET\n\r", timecode_ms_full_speed / 1000 );
											timecode_ms_full_speed = timecode_depuis_demarrage_moteur / 1000; /// ( timecode_us_next_echant + TAB_volume_slm_calib[ index_segt ] .CALIB_US_avt_motor_full_speed ) / 1000; 
											if( is_first_guess_from_abaques ){
												index_premier_segment_FULL_SPEED = index_segt; /// fixed une fois pour toute : le reste est optimisable : ca se discute e fixer ca au premier cycle... WIP
											}
											TAB_volume_slm_calib[ index_segt ].PID_tunable_sgt = true;
											is_still_ACCEL_phase = false;
										}
										
										Last_duree_step_moteur_en_cours = duree_step_US;
										timecode_depuis_demarrage_moteur += duree_step_US;
									
									/// on applique immediatement
									if( TAB_volume_slm_calib[ index_segt ].CALIB_nbr_echant != 0 ){ /// bug dernier segt... WIP : paleocode crado ??? me rappelle plus sorry, a virer / tester
										
										_motor_steps_us[ index_pas_stepper++ ] = duree_step_US;
										duree_segment_US += duree_step_US;
										
									}
								
								
							} /// fin du for nbr_pas_segment
							/// OUTPUT : _motor_steps_us
							/// OUTPUT : duree_segment_US, timecode_depuis_demarrage_moteur
							/// OUTPUT : passe les flags accel max vit max...
							/// OUTPUT fonction : a creer WIP :
							TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US = duree_segment_US; /// RECALCULED SELON CONSIGNE
						} /// a passer en fonction
						/// on recalcule le futur programme moteur en tenant compte des accelerations et vit max : WIP : a passer en fonction
						/// on recalcule le futur programme moteur en tenant compte des accelerations et vit max : WIP : a passer en fonction
						/// on recalcule le futur programme moteur en tenant compte des accelerations et vit max : WIP : a passer en fonction
						
						DUREE_Totale_cycle 							+= duree_segment_US; /// WIP : doublon paleocode a virer ???
						duree_TOTAL_theorique_US 				+= (uint32_t)duree_segment_US; /// WIP : doublon paleocode a virer ???
						
						#if	0
						brth_printf( "--sgt %i : step ideale %u duree_segment_US %u %u %u %u\n\r", 
																																		index_segt, 
																																		duree_step_ideale_US,
																																		duree_segment_US, 
																																		duree_TOTAL_theorique_US, 
																																		duree_step_US, 
																																		index_pas_stepper 
																																);
						#endif
					} /// fin du for index_segt
					
					is_first_guess_from_abaques = false;
					
					
					if( 
							is_first_guess_from_abaques == true /// WIP : Vilain paleocode: je le laisse car pas d'acces a un recovid pour tester : mais a virer !!!!! pas fier ;)
						&&	timecode_ms_full_speed == 0
					){
						brth_printf( "--------------------->  timecode_ms_full_speed %u SET baaaad !!!\n\r", duree_TOTAL_theorique_US / 1000);
						timecode_ms_full_speed = duree_TOTAL_theorique_US / 1000;
						index_premier_segment_FULL_SPEED = NBR_SEGMENTS_CALIBRATION;
					}
						
					
					
					#if	0 /// desespoir only
						for( int i_steps = 0; i_steps < index_pas_stepper; i_steps++ ){
						// for( int i_steps = 0; i_steps < 200; i_steps++ ){
							
							brth_printf( "-- %i duree step %u \n\r", 
																								i_steps,
																								_motor_steps_us[ i_steps ]
																								// NBR_SEGMENTS_CALIBRATION,
																								// (int)( round( NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE ) )
																							);
																							
						}
					#endif

					valve_inhale();
					enter_state(Insufflation);
					
					reset_Vol_mL();

						
					
					/// on demarre le sampling en IT
					/// on demarre le sampling en IT ds TAB_dp_raw_temps_moteur
					__disable_irq();
					
						uint32_t	maintenant_sampling = get_time_ms();
						index_TAB_dp_raw_temps_moteur = 0;
						denom_TAB_dp_raw_temps_moteur = 0;
						accumule_TAB_dp_raw_temps_moteur = 0;
						accumule_TAB_TIMECODE_temps_moteur = 0;
						
						TIMER_debut_sampling_temps_moteur = maintenant_sampling;
						TIMER_fin_sampling_temps_moteur 		= maintenant_sampling + duree_TOTAL_theorique_US / 1000 + DEBIT_OVERSAMPLING_TIME;
						is_running_sampling_temps_moteur 		= true; /// on demarre l'acquisition en IT
						TIMER_fin_accel_moteur_ms					= maintenant_sampling + timecode_ms_full_speed;
						
					__enable_irq();
					/// on demarre le sampling en IT
					/// on demarre le sampling en IT
					

					
					/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					
					
					
					#define			ESSAI_PORTAGE_OLD_COD					0
					// _steps = index_pas_stepper;
					#if					ESSAI_PORTAGE_OLD_COD 		==		0  /// bricolage adrien le temps du dev
						
						motor_press( _motor_steps_us, index_pas_stepper ); /// index_pas_stepper en remplacement de _steps);
						
						
					#else
					
						// motor_press(_motor_steps_us, _steps);
						motor_press(_motor_steps_us, index_pas_stepper); /// index_pas_stepper en remplacement de _steps);
						
						#if			NIVEAU_VERBOSE_DEBUG		>=				3
						// brth_print("BRTH: Insuflation\n");      
						#endif
						while(Insufflation == _state ) { /// CONTROLE GENERAL IHM
						  if (Pmax <= read_Paw_cmH2O()) {
							  brth_printf("BRTH: Paw [%ld]> Pmax --> Exhalation\n", (int32_t)(read_Paw_cmH2O()));
							  enter_state(Exhalation);
							  break;
						  } else if (VT <= read_Vol_mL()) {
							  brth_printf("BRTH: vol [%ld]>= VT --> Plateau\n", (int32_t)(read_Vol_mL()));
							  enter_state(Plateau);
							  break;
						  } else 
						  if( Ti <= (get_time_ms() - _cycle_start_ms) ) {
							  brth_printf("BRTH: dt [%lu]>= Ti\n", (get_time_ms() - _cycle_start_ms));
							  enter_state(Plateau);
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
					
					#if	0
						brth_printf( "--- Duree cycle... %u\n\r", TIMER_fin_sampling_temps_moteur - TIMER_debut_sampling_temps_moteur );
					#endif
					
					// uint32_t			maintenant = get_time_ms() ;
					
					
					uint32_t	temps_restant_sur_phase_moteur = get_time_ms() - maintenant_sampling + duree_TOTAL_theorique_US / 1000;
					
					/// USE_PID_ON_AUTO_CALIB		==		0
					/// on laisse faire l'IT pour acquisition ttes les 5*4 = 20 ms
					wait_ms( temps_restant_sur_phase_moteur ); /// duree_TOTAL_theorique_US / 1000 );
					
							
					
					/// retour moteur HOME
					/// retour moteur HOME
					/// retour moteur HOME
					motor_release();
					/// retour moteur HOME
					/// retour moteur HOME
					/// retour moteur HOME
					
					#if			NIVEAU_VERBOSE_DEBUG			>=		1
						if( index_TAB_dp_raw_temps_moteur == 0 ){ /// detection des capteurs i2c plantés...
							brth_printf( "---fin sampling %i < %i\n\r", index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED, NBR_VALEURS_TAB_debits_temps_moteur );
						}
					#endif
					


					/// on attends la fin du sampling DEBIT
					/// on attends la fin du sampling DEBIT
					uint32_t	maintenant = get_time_ms() ;
					while( maintenant < TIMER_fin_sampling_temps_moteur ){
						wait_ms( 1 );
						maintenant = get_time_ms() ;
					}
					
					/// on arrete le sampling en interrupt ds TAB_dp_raw_temps_moteur
					/// on arrete le sampling en interrupt ds TAB_dp_raw_temps_moteur
					if( maintenant >= TIMER_fin_sampling_temps_moteur ){
						// index_TAB_dp_raw_temps_moteur = 0;
						/// impossible de sampler ds l'IT avec ces valeurs :
						// TIMER_debut_sampling_temps_moteur = 1;
						// TIMER_fin_sampling_temps_moteur = 0;
						is_running_sampling_temps_moteur = false;
						
						if(
							index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED >= NBR_VALEURS_TAB_debits_temps_moteur
						){
								brth_printf( "\n\r\n\r\n\r---hardfault_CRASH_ME : fin sampling %i < %i\n\r\n\r\n\r", index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED, NBR_VALEURS_TAB_debits_temps_moteur );
								motor_release();
								wait_ms( 3000 );
								hardfault_CRASH_ME();
						}
						
						
					}
					#else
						
						motor_release();

						/// on arrete le sampling en interrupt ds TAB_dp_raw_temps_moteur
						/// on arrete le sampling en interrupt ds TAB_dp_raw_temps_moteur
						uint32_t	maintenant = get_time_ms() ;
						if( maintenant >= TIMER_fin_sampling_temps_moteur ){
							// index_TAB_dp_raw_temps_moteur = 0;
							/// impossible de sampler ds l'IT avec ces valeurs :
							// TIMER_debut_sampling_temps_moteur = 1;
							// TIMER_fin_sampling_temps_moteur = 0;
							is_running_sampling_temps_moteur = false;
							
							if(
								index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED >= NBR_VALEURS_TAB_debits_temps_moteur
							){
									brth_printf( "\n\r\n\r\n\r---hardfault_CRASH_ME : fin sampling %i < %i\n\r\n\r\n\r", index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED, NBR_VALEURS_TAB_debits_temps_moteur );
									motor_release();
									wait_ms( 3000 );
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
																																													
																																													
																																													(int)( (float)timecode_ms_full_speed/( DUREE_Totale_cycle /1000 ) * 100 ),
																																													timecode_ms_full_speed,
																																													DUREE_Totale_cycle / 1000
																																													
																																											);
							
					
					}
					#endif
					/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					
						#if					ESSAI_PORTAGE_OLD_COD 		==		0 /// bricolage adrien le temps du dev
						
							wait_ms( 750 ); /// 
							
							enter_state(Exhalation);
							valve_exhale();
							
							wait_ms( 1700 ); /// 
							
										#if		LIMITE_CYCLES_INSPI_TEST_DEBUG		>		0
										static	int	compteur = LIMITE_CYCLES_INSPI_TEST_DEBUG;
										if( compteur-- == 0 ){
											while(1);
										}
										#endif
										
							// prgr_moteur( go_home );
							while( is_motor_home() == false );
							
						#else /// WIP : a integrer !!!
							
							while(Plateau == _state) { /// CONTROLE GENERAL IHM
							if (Pmax <= read_Paw_cmH2O()) { 
								brth_print("BRTH: Paw > Pmax --> Exhalation\n");
								enter_state(Exhalation);
							} else if ( is_command_Tpins_expired() && (Tplat <= (get_time_ms() - _state_start_ms)) ) {
								brth_print("BRTH: Tpins expired && (dt > Tplat)\n");
								enter_state(Exhalation);
							}
							wait_ms(10);
							}
							VTi= read_Vol_mL();
							valve_exhale();
							
							
							while(Exhalation == _state) { /// CONTROLE GENERAL IHM
							  if ( T <= (get_time_ms() - _cycle_start_ms )) { 
								  uint32_t t_ms = get_time_ms();


								  EoI_ratio =  (float)(t_ms-_cycle_start_ms)/(_state_start_ms-_cycle_start_ms);
								  FR_pm     = 1./(((float)(t_ms-_cycle_start_ms))/1000/60);

								  xEventGroupSetBits(brthCycleState, BRTH_RESULT_UPDATED);

								  // TODO regulation_pep();
								  enter_state(Finished);
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

		enter_state(Insufflation);

		// TODO: Take into account the time to compute adaptation for the FR calculation ??!!??

		// Get current controller settings
		uint32_t T        		= get_setting_T_ms      ();
		float    VT       		= get_setting_VT_mL     ();
		float    VM       		= get_setting_Vmax_Lpm  ();
		float    Pmax     	= get_setting_Pmax_cmH2O();
		uint32_t Tplat    	= get_setting_Tplat_ms  ();

		float VTi=0.;
		float VTe=0.;

		

		brth_printf("BRTH: T     : %ld\n", T);
		brth_printf("BRTH: VT    : %ld\n", (uint32_t)(VT*100));
		brth_printf("BRTH: VM    : %ld\n", (uint32_t)(VM*100));
		brth_printf("BRTH: Pmax  : %ld\n", (uint32_t)(Pmax*100));
		brth_printf("BRTH: Tplat : %ld\n", Tplat);

	  

      // Compute adaptation based on current settings and previous collected data if any.
      uint32_t Ti = T*1/3;  // TODO: Check how to calculate Tinsuflation
      // brth_printf("BRTH: Ti    : %ld\n", Ti);

      // adaptation(VM, _flow_samples, _flow_samples_count, 0.001*FLOW_SAMPLING_PERIOD_MS, &A_calibrated, &B_calibrated);
      // _flow_samples_count = 0;

	  uint32_t d = 300*MOTOR_CORRECTION_USTEPS;
	  
	  
		compute_constant_motor_steps(d, _steps, _motor_steps_us);
	  
	  
	  //compute_motor_press_christophe(350000, 2000, 65000, 20, 14, 350000, 4000, _steps, _motor_steps_us);
	  brth_printf("T_C = %d Patmo = %d\n", (int) (read_temp_degreeC()*100), (int) (read_Patmo_mbar()*100));

      // Start Inhalation
      valve_inhale();
	  
		motor_press(_motor_steps_us, _steps);

		reset_Vol_mL();
		
		// brth_print("BRTH: Insuflation\n");      
		while(Insufflation == _state ) {
		  if (Pmax <= read_Paw_cmH2O()) {
			  brth_printf("BRTH: Paw [%ld]> Pmax --> Exhalation\n", (int32_t)(read_Paw_cmH2O()));
			  enter_state(Exhalation);
			  break;
		  } else if (VT <= read_Vol_mL()) {
			  brth_printf("BRTH: vol [%ld]>= VT --> Plateau\n", (int32_t)(read_Vol_mL()));
			  enter_state(Plateau);
			  break;
		  } else 
		  if( Ti <= (get_time_ms() - _cycle_start_ms) ) {
			  brth_printf("BRTH: dt [%lu]>= Ti\n", (get_time_ms() - _cycle_start_ms));
			  enter_state(Plateau);
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
	  
      motor_release();
	  
      while(Plateau == _state) {
        if (Pmax <= read_Paw_cmH2O()) { 
            brth_print("BRTH: Paw > Pmax --> Exhalation\n");
            enter_state(Exhalation);
        } else if ( is_command_Tpins_expired() && (Tplat <= (get_time_ms() - _state_start_ms)) ) {
            brth_print("BRTH: Tpins expired && (dt > Tplat)\n");
            enter_state(Exhalation);
        }
        wait_ms(10);
      }
      VTi= read_Vol_mL();
      valve_exhale();
	  
	  
      while(Exhalation == _state) { 
          if ( T <= (get_time_ms() - _cycle_start_ms )) { 
              uint32_t t_ms = get_time_ms();


              EoI_ratio =  (float)(t_ms-_cycle_start_ms)/(_state_start_ms-_cycle_start_ms);
              FR_pm     = 1./(((float)(t_ms-_cycle_start_ms))/1000/60);

              xEventGroupSetBits(brthCycleState, BRTH_RESULT_UPDATED);

              // TODO regulation_pep();
              enter_state(Finished);
          }
        wait_ms(10);
      }
      VTe = VTe_start - read_Vol_mL();





      events= xEventGroupGetBits(ctrlEventFlags);
    } while ( ( events & BREATHING_RUN_FLAG ) != 0 );

    brth_printf("BRTH: Stopping\n");      

    wait_ms(200);
    xEventGroupSetBits(ctrlEventFlags, BREATHING_STOPPED_FLAG);

  }
}











static void enter_state(BreathingState newState) {
  _state= newState;
  _state_start_ms = get_time_ms();
  EventBits_t brthState =0;
  switch(_state) {
    case Insufflation:
      _cycle_start_ms = get_time_ms();
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
  xEventGroupClearBits(brthCycleState, (BRTH_CYCLE_INSUFLATION | BRTH_CYCLE_PLATEAU | BRTH_CYCLE_EXHALATION | BRTH_CYCLE_FINISHED) );
  xEventGroupSetBits(brthCycleState, brthState);
}



// static float compte_motor_step_time(uint32_t step_number, float desired_flow_Ls, float A, float B, float speed) {
// 	float res = (0.8*A*speed*speed*step_number) + B * speed;
// 	res = res / desired_flow_Ls;
// 	if (res * 1000000 < 200) {return 200;}
// 	else {return res * 1000000.;}
// }


// static void adaptation(float target_flow_Lm, float* flow_samples, uint32_t nb_samples, float time_step_sec, float* A, float* B) {
//   if(nb_samples==0) return;
// //************************************************* PID ZONE ********************************************//
// 		// Compute average flow and slope to adjust A_calibrated and B_calibrated
// 		float P_plateau_slope = 0.1;
// 		float P_plateau_mean = 0.2;
// 		uint32_t low;
// 		uint32_t high;
// 		if(get_plateau(flow_samples, nb_samples, time_step_sec, 10, &low, &high) == 0) {
// //			brth_printf("plateau found from sample %lu to %lu\n", low, high);
// 		} else {
// //			brth_printf("plateau NOT found, considering from sample %lu to %lu\n", low, high);
// 		}
// 		float plateau_slope = linear_fit(flow_samples+low, high-low-1, time_step_sec, &plateau_slope);
// 		float plateau_mean = 0;
// 		for(uint32_t i=low; i<high; i++) {
// 			plateau_mean += flow_samples[i];
// 		}
// 		plateau_mean = plateau_mean/(high-low);
// //		brth_printf("plateau slope : %ld\n",(int32_t)(1000*plateau_slope));
// //		brth_printf("plateau mean : %ld\n",(int32_t)(1000*plateau_mean));

// 		float error_mean = plateau_mean - (target_flow_Lm/60.);

// 		*A += plateau_slope * P_plateau_slope;
// 		*B += error_mean * P_plateau_mean;
// //		brth_printf("A = %ld\n", (int32_t)(1000*(*A)));
// //		brth_printf("B = %ld\n", (int32_t)(1000*(*B)));

// }

// // Compute slope of samples fetched with specified time_step
// // Returns 	R  if fit is ok
// // 			-1 if fit is not possible
// static float linear_fit(float* samples, uint32_t samples_len, float time_step_sec, float* slope){
// 	float sumx=0,sumy=0,sumxy=0,sumx2=0, sumy2=0;
// 	for(uint32_t i=0;i<samples_len;i++) {
// 		sumx  = sumx + (float)i * time_step_sec;
// 		sumx2 = sumx2 + (float)i*time_step_sec*(float)i*time_step_sec;
// 		sumy  = sumy + *(samples+i);
// 		sumy2 = sumy2 + (*(samples+i)) * (*(samples+i));
// 		sumxy = sumxy + (float)i*(time_step_sec)* (*(samples+i));
// 	}
// 	float denom = (samples_len * sumx2 - (sumx * sumx));
// 	if(denom == 0.) {
// //		brth_printf("Calibration of A is not possible\n");
// 		return 1;
// 	}
// 	// compute slope a
// 	*slope = (samples_len * sumxy  -  sumx * sumy) / denom;

// 	// compute correlation coefficient
// 	return (sumxy - sumx * sumy / samples_len) / sqrtf((sumx2 - (sumx*sumx)/samples_len) * (sumy2 - (sumy*sumy)/samples_len));
// }

// static int32_t get_plateau(float* samples, uint32_t samples_len, float time_step_sec, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound){
// 	if(windows_number < 2 || windows_number > 30) {return -1;}
// 	float slopes[30];
// 	*high_bound = samples_len-1;
// 	// Compute slope for time windows to detect when signal start increasing/decreasing
// 	for(uint32_t window=0; window<windows_number; window++) {
// 		float r = linear_fit(samples+window*(samples_len/windows_number), samples_len/windows_number, time_step_sec, slopes+window);
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
// 	return 1;
// }





float get_breathing_EoI_ratio()     { return EoI_ratio; }
float get_breathing_FR_pm()         { return FR_pm; }
float get_breathing_VTe_mL()        { return VTe_mL; }
float get_breathing_VMe_Lpm()       { return Pcrete_cmH2O; }
float get_breathing_Pcrete_cmH2O()  { return Pcrete_cmH2O; }
float get_Pplat_cmH20()             { return Pplat_cmH2O; }
float get_PEP_cmH2O()               { return PEP_cmH2O; }


