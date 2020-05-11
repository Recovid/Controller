#ifndef __CONST_CALIB_H__
#define __CONST_CALIB_H__

#include "compute_motor.h"


const	uint16_t		CONST_calib_90_lpm[ 64 ] ;
const	uint16_t		CONST_calib_80_lpm[ 64 ] ;
const	uint16_t		CONST_calib_70_lpm[ 64 ] ;
const	uint16_t		CONST_calib_60_lpm[ 64 ] ;
const	uint16_t		CONST_calib_50_lpm[ 64 ] ;
const	uint16_t		CONST_calib_40_lpm[ 64 ] ;
const	uint16_t		CONST_calib_30_lpm[ 64 ] ;
const	uint16_t		CONST_calib_20_lpm[ 64 ] ;
const	uint16_t		CONST_calib_10_lpm[ 64 ] ;
const	uint16_t		CONST_calib_90_lpm[ 64 ] ;


/// ajouts Adrien
/// ajouts Adrien		WIP : a deplacer
/// ajouts Adrien


			/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la consigne de debit est 			DEBIT_CIBLE_SLM
			/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la consigne de debit est 			DEBIT_CIBLE_SLM
			/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la consigne de debit est 			DEBIT_CIBLE_SLM
			// #define					NBR_PAS_MOTEUR_MAX													MAX_MOTOR_STEPS  /// 4800 //4480
			// #define					NBR_SEGMENTS_CALIBRATION										64
			
			// #define					NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE				( (float)NBR_PAS_MOTEUR_MAX / NBR_SEGMENTS_CALIBRATION )
			
			#define					VITESSE_DEMARRAGE_CALIB											( 2048 ) // us pas moteur * MOTOR_CORRECTION_USTEPS
			
			#define					NBR_ITERATION_CALIB														3
			
			
			#define					MAX_ACCEL_MOTEUR_POSITIF										( 38 ) /// 40
			#define					MAX_ACCEL_MOTEUR_NEGATIF									( 38 ) /// 45
			#define					MAX_ACCEL_MOTEUR_POSITIF_ON_PLATEAU		( 38 ) /// 40
			#define					MAX_ACCEL_MOTEUR_NEGATIF_ON_PLATEAU		( 38 ) /// 45
			
			#define					DUREE_STEP_MIN_MOTOR												380 /// cad vitesse max
			#define					DUREE_ARRET_MOTEUR_PR_ACCEL							2048
			
			/// P meilleur que I ???
			#define					USE_PID_ON_AUTO_CALIB												1
			#define							FACTEUR_P_PID_CONSIGNE_DEBIT													0.20f /// 0.33f max
			#define									LIMITE_BASSE_DESACTIVATION_COMPOSANTE_P_PID					(-1500.0f/1000) /// on desactivera la 		composante_PID_P_factor_SEGMENT_complet		a l'approche de la convergence, pour ne laisser que la composante Integral
			#define							FACTEUR_I_PID_CONSIGNE_DEBIT_ERR_NEGATIVE					0.0004f /// mode pas de panik : 0.00005f
			#define							FACTEUR_P_PID_CONSIGNE_DEBIT_ERR_POSITIVE						0.50f  /// en cas de debranchement patient par ex : on diminue en mode vlan
			
			/// securite tuyaux bouched : variation debit instantane
			
			#define					MAX_INCREMENT_INTEGRALE_ACCUMULEE_PID_POSITIF					(8.5f /1000) /// 2000 : on empeche les tres grosses ponctuelles /// todo_lyon : 
			#define					MAX_INCREMENT_INTEGRALE_ACCUMULEE_PID_NEGATIF				(2.5f /1000) /// 2000 : on empeche les tres grosses ponctuelles /// todo_lyon : 
			#define					MAX_INTEGRALE_ACCUMULEE_PID																(15.0f /1000) /// 2000 : on empeche les tres grosses ponctuelles /// todo_lyon : 
			
			#define				PROPORTION_FACTEUR_LINEARITE_PLATEAU							30.5f /// diminuer pour ajouter de la resolution
			#define						MINI_FACTEUR_LINEARITE_PLATEAU												0.60f
			#define						MAXI_FACTEUR_LINEARITE_PLATEAU											0.90f
			#define						FACT_PROPORTIONNEL_in_LINEARITE_NEGATIF						0.75f /// pour converger + rapidement vers erreur	
			#define						FACT_PROPORTIONNEL_in_LINEARITE_POSITIF							2.75f /// MODE PANIK OVERSHOOT : pour converger + rapidement vers erreur				
			
			/// 10 - 40 marche tres bien pour les compliances basses : il va falloir tenir compte de la Paw : voir 		Segment_error_time_sliced()		WIP
			#define							TIME_SLICING_ERROR_debut										10 /// ms : 10
			#define							TIME_SLICING_ERROR_fin												40 /// ms : 40
			
			
			
			#define					ENABLE_GESTION_Ti																0
			#define					ENABLE_LISSAGE_DECELERATION									0
			
			
			
			#define					ENABLE_LISSAGE_COURBE_FACT_INTEGRALE			0 /// NE PAS UTILISER c'est foireux...
			#define							FACTEUR_LISSAGE_NM1													0.08f
			#define							FACTEUR_LISSAGE_NP1													0.08f
			
			
			/// sampling debit IT pendant PHASE MOTEUR inspi
			#define					DEBIT_OVERSAMPLING_TIME											0 /// ms apres arret moteur (toujours un peu de débit) : mais on en tient pas compte finalement pour la pid
			
						
			#define					USE_PID_VENTILATION_Verreur_accel							0 /// WIP : pas encore tested, pas meme compiled
			#define							FACTEUR_I_PID_VENTILATION_ERREUR_PHASE_ACCEL				0.0002f
			
			
			// #define					DEBIT_CIBLE_SLM																60 /// slm
			// #if							DEBIT_CIBLE_SLM		< 		10
				// #error 		"DEBIT_CIBLE_SLM trop bas..."
			// #endif
			
			#define					DEBUG_PRINTF_CYCLE_PID											0 /// ---DEBUT sampling data
			#define					DEBUG_PRINTF_PR_CALIB_A_VIDE								0
			
			#define					ENABLE_PID_REGULATION												1
			
			
			#define					NIVEAU_VERBOSE_DEBUG												3
			/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la consigne de debit est 			DEBIT_CIBLE_SLM
			/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la consigne de debit est 			DEBIT_CIBLE_SLM
			/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la consigne de debit est 			DEBIT_CIBLE_SLM








void		hardfault_CRASH_ME();

typedef struct  {
	uint8_t		CALIB_nbr_echant;
	uint32_t	CALIB_duree_sgt_US;
	float			CALIB_result_volume_segment;
	uint32_t	CALIB_US_avt_motor_full_speed;
	float			debit_slm_en_cours;
	
	float			PID_ERREUR_Debit;
	float			PID_i_factor;
	bool			max_error_checked;
	
	bool			PID_tunable_sgt;
	bool			MODE_Panique;
	bool			is_vitesse_max_SEGMENT; ///						= false;
	bool			is_accel_max_SEGMENT; ///						= false;
} mesure_volume;

#define						NBR_SEGMENTS_CALIBRATION												64 /// WIP on aurait due faire +, en fait ca passe en RAM
mesure_volume		TAB_volume_slm_calib[ NBR_SEGMENTS_CALIBRATION ];

#define		NBR_VALEURS_TAB_debits_temps_moteur		800 /// besoin de marge pour Ti tres long a gros volumes : il y a de la RAM a gagner ici : essayer avec Ti max : WIP : faire freq variable
#define		DIVISEUR_NBR_VALEURS_SAMPLED				1 /// WIP : faire freq variable

typedef struct  {
			int16_t	dp_raw;
			// int16_t	Paw; /// WIP : pr calculer le time slicing de debit : low res surement suffisante !!!
			uint32_t	timecode_sample_MS;
} samples_debit;

/// WIP : a placer en archi apres validation fonctionnement complet
volatile int32_t							GLOB_index_TAB_dp_raw_temps_moteur;
volatile int64_t							GLOB_accumule_TAB_dp_raw_temps_moteur;
volatile uint64_t						GLOB_accumule_TAB_TIMECODE_temps_moteur;
volatile int16_t							GLOB_denom_TAB_dp_raw_temps_moteur;
volatile samples_debit			TAB_dp_raw_temps_moteur[ NBR_VALEURS_TAB_debits_temps_moteur  ]; /// 200 Hz : resta a calculer : _current_flow_slm = compute_corrected_flow(dp_raw);

volatile uint64_t						GLOB_TIMER_ms_debut_sampling_temps_moteur;
volatile uint32_t						GLOB_timecode_ms_full_speed;
volatile uint8_t							GLOB_index_sgt_full_speed;
volatile uint64_t						GLOB_TIMER_fin_accel_moteur_ms;
volatile uint64_t						GLOB_TIMER_ms_fin_sampling_temps_moteur;
volatile bool								GLOB_is_running_sampling_temps_moteur;


volatile int									GLOB_NBR_pas_moteur_par_segment;
/// ajouts Adrien
/// ajouts Adrien
/// ajouts Adrien



volatile bool				GLOB_is_first_guess_from_abaques; /// 						= true;
volatile uint32_t		GLOB_index_premier_segment_FULL_SPEED; /// 	= 0;
volatile uint32_t		GLOB_NBR_de_cycles_before_auto_crash_test; /// 	= 0; /// 0 pr desactiver le test crash


volatile float				GLOB_FACTEUR_linearite_plateau_inspi;
volatile float				GLOB_MOYENNE_facteur_I_PID; /// = 0;
volatile float				GLOB_MOYENNE_erreur; /// 				= 0;
volatile uint32_t		GLOB_index_pas_stepper; /// = 0;
volatile uint32_t		GLOB_duree_TOTAL_theorique_US; /// = 0;


volatile float				GLOB_debit_from_error_slm; /// = 0; /// provenant essentiellement de la phase d'acceleration qu'il va falloir compenser en augmentation de dit (Ti restant fixed)
volatile float				GLOB_Volume_erreur_phase_accel; /// = 0;


volatile uint32_t 		g_motor_steps_us[ MAX_MOTOR_STEPS ]; /// = {0}; // TODO: Make it configurable with a define. This represent a physical limit a the system.


/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// 			FUNCTIONS
bool		load_default_calib_from_debit(	float debit_consigne_slm );
bool		init_variables_PID(		float		debit_consigne_slm );
void 		reset_Vol_error_phase_accel(); /// GLOB_Volume_erreur_phase_accel = -12345.0f; /// sera remesured ds interrupt i2c

void		start_sampling_temps_moteur( uint32_t maintenant_sampling );
void		stop_sampling_temps_moteur( );

float		Segment_error_time_sliced(
															/// INPUT : TAB_dp_raw_temps_moteur
															int				index_segt,
															uint32_t		TIMER_US_debut_segment, 						/// INPUT : TIMER_US_debut_segment
															uint32_t		TIMER_US_fin_segment, 								//// INPUT : TIMER_US_fin_segment
															uint32_t		TIME_SLICING_ERROR_debut__, 					/// INPUT : TIME_SLICING_ERROR_debut
															uint32_t		TIME_SLICING_ERROR_fin__, 						/// INPUT : TIME_SLICING_ERROR_fin
															/// important que ces 2 variables soient JUSTES et mise à jour !!!																				WIP : a verifier
															uint64_t		GLOB_TIMER_fin_accel_moteur_ms, 								// GLOB_TIMER_fin_accel_moteur_ms = 1;
															uint64_t		GLOB_TIMER_ms_fin_sampling_temps_moteur 			// GLOB_TIMER_ms_fin_sampling_temps_moteur = 0;
);

/// todo WIP struct pr globales vraies, et arguments compute_PID_factors pr usage uniqt ds boucle breathing
/// todo WIP struct pr globales vraies, et arguments compute_PID_factors pr usage uniqt ds boucle breathing
bool		compute_PID_factors(
															float 		debit_consigne_slm,
															uint32_t Ti
															/// GLOB_Volume_erreur_phase_accel
														);
/// todo WIP struct pr globales vraies, et arguments compute_PID_factors pr usage uniqt ds boucle breathing
/// todo WIP struct pr globales vraies, et arguments compute_PID_factors pr usage uniqt ds boucle breathing


#endif
