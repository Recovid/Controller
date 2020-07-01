#ifndef __CONST_CALIB_H__
#define __CONST_CALIB_H__

#include <math.h>

#define			RANGE_RACINE_CONST_TAB			1200
const float		TAB_sqrtf[ RANGE_RACINE_CONST_TAB ];
const uint16_t CONST_calib_90_lpm[64];
const uint16_t CONST_calib_80_lpm[64];
const uint16_t CONST_calib_70_lpm[64];
const uint16_t CONST_calib_60_lpm[64];
const uint16_t CONST_calib_50_lpm[64];
const uint16_t CONST_calib_40_lpm[64];
const uint16_t CONST_calib_30_lpm[64];
const uint16_t CONST_calib_20_lpm[64];
const uint16_t CONST_calib_10_lpm[64];
const uint16_t CONST_calib_90_lpm[64];

/// ajouts Adrien
/// ajouts Adrien		WIP : a deplacer
/// ajouts Adrien

/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la
/// consigne de debit est 			DEBIT_CIBLE_SLM ceci est un WIP
/// ;)		besoin de temps pour fixer ca proprement : la consigne de debit
/// est 			DEBIT_CIBLE_SLM ceci est un WIP ;)
/// besoin de temps pour fixer ca proprement : la consigne de debit est
/// DEBIT_CIBLE_SLM
// #define					NBR_PAS_MOTEUR_MAX
// MAX_MOTOR_STEPS  /// 4800 //4480 #define
// NBR_SEGMENTS_CALIBRATION
// 64

// #define					NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE
// ( (float)NBR_PAS_MOTEUR_MAX / NBR_SEGMENTS_CALIBRATION )

#define VITESSE_DEMARRAGE_CALIB            													(2048) // us pas moteur * MOTOR_CORRECTION_USTEPS

#define NBR_ITERATION_CALIB 3

#define MAX_ACCEL_MOTEUR_POSITIF 															(38)            /// 40
#define MAX_ACCEL_MOTEUR_NEGATIF 															(50)            /// 45
#define MAX_ACCEL_MOTEUR_POSITIF_ON_PLATEAU 										(38) /// 40
#define MAX_ACCEL_MOTEUR_NEGATIF_ON_PLATEAU 									(50) /// 45

#define DUREE_STEP_MIN_MOTOR 380 /// cad vitesse max
#define DUREE_ARRET_MOTEUR_PR_ACCEL 2048

/// P meilleur que I ???
#define USE_PID_ON_AUTO_CALIB 																	1
#define FACTEUR_P_PID_CONSIGNE_DEBIT 														0.20f /// 0.33f max
#define LIMITE_BASSE_DESACTIVATION_COMPOSANTE_P_PID 						(-1500.0f /1000) /// on desactivera la 		composante_PID_P_factor_SEGMENT_complet


 /// a l'approche de la convergence, pour ne laisser que la composante
 /// Integral
#define FACTEUR_I_PID_CONSIGNE_DEBIT_ERR_NEGATIVE  							0.0004f /// mode pas de panik : 0.00005f
#define FACTEUR_P_PID_CONSIGNE_DEBIT_ERR_POSITIVE  							0.50f /// en cas de debranchement patient par ex : on diminue en mode vlan

/// securite tuyaux bouched : variation debit instantane
#define MAX_INCREMENT_INTEGRALE_ACCUMULEE_PID_POSITIF 					(8.5f / 1000) /// 2000 : on empeche les tres grosses ponctuelles /// todo_lyon :
#define MAX_INCREMENT_INTEGRALE_ACCUMULEE_PID_NEGATIF 				(2.5f / 1000) /// 2000 : on empeche les tres grosses ponctuelles /// todo_lyon :
#define MAX_INTEGRALE_ACCUMULEE_PID                   									(15.0f / 1000) /// 2000 : on empeche les tres grosses ponctuelles /// todo_lyon :

#define PROPORTION_FACTEUR_LINEARITE_PLATEAU                                  	30.5f /// diminuer pour ajouter de la resolution
#define MINI_FACTEUR_LINEARITE_PLATEAU 													0.60f
#define MAXI_FACTEUR_LINEARITE_PLATEAU 													0.90f
#define FACT_PROPORTIONNEL_in_LINEARITE_NEGATIF    								0.75f /// pour converger + rapidement vers erreur
#define FACT_PROPORTIONNEL_in_LINEARITE_POSITIF    								2.75f /// MODE PANIK OVERSHOOT : pour converger + rapidement vers erreur

/// 10 - 40 marche tres bien pour les compliances basses : il va falloir tenir
/// compte de la Paw : voir 		Segment_error_time_sliced()
/// WIP

#define	ENABLE_CORRECTION_FABRICE_GERMAIN										1
				#define		MODEL_ERR_PAR_PHASES													1 /// 0 : ancienne implem
				
#define	ENABLE_CORRECTION_FARRUG															0
#define	ENABLE_CONVERSION_BTPS																	0 /// 0 : ATPD

#if		ENABLE_CORRECTION_FABRICE_GERMAIN			==						1
	
	#if		ENABLE_CORRECTION_FARRUG							==						1
		#define TIME_SLICING_ERROR_debut 																20 /// ms : 70
		#define TIME_SLICING_ERROR_fin 																	50   /// ms : 120

	#else
		
	
		#if		MODEL_ERR_PAR_PHASES									==							1
			// #error "zzzzzzzzzzzzzzzzz"
			#define TIME_SLICING_ERROR_debut 															10 /// ms : 70
			#define TIME_SLICING_ERROR_fin 																	40   /// ms : 120

		#else
			#define TIME_SLICING_ERROR_debut 															75 /// ms : 70
			#define TIME_SLICING_ERROR_fin 																	110   /// ms : 120
		#endif
	#endif
#else
	#define TIME_SLICING_ERROR_debut 																20 /// ms : 10
	#define TIME_SLICING_ERROR_fin 																	50   /// ms : 40
#endif

#define ENABLE_GESTION_Ti 0
#define ENABLE_LISSAGE_DECELERATION 0

#define ENABLE_LISSAGE_COURBE_FACT_INTEGRALE 									1 /// NE PAS UTILISER c'est foireux...
#define FACTEUR_LISSAGE_NM1 																				0.08f
#define FACTEUR_LISSAGE_NP1																				0.08f


#define	ENABLE_ASSERVISSEMENT_PEP															0



#define	VOLUME_PARASITE_DECELERATION													72 /// mL /// par defaut approximed pour 60 lpm
#define	ENABLE_ADAPTATION_DURING_EXPIRATION									1

/// sampling debit IT pendant PHASE MOTEUR inspi
#define DEBIT_OVERSAMPLING_TIME             0 /// ms apres arret moteur (toujours un peu de débit) : mais on en tient pas
                                              /// compte finalement pour la pid

#define USE_PID_VENTILATION_Verreur_accel   													1 /// WIP : pas encore tested, pas meme compiled
#define FACTEUR_I_PID_VENTILATION_ERREUR_PHASE_ACCEL 					0.0002f

// #define					DEBIT_CIBLE_SLM
// 60 /// slm
// #if							DEBIT_CIBLE_SLM		<
// 10 #error 		"DEBIT_CIBLE_SLM trop bas..."
// #endif

#define DEBUG_PRINTF_CYCLE_PID 0 /// ---DEBUT sampling data
#define DEBUG_PRINTF_PR_CALIB_A_VIDE 0

#define ENABLE_PID_REGULATION 1

#define NIVEAU_VERBOSE_DEBUG 1
#define						VERBOSE_PID_PHASE_DEBUT					0
/// ceci est un WIP ;)		besoin de temps pour fixer ca proprement : la
/// consigne de debit est 			DEBIT_CIBLE_SLM ceci est un WIP
/// ;)		besoin de temps pour fixer ca proprement : la consigne de debit
/// est 			DEBIT_CIBLE_SLM ceci est un WIP ;)
/// besoin de temps pour fixer ca proprement : la consigne de debit est
/// DEBIT_CIBLE_SLM


// static inline uint16_t get_time_us(); /// { return timer_us.Instance->CNT; }
#define SAMPLING_PERIOD_MS      (5)
uint16_t	get_time_us_extern();

#if	ENABLE_CORRECTION_FABRICE_GERMAIN		==		1
	
	 // #error "--------------------"
	// #include "../platforms/recovid_revB/core/recovid_revB.h"
	
	#define		NBR_ECHANTILLONS_FITTING_MODELE_ERREUR				5 ///  SAMPLING_PERIOD_MS*5 = 25 ms pr fitting Fabrice : inutile sur derniere implem : todo
	
	static volatile float buffer_flow_slm[ NBR_ECHANTILLONS_FITTING_MODELE_ERREUR * 4 ]				={0};
	static volatile float buffer_Paw_cmH2O[ NBR_ECHANTILLONS_FITTING_MODELE_ERREUR * 4 ]			={0};
	// static volatile float buffer_RPaw_cmH2O[ NBR_ECHANTILLONS_FITTING_MODELE_ERREUR * 4 ]		={0};
	static volatile uint32_t buffer_time_echant_US[ NBR_ECHANTILLONS_FITTING_MODELE_ERREUR * 4 ]	={0};
	
	/// a verifier : reinit 		ds 		init_variables_PID()		et		
	static volatile float MODELE_ERR_PNEUMO_paw_max = 0;
	static volatile float MODELE_ERR_PNEUMO_volume_pneumo = 0; /// MODELE_ERR_PNEUMO_volume_pneumo ------> faut-il repasser a zero a chaque fin de cycle ????????
	static volatile float MODELE_ERR_PNEUMO_volumeplus_pneumo = 0;
	// static volatile uint16_t last_flow_t_us;
	
	
	/// MODELE_ERR_PNEUMO_volume_pneumo		et			MODELE_ERR_PNEUMO_volumeplus_pneumo ------> faut-il repasser a zero a chaque fin de cycle ????????
	/// MODELE_ERR_PNEUMO_paw_max 					a la reinit du systeme : init_variables_PID()
#endif





typedef struct {
  uint8_t CALIB_nbr_echant;
  uint32_t CALIB_duree_sgt_US;
  float CALIB_result_volume_segment;
  uint32_t CALIB_US_avt_motor_full_speed;
  float debit_slm_en_cours;

  float PID_ERREUR_Debit;
  float PID_i_factor;
  bool max_error_checked;

  bool PID_tunable_sgt;
  bool MODE_Panique;
  bool is_vitesse_max_SEGMENT; ///						=
                               ///false;
  bool
      is_accel_max_SEGMENT; ///						= false;
} mesure_volume;

#define NBR_SEGMENTS_CALIBRATION            64 /// WIP on aurait due faire +, en fait ca passe en RAM
mesure_volume TAB_volume_slm_calib[NBR_SEGMENTS_CALIBRATION];

#define NBR_VALEURS_TAB_debits_temps_moteur  800 /// besoin de marge pour Ti tres long a gros volumes : il y a de la RAM a
      /// gagner ici : essayer avec Ti max : WIP : faire freq variable

typedef struct {
  int16_t dp_raw;
  // int16_t	Paw; /// WIP : pr calculer le time slicing de debit : low res
  // surement suffisante !!!
  uint32_t timecode_sample_MS;
} samples_debit;

/// WIP : a placer en archi apres validation fonctionnement complet
volatile int32_t GLOB_index_TAB_dp_raw_temps_moteur;
volatile int64_t GLOB_accumule_TAB_dp_raw_temps_moteur;
volatile uint64_t GLOB_accumule_TAB_TIMECODE_temps_moteur;
volatile int16_t GLOB_denom_TAB_dp_raw_temps_moteur;
volatile samples_debit TAB_dp_raw_temps_moteur [NBR_VALEURS_TAB_debits_temps_moteur]; /// 200 Hz : resta a calculer :
                                           /// _current_flow_slm =
                                           /// compute_corrected_flow(dp_raw);

volatile uint64_t 		GLOB_TIMER_ms_debut_sampling_temps_moteur;

volatile uint32_t 		GLOB_timecode_ms_full_speed;
volatile uint8_t 			GLOB_index_sgt_full_speed;

volatile uint32_t						GLOB_Verif_Tinsu;
volatile uint32_t						GLOB_index_sample_full_speed;
volatile uint32_t						GLOB_index_sample_GO_motor_stop;
volatile uint32_t						GLOB_index_sample_motor_stop_DONE;
volatile uint32_t						GLOB_index_sample_FIN_plateau;
volatile uint32_t						GLOB_timecode_ms_full_speed;

/// voir			USE_PID_VENTILATION_Verreur_accel :
volatile float								GLOB_Volume_BEFORE_motor_stop;
volatile float								GLOB_Volume_AFTER_motor_stop;
volatile float								GLOB_Volume_AFTER_Plateau;
			
			

// volatile uint64_t 		GLOB_TIMER_fin_accel_moteur_ms;
volatile uint64_t 		GLOB_TIMECODE_ms_fin_sampling_temps_moteur;


volatile bool 				GLOB_is_running_sampling_temps_moteur;

volatile int GLOB_NBR_pas_moteur_par_segment;
/// ajouts Adrien
/// ajouts Adrien
/// ajouts Adrien

volatile bool GLOB_is_first_guess_from_abaques;          /// 						=
                                                         /// true;
volatile uint32_t GLOB_index_premier_segment_FULL_SPEED; /// 	= 0;
volatile uint32_t
    GLOB_NBR_de_cycles_before_auto_crash_test; /// 	= 0; /// 0 pr desactiver
                                               /// le test crash

volatile float GLOB_FACTEUR_linearite_plateau_inspi;
volatile float GLOB_MOYENNE_facteur_I_PID;       /// = 0;
volatile float GLOB_MOYENNE_erreur;              /// 				= 0;
volatile uint32_t GLOB_index_pas_stepper;        /// = 0;
volatile uint32_t GLOB_duree_TOTAL_theorique_US; /// = 0;

volatile float GLOB_debit_to_compensate_ERROR_accel; /// = 0; /// provenant essentiellement de la
                               /// phase d'acceleration qu'il va falloir
                               /// compenser en augmentation de dit (Ti restant
                               /// fixed)
							   
volatile float GLOBALE_pep;
			   
/// Volumes parasites :
volatile float GLOB_Volume_erreur_phase_accel; /// = 0;
volatile float GLOB_Volume_insu_AFTER_motor_stop; /// = 0;
volatile float GLOB_TOTAL_Volume_insu_AFTER_motor_stop; /// = 0;
		  
/// 			FUNCTIONS
float compute_corrected_flow( int16_t read, float Paw );
bool load_default_calib_from_debit(float debit_consigne_slm);
void init_variables_PID(float debit_consigne_slm);
void reset_Vol_error_phase_accel(); /// GLOB_Volume_erreur_phase_accel =
                                    /// -12345.0f; /// sera remesured ds
                                    /// interrupt i2c
#if	ENABLE_CORRECTION_FABRICE_GERMAIN		==		1
typedef enum {
	INSPIRATION_MODEL_ERR,
	EXPIRATION_MODEL_ERR,
	PLATEAU_MODEL_ERR
} modele_erreur_PHASE;

volatile float	ACCUMULATION_err_max_EXPI;

volatile	modele_erreur_PHASE	GLOB_PHASE___modele_erreur; /// = 1;
volatile	float GLOB_last_flow_slm_brut; /// = 1;
void reinit_Modele_erreur_Pneumo(); /// notamment reinit modele erreur pneumo pr compute_corrected_flow() /// CHANGEMENTS_POST_LYON_0
void reset_volumes_Modele_erreur_Pneumo(); /// /// pour usage cycle a cycle
#endif


void start_sampling_temps_moteur(uint32_t maintenant_sampling);
void stop_sampling_temps_moteur();

float Segment_error_time_sliced(
													int		index_segt,
													float*   flow_samples_Lpm,                         /// INPUT : Flow samples
													uint32_t flow_samples_period_ms,                   /// INPUT : Flow samples period
													uint32_t flow_samples_count,   
													uint32_t TIMER_US_debut_segment,                   /// INPUT : TIMER_US_debut_segment
													uint32_t TIMER_US_fin_segment,                     /// INPUT : TIMER_US_fin_segment
													uint32_t time_shift_start,                         /// INPUT : TIME_SLICING_ERROR_debut
													uint32_t time_shift_end                           /// INPUT : TIME_SLICING_ERROR_fin
													// uint32_t GLOB_timecode_ms_full_speed,           /// INPUT : Time end of acceleration
													// uint32_t GLOB_TIMECODE_ms_fin_sampling_temps_moteur
												); /// INPUT : Time end of plateau

/// todo WIP struct pr globales vraies, et arguments compute_PID_factors pr
/// usage uniqt ds boucle breathing todo WIP struct pr globales vraies, et
/// arguments compute_PID_factors pr usage uniqt ds boucle breathing
bool compute_PID_factors(float debit_consigne_slm, uint32_t Ti
                         /// GLOB_Volume_erreur_phase_accel
);
/// todo WIP struct pr globales vraies, et arguments compute_PID_factors pr
/// usage uniqt ds boucle breathing todo WIP struct pr globales vraies, et
/// arguments compute_PID_factors pr usage uniqt ds boucle breathing

#endif
