#include "breathing.h"
#include "common.h"
#include "config.h"
#include "controller.h"
#include "platform.h"
#include "adaptation.h"

#include <stdint.h>
#include <inttypes.h>
#include <compute_motor.h>
#include <math.h>
#include "const_calibs.h"


float Segment_error_time_sliced( float*   flow_samples_Lpm,                       /// INPUT : Flow samples
                                 uint32_t flow_samples_period_ms,                 /// INPUT : Flow samples period
                                 uint32_t TIMER_US_debut_segment,                 /// INPUT : TIMER_US_debut_segment
                                 uint32_t TIMER_US_fin_segment,                   /// INPUT : TIMER_US_fin_segment
                                 uint32_t time_shift_start,                       /// INPUT : TIME_SLICING_ERROR_debut
                                 uint32_t time_shift_end,                         /// INPUT : TIME_SLICING_ERROR_fin
                                 uint32_t GLOB_TIMER_fin_accel_moteur_ms,         /// INPUT : Time end of acceleration
                                 uint32_t GLOB_TIMER_ms_fin_sampling_temps_moteur)/// INPUT : Time end of plateau
{
  float Debit_moyen_SEGMENT = 0.f;
  int denom = 0;

  if ( (TIMER_US_fin_segment / 1000)  <= GLOB_TIMER_fin_accel_moteur_ms) 
  {
    /// segment inutilisable car accel initiale
    return 0.0f;
  }
  
  /// d'abord on cherche la Paw du segment
  uint32_t timecode_milieu_segment_MS = TIMER_US_debut_segment + (TIMER_US_fin_segment - TIMER_US_debut_segment) / 2; 
  timecode_milieu_segment_MS /= 1000;

  uint32_t sample_idx_debut =  ((timecode_milieu_segment_MS + time_shift_start)) / flow_samples_period_ms;
  uint32_t t_end_of_measure =  MIN( (timecode_milieu_segment_MS + time_shift_end), GLOB_TIMER_ms_fin_sampling_temps_moteur);
  uint32_t sample_idx_fin   =  ( t_end_of_measure / flow_samples_period_ms);

  for (uint32_t index_TAB_sampl_DEBITS = sample_idx_debut ; index_TAB_sampl_DEBITS < sample_idx_fin; index_TAB_sampl_DEBITS++) {
    //Find the sample between timecode_milieu_segment_MS + time_shift_start and timecode_milieu_segment_MS + time_shift_end 
    Debit_moyen_SEGMENT += flow_samples_Lpm[index_TAB_sampl_DEBITS];
    // Timecode_moyen_Debit_SEGMENT 	+= TAB_dp_raw_temps_moteur[
    // index_TAB_sampl_DEBITS ].timecode_sample_MS;
    denom++;
  } /// fin du for

  if (denom == 0) {
    printf("--- PANIQUE SGT : de moyenne possible : on prends le milieu\n");
    printf("T debut = %d\n", TIMER_US_debut_segment);
    printf("T fin   = %d\n", TIMER_US_fin_segment  );
    Debit_moyen_SEGMENT =  flow_samples_Lpm[sample_idx_debut];
    /// ON SAMPLE L'ECHANTILLON LE PLUS PROCHE : la frequence d'echantillonage n'est pas stable...
  }
  else {
    Debit_moyen_SEGMENT /= denom;
  }
  return Debit_moyen_SEGMENT;
}
/// OUPUT : Debit_slm_segment

void reset_Vol_error_phase_accel() {
  return;
}

bool load_default_calib_from_debit(float target_Pdiff_Lpm) {

  printf("----> load_default_calib_from_debit %i\n\r", (int)target_Pdiff_Lpm);
  
  const uint16_t* calib_lesser; // pointer to the calib tab (lesser bound)
  const uint16_t* calib_upper; // pointer to the calib tab (upper bound)
  float proportion_TAB_du_bas = 0.f;
  if (target_Pdiff_Lpm < 20)  /// on prends 		CONST_calib_10_lpm
  {  
    proportion_TAB_du_bas = (20.0f - target_Pdiff_Lpm) / 10;
    calib_lesser = CONST_calib_10_lpm;
    calib_upper   = CONST_calib_20_lpm;
  }
  else if (target_Pdiff_Lpm < 30)  /// on prends CONST_calib_20_lpm
  {  
    proportion_TAB_du_bas = (30.0f - target_Pdiff_Lpm) / 10;
    calib_lesser = CONST_calib_20_lpm;
    calib_upper   = CONST_calib_30_lpm;
  }
  else if (target_Pdiff_Lpm < 40) /// on prends CONST_calib_30_lpm
  {
    proportion_TAB_du_bas = (40.0f - target_Pdiff_Lpm) / 10;
    calib_lesser = CONST_calib_30_lpm;
    calib_upper   = CONST_calib_40_lpm;
  } 
  else if (target_Pdiff_Lpm < 50) /// on prends CONST_calib_40_lpm
  {
    proportion_TAB_du_bas = (50.0f - target_Pdiff_Lpm) / 10;
    calib_lesser = CONST_calib_40_lpm;
    calib_upper   = CONST_calib_50_lpm;
  } 
  else if (target_Pdiff_Lpm < 60)  /// on prends CONST_calib_50_lpm
  {
    proportion_TAB_du_bas = (60.0f - target_Pdiff_Lpm) / 10;
    calib_lesser = CONST_calib_50_lpm;
    calib_upper   = CONST_calib_60_lpm;
  } 
  else if (target_Pdiff_Lpm < 70)  /// on prends CONST_calib_60_lpm
  {
    proportion_TAB_du_bas = (70.0f - target_Pdiff_Lpm) / 10;
    calib_lesser = CONST_calib_60_lpm;
    calib_upper   = CONST_calib_70_lpm;
  } 
  else if (target_Pdiff_Lpm < 80) /// on prends CONST_calib_70_lpm
  {
    proportion_TAB_du_bas = (80.0f - target_Pdiff_Lpm) / 10;
    calib_lesser = CONST_calib_70_lpm;
    calib_upper   = CONST_calib_80_lpm;
  }
  else if (target_Pdiff_Lpm < 90) /// on prends CONST_calib_80_lpm
  {
    proportion_TAB_du_bas = (90.0f - target_Pdiff_Lpm) / 10;
    calib_lesser = CONST_calib_80_lpm;
    calib_upper   = CONST_calib_90_lpm;
  } 
  else 
  {
    proportion_TAB_du_bas = 0;
    calib_lesser   = CONST_calib_90_lpm; //We don't care about the downer bound
    calib_upper   = CONST_calib_90_lpm;
  }

  for (int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++) 
  {  
    TAB_volume_slm_calib[index_segt].CALIB_result_volume_segment = ((float) calib_lesser[index_segt] / (1000.f * 1000.f)) * proportion_TAB_du_bas +
                                                                   ((float)  calib_upper[index_segt] / (1000.f * 1000.f)) * (1.0f - proportion_TAB_du_bas);
    TAB_volume_slm_calib[index_segt].CALIB_nbr_echant = 1;
  }
  return true;
}

void init_variables_PID(float target_Pdiff_Lpm) {

  /// reinit tab
  for (int i = 0; i < NBR_SEGMENTS_CALIBRATION; i++) {
    TAB_volume_slm_calib[i].CALIB_nbr_echant = 0;
    TAB_volume_slm_calib[i].CALIB_duree_sgt_US = 0;
    TAB_volume_slm_calib[i].CALIB_result_volume_segment = 0;
    TAB_volume_slm_calib[i].CALIB_US_avt_motor_full_speed = 0;
    TAB_volume_slm_calib[i].debit_slm_en_cours = 0;

#if USE_PID_ON_AUTO_CALIB == 1
    TAB_volume_slm_calib[i].PID_ERREUR_Debit = 0.0f;
    // TAB_volume_slm_calib[ i ] .PID_Comp_Volume_Debit 				=
    // 0.0f; TAB_volume_slm_calib[ i ] .PID_p_factor
    // = 0.0f;
    TAB_volume_slm_calib[i].PID_i_factor = 0.0f;

    TAB_volume_slm_calib[i].max_error_checked = false;
    TAB_volume_slm_calib[i].PID_tunable_sgt = true;
    TAB_volume_slm_calib[i].MODE_Panique = false;
    TAB_volume_slm_calib[i].is_vitesse_max_SEGMENT = false;
    TAB_volume_slm_calib[i].is_accel_max_SEGMENT = false;
    // TAB_volume_slm_calib[ i ] .is_TUNABLE_segment					=
    // true;
#endif
  }
  /// reinit tab
  /// reinit tab
  /// reinit tab

#define NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE ((float)MAX_MOTOR_STEPS / NBR_SEGMENTS_CALIBRATION)

  /// todo WIP struct pr globales vraies, et arguments compute_PID_factors pr
  /// usage uniqt ds boucle breathing todo WIP struct pr globales vraies, et
  /// arguments compute_PID_factors pr usage uniqt ds boucle breathing
  GLOB_is_first_guess_from_abaques = true;
  GLOB_index_premier_segment_FULL_SPEED = 0;
  GLOB_NBR_de_cycles_before_auto_crash_test = 0; /// 0 pr desactiver le test crash
  GLOB_NBR_pas_moteur_par_segment = floor(NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE);
  GLOB_FACTEUR_linearite_plateau_inspi = 0;
  GLOB_MOYENNE_facteur_I_PID = 0;
  GLOB_MOYENNE_erreur = 0;
  GLOB_index_pas_stepper = 0;
  GLOB_duree_TOTAL_theorique_US = 0;
  GLOB_debit_from_error_slm = 0;
  /// todo WIP struct pr globales vraies, et arguments compute_PID_factors pr
  /// usage uniqt ds boucle breathing todo WIP struct pr globales vraies, et
  /// arguments compute_PID_factors pr usage uniqt ds boucle breathing

  printf("----> init_variables_PID %i\n\r", (int)(target_Pdiff_Lpm));

  /// charge un programme de 		TAB_volume_slm_calib[ index_segt ]
  /// .CALIB_result_volume_segment 		par defaut, ok a vide, la PID fera le
  /// reste
  if (load_default_calib_from_debit(target_Pdiff_Lpm) == false) {
   printf("HUGE ERROR : %s:%d\n",__FILE__, __LINE__);
   while(true);
  }
}

uint32_t adaptation( float     target_VT_mL,
                     float     target_Pdiff_Lpm, /// GLOB_debit_from_error_slm sera ajouted ds corps function, mais GLOB_Volume_erreur_phase_accel a passer en argument
                     uint32_t  flow_samples_period_ms,                           
                     uint32_t  flow_samples_count,                               
                     float*    flow_samples_Lpm,                                 
                     uint32_t  motor_max_steps,
                     uint32_t* motor_steps_us)
{ 
  static bool initialized = false;
  if(! initialized)
  {
    init_variables_PID(target_Pdiff_Lpm);
    initialized = true;
  }

  printf("------------------ compute_PID_factors DEBUT %i lpm %i ms\n\r", (int)target_Pdiff_Lpm, (int)get_setting_Tinsu_ms()*2);

  float duree_ACCEL = 0;
  float duree_FULL_SPEED = 0;

#define LIMITE_CYCLES_INSPI_TEST_DEBUG 0 /// 0 pr boucle infinie
  /// calculs PID

  if (flow_samples_count != 0) { /// on a des datas dispo
   /// Nouvel algo PID
   // bool			MODE_Panique = false;
   uint32_t index_step_moteur_en_cours = 0; /// pr analyse ds 	g_motor_steps_us[ ]
   uint32_t TIMER_US_debut_segment     = 0;
   uint32_t TIMER_US_fin_segment       = TAB_volume_slm_calib[0].CALIB_duree_sgt_US;

   for (int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++)  /// ds 		TAB_volume_slm_calib
   {
    ///
    /// ETAPE 1 : on recupere l'ERREUR sur le debit comme future base de la
    /// PID

    /// il faut faire un time slicing car on a un temps de reponse entre la
    /// commande moteur et son effet sur le débits todo reecrire en prenant en
    /// compte le tableau de debit du debut à la fin du temps moteur :
    /// beaucoup + precis

    /// ON FAIT LA MOYENNE : 	ATTENTION  INSTABLE CAR FREQUENCE ECHANTILLONAGE
    /// CAPTEUR PAS FIXE...
    printf("duree_theorique = %d get_cycle_insuflation_duration = %d\n", 
      (GLOB_duree_TOTAL_theorique_US / 1000), 
      get_cycle_insuflation_duration()-50);
    float Debit_slm_segment = Segment_error_time_sliced(
      flow_samples_Lpm,
      flow_samples_period_ms,
      TIMER_US_debut_segment,        /// INPUT : TIMER_US_debut_segment
      TIMER_US_fin_segment,          /// INPUT : TIMER_US_fin_segment
      TIME_SLICING_ERROR_debut,      /// INPUT : TIME_SLICING_ERROR_debut
      TIME_SLICING_ERROR_fin,        /// INPUT : TIME_SLICING_ERROR_fin
      GLOB_timecode_ms_full_speed,
      MIN((GLOB_duree_TOTAL_theorique_US / 1000), get_cycle_insuflation_duration())); // GLOB_TIMER_fin_accel_moteur_ms

    /// MAJ 		ERREUR
    TAB_volume_slm_calib[index_segt].PID_ERREUR_Debit = Debit_slm_segment - (target_Pdiff_Lpm
#if USE_PID_VENTILATION_Verreur_accel == 1
      // +	GLOB_debit_from_error_slm
#endif
      );
    /// MAJ 		ERREUR

    printf( "---deb %d mes %i Err %i\n\r", index_segt, (int)(Debit_slm_segment * 1000), (int)(TAB_volume_slm_calib[index_segt].PID_ERREUR_Debit * 1000));

    /// on recupere l'erreur du SEGMENT
    float ERREUR_debit_segment = TAB_volume_slm_calib[index_segt].PID_ERREUR_Debit;
    float DUREE_segment_US = TAB_volume_slm_calib[index_segt].CALIB_duree_sgt_US;

    ///
    /// ETAPE 2 : on check l'état de la commande MOTEUR		+
    /// on applique la PID sur l'erreur du segment

    /// a virer toute cette section :
    /// puis on recupere l'erreur sur le segment
    // ERREUR_debit_segment > 5000.0f / 1000
    if (0) /// WIP
    {
     ///				MODE PANIQUE ;)
     /// que le segment soit tunable ou pas ne change rien : on a OVERSHOOT
     /// !!!!!

     /// on repart de zero : mais il fadrait deplacer ca : WIP
     if (load_default_calib_from_debit(target_Pdiff_Lpm) == false)
     {
      printf("HUGE ERROR : %s:%d\n",__FILE__, __LINE__);
      while(true);
     }


     /// ALARME : debranchement patient
     /// MAJ indispensable
     index_step_moteur_en_cours += GLOB_NBR_pas_moteur_par_segment; /// pr analyse sgt suivant
     /// MAJ indispensable
     TAB_volume_slm_calib[index_segt].MODE_Panique = true;
    } 
    else if (0) /// WIP : marche pas trop mal, mais peut etre deplacer ca en
     /// mode global : pas segment par segment
     /// ERREUR_debit_segment > 1500.0f / 1000
    {
     TAB_volume_slm_calib[index_segt].PID_i_factor *= FACTEUR_P_PID_CONSIGNE_DEBIT_ERR_POSITIVE; /// et vlan
     // TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US
     // *=	1.0f  + ERREUR_debit_segment / ( target_Pdiff_Lpm +
     // target_Pdiff_Lpm );	/// et vlan

     printf("---SGT %i : OVERSHOT !!! ERR %i\n\r", index_segt,
       (int)(ERREUR_debit_segment * 1000));

     /// MAJ indispensable
     // index_step_moteur_en_cours += GLOB_NBR_pas_moteur_par_segment; /// pr
     // analyse sgt suivant
     /// MAJ indispensable

     TAB_volume_slm_calib[index_segt].MODE_Panique = true;
    }
    else { /// l'erreur est negative			PAS DE PANIQUE

     /// on verifie les commandes moteurs du cycle precedant		--->
     /// on passe		is_vitesse_max_SEGMENT		et
     /// is_accel_max_SEGMENT
     if (DUREE_segment_US < GLOB_NBR_pas_moteur_par_segment * DUREE_STEP_MIN_MOTOR) /// cad SEGMENT a vitesse max
     {
      printf("---ERREUR !!! duree %i < duree min %i from %i\n\r",
        (int)(DUREE_segment_US),
        GLOB_NBR_pas_moteur_par_segment * DUREE_STEP_MIN_MOTOR,
        GLOB_NBR_pas_moteur_par_segment);
      /// ALARME et PANIQUE : WIP

      TAB_volume_slm_calib[index_segt].MODE_Panique = true;
      // TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt = false;
      // TAB_volume_slm_calib[ index_segt ] .is_vitesse_max_SEGMENT = true;

      /// MAJ indispensable
      // index_step_moteur_en_cours += GLOB_NBR_pas_moteur_par_segment; ///
      // pr analyse sgt suivant
      /// MAJ indispensable

     } 
     else if (DUREE_segment_US == GLOB_NBR_pas_moteur_par_segment * DUREE_STEP_MIN_MOTOR) /// cad SEGMENT a vitesse max
     {
      /// SEGMENT non tunable
      // TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt = false;
      // TAB_volume_slm_calib[ index_segt ] .is_vitesse_max_SEGMENT = true;

      /// MAJ indispensable
      // index_step_moteur_en_cours += GLOB_NBR_pas_moteur_par_segment; ///
      // pr analyse sgt suivant
      /// MAJ indispensable

     }         /// else : TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt == true
     /// implicite

    } /// fin de l'erreur is negative			PAS DE PANIQUE

#if DEBUG_PRINTF_CYCLE_PID == 1 /// ---DEBUT sampling data
    /// RESUME :
    printf(
      "---SGT %i  : Panik %u || TUNABLE %u : %u %u || Dur_step %u || ERR "
      "%i || PID %i\n\r",
      index_segt, TAB_volume_slm_calib[index_segt].MODE_Panique,
      TAB_volume_slm_calib[index_segt].PID_tunable_sgt,
      TAB_volume_slm_calib[index_segt].is_vitesse_max_SEGMENT,
      TAB_volume_slm_calib[index_segt].is_accel_max_SEGMENT,

      TAB_volume_slm_calib[index_segt].CALIB_duree_sgt_US,
      (int)(ERREUR_debit_segment *
       1000), /// TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit
      // Debit_moyen_dp_raw_SEGMENT,
      (int)(TAB_volume_slm_calib[index_segt].PID_i_factor * 1000 * 1000)

      );

#endif

    /// MAJ
    TIMER_US_debut_segment = TIMER_US_fin_segment;
    if(index_segt + 1 < NBR_SEGMENTS_CALIBRATION)
     TIMER_US_fin_segment   += TAB_volume_slm_calib[index_segt+1].CALIB_duree_sgt_US;

   } /// fin du for		NBR_SEGMENTS_CALIBRATION
   ///
   /// ETAPE 2 : analyse linearite erreur : WIP une fois que periodicite
   /// capteurs  : a ameliorer

   /// on calcule un facteur P sur les GROSSES ERREURS UNIQUEMENT, pour faire
   /// converger + rapidement : sera accumuled ds		PID_i_factor
   /// WIP
   ///		-	la moyenne de l'erreur
   ///		-	le max de l'erreur lorsque celui ci est UNDERSHOOTED

   float composante_PID_P_factor_SEGMENT_complet = 0.0f;
   // float			GLOB_FACTEUR_linearite_plateau_inspi;

   // float			GLOB_MOYENNE_facteur_I_PID = 0;
   float SOMME_ABS_erreur = 0;
   // float			GLOB_MOYENNE_erreur 				=
   // 0;
   float MAX_erreur = -123456789;
   float MIN_erreur = 123456789;
   int denom = 0;

   for (int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++) { /// ds 		TAB_volume_slm_calib

    if (TAB_volume_slm_calib[index_segt].is_vitesse_max_SEGMENT == false) /// on ne fait la moyenne que de ce qui est sur le plateau de
     /// debit : on ne tient pas compte de la phase d'accel initiale
     /// on pourrait aussi utiliser 		GLOB_timecode_ms_full_speed		--->
     /// equivalent
    {
     SOMME_ABS_erreur += TAB_volume_slm_calib[index_segt].PID_ERREUR_Debit * TAB_volume_slm_calib[index_segt].PID_ERREUR_Debit;

     GLOB_MOYENNE_erreur += TAB_volume_slm_calib[index_segt].PID_ERREUR_Debit;
     GLOB_MOYENNE_facteur_I_PID += TAB_volume_slm_calib[index_segt].PID_i_factor;
     denom++;

     if (TAB_volume_slm_calib[index_segt].PID_ERREUR_Debit > MAX_erreur) 
     {
      MAX_erreur = TAB_volume_slm_calib[index_segt].PID_ERREUR_Debit;
     }

     if (TAB_volume_slm_calib[index_segt].PID_ERREUR_Debit < MIN_erreur) 
     {
      MIN_erreur = TAB_volume_slm_calib[index_segt].PID_ERREUR_Debit;
     }
    }
   } /// fin du for
   GLOB_MOYENNE_erreur /= denom;
   GLOB_MOYENNE_facteur_I_PID /= denom;
   SOMME_ABS_erreur /= denom;

   /// linearite du plateau
   GLOB_FACTEUR_linearite_plateau_inspi = fabs(GLOB_MOYENNE_erreur - MAX_erreur) / PROPORTION_FACTEUR_LINEARITE_PLATEAU; /// ajustement fin
   /// PID_i_factor
   // GLOB_FACTEUR_linearite_plateau_inspi = ( fabs( GLOB_MOYENNE_erreur ) -
   // SOMME_ABS_erreur ) / ( PROPORTION_FACTEUR_LINEARITE_PLATEAU / 6 ); ///
   // ajustement fin		PID_i_factor

   /// ecart moyen a consigne
   if (GLOB_MOYENNE_erreur >
     0) { /// MODE_PANIQUE : overshoot on cherche a revenir en leger negatif
    GLOB_FACTEUR_linearite_plateau_inspi *= GLOB_MOYENNE_erreur * FACT_PROPORTIONNEL_in_LINEARITE_POSITIF;
    if (GLOB_FACTEUR_linearite_plateau_inspi < MINI_FACTEUR_LINEARITE_PLATEAU * 4)
     GLOB_FACTEUR_linearite_plateau_inspi = MINI_FACTEUR_LINEARITE_PLATEAU * 4;

   } 
   else {
    GLOB_FACTEUR_linearite_plateau_inspi *= -GLOB_MOYENNE_erreur * FACT_PROPORTIONNEL_in_LINEARITE_NEGATIF;

    if (GLOB_FACTEUR_linearite_plateau_inspi < MINI_FACTEUR_LINEARITE_PLATEAU)
     GLOB_FACTEUR_linearite_plateau_inspi = MINI_FACTEUR_LINEARITE_PLATEAU;
   }

   if (GLOB_FACTEUR_linearite_plateau_inspi > MAXI_FACTEUR_LINEARITE_PLATEAU)
    GLOB_FACTEUR_linearite_plateau_inspi = MAXI_FACTEUR_LINEARITE_PLATEAU;

   /// OUTPUT : GLOB_MOYENNE_erreur
   /// OUTPUT : MAX_erreur
   /// OUTPUT : GLOB_FACTEUR_linearite_plateau_inspi
   
   /// on applique tout ca :
   for (int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++) /// ds 		TAB_volume_slm_calib
   {
    if (TAB_volume_slm_calib[index_segt].PID_tunable_sgt == false) {

    } 
    else { /// cad TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt == true
     /// on calcule la PID
     float increment_INTEGRALE = ( TAB_volume_slm_calib[index_segt].PID_ERREUR_Debit) *
      FACTEUR_I_PID_CONSIGNE_DEBIT_ERR_NEGATIVE
      * GLOB_FACTEUR_linearite_plateau_inspi; /// va diminuer l'impact de l'integrale a mesure que le plateau apparait

     if (increment_INTEGRALE > MAX_INCREMENT_INTEGRALE_ACCUMULEE_PID_POSITIF) 
     {
      increment_INTEGRALE = MAX_INCREMENT_INTEGRALE_ACCUMULEE_PID_POSITIF;
#if NIVEAU_VERBOSE_DEBUG >= 1
      if (index_segt >= GLOB_index_sgt_full_speed) {
       printf("\t\t---limitation sgt ++ %i : %i\n\r", index_segt, (int)(increment_INTEGRALE * 1000));
      }
#endif
     } 
     else if (increment_INTEGRALE < -MAX_INCREMENT_INTEGRALE_ACCUMULEE_PID_NEGATIF) 
     {
      increment_INTEGRALE = -MAX_INCREMENT_INTEGRALE_ACCUMULEE_PID_NEGATIF;
#if NIVEAU_VERBOSE_DEBUG >= 1
      if (index_segt >= GLOB_index_sgt_full_speed) {
       printf("\t\t---limitation sgt -- %i : %i\n\r", index_segt, (int)(increment_INTEGRALE * 1000));
      }
#endif
     }
     TAB_volume_slm_calib[index_segt].PID_i_factor += increment_INTEGRALE;
    }

    /// on applique tout ca :
   } /// fin du for 		NBR_SEGMENTS_CALIBRATION

   /// Nouvel algo PID

   /// on limite les facteurs de PID PID_i_factor : pas plus de 20000 !!!
   /// WIP	:		retuner avec nouveaux facteurs define :
   /// imperatif securite patient : tube bouched...
   for (uint32_t index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++) {

    if (TAB_volume_slm_calib[index_segt].PID_i_factor > MAX_INTEGRALE_ACCUMULEE_PID) // &&	TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == false
    {
     if ( index_segt < GLOB_index_premier_segment_FULL_SPEED) // TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == true
     {
      TAB_volume_slm_calib[index_segt].PID_i_factor = 0; /// par defaut, on garde ceinture brettelles
     } 
     else {
      TAB_volume_slm_calib[index_segt].PID_i_factor = MAX_INTEGRALE_ACCUMULEE_PID;
#if NIVEAU_VERBOSE_DEBUG >= 1
      printf( "---limitation++ %lu : %i\n\r", index_segt, (int)(TAB_volume_slm_calib[index_segt].PID_i_factor * 1000));
#endif
     }

    } 
    else if (TAB_volume_slm_calib[index_segt].PID_i_factor < -MAX_INTEGRALE_ACCUMULEE_PID) // &&	TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == false
    {
     if (index_segt < GLOB_index_premier_segment_FULL_SPEED) // TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == true
     {
      TAB_volume_slm_calib[index_segt].PID_i_factor =
       0; /// par defaut, on garde ceinture brettelles
     } 
     else {
      TAB_volume_slm_calib[index_segt].PID_i_factor =
       -MAX_INTEGRALE_ACCUMULEE_PID;
#if NIVEAU_VERBOSE_DEBUG >= 1
      printf(
        "---limitation-- %lu : %i\n\r", index_segt,
        (int)(TAB_volume_slm_calib[index_segt].PID_i_factor * 1000));
#endif
     }
    }
   }
   /// on limite les facteurs de PID PID_i_factor : pas plus de 20000 !!!

   /// LISSAGE param PID
   if (ENABLE_LISSAGE_COURBE_FACT_INTEGRALE == 1) /// WIP : pas terrible : ne pas utiliser, pour memoire : a virer
   {
    uint8_t TAB_tri_index_error[NBR_SEGMENTS_CALIBRATION];

    for (int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++)  /// ds 		TAB_volume_slm_calib
    {  
     float MAX_erreur = 0.0f;

     /// on cherche l'err max suivante :
     int index_SEGMENT_max_error_en_cours = -1;
     for (int index_segt_look_for_err = 0; index_segt_look_for_err < NBR_SEGMENTS_CALIBRATION; index_segt_look_for_err++)  /// ds
     {                                  /// TAB_volume_slm_calib

      if (fabs(TAB_volume_slm_calib[index_segt_look_for_err].PID_ERREUR_Debit) > MAX_erreur &&
        TAB_volume_slm_calib[index_segt_look_for_err].max_error_checked == false) /// pas deja retenu : WIP : foireux, a virer
      {
       index_SEGMENT_max_error_en_cours = index_segt_look_for_err;
       MAX_erreur = fabs(
         TAB_volume_slm_calib[index_segt_look_for_err].PID_ERREUR_Debit);
      }
     }

     if (index_SEGMENT_max_error_en_cours == -1)
      break; /// on a fini
     /// else

     // printf( "-------------------------------------------- %i Max
     // error %i\n\r", index_segt, (int)( TAB_volume_slm_calib[
     // index_SEGMENT_max_error_en_cours ] .PID_ERREUR_Debit * 1000 ) );

     /// on ventile PID_i_factor dans les segments adjacents
     if (index_SEGMENT_max_error_en_cours > 0)  /// gestion nm1
     {  
      TAB_volume_slm_calib[index_SEGMENT_max_error_en_cours - 1] .PID_i_factor = 
       TAB_volume_slm_calib[index_SEGMENT_max_error_en_cours - 1].PID_i_factor * (1.0f - FACTEUR_LISSAGE_NM1) 
       + TAB_volume_slm_calib[index_SEGMENT_max_error_en_cours] .PID_i_factor * FACTEUR_LISSAGE_NM1;
     }

     if (index_SEGMENT_max_error_en_cours < NBR_SEGMENTS_CALIBRATION - 1) { /// gestion nm1
      TAB_volume_slm_calib[index_SEGMENT_max_error_en_cours + 1] .PID_i_factor =
       TAB_volume_slm_calib[index_SEGMENT_max_error_en_cours + 1] .PID_i_factor * (1.0f - FACTEUR_LISSAGE_NP1) +
       TAB_volume_slm_calib[index_SEGMENT_max_error_en_cours] .PID_i_factor * FACTEUR_LISSAGE_NP1;
     }

     /// MAJ
     TAB_volume_slm_calib[index_SEGMENT_max_error_en_cours] .max_error_checked = true;
    } /// fin du for
   }

   /// LISSAGE param PID
   /// LISSAGE param PID
   /// LISSAGE param PID
   for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ )
   {
     brth_printf( "%i\n", (int)( TAB_volume_slm_calib[ index_segt ].PID_ERREUR_Debit * 10 ));
   }
#if DEBUG_PRINTF_CYCLE_PID == 1 /// ---DEBUT sampling data
   printf("---FIN   sampling data %i\n\r", GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED);
#endif
  } 
  else {
    printf( "-- No data for PID\n");    /// Alarm(); /// WIP
  }
  

  /// ETAPE 2 : on prepare la commande moteur
#if USE_PID_VENTILATION_Verreur_accel == 1 /// WIP
  float vol_during_acc_mL = 0.f;
  uint32_t t_acc = 0;
  printf("GLOB_timecode_ms_full_speed = %d\n", GLOB_timecode_ms_full_speed);
  uint32_t i =0;
  for(i = 0; t_acc < (GLOB_timecode_ms_full_speed*1000) ; i++)
  {
    t_acc             += motor_steps_us[i];
    if( ((t_acc/1000) % flow_samples_period_ms) == 0)
    {
      uint32_t idx = (t_acc/1000) / flow_samples_period_ms;
      vol_during_acc_mL += ( flow_samples_Lpm[idx] / 60.f) * (flow_samples_period_ms); //LPM -> mLms => /60/1000*1000 => /60
    }
  }
  //Mettre a jour GLOB_debit_from_error_slm
  GLOB_debit_from_error_slm = GLOB_Volume_erreur_phase_accel / duree_FULL_SPEED; /// on pourrait aussi estimer direct l'aire du triangle PID...

#endif

  GLOB_index_pas_stepper = 0;
  GLOB_duree_TOTAL_theorique_US = 0;
  int32_t Last_duree_step_moteur_en_cours = DUREE_ARRET_MOTEUR_PR_ACCEL; /// pr gestion accel uniqt
  uint32_t timecode_US_depuis_demarrage_moteur = 0;
  bool is_still_ACCEL_phase = true; /// phase ACCEL au demarrage, avant FULL SPEED
  uint32_t DUREE_Totale_cycle = 0;

  // printf( "---GLOB_NBR_pas_moteur_par_segment %i\n\r",
  // GLOB_NBR_pas_moteur_par_segment ); printf(
  // "---NBR_SEGMENTS_CALIBRATION %i\n\r", NBR_SEGMENTS_CALIBRATION );

  ///  todo : passer les 		is_accel_max_SEGMENT 		ici
  ///  todo : passer les 		 is_vitesse_max_SEGMENT		ici
  for (int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++)  /// pour chaque segment, on calcule la duree pour assurer
  {                                                                              /// le debit consigne (CALIB_result_volume_segment -> debit)
    bool is_accel_max_segt = true;
    bool is_vitesse_max_segt = true;

    float VOLUME_segment_corriged = TAB_volume_slm_calib[index_segt].CALIB_result_volume_segment;
                                         /// from		TAB_volume_slm_calib[
                                         /// index_segt
                                         /// ].CALIB_result_volume_segment
                                         /// cad		const_calibs.h

#if USE_PID_ON_AUTO_CALIB == 1
#if ENABLE_PID_REGULATION == 1
    VOLUME_segment_corriged += ((TAB_volume_slm_calib[index_segt].PID_i_factor));
#endif
#endif
    /// vol / debit = US
    uint32_t duree_segment_US = (uint32_t)(round((float)((VOLUME_segment_corriged) / (target_Pdiff_Lpm)*60 * 1000 * 1000)));

//#if USE_PID_VENTILATION_Verreur_accel == 1
    // +	GLOB_debit_from_error_slm /// todo : on accumule le VOL d'erreur pdt
    // accel : A VENTILER SUR target_Pdiff_Lpm
//#endif
//
//
#if DEBUG_PRINTF_PR_CALIB_A_VIDE == 1
    printf("%i\n\r", (int)(VOLUME_segment_corriged * 1000 * 1000)); /// pratique pr creer les const_calibs.h
                                                                         /// WIP	: a refaire avec  nouveaux mors
#endif

    uint32_t duree_step_ideale_US = round((float)duree_segment_US / GLOB_NBR_pas_moteur_par_segment); /// linearised : todo : faire
                                                                                                      /// varier le nombre de pas POUR
                                                                                                      /// EVITER DE TOUT DEPLACER

    /// on s'apprete a recalculer :
    TAB_volume_slm_calib[index_segt].CALIB_US_avt_motor_full_speed = 0;
    if (GLOB_is_first_guess_from_abaques == true) /// uniquement au premier tour : WIP : A VERIFIER, attention paleocode !!!!
    { 
      TAB_volume_slm_calib[index_segt].PID_tunable_sgt = false;
    }
    /// on recalcule le futur programme moteur en tenant compte des
    /// accelerations et vit max : WIP : a passer en fonction on recalcule le
    /// futur programme moteur en tenant compte des accelerations et vit max :
    /// WIP : a passer en fonction on recalcule le futur programme moteur en
    /// tenant compte des accelerations et vit max : WIP : a passer en fonction

    /// INPUT : g_motor_steps_us
    /// INPUT : duree_step_ideale_US
    /// INPUT : Last_duree_step_moteur_en_cours en ref a MAJ
    /// INPUT : timecode_US_depuis_demarrage_moteur en ref a MAJ
    /// INPUT : duree_segment_US en ref a MAJ

#if ENABLE_GESTION_Ti == 1
    bool super_break = false;
#endif
    uint32_t duree_step_US;
    { /// a passer en fonction : WIP : pas mal de globales a proprifier
      /// commande_steps_moteur_from_duree_segment_US_ideal()
      duree_segment_US = 0;
      for (int nbr_pas_segment = 0; nbr_pas_segment < GLOB_NBR_pas_moteur_par_segment; nbr_pas_segment++)
      {
        ///
        /// on consitute la MAP DMA de controle moteur :
        /// gestion acceleration
        /// analyse vitesse
        /// analyse vitesse : ok coherent : permet de passer
        /// GLOB_timecode_ms_full_speed		et
        /// is_still_ACCEL_phase
        if (duree_step_ideale_US <= DUREE_STEP_MIN_MOTOR) {
          /// vitesse impossible
          duree_step_ideale_US = DUREE_STEP_MIN_MOTOR;
        } 
        else {
          /// MAJ :
          is_vitesse_max_segt = false; /// pr tout le segment du coup
        }

        /// sommes nous sur le plateau ??
        if (is_still_ACCEL_phase == true && is_vitesse_max_segt == false)  /// on fixe le point de fin d'accel
        {
          GLOB_index_sgt_full_speed = index_segt;
          GLOB_timecode_ms_full_speed = timecode_US_depuis_demarrage_moteur / 1000; /// ( timecode_us_next_echant + TAB_volume_slm_calib[
                    /// index_segt ] .CALIB_US_avt_motor_full_speed ) / 1000;
          // printf( "--------------------->  GLOB_timecode_ms_full_speed
          // %u SET at segt %i\n\r", GLOB_timecode_ms_full_speed, index_segt );

          if (GLOB_is_first_guess_from_abaques) 
          {
            GLOB_index_premier_segment_FULL_SPEED = index_segt; /// fixed une fois pour toute : le reste est
                                                                /// optimisable : ca se discute e fixer ca au
                                                                /// premier cycle... WIP
          }
          TAB_volume_slm_calib[index_segt].PID_tunable_sgt = true;

          is_still_ACCEL_phase = false; /// MAJ : 1 seul par segment
        }

        /// analyse acceleration
        /// analyse acceleration
        int32_t Accel_moteur =
            (int32_t)Last_duree_step_moteur_en_cours - duree_step_ideale_US;
        int MAX_accel_negatif = MAX_ACCEL_MOTEUR_NEGATIF;
        int MAX_accel_positif = MAX_ACCEL_MOTEUR_POSITIF;

        if (index_segt >= GLOB_index_sgt_full_speed)  /// on est sur le plateau, soyons + smooth...
        {
          MAX_accel_negatif = MAX_ACCEL_MOTEUR_NEGATIF_ON_PLATEAU;
          MAX_accel_positif = MAX_ACCEL_MOTEUR_POSITIF_ON_PLATEAU;
        }

        if (-Accel_moteur > MAX_accel_negatif) 
        {  /// phase d'acceleration intiale
          duree_step_US = Last_duree_step_moteur_en_cours + MAX_accel_negatif;
          TAB_volume_slm_calib[index_segt].CALIB_US_avt_motor_full_speed += duree_step_US;
        } 
        else if (Accel_moteur > MAX_accel_positif) 
        {
          /// phase d'acceleration intiale
          duree_step_US = Last_duree_step_moteur_en_cours - MAX_accel_positif;
          TAB_volume_slm_calib[index_segt].CALIB_US_avt_motor_full_speed += duree_step_US;

        } 
        else 
        {
          /// phase FULL SPEED
          duree_step_US = duree_step_ideale_US;

          if (GLOB_is_first_guess_from_abaques == true) {
            TAB_volume_slm_calib[index_segt].PID_tunable_sgt = true; /// par defaut, tous les autres segment restent tunables
                      /// par la PID : WIP
          }

          /// MAJ :
          is_accel_max_segt = false; /// pr tout le segment du coup
        }

        Last_duree_step_moteur_en_cours = duree_step_US;
        timecode_US_depuis_demarrage_moteur += duree_step_US;

        /// on applique immediatement
        // if( TAB_volume_slm_calib[ index_segt ].CALIB_nbr_echant != 0 ){ ///
        // bug dernier segt... WIP : paleocode crado ??? me rappelle plus sorry,
        // a virer / tester

        // printf( "---- GLOB_index_pas_stepper %i\n\r",
        // GLOB_index_pas_stepper );
        motor_steps_us[GLOB_index_pas_stepper++] = duree_step_US;

        duree_segment_US += duree_step_US;
        // }
#if ENABLE_GESTION_Ti == 1
        if (DUREE_Totale_cycle + duree_segment_US > round((float)get_setting_Tinsu_ms()*2 / 1000)) 
        { /// on vient de depasser Ti !!! cut !!
          super_break = true;
          break;
        }
#endif

      } 

      /// fin du for nbr_pas_segment
      /// on MAJ immediatemment pour ce segment :
      // printf( "---AVANT %i\n\r", index_segt );

      TAB_volume_slm_calib[index_segt].is_accel_max_SEGMENT = is_accel_max_segt;
      TAB_volume_slm_calib[index_segt].is_vitesse_max_SEGMENT = is_vitesse_max_segt;
      /// OUTPUT : g_motor_steps_us
      /// OUTPUT : duree_segment_US, timecode_US_depuis_demarrage_moteur
      /// OUTPUT : passe les flags accel max vit max...
      /// OUTPUT fonction : a creer WIP :
      TAB_volume_slm_calib[index_segt].CALIB_duree_sgt_US = duree_segment_US; /// RECALCULED SELON CONSIGNE
      // printf( "---APRES %i\n\r", index_segt );
    } /// a passer en fonction
    /// on recalcule le futur programme moteur en tenant compte des
    /// accelerations et vit max : WIP : a passer en fonction on recalcule le
    /// futur programme moteur en tenant compte des accelerations et vit max :
    /// WIP : a passer en fonction on recalcule le futur programme moteur en
    /// tenant compte des accelerations et vit max : WIP : a passer en fonction

    DUREE_Totale_cycle += duree_segment_US; /// WIP : doublon paleocode a virer ???
    GLOB_duree_TOTAL_theorique_US += (uint32_t)duree_segment_US; /// WIP : doublon paleocode a virer ???

#if ENABLE_GESTION_Ti == 1
    if (super_break == true) { /// on vient de depasser Ti !!! cut !!
      break;
    }
#endif
  } /// fin du for index_segt

#if ENABLE_LISSAGE_DECELERATION == 1
  int nbr_pas_restant = MAX_MOTOR_STEPS - GLOB_index_pas_stepper;
  // if(
  // nbr_pas_restant > 0
  // ){
  while (nbr_pas_restant-- > 0) {
    if (Last_duree_step_moteur_en_cours > MAX_ACCEL_MOTEUR_NEGATIF) {
      Last_duree_step_moteur_en_cours -= MAX_ACCEL_MOTEUR_NEGATIF;
      motor_steps_us[GLOB_index_pas_stepper++] = Last_duree_step_moteur_en_cours;

    } else {
      Last_duree_step_moteur_en_cours = 0;
      motor_steps_us[GLOB_index_pas_stepper++] = Last_duree_step_moteur_en_cours;
      break;
    }
  }
  // }
#endif

  // printf( "---GLOB_NBR_pas_moteur_par_segment %i\n\r",
  // GLOB_NBR_pas_moteur_par_segment );

  GLOB_is_first_guess_from_abaques = false;

  // if(
  // GLOB_is_first_guess_from_abaques == true /// WIP : Vilain paleocode: je le
  // laisse car pas d'acces a un recovid pour tester : mais a virer !!!!! pas
  // fier ;)
  // &&	GLOB_timecode_ms_full_speed == 0
  // ){
  // printf( "--------------------->  GLOB_timecode_ms_full_speed %u SET
  // baaaad !!!\n\r", GLOB_duree_TOTAL_theorique_US / 1000);
  // GLOB_timecode_ms_full_speed = GLOB_duree_TOTAL_theorique_US / 1000;
  // GLOB_index_premier_segment_FULL_SPEED = NBR_SEGMENTS_CALIBRATION;
  // }

  /// OUTPUT : GLOB_MOYENNE_facteur_I_PID
  /// OUTPUT : GLOB_FACTEUR_linearite_plateau_inspi
  /// OUTPUT : GLOB_index_pas_stepper
  /// OUTPUT : GLOB_duree_TOTAL_theorique_US

  /// BLOC_PID 0 FIN
  /// BLOC_PID 0 FIN
  /// BLOC_PID 0 FIN

  printf("------------------ compute_PID_factors FIN\n\r");

  float    V_must_be_zero = get_cycle_VTe_mL();
  float    VTe = get_cycle_VTi_mL() - V_must_be_zero; /// CONTROLE GENERAL IHM
  float    pourcentage_erreur = ( V_must_be_zero ) / VTe; /// MODIF_POST_COMMIT
  printf("----ERR\t %i\t %i\t LinACCEL %ims\t VTi %i VTe %i Verr_Acc %i ->deberr %i pmill %i\tmoyPIDI:\t %i\n",
    (int)(GLOB_MOYENNE_erreur*1000),
    (int)(GLOB_FACTEUR_linearite_plateau_inspi*1000),
    (int)GLOB_timecode_ms_full_speed,
    (int)(get_cycle_VTi_mL()*1000),
    (int)(VTe*1000),
    (int)(GLOB_Volume_erreur_phase_accel*1000),
    (int)(GLOB_debit_from_error_slm*1000),
    (int)(pourcentage_erreur*1000),
    (int)(GLOB_MOYENNE_facteur_I_PID*1000*1000));

  return GLOB_index_pas_stepper-1;
} /// fin de 		compute_PID_factors()

/// OUPUT ETAPE 1 Analyse error : dans l'ordre
// TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit
// TAB_volume_slm_calib[ index_segt ] .MODE_Panique
// GLOB_MOYENNE_erreur
// GLOB_MOYENNE_facteur_I_PID
// GLOB_FACTEUR_linearite_plateau_inspi
// TAB_volume_slm_calib[ index_segt ] .PID_i_factor
// TAB_volume_slm_calib[ index_SEGMENT_max_error_en_cours ] .max_error_checked
// /// foireux a virer GLOB_debit_from_error_slm

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
// 
//


const	uint16_t		CONST_calib_90_lpm[ 64 ] =  { 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 396,
 6880,
 9719,
 11796,
 13742,
 15204,
 16127,
 16951,
 17812,
 18751,
 19799,
 20772,
 22135,
 23859,
 25575,
 27090,
 28597,
 29552,
 29930,
 30467,
 31443,
 32356,
 31619,
 30310,
 29078,
 30432,
 32071,
 34260,
 35808,
 34858,
 35594,
 35410,
 35322,
 34800,
 32612,
 29158};


const	uint16_t		CONST_calib_80_lpm[ 64 ] =  { 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 2429,
 5293,
 7380,
 8884,
 10405,
 11656,
 12751,
 13808,
 14828,
 15860,
 16767,
 17712,
 18402,
 19187,
 20021,
 20973,
 22110,
 23353,
 24763,
 26078,
 27207,
 28096,
 28807,
 29391,
 29732,
 30098,
 30395,
 30438,
 30144,
 29546,
 29622,
 30451,
 32398,
 34527,
 35113,
 35085,
 34193,
 33609,
 33098,
 33154,
 30813};


const	uint16_t		CONST_calib_70_lpm[ 64 ] =  { 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 2362,
 4018,
 5608,
 6908,
 8125,
 9160,
 10146,
 11114,
 12015,
 12912,
 13938,
 15075,
 16158,
 17116,
 18091,
 18758,
 19397,
 19853,
 20476,
 21428,
 22506,
 23718,
 25013,
 26164,
 27058,
 27789,
 28232,
 28539,
 28778,
 28860,
 29030,
 28984,
 28593,
 28557,
 29455,
 31099,
 32561,
 32852,
 32777,
 32697,
 32893,
 31917,
 31410,
 29476};


const	uint16_t		CONST_calib_60_lpm[ 64 ] = { 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 734,
 2260,
 3974,
 5663,
 6796,
 7757,
 8584,
 9398,
 10291,
 11244,
 12154,
 12982,
 13782,
 14597,
 15503,
 16331,
 17226,
 18188,
 18804,
 19164,
 19664,
 20039,
 20776,
 21886,
 23566,
 25144,
 26464,
 27506,
 28234,
 28828,
 29230,
 29424,
 29366,
 29394,
 29423,
 28926,
 28590,
 29691,
 32234,
 34177,
 33859,
 32419,
 31906,
 32500,
 32612,
 32650,
 31415};
 
 
const	uint16_t		CONST_calib_50_lpm[ 64 ] =  { 53,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 575,
 2121,
 3949,
 5185,
 5926,
 6593,
 7206,
 7754,
 8306,
 8759,
 9189,
 9814,
 10602,
 11573,
 12740,
 14101,
 15456,
 16581,
 17450,
 18109,
 18708,
 19352,
 19771,
 19940,
 20131,
 20636,
 21630,
 22810,
 24071,
 25079,
 25931,
 26610,
 27090,
 27458,
 27610,
 27777,
 27845,
 27914,
 28023,
 28144,
 28249,
 29715,
 31313,
 31503,
 30266,
 29530,
 29036,
 29029,
 29469,
 29467,
 29482};

 
const	uint16_t		CONST_calib_40_lpm[ 64 ] =   { 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 741,
 3209,
 4903,
 5584,
 6191,
 6470,
 6652,
 6878,
 7216,
 7751,
 8451,
 9112,
 9847,
 10435,
 10918,
 11323,
 11883,
 13093,
 14911,
 17106,
 18818,
 19567,
 19949,
 20444,
 20862,
 20852,
 20577,
 20537,
 21388,
 22974,
 24688,
 25944,
 26691,
 27251,
 27654,
 27818,
 27970,
 27982,
 27999,
 27973,
 27947,
 28128,
 28332,
 29504,
 32277,
 32926,
 31188,
 30194,
 30224,
 31090,
 32831,
 30900,
 30197,
 31439};


const	uint16_t		CONST_calib_30_lpm[ 64 ] =  { 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1,
 1842,
 3308,
 4587,
 5781,
 6883,
 7183,
 7074,
 6941,
 7069,
 7294,
 7772,
 8354,
 8937,
 9497,
 9964,
 10409,
 10851,
 11415,
 12099,
 12740,
 13344,
 14017,
 14996,
 16398,
 17726,
 18638,
 18999,
 19076,
 19074,
 19141,
 19503,
 20736,
 22738,
 24509,
 25659,
 26364,
 26742,
 27017,
 27164,
 27234,
 27203,
 27280,
 27311,
 27487,
 28066,
 29418,
 33886,
 31423,
 27937,
 39274,
 38688,
 17751,
 19236,
 23060,
 21660,
 19870};


const	uint16_t		CONST_calib_20_lpm[ 64 ] =  { 1,
 1,
 1,
 1,
 1,
 1,
 1934,
 3164,
 3816,
 4035,
 4242,
 4521,
 4975,
 5538,
 6199,
 7051,
 7896,
 8615,
 9284,
 9685,
 9888,
 10159,
 10585,
 10767,
 10940,
 11246,
 11771,
 12417,
 13205,
 13977,
 14953,
 16186,
 17207,
 17920,
 18113,
 18045,
 18342,
 18744,
 19120,
 19830,
 21521,
 23224,
 24403,
 25169,
 25522,
 25810,
 25997,
 26345,
 26615,
 26660,
 26835,
 27032,
 27649,
 28894,
 31035,
 26717,
 29071,
 35135,
 31022,
 23239,
 23240,
 22302,
 20480,
 17701};



const	uint16_t		CONST_calib_10_lpm[ 64 ] =  { 1,
 1,
 1,
 1,
 1,
 1,
 1752,
 2832,
 3061,
 3079,
 3204,
 3497,
 3806,
 4258,
 4650,
 5054,
 5532,
 7187,
 14130,
 11883,
 52,
 2986,
 6720,
 8023,
 8238,
 8546,
 8956,
 9480,
 10201,
 10945,
 11699,
 12349,
 13300,
 14694,
 15482,
 14558,
 13337,
 14321,
 14807,
 15420,
 16084,
 17809,
 18989,
 20331,
 20677,
 21034,
 21279,
 21616,
 21597,
 21801,
 21870,
 22201,
 23024,
 23232,
 22737,
 29757,
 20406,
 29462,
 26219,
 13621,
 12471,
 16067,
 17276,
 15044};

