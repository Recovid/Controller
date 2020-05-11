#include "common.h"
#include "controller.h"
#include "breathing.h"
#include "platform.h"
#include <inttypes.h>
#include "../platforms/recovid_revB/HAL/CMSIS/Include/cmsis_gcc.h"
#include "config.h"

#include <compute_motor.h>
#include <math.h>



void		start_sampling_temps_moteur( uint32_t maintenant_sampling ){
	
	/// on demarre le sampling en IT
	__disable_irq();
	
		GLOB_index_TAB_dp_raw_temps_moteur = 0;
		GLOB_denom_TAB_dp_raw_temps_moteur = 0;
		GLOB_accumule_TAB_dp_raw_temps_moteur = 0;
		GLOB_accumule_TAB_TIMECODE_temps_moteur = 0;
		
		GLOB_TIMER_ms_debut_sampling_temps_moteur = maintenant_sampling;
		GLOB_TIMER_ms_fin_sampling_temps_moteur 		= maintenant_sampling + GLOB_duree_TOTAL_theorique_US / 1000 + DEBIT_OVERSAMPLING_TIME;
		GLOB_is_running_sampling_temps_moteur 				= true; /// on demarre l'acquisition en IT
		GLOB_TIMER_fin_accel_moteur_ms							= maintenant_sampling + GLOB_timecode_ms_full_speed;
		
	__enable_irq();
	/// on demarre le sampling en IT
	/// on demarre le sampling en IT
	
	return;
}

void		stop_sampling_temps_moteur( ){
	GLOB_is_running_sampling_temps_moteur = false;
	return;
}

					
					
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
){
							
		int64_t			Debit_moyen_dp_raw_SEGMENT = 0;
		// uint64_t		Timecode_moyen_Debit_SEGMENT = 0;
		int				denom = 0;
		
		if(
				TIMER_US_fin_segment / 1000 + TIME_SLICING_ERROR_fin <= GLOB_TIMER_fin_accel_moteur_ms
		){
			/// segment inutilisable car accel initiale
			return 0.0f;
		}
		
		/// d'abord on cherche la Paw du segment
		uint64_t		timecode_milieu_segment_MS = TIMER_US_debut_segment + (TIMER_US_fin_segment - TIMER_US_debut_segment)  / 2;
							timecode_milieu_segment_MS /= 1000;
							
		// uint8_t		Paw_milieu_segment = 0;
		// for( int index_TAB_sampl_DEBITS = 0;  index_TAB_sampl_DEBITS < GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED; index_TAB_sampl_DEBITS++ ){
			
			// if( timecode_milieu_segment_MS >= TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS ){
				// Paw_milieu_segment = TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].Paw;
				// break;
			// }
		// }
		
		#if		1 
		int		Time_slicing_DEBUT 	= TIME_SLICING_ERROR_debut__;
		int		Time_slicing___FIN 		= TIME_SLICING_ERROR_fin__;
		#else /// WIP : il y a une relation fine a Paw certainement : en tout cas a compliance faible, dc Paw elevee, on observe que la fin du segment oscille un peu
		int		Time_slicing_DEBUT 	= TIME_SLICING_ERROR_debut__ 	- Paw_milieu_segment / 5; if( Time_slicing_DEBUT < 0 ) Time_slicing_DEBUT = 0;
		int		Time_slicing___FIN 		= TIME_SLICING_ERROR_fin__ 		- Paw_milieu_segment / 5; if( Time_slicing___FIN < 10 ) Time_slicing___FIN = 10;
		#endif
		
		#if	1
		
			for( int index_TAB_sampl_DEBITS = 0;  index_TAB_sampl_DEBITS < GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED; index_TAB_sampl_DEBITS++ ){
			
				if(
								// TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS  >= TIMER_US_debut_segment / 1000 + Time_slicing_DEBUT
								TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS  >= timecode_milieu_segment_MS + Time_slicing_DEBUT
						&&	TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS >= GLOB_TIMER_fin_accel_moteur_ms
					// &&	TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS  <= TIMER_US_fin_segment
				){
					
					if( 
							// TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS  > TIMER_US_fin_segment / 1000 + Time_slicing___FIN
							TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS  > timecode_milieu_segment_MS + Time_slicing___FIN
						||	TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS  > GLOB_TIMER_ms_fin_sampling_temps_moteur
					){
						if( denom == 0 ){
							Debit_moyen_dp_raw_SEGMENT = TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].dp_raw;
							denom = 1;
							break; /// cdtion de sortie
						} else {
							break; /// cdtion de sortie
						}
					}

					Debit_moyen_dp_raw_SEGMENT 					+= TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].dp_raw;
					// Timecode_moyen_Debit_SEGMENT 	+= TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS;
					
					denom++;
				}
			} /// fin du for 
			
		#else
			
				int32_t	meilleur_ecart = 1000 * 1000;
				int			meilleur_index_trouved = ( GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED ) / 2; /// milieu par defaut
				
				for( int index_TAB_sampl_DEBITS = 0;  index_TAB_sampl_DEBITS < GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED; index_TAB_sampl_DEBITS++ ){
					
					int32_t 		ecart;
					// if( 
							// timecode_milieu_segment_MS > TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS + Time_slicing___FIN
					// ){
						ecart = (int32_t)timecode_milieu_segment_MS + Time_slicing___FIN - TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS;
					// } else {
						// ecart = timecode_milieu_segment_MS - TAB_dp_raw_temps_moteur[ index_TAB_sampl_DEBITS ].timecode_sample_MS;
					// }
					
					if( abs( ecart ) < meilleur_ecart ){
						meilleur_ecart = abs( ecart );
						meilleur_index_trouved = index_TAB_sampl_DEBITS;
					}
					
				}
				/// meilleur_index_trouved
				
				Debit_moyen_dp_raw_SEGMENT = TAB_dp_raw_temps_moteur[ meilleur_index_trouved ].dp_raw;
				
				return -(float)( Debit_moyen_dp_raw_SEGMENT / 105.0f );
				// denom = 1;
			
		#endif
		
		if(
					index_segt 		< NBR_SEGMENTS_CALIBRATION
			&&	index_segt 		>= NBR_SEGMENTS_CALIBRATION - 3
			&&	denom 			<= 0
		){
			
			uint32_t		timecode_le_plus_proche = 0;
			
			/// TIME SLICING impossible....
			for( int index_TAB_sampl_DEBITS = ( GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED ) - 1;  index_TAB_sampl_DEBITS >= 0; index_TAB_sampl_DEBITS-- ){
			
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
			// brth_printf( "\n\r\n\r\n\r\n\r\n\r--- PANIQUE SGT %i : pas de sample debit within %i !!! %i sample at time %u us : Paw %u\n\r\n\r\n\r\n\r\n\r\n\r", 
			brth_printf( "--- PANIQUE SGT %i : pas de sample debit within %i !!! %i sample at time %u us\n\r", 
																																																			index_segt,
																																																			GLOB_index_TAB_dp_raw_temps_moteur,
																																																			denom,
																																																			(uint32_t)TIMER_US_fin_segment - TIMER_US_debut_segment
																																																			// Paw_milieu_segment
																																																		);
																																																		
			
			#if	1 /// ON SAMPLE L'ECHANTILLON LE PLUS PROCHE : la frequence d'echantillonage n'est pas stable...
			
				uint32_t	meilleur_ecart = 1000 * 1000;
				int			meilleur_index_trouved = ( GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED ) / 2; /// milieu par defaut
				
				for( int index_TAB_sampl_DEBITS = 0;  index_TAB_sampl_DEBITS < GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED; index_TAB_sampl_DEBITS++ ){
					
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
						
void reset_Vol_error_phase_accel(){
	GLOB_Volume_erreur_phase_accel = -12345.0f; /// sera remesured ds interrupt i2c
	return;
}

bool		load_default_calib_from_debit(	float debit_consigne_slm ){
	
	brth_printf( "----> load_default_calib_from_debit %i\n\r", (int)debit_consigne_slm );
	
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
	
	
	
	
	brth_printf( "erreur load_default_calib_from_debit\n\r" );
	return false;
}




bool		init_variables_PID(
															float		debit_consigne_slm
){
	
			/// On arrete + reinit sampling variables : ne demarrera pas le sampling par defaut ds TAB_dp_raw_temps_moteur
		__disable_irq();
		
				// GLOB_TIMER_ms_debut_sampling_temps_moteur = 1;
				// GLOB_TIMER_ms_fin_sampling_temps_moteur = 0;
				GLOB_is_running_sampling_temps_moteur 	= false; /// on arrete sampling en IT
				GLOB_index_TAB_dp_raw_temps_moteur = 0;
				GLOB_denom_TAB_dp_raw_temps_moteur = 0;
				GLOB_accumule_TAB_dp_raw_temps_moteur = 0;
				GLOB_accumule_TAB_TIMECODE_temps_moteur = 0;
				
		__enable_irq();
		
		/// ou bien : 
		// stop_sampling_temps_moteur( ); /// GLOB_is_running_sampling_temps_moteur
		
		
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
			TAB_volume_slm_calib[ i ] .CALIB_result_volume_segment 		= 0;
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
		
		#define					NBR_PAS_MOTEUR_MAX														MAX_MOTOR_STEPS  /// 4800 //4480
		#define					NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE				( (float)NBR_PAS_MOTEUR_MAX / NBR_SEGMENTS_CALIBRATION )
		
		/// todo WIP struct pr globales vraies, et arguments compute_PID_factors pr usage uniqt ds boucle breathing
		/// todo WIP struct pr globales vraies, et arguments compute_PID_factors pr usage uniqt ds boucle breathing
		GLOB_is_first_guess_from_abaques 						= true;
		GLOB_index_premier_segment_FULL_SPEED 	= 0;
		GLOB_NBR_de_cycles_before_auto_crash_test 	= 0; /// 0 pr desactiver le test crash
		GLOB_NBR_pas_moteur_par_segment 				= floor( NBR_PAS_MOTEUR_par_SEGMENT_ANALYSE );
		GLOB_FACTEUR_linearite_plateau_inspi 			= 0;
		GLOB_MOYENNE_facteur_I_PID 							= 0;
		GLOB_MOYENNE_erreur 										= 0;
		GLOB_index_pas_stepper 										= 0;
		GLOB_duree_TOTAL_theorique_US 						= 0;
		GLOB_debit_from_error_slm 									= 0;
		GLOB_Volume_erreur_phase_accel 						= -12345.0f;
		
		/// todo WIP struct pr globales vraies, et arguments compute_PID_factors pr usage uniqt ds boucle breathing
		/// todo WIP struct pr globales vraies, et arguments compute_PID_factors pr usage uniqt ds boucle breathing
		
		
		/// reinit :
		reset_Vol_error_phase_accel(); /// GLOB_Volume_erreur_phase_accel = -12345.0f; /// sera remesured ds interrupt i2c
		
		brth_printf( "----> init_variables_PID %i\n\r", (int)( debit_consigne_slm ) );
		
		/// charge un programme de 		TAB_volume_slm_calib[ index_segt ] .CALIB_result_volume_segment 		par defaut, ok a vide, la PID fera le reste
		if( load_default_calib_from_debit( debit_consigne_slm ) == false ){
			hardfault_CRASH_ME();				 /// gasp ;)
		}
	
}





bool			compute_PID_factors(

																	float 			debit_consigne_slm, 		/// GLOB_debit_from_error_slm 		sera ajouted ds corps function, mais GLOB_Volume_erreur_phase_accel a passer en argument
																	uint32_t 		Ti 										/// ms
																	// GLOB_Volume_erreur_phase_accel
																	/// BLOC_PID 0 DEBUT
																	/// BLOC_PID 0 DEBUT
																	/// BLOC_PID 0 DEBUT

																	/// INPUT : debit_consigne_slm
){
	brth_printf( "------------------ compute_PID_factors DEBUT %i lpm %i ms\n\r", (int)debit_consigne_slm, (int)Ti );
	
						float				duree_ACCEL = 0;
						float				duree_FULL_SPEED = 0;
						// float				GLOB_FACTEUR_linearite_plateau_inspi;
						// float				GLOB_MOYENNE_facteur_I_PID = 0;
						// float				GLOB_MOYENNE_erreur 				= 0;
						
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
							
						if( GLOB_index_TAB_dp_raw_temps_moteur > 0 ){ /// on a des datas dispo
								
								#if				DEBUG_PRINTF_CYCLE_PID			==			1 /// ---DEBUT sampling data
								brth_printf( "---DEBUT sampling data %i\n\r", GLOB_index_TAB_dp_raw_temps_moteur /  DIVISEUR_NBR_VALEURS_SAMPLED );
								
									uint32_t			maintenant = get_time_ms() ;
									
									for( int	index_TAB = 0; index_TAB < GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED; index_TAB++ ){
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
								uint32_t		index_step_moteur_en_cours = 0; /// pr analyse ds 	g_motor_steps_us[ ]	
								uint32_t		TIMER_US_debut_segment 	= GLOB_TIMER_ms_debut_sampling_temps_moteur * 1000;
								uint32_t		TIMER_US_fin_segment 		= TIMER_US_debut_segment + TAB_volume_slm_calib[ 0 ] .CALIB_duree_sgt_US;
								uint32_t		DUREE_Totale_cycle__controle_Ti = 0; /// MODIF_POST_COMMIT
								
								
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
									

									#if		ENABLE_GESTION_Ti		==		1
									if( DUREE_Totale_cycle__controle_Ti >  Ti*1000 ){ /// on vient de depasser Ti !!! cut !! : MODIF_POST_COMMIT
										
										// brth_printf( "------------------------------------------------------------> break deb err : %i > %i\n\r", 
																																															// (int)DUREE_Totale_cycle__controle_Ti,
																																															// (int)Ti
																																														// ); /// MODIF_POST_COMMIT
										
										TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit = 0;
										TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US = 0;
										// super_break = true;
										continue;
									}
									#endif
									
									
									/// ON FAIT LA MOYENNE : 	ATTENTION  INSTABLE CAR FREQUENCE ECHANTILLONAGE CAPTEUR PAS FIXE...
									float		Debit_slm_segment = Segment_error_time_sliced(
																																			/// INPUT : TAB_dp_raw_temps_moteur
																																			(int)index_segt,
																																			(uint32_t)TIMER_US_debut_segment, 							/// INPUT : TIMER_US_debut_segment
																																			(uint32_t)TIMER_US_fin_segment, 									//// INPUT : TIMER_US_fin_segment
																																			(uint32_t)TIME_SLICING_ERROR_debut, 					/// INPUT : TIME_SLICING_ERROR_debut
																																			(uint32_t)TIME_SLICING_ERROR_fin, 							/// INPUT : TIME_SLICING_ERROR_fin
																																			/// important que ces 2 variables soient JUSTES et mise à jour !!!
																																			(uint64_t)GLOB_TIMER_fin_accel_moteur_ms, 						// GLOB_TIMER_fin_accel_moteur_ms = 1;
																																			(uint64_t)GLOB_TIMER_ms_fin_sampling_temps_moteur				// GLOB_TIMER_ms_fin_sampling_temps_moteur = 0;
																																	);

									
									
									/// MAJ 		ERREUR 
									/// MAJ 		ERREUR 
									/// MAJ 		ERREUR 
									TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit = 
																																							Debit_slm_segment
																																						-	(
																																								debit_consigne_slm
																																							#if			USE_PID_VENTILATION_Verreur_accel		==		1
																																							+	GLOB_debit_from_error_slm
																																							#endif
																																							)
																																					;
									/// MAJ 		ERREUR 
									/// MAJ 		ERREUR 
									/// MAJ 		ERREUR 
																																			
									// brth_printf( "---deb mes %i Err %i\n\r", (int)( Debit_slm_segment * 1000 ), (int)( TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit * 1000 ) );
									
									
									/// on recupere l'erreur du SEGMENT
									float		ERREUR_debit_segment 		= TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit;
									float		DUREE_segment_US			= TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US;
									
									/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
									/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
									/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
									///
									/// ETAPE 2 : on check l'état de la commande MOTEUR		+		on applique la PID sur l'erreur du segment
									
									/// on va tout recalculer :
									
									#if	0
									// TAB_volume_slm_calib[ index_segt ] .is_vitesse_max_SEGMENT						= false;
									// TAB_volume_slm_calib[ index_segt ] .is_accel_max_SEGMENT 						= false;
									// TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt											= true;
									TAB_volume_slm_calib[ index_segt ] .max_error_checked									= false;
									
									{ /// ANALYSE sur cycle precedant :
									
										/// on cherche si on a atteinds ACCEL_max
										bool		is_vitesse_max = true;
										bool		is_accel__max = true;
										
										for( int		index_STEP_moteur = 0; index_STEP_moteur < GLOB_NBR_pas_moteur_par_segment; index_STEP_moteur++ ){
											// index_step_moteur_en_cours += GLOB_NBR_pas_moteur_par_segment;
											/// MAJ indispensable 
											/// MAJ indispensable 
											index_step_moteur_en_cours++;
											/// MAJ indispensable 
											/// MAJ indispensable 
											
											int			duree_step_NM1 = (int)g_motor_steps_us[ index_step_moteur_en_cours - 1 ] ;
											int			duree_step_N 		= (int)g_motor_steps_us[ index_step_moteur_en_cours      ] ;
											int			accel_en_cours 	= duree_step_NM1 - duree_step_N;
											if(
													( accel_en_cours > 0 		&& accel_en_cours >= MAX_ACCEL_MOTEUR_POSITIF )
												||	( accel_en_cours <= 0 	&& accel_en_cours <= -MAX_ACCEL_MOTEUR_NEGATIF )
											){
												// TAB_volume_slm_calib[ index_segt ] .is_accel_max_SEGMENT = false;
												is_accel__max = false; /// faux : a virer
											}
											
											
											if(
													(int64_t)g_motor_steps_us[ index_step_moteur_en_cours ]  > DUREE_STEP_MIN_MOTOR
											){
												// TAB_volume_slm_calib[ index_segt ] .is_vitesse_max_SEGMENT = false;
												is_vitesse_max = false; /// faux : a virer
											}
											/// no break allowed !!!
											
										} /// fin du for GLOB_NBR_pas_moteur_par_segment
										/// OUTPUT :
										// TAB_volume_slm_calib[ index_segt ] .is_accel_max_SEGMENT 			= is_accel__max; /// par defaut
										// TAB_volume_slm_calib[ index_segt ] .is_vitesse_max_SEGMENT			= is_vitesse_max; /// par defaut
										// TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt = false;
									}
									#endif
									
							
									/// a virer toute cette section :
									/// puis on recupere l'erreur sur le segment
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
										
										index_step_moteur_en_cours += GLOB_NBR_pas_moteur_par_segment; /// pr analyse sgt suivant
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
										// index_step_moteur_en_cours += GLOB_NBR_pas_moteur_par_segment; /// pr analyse sgt suivant
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
										
										/// MODIF_POST_COMMIT
/* 										if(
												DUREE_segment_US		< GLOB_NBR_pas_moteur_par_segment * DUREE_STEP_MIN_MOTOR /// cad SEGMENT a vitesse max
										){
											brth_printf( "---ERREUR !!! duree %i < duree min %i from %i\n\r", 
																																									(int)( DUREE_segment_US ),
																																									GLOB_NBR_pas_moteur_par_segment * DUREE_STEP_MIN_MOTOR,
																																									GLOB_NBR_pas_moteur_par_segment 
																																								);
											/// ALARME et PANIQUE : WIP
											
											TAB_volume_slm_calib[ index_segt ] .MODE_Panique = true;
											// TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt = false;
											// TAB_volume_slm_calib[ index_segt ] .is_vitesse_max_SEGMENT = true;
											
											/// MAJ indispensable 
											/// MAJ indispensable 
											// index_step_moteur_en_cours += GLOB_NBR_pas_moteur_par_segment; /// pr analyse sgt suivant
											/// MAJ indispensable 
											/// MAJ indispensable 
											
										} else  */
											
										if(
												DUREE_segment_US		== GLOB_NBR_pas_moteur_par_segment * DUREE_STEP_MIN_MOTOR /// cad SEGMENT a vitesse max
										){
											/// SEGMENT non tunable
											// TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt = false;
											// TAB_volume_slm_calib[ index_segt ] .is_vitesse_max_SEGMENT = true;
											
											/// MAJ indispensable 
											/// MAJ indispensable 
											// index_step_moteur_en_cours += GLOB_NBR_pas_moteur_par_segment; /// pr analyse sgt suivant
											/// MAJ indispensable 
											/// MAJ indispensable 
											
										} else {
											
											
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
									
									#if		ENABLE_GESTION_Ti		==		1
									DUREE_Totale_cycle__controle_Ti += DUREE_segment_US; ///	MODIF_POST_COMMIT		= TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US;;
									#endif
									
									
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
								// float			GLOB_FACTEUR_linearite_plateau_inspi;
							
								// float			GLOB_MOYENNE_facteur_I_PID = 0; 
								float			SOMME_ABS_erreur			= 0;
								// float			GLOB_MOYENNE_erreur 				= 0;
								float			MAX_erreur 							= -123456789;
								float			MIN_erreur 							= 123456789;
								int				denom 									= 0;
								
								for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){ /// ds 		TAB_volume_slm_calib
									
									
									if(
										TAB_volume_slm_calib[ index_segt ].is_vitesse_max_SEGMENT == false /// on ne fait la moyenne que de ce qui est sur le plateau de debit : on ne tient pas compte de la phase d'accel initiale
										/// on pourrait aussi utiliser 		GLOB_timecode_ms_full_speed		--->		equivalent
									){
										SOMME_ABS_erreur				+= TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit * TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit;
										
										GLOB_MOYENNE_erreur 					+= TAB_volume_slm_calib[ index_segt ] .PID_ERREUR_Debit;
										GLOB_MOYENNE_facteur_I_PID 	+= TAB_volume_slm_calib[ index_segt ] .PID_i_factor;
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
								GLOB_MOYENNE_erreur 					/= denom;
								GLOB_MOYENNE_facteur_I_PID 	/= denom;
								SOMME_ABS_erreur				/= denom;
								
								/// linearite du plateau
								GLOB_FACTEUR_linearite_plateau_inspi = fabs( GLOB_MOYENNE_erreur - MAX_erreur ) / PROPORTION_FACTEUR_LINEARITE_PLATEAU; /// ajustement fin		PID_i_factor
								// GLOB_FACTEUR_linearite_plateau_inspi = ( fabs( GLOB_MOYENNE_erreur ) - SOMME_ABS_erreur ) / ( PROPORTION_FACTEUR_LINEARITE_PLATEAU / 6 ); /// ajustement fin		PID_i_factor

								/// ecart moyen a consigne
								if( GLOB_MOYENNE_erreur > 0 ){ /// MODE_PANIQUE : overshoot on cherche a revenir en leger negatif
									GLOB_FACTEUR_linearite_plateau_inspi *=  GLOB_MOYENNE_erreur * FACT_PROPORTIONNEL_in_LINEARITE_POSITIF;
									if( GLOB_FACTEUR_linearite_plateau_inspi < MINI_FACTEUR_LINEARITE_PLATEAU*4 ) 	GLOB_FACTEUR_linearite_plateau_inspi = MINI_FACTEUR_LINEARITE_PLATEAU*4;
									
								} else {
									GLOB_FACTEUR_linearite_plateau_inspi *= -GLOB_MOYENNE_erreur * FACT_PROPORTIONNEL_in_LINEARITE_NEGATIF;
									
									if( GLOB_FACTEUR_linearite_plateau_inspi < MINI_FACTEUR_LINEARITE_PLATEAU ) 	GLOB_FACTEUR_linearite_plateau_inspi = MINI_FACTEUR_LINEARITE_PLATEAU;
								}
								
								
								if( GLOB_FACTEUR_linearite_plateau_inspi > MAXI_FACTEUR_LINEARITE_PLATEAU ) 	GLOB_FACTEUR_linearite_plateau_inspi = MAXI_FACTEUR_LINEARITE_PLATEAU;
								
								/// OUTPUT : GLOB_MOYENNE_erreur
								/// OUTPUT : MAX_erreur
								/// OUTPUT : GLOB_FACTEUR_linearite_plateau_inspi
								
								
								/// on applique tout ca :
								/// on applique tout ca :
								/// on applique tout ca :
								for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){ /// ds 		TAB_volume_slm_calib
								
									if( TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt == false ){
										
									} else { /// cad TAB_volume_slm_calib[ index_segt ] .PID_tunable_sgt == true
										/// on calcule la PID
										float		increment_INTEGRALE   	= 	
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
																																		
																																		* 	GLOB_FACTEUR_linearite_plateau_inspi /// va diminuer l'impact de l'integrale a mesure que le plateau apparait
																																	; 
										
										if( increment_INTEGRALE > MAX_INCREMENT_INTEGRALE_ACCUMULEE_PID_POSITIF ){
											increment_INTEGRALE = MAX_INCREMENT_INTEGRALE_ACCUMULEE_PID_POSITIF;
											#if			NIVEAU_VERBOSE_DEBUG		>=				1
												if( index_segt >=  GLOB_index_sgt_full_speed ){
													brth_printf( "\t\t---limitation sgt ++ %i : %i\n\r", index_segt, (int)( increment_INTEGRALE* 1000 ) );
												}
											#endif
										}
										else if( increment_INTEGRALE < -MAX_INCREMENT_INTEGRALE_ACCUMULEE_PID_NEGATIF ){
											increment_INTEGRALE = -MAX_INCREMENT_INTEGRALE_ACCUMULEE_PID_NEGATIF;
											#if			NIVEAU_VERBOSE_DEBUG		>=				1
												if( index_segt >=  GLOB_index_sgt_full_speed ){
													brth_printf( "\t\t---limitation sgt -- %i : %i\n\r", index_segt, (int)( increment_INTEGRALE* 1000 ) );
												}
											#endif
										}
										
										TAB_volume_slm_calib[ index_segt ] .PID_i_factor += increment_INTEGRALE;
											
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
								
							} /// fin du for 		NBR_SEGMENTS_CALIBRATION
							
							/// Nouvel algo PID
							/// Nouvel algo PID
							/// Nouvel algo PID
							/// Nouvel algo PID
							

							
							/// on limite les facteurs de PID PID_i_factor : pas plus de 20000 !!!
							/// on limite les facteurs de PID PID_i_factor : pas plus de 20000 !!!		WIP	:		retuner avec nouveaux facteurs define : imperatif securite patient : tube bouched...
							for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){
								
								if( 
											TAB_volume_slm_calib[ index_segt ] .PID_i_factor > MAX_INTEGRALE_ACCUMULEE_PID
									// &&	TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == false
								){
									if(
											// TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == true
											index_segt < GLOB_index_premier_segment_FULL_SPEED
									){
										TAB_volume_slm_calib[ index_segt ] .PID_i_factor = 0; /// par defaut, on garde ceinture brettelles
									} else {
										TAB_volume_slm_calib[ index_segt ] .PID_i_factor = MAX_INTEGRALE_ACCUMULEE_PID;
										#if			NIVEAU_VERBOSE_DEBUG		>=				1
										brth_printf( "---limitation++ %i : %i\n\r", index_segt, (int)( TAB_volume_slm_calib[ index_segt ] .PID_i_factor * 1000 ) );
										#endif
									}
									
								}
								else 
								if( 
											TAB_volume_slm_calib[ index_segt ] .PID_i_factor < -MAX_INTEGRALE_ACCUMULEE_PID
									// &&	TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == false
								){
									
									if(
											// TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT == true
											index_segt < GLOB_index_premier_segment_FULL_SPEED
									){
										TAB_volume_slm_calib[ index_segt ] .PID_i_factor = 0; /// par defaut, on garde ceinture brettelles
									} else {
										TAB_volume_slm_calib[ index_segt ] .PID_i_factor = -MAX_INTEGRALE_ACCUMULEE_PID;
										#if			NIVEAU_VERBOSE_DEBUG		>=				1
											brth_printf( "---limitation-- %i : %i\n\r", index_segt, (int)( TAB_volume_slm_calib[ index_segt ] .PID_i_factor * 1000 ) );
										#endif
									}
									
								}
							}
							/// on limite les facteurs de PID PID_i_factor : pas plus de 20000 !!!
							/// on limite les facteurs de PID PID_i_factor : pas plus de 20000 !!!
							
							
							/// LISSAGE param PID
							/// LISSAGE param PID			a virer : pas pertinent
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
							brth_printf( "---FIN   sampling data %i\n\r", GLOB_index_TAB_dp_raw_temps_moteur /  DIVISEUR_NBR_VALEURS_SAMPLED );
							#endif
						}
						else {
							
							brth_printf( "-- No data for PID %i on %i\n\r", GLOB_index_TAB_dp_raw_temps_moteur / DIVISEUR_NBR_VALEURS_SAMPLED, NBR_VALEURS_TAB_debits_temps_moteur * DIVISEUR_NBR_VALEURS_SAMPLED );
							/// Alarm(); /// WIP 
						}
							

						
						

						/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// ETAPE 2 : on prepare la commande moteur
						/// ETAPE 2 : on prepare la commande moteur
						/// ETAPE 2 : on prepare la commande moteur
						
						#if	USE_PID_VENTILATION_Verreur_accel		==		1 /// WIP : 
								/// MAJ : WIP a tester
								#if	0
								if( 
											GLOB_Volume_erreur_phase_accel != -12345.0f
									&&	GLOB_timecode_ms_full_speed	!= 0
								){ /// MODIF_POST_COMMIT
									GLOB_debit_from_error_slm = -GLOB_Volume_erreur_phase_accel / GLOB_timecode_ms_full_speed; /// on pourrait aussi estimer direct l'aire du triangle defini par la pente d'accel max, ou laisser faire la PID...
								
								
									brth_printf( "--V %i deb %i : tim %i\n\r", 
																								(int)( GLOB_Volume_erreur_phase_accel * 1000 ),
																								(int)( GLOB_debit_from_error_slm * 1000 ),
																								GLOB_timecode_ms_full_speed 
																							);
								}
								// else {
									// GLOB_debit_from_error_slm = 0;
								// }
								#endif
						#else
								wait_ms( 4 ); /// on laisse respirer le CPU
						#endif


						GLOB_index_pas_stepper = 0;
						GLOB_duree_TOTAL_theorique_US = 0;
						int32_t	Last_duree_step_moteur_en_cours = DUREE_ARRET_MOTEUR_PR_ACCEL; /// pr gestion accel uniqt
						uint32_t	timecode_US_depuis_demarrage_moteur = 0;
						bool		is_still_ACCEL_phase = true; /// phase ACCEL au demarrage, avant FULL SPEED
						uint32_t 	DUREE_Totale_cycle = 0;
						
						// brth_printf( "---GLOB_NBR_pas_moteur_par_segment %i\n\r", GLOB_NBR_pas_moteur_par_segment );
						// brth_printf( "---NBR_SEGMENTS_CALIBRATION %i\n\r", NBR_SEGMENTS_CALIBRATION );
						
						///  todo : passer les 		is_accel_max_SEGMENT 		ici
						///  todo : passer les 		 is_vitesse_max_SEGMENT		ici
						for( int index_segt = 0; index_segt < NBR_SEGMENTS_CALIBRATION; index_segt++ ){ /// pour chaque segment, on calcule la duree pour assurer le debit consigne (CALIB_result_volume_segment -> debit)
							
							bool	is_accel_max_segt 		= true;
							bool	is_vitesse_max_segt 	= true;
						
							wait_ms( 1 ); /// on laisse du temps CPU pour la COM IHM
							// printf("COUCOU  %d  %d\n",  __LINE__,index_segt);
							
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
							// printf("COUCOU  %d  %d\n",  __LINE__,index_segt);
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
																																						+	GLOB_debit_from_error_slm /// todo : on accumule le VOL d'erreur pdt accel : A VENTILER SUR debit_consigne_slm
																																						#endif
																																						
																																					) * 60 * 1000 * 1000
																																	)
																																	

																														)
																								);
							// printf("COUCOU  %d  %d\n",  __LINE__,index_segt);
							#if		DEBUG_PRINTF_PR_CALIB_A_VIDE		==		1
								// brth_printf( "%i %i %i\n\r", index_segt, duree_segment_US, (int)( debit_consigne_slm ) );
								brth_printf( "%i\n\r",
																		(int)( VOLUME_segment_corriged *  1000 * 1000 )
																		
																); /// pratique pr creer les const_calibs.h		WIP	: a refaire avec nouveaux mors
							#endif
							
							uint32_t	duree_step_ideale_US = round( (float)duree_segment_US / GLOB_NBR_pas_moteur_par_segment ); /// linearised : todo : faire varier le nombre de pas POUR EVITER DE TOUT DEPLACER
							
							// printf("COUCOU  %d  %d\n",  __LINE__,index_segt);
							/// on s'apprete a recalculer :
							TAB_volume_slm_calib[ index_segt ].CALIB_US_avt_motor_full_speed  = 0;
							if( GLOB_is_first_guess_from_abaques == true ){ /// uniquement au premier tour : WIP : A VERIFIER, attention paleocode !!!!
								TAB_volume_slm_calib[ index_segt ].PID_tunable_sgt  = false;
							}
							// printf("COUCOU  %d  %d\n",  __LINE__,index_segt);
							/// on recalcule le futur programme moteur en tenant compte des accelerations et vit max : WIP : a passer en fonction
							/// on recalcule le futur programme moteur en tenant compte des accelerations et vit max : WIP : a passer en fonction
							/// on recalcule le futur programme moteur en tenant compte des accelerations et vit max : WIP : a passer en fonction
							
							/// INPUT : g_motor_steps_us
							/// INPUT : duree_step_ideale_US
							/// INPUT : Last_duree_step_moteur_en_cours en ref a MAJ
							/// INPUT : timecode_US_depuis_demarrage_moteur en ref a MAJ
							/// INPUT : duree_segment_US en ref a MAJ
							
							// printf("COUCOU  %d  %d\n",  __LINE__,index_segt);
							#if		ENABLE_GESTION_Ti		==		1
							bool		super_break = false;
							#endif
							uint32_t	duree_step_US;
							{ /// a passer en fonction : WIP : pas mal de globales a proprifier			commande_steps_moteur_from_duree_segment_US_ideal()
								duree_segment_US = 0;
								
								for( int nbr_pas_segment = 0; nbr_pas_segment < GLOB_NBR_pas_moteur_par_segment; nbr_pas_segment++ ){
									
										/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
										/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
										///
										/// on consitute la MAP DMA de controle moteur :
										/// gestion acceleration
											// printf("COUCOU  %d  %d\n",  __LINE__,index_segt);
											/// analyse vitesse
											/// analyse vitesse : ok coherent : permet de passer 			GLOB_timecode_ms_full_speed		et			is_still_ACCEL_phase
											if( duree_step_ideale_US <= DUREE_STEP_MIN_MOTOR ){
												/// vitesse impossible
												duree_step_ideale_US = DUREE_STEP_MIN_MOTOR;
											} else {
												/// MAJ :
												is_vitesse_max_segt = false; /// pr tout le segment du coup
											}
											// printf("COUCOU  %d  %d\n",  __LINE__,index_segt);
											
											/// sommes nous sur le plateau ??
											if( 
														is_still_ACCEL_phase 		== true
												&&	is_vitesse_max_segt 			== false
											){ /// on fixe le point de fin d'accel
												
												GLOB_index_sgt_full_speed 			= index_segt;
												GLOB_timecode_ms_full_speed 	= timecode_US_depuis_demarrage_moteur / 1000; /// ( timecode_us_next_echant + TAB_volume_slm_calib[ index_segt ] .CALIB_US_avt_motor_full_speed ) / 1000; 
												// brth_printf( "--------------------->  GLOB_timecode_ms_full_speed %u SET at segt %i\n\r", GLOB_timecode_ms_full_speed, index_segt );
												
												if( GLOB_is_first_guess_from_abaques ){
													GLOB_index_premier_segment_FULL_SPEED = index_segt; /// fixed une fois pour toute : le reste est optimisable : ca se discute e fixer ca au premier cycle... WIP
												}
												TAB_volume_slm_calib[ index_segt ].PID_tunable_sgt = true;
												
												is_still_ACCEL_phase = false; /// MAJ : 1 seul par segment
												// printf("COUCOU  %d  %d\n",  __LINE__,index_segt);
											}
											
											
																						// printf("COUCOU  %d  %d\n",  __LINE__,index_segt);
											/// analyse acceleration
											/// analyse acceleration
											int32_t 		Accel_moteur = (int32_t)Last_duree_step_moteur_en_cours - duree_step_ideale_US;
											int				MAX_accel_negatif 	= MAX_ACCEL_MOTEUR_NEGATIF;
											int				MAX_accel_positif 		= MAX_ACCEL_MOTEUR_POSITIF;
											
											if( index_segt >= GLOB_index_sgt_full_speed ){ /// on est sur le plateau, soyons + smooth...
												MAX_accel_negatif = MAX_ACCEL_MOTEUR_NEGATIF_ON_PLATEAU;
												MAX_accel_positif = MAX_ACCEL_MOTEUR_POSITIF_ON_PLATEAU;
											}
											
											if(
															-Accel_moteur > MAX_accel_negatif
											){
												/// phase d'acceleration intiale
												duree_step_US = Last_duree_step_moteur_en_cours + MAX_accel_negatif;
												TAB_volume_slm_calib[ index_segt ].CALIB_US_avt_motor_full_speed += duree_step_US;
											} else if(
															Accel_moteur > MAX_accel_positif
											){
												/// phase d'acceleration intiale
												duree_step_US = Last_duree_step_moteur_en_cours - MAX_accel_positif;
												TAB_volume_slm_calib[ index_segt ].CALIB_US_avt_motor_full_speed += duree_step_US;
												
												
											} else  {
												/// phase FULL SPEED
												duree_step_US = duree_step_ideale_US;
												
												if( GLOB_is_first_guess_from_abaques == true ){
													TAB_volume_slm_calib[ index_segt ].PID_tunable_sgt = true; /// par defaut, tous les autres segment restent tunables par la PID : WIP
												}
												
												/// MAJ :
												is_accel_max_segt = false; /// pr tout le segment du coup
											}
											
											Last_duree_step_moteur_en_cours = duree_step_US;
											timecode_US_depuis_demarrage_moteur += duree_step_US;
											
										/// on applique immediatement
										// if( TAB_volume_slm_calib[ index_segt ].CALIB_nbr_echant != 0 ){ /// bug dernier segt... WIP : paleocode crado ??? me rappelle plus sorry, a virer / tester
										
											// brth_printf( "---- GLOB_index_pas_stepper %i\n\r", GLOB_index_pas_stepper );
											g_motor_steps_us[ GLOB_index_pas_stepper++ ] = duree_step_US;
											
											duree_segment_US += duree_step_US;
											
										// }
											#if		ENABLE_GESTION_Ti		==		1
											if( round( (float)( DUREE_Totale_cycle + duree_segment_US ) / 1000 ) >  Ti ){ /// on vient de depasser Ti !!! cut !! : MODIF_POST_COMMIT
												
												brth_printf( "------------------------------------------------------------> break : %i\n\r", (int)round( (float)( DUREE_Totale_cycle + duree_segment_US ) / 1000 )  ); /// MODIF_POST_COMMIT
												
												super_break = true;
												break;
											}
											#endif
									
								} /// fin du for nbr_pas_segment
								/// on MAJ immediatemment pour ce segment :
								// brth_printf( "---AVANT %i\n\r", index_segt );

								TAB_volume_slm_calib[ index_segt ].is_accel_max_SEGMENT 		= is_accel_max_segt;
								TAB_volume_slm_calib[ index_segt ].is_vitesse_max_SEGMENT 	= is_vitesse_max_segt;
								/// OUTPUT : g_motor_steps_us
								/// OUTPUT : duree_segment_US, timecode_US_depuis_demarrage_moteur
								/// OUTPUT : passe les flags accel max vit max...
								/// OUTPUT fonction : a creer WIP :
								TAB_volume_slm_calib[ index_segt ] .CALIB_duree_sgt_US = duree_segment_US; /// RECALCULED SELON CONSIGNE
								
								// brth_printf( "---APRES %i\n\r", index_segt );
							} /// a passer en fonction
							/// on recalcule le futur programme moteur en tenant compte des accelerations et vit max : WIP : a passer en fonction
							/// on recalcule le futur programme moteur en tenant compte des accelerations et vit max : WIP : a passer en fonction
							/// on recalcule le futur programme moteur en tenant compte des accelerations et vit max : WIP : a passer en fonction
							
							DUREE_Totale_cycle 								+= duree_segment_US; /// WIP : doublon paleocode a virer ???
							GLOB_duree_TOTAL_theorique_US 		+= (uint32_t)duree_segment_US; /// WIP : doublon paleocode a virer ???
							
							#if	0
							brth_printf( "--sgt %i : step ideale %u duree_segment_US %u %u %u %u\n\r", 
																																			index_segt, 
																																			duree_step_ideale_US,
																																			duree_segment_US, 
																																			GLOB_duree_TOTAL_theorique_US, 
																																			duree_step_US, 
																																			GLOB_index_pas_stepper 
																																	);
							#endif
							
							#if		ENABLE_GESTION_Ti		==		1
							if( super_break == true ){ /// on vient de depasser Ti !!! cut !!
								break;
							}
							#endif
						} /// fin du for index_segt
						
						
						#if		ENABLE_LISSAGE_DECELERATION			==		1
						int	nbr_pas_restant = MAX_MOTOR_STEPS - GLOB_index_pas_stepper;
						
						int	decceleration = DECCELERATION_VALUE;
						int	nbr_pas_necessaires = ( 4096 - Last_duree_step_moteur_en_cours ) / decceleration;
						if( nbr_pas_restant < nbr_pas_necessaires ){
							decceleration = ( 4096 - Last_duree_step_moteur_en_cours ) / nbr_pas_restant;
						}
						
						brth_printf( "---pas restant %i\n\r", nbr_pas_restant );
						
						while( 
										nbr_pas_restant-- > 0
								&&	decceleration > 0
						){
							if( Last_duree_step_moteur_en_cours < 4096 ){
								decceleration += DECCELERATION_VALUE;
							} else {
								decceleration -= DECCELERATION_VALUE * 2;
							}
							
							Last_duree_step_moteur_en_cours += decceleration;
							g_motor_steps_us[ GLOB_index_pas_stepper++ ] = Last_duree_step_moteur_en_cours;
							DUREE_Totale_cycle 								+= Last_duree_step_moteur_en_cours; /// WIP : doublon paleocode a virer ??? /// MODIF_POST_COMMIT
							GLOB_duree_TOTAL_theorique_US 		+= (uint32_t)Last_duree_step_moteur_en_cours; /// WIP : doublon paleocode a virer ???
						}
						
						// while( 
										// nbr_pas_restant-- > 0
						// ){
							// g_motor_steps_us[ GLOB_index_pas_stepper++ ] = 0; /// MODIF_POST_COMMIT
						// }
						#endif
						
						// brth_printf( "---GLOB_NBR_pas_moteur_par_segment %i\n\r", GLOB_NBR_pas_moteur_par_segment );
						
						GLOB_is_first_guess_from_abaques = false;
						
						
						// if( 
								// GLOB_is_first_guess_from_abaques == true /// WIP : Vilain paleocode: je le laisse car pas d'acces a un recovid pour tester : mais a virer !!!!! pas fier ;)
							// &&	GLOB_timecode_ms_full_speed == 0
						// ){
							// brth_printf( "--------------------->  GLOB_timecode_ms_full_speed %u SET baaaad !!!\n\r", GLOB_duree_TOTAL_theorique_US / 1000);
							// GLOB_timecode_ms_full_speed = GLOB_duree_TOTAL_theorique_US / 1000;
							// GLOB_index_premier_segment_FULL_SPEED = NBR_SEGMENTS_CALIBRATION;
						// }
							
						
						
						#if	0 /// desespoir only
							for( int i_steps = 0; i_steps < GLOB_index_pas_stepper; i_steps++ ){
							// for( int i_steps = 0; i_steps < 200; i_steps++ ){
								
								brth_printf( "-- %i duree step %u \n\r", 
																									i_steps,
																									g_motor_steps_us[ i_steps ]
																									// NBR_SEGMENTS_CALIBRATION,
																									// (int)( round( GLOB_NBR_pas_moteur_par_segment ) )
																								);
																								
							}
						#endif
	/// reinit
	GLOB_Volume_erreur_phase_accel = -12345.0f; /// sera remesured ds interrupt i2c
	
	/// OUTPUT : GLOB_MOYENNE_facteur_I_PID
	/// OUTPUT : GLOB_FACTEUR_linearite_plateau_inspi
	/// OUTPUT : GLOB_index_pas_stepper
	/// OUTPUT : GLOB_duree_TOTAL_theorique_US

	/// BLOC_PID 0 FIN	
	/// BLOC_PID 0 FIN	
	/// BLOC_PID 0 FIN	
	
	brth_printf( "------------------ compute_PID_factors FIN\n\r" );
	
} /// fin de 		compute_PID_factors()

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













