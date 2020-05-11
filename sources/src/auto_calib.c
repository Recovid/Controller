#include "common.h"
#include "controller.h"
#include "breathing.h"
#include "platform.h"
#include <inttypes.h>
#include "../platforms/recovid_revB/HAL/CMSIS/Include/cmsis_gcc.h"
#include "config.h"

#include <compute_motor.h>
#include <math.h>

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


	  

			/// Proces AUTO CALIBRATION des volumes pour chaque segment du ballon ambu
			/// Proces AUTO CALIBRATION des volumes pour chaque segment du ballon ambu
			/// Proces AUTO CALIBRATION des volumes pour chaque segment du ballon ambu
			
			
			
			#if	0 /// Algo Adrien : auto calibration volumes pour chaque segment
			
					#define		DEBUG_wait_duree				0
					
					int32_t					vitesse_cste_moteur_US 															= VITESSE_DEMARRAGE_CALIB;
					int							nbr_tours_calib = NBR_ITERATION_CALIB;
					
					
					
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
						
						
						enterg_state(Insufflation);
						valve_inhale();
						
						/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						/// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						///
						/// DEBUT CALIBRATION
					
						
						float		duree_segment_analyse_US = vitesse_cste_moteur_US * GLOB_NBR_pas_moteur_par_segment;
						// EN REMPLACEMENT DE : compute_constant_motor_steps(d, _steps, g_motor_steps_us);
						
						uint64_t	GLOB_duree_TOTAL_theorique_US = 0;
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
							g_motor_steps_us[ i_step ] = duree_step_US;
							
							
							duree_SEGMENT_theorique_US += duree_step_US;
							if( 
										i_step > 0
								&&	( i_step % GLOB_NBR_pas_moteur_par_segment ) == 0
							){
								
								#if			DEBUG_wait_duree		==		1
										brth_printf("----todo Segt %i : duree : %u\n", index_segt, duree_SEGMENT_theorique_US );
								#endif
								
								TAB_volume_slm_calib[ index_segt++ ].CALIB_duree_sgt_US = duree_SEGMENT_theorique_US;
								
								/// MAJ
								duree_SEGMENT_theorique_US = 0;
							}
							
							GLOB_duree_TOTAL_theorique_US += duree_step_US;
							
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
							brth_printf( "-- %"PRIu32" %i\n\r", g_motor_steps_us[ i_step ], i_step  );
						}
						#endif
						
						motor_press( g_motor_steps_us, NBR_PAS_MOTEUR_MAX );
						
						/// on demarre le sampling en IT
						/// on demarre le sampling en IT ds TAB_dp_raw_temps_moteur
						
						uint32_t	maintenant = get_time_ms();
						
						__disable_irq();
						
							GLOB_index_TAB_dp_raw_temps_moteur = 0;
							GLOB_denom_TAB_dp_raw_temps_moteur = 0;
							GLOB_accumule_TAB_dp_raw_temps_moteur = 0;
							GLOB_accumule_TAB_TIMECODE_temps_moteur = 0;
							
							GLOB_TIMER_ms_debut_sampling_temps_moteur 	= maintenant;
							GLOB_TIMER_ms_fin_sampling_temps_moteur 		= maintenant + GLOB_duree_TOTAL_theorique_US / 1000 + DEBIT_OVERSAMPLING_TIME;
							GLOB_is_running_sampling_temps_moteur 		= true; /// on demarre l'acquisition en IT
							
						__enable_irq();
						
						brth_printf( "---sampling %u\n\r", GLOB_TIMER_ms_debut_sampling_temps_moteur - GLOB_TIMER_ms_debut_sampling_temps_moteur );
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
							
						enterg_state(Exhalation);
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
	
	
	
	
			