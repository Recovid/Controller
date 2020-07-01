#include "recovid_revB.h"
#include "platform.h"
#include"../../../src/adrien/const_calibs.h"

#include <math.h>

static TIM_HandleTypeDef *_motor_tim = NULL;


static volatile bool _moving;
static volatile bool _homing;
static volatile bool _home;
static volatile bool _limit_sw_A;
static volatile bool _limit_sw_B;
static volatile bool _active;

static void period_elapsed_callback(TIM_HandleTypeDef *tim);
static void do_motor_stop();
static void check_home();



bool init_motor(uint32_t home_step_us)
{
    if (_motor_tim == NULL)
    {
        _motor_tim = &motor_tim;
        // register IT callbacks
        HAL_TIM_RegisterCallback(_motor_tim, HAL_TIM_PERIOD_ELAPSED_CB_ID, period_elapsed_callback);

        _active = HAL_GPIO_ReadPin(MOTOR_ACTIVE_GPIO_Port, MOTOR_ACTIVE_Pin);

        _limit_sw_A = !HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin);
        _limit_sw_B = !HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin);
        check_home();
        HAL_TIM_Base_Start(_motor_tim);
        motor_enable(true);

        if (_home)
        {
            taskENTER_CRITICAL();
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_PRESS_DIR);
            _motor_tim->Init.Period = home_step_us;
            HAL_TIM_Base_Init(_motor_tim);
            _motor_tim->Instance->CNT=0;
            _moving = true;
            _homing = false;
            HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);
            taskEXIT_CRITICAL();
            while (_home) 
            {
                wait_ms(2);
            }
            wait_ms(200);
            motor_stop();
            wait_ms(500);
        }
        taskENTER_CRITICAL();
        _homing = true;
        _moving = true;
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_RELEASE_DIR);
        _motor_tim->Init.Period = home_step_us;
        HAL_TIM_Base_Init(_motor_tim);
        _motor_tim->Instance->CNT=0;
        HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);
        taskEXIT_CRITICAL();
        while (!_home)
        {
            wait_ms(2);
        }
    }
    return true;
}


bool motor_release(uint32_t step_us)
{
    motor_stop();
    if (!_home)
    {
        taskENTER_CRITICAL();
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_RELEASE_DIR);
        _moving = true;
        _homing = true;
        _motor_tim->Init.Period = step_us;
        HAL_TIM_Base_Init(_motor_tim);
        _motor_tim->Instance->CNT=0;
        HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);
        taskEXIT_CRITICAL();
    }
    return true;
}

static volatile uint32_t *			TAB_DMA = NULL;
static volatile unsigned int 								remember_nb_steps = 0;

bool motor_press(uint32_t *steps_profile_us, unsigned int nb_steps)
{
    
	
	motor_stop();
	
	/// remember :
	TAB_DMA = steps_profile_us;
	remember_nb_steps = nb_steps;
	
    if (nb_steps > 0)
    {
        taskENTER_CRITICAL();
				motor_enable(true);
				HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_PRESS_DIR);
				_moving = true;
				_motor_tim->Init.Period = steps_profile_us[0];
				HAL_TIM_Base_Init(_motor_tim);
				_motor_tim->Instance->CNT=0;
				HAL_DMA_Init(_motor_tim->hdma[TIM_DMA_ID_UPDATE]);
				HAL_TIM_PWM_Start_IT(_motor_tim, MOTOR_TIM_CHANNEL);
				HAL_TIM_DMABurst_MultiWriteStart(_motor_tim, TIM_DMABASE_ARR, TIM_DMA_UPDATE, &steps_profile_us[1], TIM_DMABURSTLENGTH_1TRANSFER, nb_steps - 1);
        taskEXIT_CRITICAL();
		
		#if	ENABLE_CORRECTION_FABRICE_GERMAIN		==		1 &&		MODEL_ERR_PAR_PHASES		==		1
			GLOB_PHASE___modele_erreur = INSPIRATION_MODEL_ERR; /// nouvelle implem FABRICE : 3 phases 
		#endif
    }
    return true;
}

bool motor_stop()
{
    taskENTER_CRITICAL();
    if (_moving)
    {
        do_motor_stop();
    }
    taskEXIT_CRITICAL();
    return true;
}

void motor_enable(bool ena)
{
    HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, ena ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


bool is_motor_moving()
{
    return _moving;
}

bool is_motor_home()
{
    return _home;
}

bool is_motor_ok()
{
    return _motor_tim != NULL;
}


void motor_limit_sw_A_irq()
{
    __disable_irq();
    _limit_sw_A = !HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin);
    check_home();
    __enable_irq();
}

void motor_limit_sw_B_irq()
{
    __disable_irq();
    _limit_sw_B = !HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin);
    check_home();
    __enable_irq();
}

void motor_active_irq()
{
    static uint32_t last_time = 0;
    uint32_t time = HAL_GetTick();
    if (time - last_time > 10)
    {
        _active = HAL_GPIO_ReadPin(MOTOR_ACTIVE_GPIO_Port, MOTOR_ACTIVE_Pin);
    }
    last_time = time;
}


static void period_elapsed_callback(TIM_HandleTypeDef *tim)
{
    __disable_irq();
    do_motor_stop();
    __enable_irq();
}


static void check_home()
{
    if (_limit_sw_A && _limit_sw_B)
    {
        _home = true;
        if (_homing)
        {
            do_motor_stop();
            _homing = false;
        }
    }
    else
    {
        _home = false;
    }
}

static void do_motor_stop( ) {
    // uint32_t *steps_profile_us, unsigned int nb_steps
	if (_moving)
    {
        #if		0
		
			HAL_TIM_DMABurst_WriteStop(_motor_tim, TIM_DMA_ID_UPDATE);
			HAL_DMA_DeInit(_motor_tim->hdma[TIM_DMA_ID_UPDATE]);
			HAL_TIM_PWM_Stop(_motor_tim, MOTOR_TIM_CHANNEL);
			
		#else
			__disable_irq(); /// URGENCE on a atteinds le volume, on s'arrete proprement
		
					uint32_t		TIMER_ms_at_motor_stop = get_time_ms();
					
					/// on recupere l'index ds le buffer DMA en cours
					int 				index_DMA_en_cours = __HAL_DMA_GET_COUNTER( _motor_tim->hdma[TIM_DMA_ID_UPDATE] ); /// Frantz : NDT : g_motor_steps_us
					int				nbr_pas_moteur_restant = remember_nb_steps - index_DMA_en_cours;
					
					
					uint32_t		duree_US_tampon_le_temps_du_rewrite = 0;
					
					bool			rewrite_was_possible = true;
					
					// #define		NBR_PAS_MOTEUR_RESTANT											10
					if( 
									TAB_DMA														!= NULL
						&&		remember_nb_steps 									!= 0 
						&&		TAB_DMA[ index_DMA_en_cours ]		> 0
						// nbr_pas_moteur_restant > NBR_PAS_MOTEUR_RESTANT : sera gere par rewrite_was_possible : + propre
					){
						
						uint32_t		Duree_pas_moteur = TAB_DMA[ index_DMA_en_cours ];
						
						#define			US_TO_WAIT_TO_REWRITE_DMA_BUFFER			1000 /// 760 constated
						while( 
										duree_US_tampon_le_temps_du_rewrite < US_TO_WAIT_TO_REWRITE_DMA_BUFFER /// securite le temps du rewrite tab DMA
						){
							
							duree_US_tampon_le_temps_du_rewrite += TAB_DMA[ index_DMA_en_cours++ ] ; /// on avance un peu ds le temps
							
							if( index_DMA_en_cours >= remember_nb_steps ){ /// on a atteinds la fin du tableau DMA : echec rewrite
								rewrite_was_possible = false;
							}
						}

						
						
						if(
									rewrite_was_possible == true
							// &&	Duree_pas_moteur > 0
						){
							
							/// on poursuit
							// int			NBR_pas_necessaires_pour_freiner = Duree_pas_moteur / 38;
							
							uint32_t		duree_US_to_wait_for_real_stop = duree_US_tampon_le_temps_du_rewrite;
							int				duree_step_recalculee = (int)Duree_pas_moteur;
							
							// #define			VITESSE_BASSE_NIVEAU_1				3000
							// #define					ACCEL_NIVEAU_1						50
							// #define					ACCEL_EVOL_1								ACCEL_NIVEAU_1 / 10
							// #define			VITESSE_BASSE_NIVEAU_2				VITESSE_BASSE_NIVEAU_1 + 512
							// #define					ACCEL_NIVEAU_2						256
							// #define					ACCEL_EVOL_2								6
							
							#define			VITESSE_BASSE_NIVEAU_1				6000
							#define					ACCEL_NIVEAU_1						500
							#define					ACCEL_EVOL_1								ACCEL_NIVEAU_1 / 10
							#define			VITESSE_BASSE_NIVEAU_2				VITESSE_BASSE_NIVEAU_1 + 8000
							#define					ACCEL_NIVEAU_2						756
							#define					ACCEL_EVOL_2								6
							
							int		Fact_decceleration = ACCEL_NIVEAU_1;
							while( 
											index_DMA_en_cours < remember_nb_steps
									&&	duree_step_recalculee < VITESSE_BASSE_NIVEAU_1
								// &&	NBR_pas_necessaires_pour_freiner--
							){ /// on gere la decceleration bnien propre
							
								/// MAJ decceleration :
								duree_step_recalculee = duree_step_recalculee + Fact_decceleration; /// motor_release
								// if( Fact_decceleration > ACCEL_EVOL_1 + 12 )
									// Fact_decceleration -= ACCEL_EVOL_1;
								// else break;
								
								TAB_DMA[ index_DMA_en_cours++ ] = duree_step_recalculee;
								duree_US_to_wait_for_real_stop += duree_step_recalculee;
							}
							
							#if		1
							// int	NBR_steps_decceleration_faible = 12;
							while( 
											index_DMA_en_cours < remember_nb_steps
									&&	duree_step_recalculee < VITESSE_BASSE_NIVEAU_2
									// &&	NBR_steps_decceleration_faible--
							){ /// on gere la decceleration bnien propre
							
								/// MAJ decceleration :
								// duree_step_recalculee = duree_step_recalculee; /// motor_release
								duree_step_recalculee = duree_step_recalculee + ACCEL_NIVEAU_2; /// motor_release
								// if( Fact_decceleration > ACCEL_EVOL_2 + 4 )
									// Fact_decceleration -= ACCEL_EVOL_2;
								// else break;
								
								
								TAB_DMA[ index_DMA_en_cours++ ] = duree_step_recalculee;
								duree_US_to_wait_for_real_stop += duree_step_recalculee;
							}
							#endif
							
							/// on ne pourra pas changer 		nb_steps 		en cours de DMA... donc on ecrit des 0 jusqu'a la fin
							while( ++index_DMA_en_cours < remember_nb_steps ){
								TAB_DMA[ index_DMA_en_cours ] = VITESSE_BASSE_NIVEAU_2; /// UINT32_MAX;
								// printf( "---stop %i\n", index_DMA_en_cours );
							}
							/// on a fini de reecrire le TAB_DMA
							/// on a fini de reecrire le TAB_DMA
							/// on a fini de reecrire le TAB_DMA
							
							
							uint32_t		maintenant = get_time_ms();
							if( maintenant >= TIMER_ms_at_motor_stop + duree_US_tampon_le_temps_du_rewrite / 1000 ){
								/// ALERTE pas forcement critique : US_TO_WAIT_TO_REWRITE_DMA_BUFFER etait trop petit
								printf( "\n\n----ALERTE do_motor_stop %u\n\n", duree_US_tampon_le_temps_du_rewrite );
								__enable_irq(); ///  
								hardfault_CRASH_ME();
							}
							
							if( maintenant >= TIMER_ms_at_motor_stop + duree_US_to_wait_for_real_stop / 1000 ){
								/// ERREUR GRAVE : US_TO_WAIT_TO_REWRITE_DMA_BUFFER etait trop petit , il y a un risque de grand nimporte quoi
								printf( "\n\n----ERREUR do_motor_stop %u\n\n", duree_US_to_wait_for_real_stop );
								__enable_irq(); ///  
								hardfault_CRASH_ME();
							}
							
							__enable_irq(); /// touboueno 
							wait_ms( duree_US_to_wait_for_real_stop / 1000 - ( maintenant - TIMER_ms_at_motor_stop )  );
							/// SUCCES le moteur vient de s'arreter tout va bien on va couper le DMA
						}
						else { /// pas assez de marge
							/// on va juste arreter brutalement
							__enable_irq();
						}
						
						/// puis on attends la fin :
					}
					else { /// pas assez de marge
						/// on va juste arreter brutalement
						__enable_irq();
					}
					
					/// reinit commande
					remember_nb_steps = 0;
					TAB_DMA = NULL;
					
					/// enfin on arrete le DMA brutalement une bonne fois pour toute :
					HAL_TIM_DMABurst_WriteStop(_motor_tim, TIM_DMA_ID_UPDATE);
					HAL_DMA_DeInit(_motor_tim->hdma[TIM_DMA_ID_UPDATE]);
					HAL_TIM_PWM_Stop(_motor_tim, MOTOR_TIM_CHANNEL);
			
		#endif
		
		#if	ENABLE_CORRECTION_FABRICE_GERMAIN		==		1 &&		MODEL_ERR_PAR_PHASES		==		1
			GLOB_PHASE___modele_erreur = PLATEAU_MODEL_ERR; /// nouvelle implem FABRICE : 3 phases 
		#endif
		
        _moving = false;
    }
	else {
		/// reinit commande
		remember_nb_steps = 0;
		TAB_DMA = NULL;
	}
}

