#include "recovid_revB.h"
#include "lowlevel.h"
#include "sensing.h"
#include "ihm_communication.h"
#include "hardware_serial.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

static TIM_HandleTypeDef* _motor_tim = NULL;

static void period_elapsed_callback(TIM_HandleTypeDef *tim);

static volatile bool _moving;
static volatile bool _homing;
static volatile bool _home;
static volatile bool _limit_sw_A;
static volatile bool _limit_sw_B;
static volatile bool _active;

static void check_home() ;
static void motor_enable(bool ena);

unsigned int print_christophe(
										unsigned int step_t_ns_init, 
										unsigned int acc_ns, 
										unsigned int step_t_min_ns, 
										unsigned int speed_down_t_ns, 
										unsigned int speed_down_t2_ps, 
										unsigned int step_t_ns_final,
										unsigned int dec_ns,
										int nb_steps)
{
	char msg[200];
	char* current;

	strcpy(msg, "step_t_ns_init ");
	current = msg + strlen(msg);
	itoa(step_t_ns_init, current, 10);
	strcpy(msg + strlen(msg),"\n"); 
	hardware_serial_write_data(msg, strlen(msg)); 

	strcpy(msg, "acc_ns ");
	current = msg + strlen(msg);
	itoa(acc_ns, current, 10);
	strcpy(msg + strlen(msg),"\n"); 
	hardware_serial_write_data(msg, strlen(msg)); 

	strcpy(msg, "step_t_min_ns ");
	current = msg + strlen(msg);
	itoa(step_t_min_ns, current, 10);
	strcpy(msg + strlen(msg),"\n"); 
	hardware_serial_write_data(msg, strlen(msg)); 

	strcpy(msg, "speed_down_t_ns ");
	current = msg + strlen(msg);
	itoa(speed_down_t_ns, current, 10);
	strcpy(msg + strlen(msg),"\n");
	hardware_serial_write_data(msg, strlen(msg)); 

	strcpy(msg, "speed_down_t2_ps ");
	current = msg + strlen(msg);
	itoa(speed_down_t2_ps, current, 10);
	strcpy(msg + strlen(msg),"\n"); 
	hardware_serial_write_data(msg, strlen(msg)); 

	strcpy(msg, "step_t_ns_final ");
	current = msg + strlen(msg);
	itoa(step_t_ns_final, current, 10);
	strcpy(msg + strlen(msg),"\n");
	hardware_serial_write_data(msg, strlen(msg)); 

	strcpy(msg, "dec_ns ");
	current = msg + strlen(msg);
	itoa(dec_ns, current, 10);
	strcpy(msg + strlen(msg),"\n"); 
	hardware_serial_write_data(msg, strlen(msg)); 


}

unsigned int compute_motor_press_christophe(
										unsigned int step_t_ns_init, 
										unsigned int acc_ns, 
										unsigned int step_t_min_ns, 
										unsigned int speed_down_t_ns, 
										unsigned int speed_down_t2_ps, 
										unsigned int step_t_ns_final,
										unsigned int dec_ns,
										int nb_steps, 
										uint16_t* steps_t_us)
{
	unsigned int step_total_us = 0;
	unsigned int pente_acc, debit_constant, pente_dec;
	for(int i = 0; i < nb_steps; i++)
	{

		//Pas acceleration
		// =SI(N°pas<Pas_temps_initial/Acc_temps,ARRONDI.INF((Pas_temps_initial-Acc_temps*N°pas),0)/1000,0)
		if( i< step_t_ns_init / acc_ns ) {
			pente_acc = (step_t_ns_init - (acc_ns*i)) / 1000;
		}
		else {
			pente_acc = 0;
		}

		//Accélération phase
		//=ARRONDI.INF((Pas_temps_min+(N°pas*Ralent_débit_lin)+(N°pas^2*Ralent_débit_2/1000))/1000,0)
		debit_constant =  ((step_t_min_ns + i*speed_down_t_ns + (i*i*speed_down_t2_ps/1000)) /1000);
		
		//Fin
		//=SI(ET(H17<=N_Pas_fin,(H17>(N_Pas_fin-(Pas_temps_fin-Pas_temps_min)/Décc_temps))),
		//	ARRONDI.INF((Pas_temps_fin/1000)+((H17-N_Pas_fin)*(Décc_temps/1000)),0)
		//else
		//	,0)
		if(nb_steps - ((step_t_ns_final - step_t_min_ns)/dec_ns)  < i &&  i < nb_steps)
			pente_dec = (step_t_ns_final / 1000)+  ((i-nb_steps)*(dec_ns/1000));
		else
			pente_dec = 0;

		steps_t_us[i] = MAX(pente_acc, MAX(debit_constant, pente_dec));
		step_total_us += steps_t_us[i];
	}
	for(int i = nb_steps; i < MOTOR_MAX; i++) 
	{
		steps_t_us[i] = UINT16_MAX;
	}
	return step_total_us;
}

uint32_t compute_constant_motor_steps(uint16_t step_t_us, uint16_t nb_steps)
{
    const uint16_t max_steps = MIN(nb_steps, COUNT_OF(steps_t_us));
	uint32_t total_time = 0;
    for(unsigned int i=0; i<MOTOR_MAX; ++i) {
        if(i < max_steps) {
               steps_t_us[i] = MAX(step_t_us, MOTOR_STEP_TIME_INIT - (A)*i);
        }
        else {
               steps_t_us[i] = MIN(UINT16_MAX, step_t_us + (A)*(i-max_steps));
        }
		total_time += step_t_us;
	}
    return max_steps;
}


uint32_t motor_press_constant(uint16_t step_t_us, uint16_t nb_steps)
{
    const uint16_t max_steps = MIN(nb_steps, COUNT_OF(steps_t_us));
	uint32_t total_time = compute_constant_motor_steps(step_t_us, nb_steps);
    motor_press(steps_t_us, nb_steps);
	return total_time;
}

bool motor_release() {
  if( !_home) {
	  motor_stop();
	  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_RELEASE_DIR);
	  _moving=true;
	  _homing=true;
	  _motor_tim->Instance->ARR = MOTOR_RELEASE_STEP_US;
	  HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);
  }
  return true;
}

static char buf[200];
bool motor_press(uint16_t* steps_profile_us, uint16_t nb_steps)
{
    motor_stop();
	//for(int i =0; i < nb_steps;i+=100)
	//{
	//	sprintf(buf, "step [ %d]  : %d\n", i, steps_profile_us[i]);
	//	hardware_serial_write_data(buf, strlen(buf)); 
	//}
    if (nb_steps > 0) {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_PRESS_DIR);
        _moving=true;
        _motor_tim->Instance->ARR = steps_profile_us[0];
        HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);
        HAL_TIM_DMABurst_MultiWriteStart(_motor_tim, TIM_DMABASE_ARR, TIM_DMA_UPDATE,	(uint32_t*)&steps_profile_us[1], TIM_DMABURSTLENGTH_1TRANSFER, nb_steps-1);
    }
    return true;
}

bool motor_stop() {
  HAL_TIM_PWM_Stop(_motor_tim, MOTOR_TIM_CHANNEL);
  HAL_TIM_DMABurst_WriteStop(_motor_tim, TIM_DMA_ID_UPDATE);
  _moving=false;
  return true;
}

bool is_motor_ok() {
  return _motor_tim!=NULL;
}

bool init_motor()
{
  if(_motor_tim==NULL) {
    _motor_tim= &motor_tim;
    // register IT callbacks
    _motor_tim->PeriodElapsedCallback = period_elapsed_callback;

    _active= HAL_GPIO_ReadPin(MOTOR_ACTIVE_GPIO_Port, MOTOR_ACTIVE_Pin);

    _limit_sw_A= ! HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin);
    _limit_sw_B= ! HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin);
    check_home();

    motor_enable(true);

    if(_limit_sw_A || _limit_sw_B) {
      HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_PRESS_DIR);
      _motor_tim->Instance->ARR = MOTOR_HOME_STEP_US;
      _moving = true;
      _homing= false;
      HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);    
      while(_limit_sw_A || _limit_sw_B);
      HAL_Delay(200);
      motor_stop();
    }
    _homing=true;
    _moving=true;
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_RELEASE_DIR);
    _motor_tim->Instance->ARR = MOTOR_HOME_STEP_US;
    HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);    
    while(!_home);
  }
  return true;
}

static void period_elapsed_callback(TIM_HandleTypeDef *tim)
{
  motor_stop();
}

void motor_limit_sw_A_irq() {
  static uint32_t last_time=0;
  uint32_t time= HAL_GetTick();
  if(time-last_time>25) {
    _limit_sw_A= ! HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin);
    _limit_sw_B= ! HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin);
//	  _limit_sw_A ? light_yellow(On) : light_yellow(Off);
    check_home();
  }
  last_time=time;
}

void motor_limit_sw_B_irq() {  
  static uint32_t last_time=0;
  uint32_t time= HAL_GetTick();
  if(time-last_time>25) {
    _limit_sw_B= ! HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin);
    _limit_sw_A= ! HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin);
//  	_limit_sw_B ? light_green(On) : light_green(Off);
    check_home();
  }
  last_time=time;
}

void motor_active_irq() {
  static uint32_t last_time=0;
  uint32_t time= HAL_GetTick();
  if(time-last_time>50) {
    _active=HAL_GPIO_ReadPin(MOTOR_ACTIVE_GPIO_Port, MOTOR_ACTIVE_Pin);
  }
  last_time=time;
}


static void check_home() {
  if(_limit_sw_A && _limit_sw_B) {
    _home=true;
    if(_homing) {
      motor_stop();
      _homing= false;
    }
  } else {
    _home=false;
  }
}

static void motor_enable(bool ena) {
	HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, ena?GPIO_PIN_SET:GPIO_PIN_RESET);
}

static void print_samples_Q(float* samples_Q_Lps, uint16_t* samples_Q_Lps_dt_us, int nb_samples)
{
	static char msg[200];
	strcpy(msg, "\nQ ");
	itoa(nb_samples, msg+3, 10);
	hardware_serial_write_data(msg, strlen(msg));
	msg[0] = '\n';
	for(unsigned int j=0; j < nb_samples; j++)
	{
		char* current = itoa( (int) (samples_Q_Lps[j] * 1000.0f), msg+1, 10);
		char* next = current + strlen(current);
		next[0] = ' ';
		next++;
		itoa( (int) samples_Q_Lps_dt_us[j], next, 10);
		hardware_serial_write_data(msg, strlen(msg));
		wait_ms(1);
	}
}

static void print_samples_P(uint16_t* samples_P, uint16_t* samples_P_dt_us, int nb_samples)
{
	static char msg[200];
	strcpy(msg, "\nP ");
	itoa(nb_samples, msg+3, 10);
	hardware_serial_write_data(msg, strlen(msg));
	msg[0] = '\n';
	for(unsigned int j=0; j < nb_samples; j++)
	{
		char* current = itoa( (int) samples_P[j], msg+1, 10);
		char* next = current + strlen(current);
		next[0] = ' ';
		next++;
		itoa( (int) samples_P_dt_us[j], next, 10);
		hardware_serial_write_data(msg, strlen(msg));
		wait_ms(1);
	}
}

static void print_steps(uint16_t* steps_t_us, unsigned int nb_steps)
{
      static char msg[200];
      strcpy(msg, "\nSteps                ");
      itoa( nb_steps , msg+6, 10);
      hardware_serial_write_data(msg, strlen(msg));
      msg[0] = '\n';
      for(unsigned int j=0; j < nb_steps; j+=100)
      {
              itoa((int) steps_t_us[j], msg+1, 10);
              hardware_serial_write_data(msg, strlen(msg));
              wait_ms(1);
      }
}

void test_motor() 
{
	unsigned int step_t_ns_init = 300000; 
	unsigned int acc_ns = 2000;
	unsigned int step_t_min_ns = 100000;
	unsigned int speed_down_t_ns = 10;
	unsigned int speed_down_t2_ps = 5;
	unsigned int step_t_ns_final = 300000;
	unsigned int dec_ns = 2000;


	const int nb_steps = 3800;
	unsigned int t_us = compute_motor_press_christophe(step_t_ns_init,
												acc_ns,
												step_t_min_ns,
												speed_down_t_ns,
												speed_down_t2_ps,
												step_t_ns_final,
												dec_ns,
												nb_steps,
												steps_t_us);
	print_christophe(step_t_ns_init, 
						acc_ns,
						step_t_min_ns,
						speed_down_t_ns,
						speed_down_t2_ps,
						step_t_ns_final,
						dec_ns,
						nb_steps);
	valve_inhale();
	sensors_start_sampling_flow();
	motor_press(steps_t_us, nb_steps);
	while(_moving);
	wait_ms(800);
	sensors_stop_sampling_flow();
	motor_release();
	valve_exhale();
	while(!_home);
	wait_ms(100);
	print_samples_Q(samples_Q_Lps, samples_Q_Lps_dt_us, get_samples_Q_index_size());
	print_samples_P(samples_P, samples_P_dt_us, get_samples_P_index_size());
	while(true);
}
