#include "recovid_revB.h"
#include "lowlevel.h"
#include "sensing.h"
#include "ihm_communication.h"
#include "hardware_serial.h"
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



void test_motor() 
{
	static char msg[200];
	for(int i = 1; i < 4; i++) 
	{
		valve_inhale();
		sensors_start_sampling_flow();
		motor_press_constant(MOTOR_STEP_TIME_US_MIN*i, 3800);
		sensors_start_sampling_flow();
		wait_ms(2000);
		motor_stop();
		valve_exhale();
		wait_ms(100);
		sensors_stop_sampling_flow();
		motor_release();
		strcpy(msg, "\nSample");
		hardware_serial_write_data(msg, strlen(msg)); 
		msg[0] = '\n';
		for(unsigned int j=0; j < COUNT_OF(samples_Q_Lps); j++)
		{
			itoa( (int) (samples_Q_Lps[j] * 1000.0f), msg+1, 10);
			hardware_serial_write_data(msg, strlen(msg)); 
			wait_ms(1);
		}

		strcpy(msg, "\nSteps");
		hardware_serial_write_data(msg, strlen(msg)); 
		msg[0] = '\n';
		for(unsigned int j=0; j < COUNT_OF(steps_t_us); j++)
		{
			itoa((int) steps_t_us[j], msg+1, 10);
			hardware_serial_write_data(msg, strlen(msg)); 
			wait_ms(1);
		}
		wait_ms(2000);
	}
}
