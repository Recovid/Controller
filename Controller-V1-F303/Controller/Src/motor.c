/*
 * bavu.c
 *
 *  Created on: Apr 11, 2020
 *      Author: rix
 */

#include <motor.h>


static motor_handle_t* motors[NB_MOTORS] = { 0 };


static motor_handle_t* get_motor_for_tim();


void step_callback(TIM_HandleTypeDef* 	tim);
void period_callback(TIM_HandleTypeDef* 	tim);


bool motor_init(motor_handle_t* motor) {

	int idx=0;
	for(idx=0; idx<NB_MOTORS; ++idx) {
		if(motors[idx]==NULL) break;
	}
	if(idx==NB_MOTORS) return false;

	motors[idx]= motor;

	motor->_moving= false;

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = motor->pulse_width_us;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(motor->tim, &sConfigOC, motor->channel);

  motor->tim->PWM_PulseFinishedCallback= step_callback;
  motor->tim->PeriodElapsedCallback= period_callback;

  return true;
}


void motor_enable(motor_handle_t* motor) { 	// Enable motor
	HAL_GPIO_WritePin(motor->ena_port, motor->ena_pin, motor->ena_inverted?GPIO_PIN_RESET:GPIO_PIN_SET);
}

void motor_disable(motor_handle_t* motor) { 	// Disable motor
	HAL_GPIO_WritePin(motor->ena_port, motor->ena_pin, motor->ena_inverted?GPIO_PIN_SET:GPIO_PIN_RESET);
}

void motor_run(motor_handle_t* motor, motor_dir_t dir, uint16_t step_time_us) {
	motor_move(motor, dir, step_time_us, -1);
}


void motor_move(motor_handle_t* motor, motor_dir_t dir, uint16_t step_time_us, int32_t nb_steps) {
	if(nb_steps==0) {
		motor->_moving=false;
		return;
	}

	HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, dir);
	motor->_remaining_steps			= nb_steps;
	motor->_moving=true;
	motor->tim->Init.Period = step_time_us;
  HAL_TIM_Base_Init(motor->tim);
	HAL_TIM_PWM_Start_IT(motor->tim, motor->channel);
}

uint32_t _dma_time;

void motor_move_profile(motor_handle_t* motor, motor_dir_t dir, uint16_t* steps_profile_us, int32_t nb_steps) {
	if(nb_steps==0) {
		motor->_moving=false;
		return;
	}
	if(nb_steps==1) {
		motor_move(motor, dir, steps_profile_us[0], 1);
		return;
	}

	HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, dir);
	motor->_steps_profile = steps_profile_us;
	motor->_remaining_steps			= nb_steps-1;
	motor->_moving=true;
	motor->tim->Init.Period = motor->_steps_profile[0];
	_dma_time= HAL_GetTick();
  HAL_TIM_Base_Init(motor->tim);
	HAL_TIM_PWM_Start(motor->tim, motor->channel);
	HAL_TIM_DMABurst_MultiWriteStart(motor->tim, TIM_DMABASE_ARR, TIM_DMA_UPDATE,	(uint32_t*)steps_profile_us, TIM_DMABURSTLENGTH_1TRANSFER, motor->_remaining_steps);
}


bool motor_is_moving(motor_handle_t* motor) {
	return motor->_moving;
}

void motor_stop(motor_handle_t* motor) {
	HAL_TIM_PWM_Stop_IT(motor->tim, motor->channel);
	motor->_moving=false;
}


static motor_handle_t* get_motor_for_tim(TIM_HandleTypeDef* 	tim) {
	for(int t=0; t<NB_MOTORS; ++t) {
		if(motors[t]!= NULL && motors[t]->tim==tim) return motors[t];
	}
	return NULL;
}


void period_callback(TIM_HandleTypeDef* 	tim) {
	motor_handle_t* motor= get_motor_for_tim(tim);
	if(motor==NULL) return;
	motor->_moving=false;
	HAL_TIM_PWM_Stop(motor->tim, motor->channel);
}

void step_callback(TIM_HandleTypeDef* 	tim) {
	motor_handle_t* motor= get_motor_for_tim(tim);
	if(motor==NULL) return;

	if(motor->_remaining_steps>0) {
		if(motor->_remaining_steps==1) {
			HAL_TIM_PWM_Stop_IT(motor->tim, motor->channel);
			motor->_moving=false;
		} else {
			--motor->_remaining_steps;
		}
	}
}

