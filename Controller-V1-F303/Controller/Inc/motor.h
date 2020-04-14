/*
 * bavu.h
 *
 *  Created on: Apr 11, 2020
 *      Author: rix
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include <stdbool.h>


#define NB_MOTORS		2


typedef enum {
	DIR_CW=GPIO_PIN_RESET,
	DIR_CCW=GPIO_PIN_SET
} motor_dir_t;



typedef struct {
	TIM_HandleTypeDef* 	tim;
	uint32_t					 	channel;
	GPIO_TypeDef* 			ena_port;
	uint32_t 						ena_pin;
	GPIO_TypeDef* 			dir_port;
	uint32_t 						dir_pin;


	volatile bool			_moving;
	volatile int32_t  _remaining_steps;
	uint16_t*   			_steps_profile;

} motor_handle_t;


bool motor_init(motor_handle_t* motor);

void motor_enable(motor_handle_t* motor); 	// Enable motor
void motor_disable(motor_handle_t* motor); 	// Disable motor

void motor_run(motor_handle_t* motor, motor_dir_t dir, uint16_t step_time_us);
void motor_move(motor_handle_t* motor, motor_dir_t dir, uint16_t step_time_us, int32_t nb_steps);
void motor_move_profile(motor_handle_t* motor, motor_dir_t dir, uint16_t* step_profile_us, int32_t nb_steps);

void motor_stop(motor_handle_t* motor);

bool motor_is_moving(motor_handle_t* motor);

#endif /* INC_MOTOR_H_ */
