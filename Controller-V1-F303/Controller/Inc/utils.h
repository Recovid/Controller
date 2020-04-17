/*
 * utils.h
 *
 *  Created on: Apr 14, 2020
 *      Author: rix
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "stm32f3xx_hal.h"



void 			init_time_us(TIM_HandleTypeDef* htim);
void 			start_time_us();
void 			stop_time_us();

uint32_t 	get_time_us();

#endif /* INC_UTILS_H_ */
