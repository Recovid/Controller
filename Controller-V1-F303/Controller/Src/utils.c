/*
 * utils.c
 *
 *  Created on: Apr 14, 2020
 *      Author: rix
 */

#include "utils.h"

static TIM_HandleTypeDef* _tim_us = 0;			// us timer


void init_time_us(TIM_HandleTypeDef* htim) {
	_tim_us= htim;
}

void start_time_us() {
	HAL_TIM_Base_Start(_tim_us);
}
void stop_time_us() {
	HAL_TIM_Base_Stop(_tim_us);
}

uint32_t get_time_us() {
	return _tim_us->Instance->CNT;
}
