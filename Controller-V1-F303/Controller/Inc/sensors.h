/*
 * sensors.h
 *
 *  Created on: Apr 13, 2020
 *      Author: rix
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "main.h"


#define ADDR_SPD610 	((uint16_t)(0x40 <<1))
//#define ADDR_NPA700B 	((uint16_t)(0x28 <<1))
#define ADDR_NPA700B 	((uint16_t)(0x76 <<1))


bool 			sensors_init(I2C_HandleTypeDef *hi2c);

void 			sensors_start();
void 			sensors_stop();

float 		sensors_get_flow(); // in slm
float 		sensors_get_pressure();
float 		sensors_get_volume();
void      sensors_reset_volume();

void 			sensors_set_flow_callback(void (*callback)(float flow, uint32_t delta_t_us));

void 			sensors_scan(I2C_HandleTypeDef *hi2c);


#endif /* INC_SENSORS_H_ */
