#include "recovid_revB.h"
#include "lowlevel.h"



bool light_nucleo(OnOff val) {
	HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, val?GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}


bool light_yellow(OnOff val) {
	HAL_GPIO_WritePin(MAT_LED_ORANGE_GPIO_Port, MAT_LED_ORANGE_Pin, val?GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}

bool light_red   (OnOff val) {
	HAL_GPIO_WritePin(MAT_LED_RED_GPIO_Port, MAT_LED_RED_Pin, val?GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}

bool light_green (OnOff val) {
	HAL_GPIO_WritePin(MAT_LED_GREEN_GPIO_Port, MAT_LED_GREEN_Pin, val?GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}
