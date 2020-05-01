#include "recovid_revB.h"
#include "platform.h"
#include "platform_config.h"



bool init_indicators() { return true; /* nothing to do */ }



bool light_nucleo(OnOff val) {
	HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, val == On ?GPIO_PIN_RESET:GPIO_PIN_SET);
  return true;
}


bool light_yellow(OnOff val) {
	HAL_GPIO_WritePin(MAT_LED_ORANGE_GPIO_Port, MAT_LED_ORANGE_Pin, val == On ?GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}

bool light_red   (OnOff val) {
	HAL_GPIO_WritePin(MAT_LED_RED_GPIO_Port, MAT_LED_RED_Pin, val == On ?GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}

bool light_green (OnOff val) {
	HAL_GPIO_WritePin(MAT_LED_GREEN_GPIO_Port, MAT_LED_GREEN_Pin, val == On ?GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}



bool buzzer_low(OnOff val) {
	HAL_GPIO_WritePin(BUZZER_HIGH_GPIO_Port, BUZZER_HIGH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_MEDIUM_GPIO_Port, BUZZER_MEDIUM_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_LOW_GPIO_Port, BUZZER_LOW_Pin, val == On ? GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}

bool buzzer_medium(OnOff val) {
	HAL_GPIO_WritePin(BUZZER_HIGH_GPIO_Port, BUZZER_HIGH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_MEDIUM_GPIO_Port, BUZZER_MEDIUM_Pin, val == On ? GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_LOW_GPIO_Port, BUZZER_LOW_Pin, val == On ? GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}

bool buzzer_high(OnOff val) {
	HAL_GPIO_WritePin(BUZZER_HIGH_GPIO_Port, BUZZER_HIGH_Pin, val == On ? GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_MEDIUM_GPIO_Port, BUZZER_MEDIUM_Pin, val == On ? GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BUZZER_LOW_GPIO_Port, BUZZER_LOW_Pin, val == On ? GPIO_PIN_SET:GPIO_PIN_RESET);
  return true;
}
