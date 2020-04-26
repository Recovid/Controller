#include "recovid_revB.h"
#include "lowlevel.h"



bool is_Failsafe_Enabled() {
  return HAL_GPIO_ReadPin(FS_Enabled_GPIO_Port, FS_Enabled_Pin) == GPIO_PIN_RESET;
}



void enable_Rpi(bool ena) {
  HAL_GPIO_WritePin(Enable_P5V_Rpi_GPIO_Port, Enable_P5V_Rpi_Pin, ena ? GPIO_PIN_RESET : GPIO_PIN_SET);
} 

