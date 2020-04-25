#include "common.h"
#include "lowlevel.h"
#include "recovid_revB.h"

#include <FreeRTOS.h>
#include <task.h>

uint32_t get_time_ms()
{
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

uint32_t wait_ms(uint32_t t_ms)
{
  if(xTaskGetCurrentTaskHandle() != NULL)
    vTaskDelay(t_ms/portTICK_PERIOD_MS);
  else {
  	HAL_Delay(t_ms);
  }
  return get_time_ms();
}