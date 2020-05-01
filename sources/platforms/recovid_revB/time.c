#include "recovid_revB.h"
#include "platform.h"
#include "platform_config.h"
#include "log_timings.h"

#include <FreeRTOS.h>
#include <task.h>

uint32_t get_time_ms()
{
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

uint32_t wait_ms(uint32_t t_ms)
{
  TaskHandle_t handle = xTaskGetCurrentTaskHandle();
  if(handle != NULL) {
    LOG_TIME_EVENT_TASK(handle, LOG_TIME_EVENT_STOP)
    vTaskDelay(t_ms/portTICK_PERIOD_MS);
    LOG_TIME_EVENT_TASK(handle, LOG_TIME_EVENT_START)
  } else {
  	HAL_Delay(t_ms);
  }
  return get_time_ms();
}
