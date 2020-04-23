//FreeRTOS Include
#include <FreeRTOS.h>
#include <stdint.h>
#include <task.h>
#include <queue.h>

//Std include
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


//Recovid include
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "tasks_recovid.h"
#include "struct_recovid.h"


uint32_t get_time_ms()
{
    return HAL_GetTick();
}

uint32_t delay(uint32_t ticks_to_wait)
{
  if(xTaskGetCurrentTaskHandle() != NULL) {
    vTaskDelay(ticks_to_wait);
  }
#ifdef stm32f303
  else {
	HAL_Delay(ticks_to_wait);
  }
#endif
  return get_time_ms();
}

uint32_t wait_ms(uint32_t t_ms)
{
  if(xTaskGetCurrentTaskHandle() != NULL) {
    TickType_t WakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&WakeTime, (t_ms/portTICK_PERIOD_MS));
  }
#ifdef stm32f303
  else {
	HAL_Delay(t_ms);
  }
#endif
  return get_time_ms();
}

int sleepPeriodic(struct periodic_task* task)
{
  int ret = 0;
  if(xTaskGetTickCount() > (task->LastWakeTime + task->PeriodMs)) {
      ret =((xTaskGetTickCount() - (task->LastWakeTime + task->xFrequency))/task->xFrequency);
      //Skip the missed cycles by resetting the lastWakeTime
      task->LastWakeTime = xTaskGetTickCount();
  }
  vTaskDelayUntil( &task->LastWakeTime, task->PeriodMs);
  return ret;
}

int initTask(struct periodic_task* task)
{
  task->xFrequency = task->PeriodMs / portTICK_PERIOD_MS;
  printf("Task %s  started\n", task->name);
  return 0;
}


struct periodic_task task_array[] = {
  { TaskMessageManagement,         1,  "Message Management",  configMAX_PRIORITIES-1, 0, 0},
  { TaskRespirationCycle,          1,  "Respiration Cycle",   configMAX_PRIORITIES-1, 0, 0},
  //{ TaskSensing,          		   25,  "Sensing Cycle",      configMAX_PRIORITIES-5, 0, 0},

};

size_t size_task_array = sizeof(task_array) / sizeof(task_array[0]);
