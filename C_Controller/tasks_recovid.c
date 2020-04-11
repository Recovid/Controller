//FreeRTOS Include
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

//Std include
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


//Recovid include
#include "tasks_recovid.h"
#include "struct_recovid.h"


long get_time_ms()
{
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

long wait_ms(long t_ms)
{
  if(xTaskGetCurrentTaskHandle() != NULL)
    vTaskDelay(t_ms/portTICK_PERIOD_MS);
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
  { TaskAlarm,                   500,  "Urgent Alarm",        0, 0, 0},
  { TaskSensing,                   1,  "Sensing",             0, 0, 0},
  { TaskMessageManagement,         1,  "Message Management",  0, 0, 0},
  { TaskRespirationCycle,          1,  "Respiration Cycle",   0, 0, 0},
};

size_t size_task_array = sizeof(task_array) / sizeof(task_array[0]);
