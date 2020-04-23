//include FreeRTOS
#include <FreeRTOS.h>
#include <task.h>

//Inclde STD
#include <stdlib.h>

//Inlcude Recovid
#include "tasks_recovid.h"
#include "sensing.h"
#include "i2c.h"

void TaskSensing(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  sensors_start();
  while (true) // A Task shall never return or exit.
  {
    sleepPeriodic(task);
  }
}
