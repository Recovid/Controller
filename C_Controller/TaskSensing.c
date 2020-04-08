//include FreeRTOS
#include <FreeRTOS.h>
#include <task.h>

//Inlcude Recovid
#include "tasks_recovid.h"
#include "controller.h"

void TaskSensing(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
	sense_and_compute();
  }
  
}
