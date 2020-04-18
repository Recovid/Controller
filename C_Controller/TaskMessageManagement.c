//include FreeRTOS
#include <FreeRTOS.h>
#include <task.h>

//Inlcude Recovid
#include "tasks_recovid.h"
#include "ihm_communication.h"

void TaskMessageManagement(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  while (true) // A Task shall never return or exit.
  {
    sleepPeriodic(task);
    send_and_recv();
  }
}
