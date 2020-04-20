//include FreeRTOS
#include <FreeRTOS.h>
#include <task.h>

//Inlcude Recovid
#include "tasks_recovid.h"
#include "sensing.h"
#include "alarms.h"
#include "controller.h"

void TaskRespirationCycle(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  while (true) // A Task shall never return or exit.
  {
    sleepPeriodic(task);

    //! The following functions never block and only use threadsafe functions to communicate with hardware
    //! Moreover, they depend on each other, hence the desire to execute them sequentially for more reliability
    //! \remark Each task adapt the task rate by monitoring the elapsed time since last operation
    sense_and_compute(current_respiration_state());
    // FIXME: this must be called only once per cycle
    update_alarms();
    cycle_respiration();
  }
}
