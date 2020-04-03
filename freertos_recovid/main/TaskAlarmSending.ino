

void TaskAlarmSending(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);

  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
    Serialprintf("%s is awake at %d", task->name, xTaskGetTickCount());
  }
}
