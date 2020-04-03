void TaskRespirationCycle(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  int time = 0;
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
    time = (time + 1) % 100;
    if(time == 0)
      Serialprintf("%s is awake at %d", task->name, xTaskGetTickCount());
  }
}
