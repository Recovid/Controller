void TaskSerialSending(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
    struct message* msg_ptr;
    if( xQueueReceive( xQueueMessage, &(msg_ptr), ( TickType_t ) 0 ) == pdPASS )
    {
      Serial.println(msg_ptr->text);
      free(msg_ptr);
    }
  }
}
