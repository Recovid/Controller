void TaskDigitalRead(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  // defined USER_BTN digital pin  has a pushbutton attached to it. Give it a name:
  uint8_t pushButton = USER_BTN;
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);
  int prevbuttonState = 1;
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
    // read the input pin:
    int buttonState = digitalRead(pushButton);
    while(buttonState == 0) {
      count_ms = 100;
      vTaskDelay(task->xFrequency);
      buttonState = digitalRead(pushButton);
    }
    count_ms = 500;
  }
}
