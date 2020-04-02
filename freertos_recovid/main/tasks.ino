#include "tasks.h"
#include "struct.h"
#include "variables.h"



int sleepPeriodic(struct periodic_task* task)
{
  int ret = 0;
  if(xTaskGetTickCount() > (task->LastWakeTime + task->PeriodMs)) {
      ret =((xTaskGetTickCount() - (task->LastWakeTime + task->xFrequency))/task->xFrequency);
      Serial.print("We are late of");
      Serial.print(ret);
      Serial.println();
      task->LastWakeTime = xTaskGetTickCount();
  }
  vTaskDelayUntil( &task->LastWakeTime, task->PeriodMs);
  return ret;
}

int initTask(struct periodic_task* task)
{
  Serial.println(task->name);
  task->xFrequency = task->PeriodMs / portTICK_PERIOD_MS;
  return 0;
}


void TaskMediumAlarm(void* task); 
void TaskUrgentAlarm(void* task); 
void TaskSensing(void* task);
void TaskMessageManagement(void* task);
void TaskRespirationCycle(void* task);
void TaskDataSending(void* task);
void TaskAlarmSending(void* task);



struct periodic_task task_array[] = {
	{ TaskMediumAlarm,            2000,  "Medium Alarm", 0, 0, 0},
  { TaskUrgentAlarm,             500,  "Urgent Alarm", 0, 0, 0},
  { TaskSensing,                   1,  "Sensing", 0, 0, 0},
  { TaskMessageManagement,         1,  "Message Management", 0, 0, 0},
  { TaskRespirationCycle,          1,  "Respiration Cycle", 0, 0, 0},
  { TaskDataSending,              40,  "Sending Datas", 0, 0, 0},
  { TaskDataSending,             100,  "Sending Alarms", 0, 0, 0}, 
  { TaskDigitalRead,               1,  "Digital Read", 0, 0, 0},
  { TaskBlink,                  1000,  "Blink", 0, 0, 0},
};

size_t size_task_array = sizeof(task_array) / sizeof(task_array[0]);



void TaskAlarmSending(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
    Serial.print(task->name);
    Serial.print("is awake at");
    Serial.println(xTaskGetTickCount());
  }
  
}

void TaskDataSending(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
    Serial.print(task->name);
    Serial.print("is awake at");
    Serial.println(xTaskGetTickCount());
  }
  
}


void TaskMessageManagement(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
    Serial.print(task->name);
    Serial.print("is awake at");
    Serial.println(xTaskGetTickCount());
  }
  
}

void TaskRespirationCycle(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
    Serial.print(task->name);
    Serial.print("is awake at");
    Serial.println(xTaskGetTickCount());
  }
  
}

void TaskSensing(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
    Serial.print(task->name);
    Serial.print("is awake at");
    Serial.println(xTaskGetTickCount());
  }
  
}

void TaskMediumAlarm(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
    Serial.print(task->name);
    Serial.print("is awake at");
    Serial.println(xTaskGetTickCount());
  }
  
}

void TaskUrgentAlarm(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
    Serial.print(task->name);
    Serial.print("is awake at");
    Serial.println(xTaskGetTickCount());
  } 
}


/*--------------------------------------------------*/
/*---------------------- Exemple Tasks -------------*/
/*--------------------------------------------------*/
void TaskBlink(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
   // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( count_ms / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( count_ms / portTICK_PERIOD_MS ); // wait for one second
  }
}

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


//struct periodic_task task_array[] = {
//  { TaskDigitalRead,            1,  "Digital Read", 0, 0, 0},
//  { TaskBlink,               1000,  "Blink", 0, 0, 0},
//};
// size_t size_task_array = sizeof(task_array) / sizeof(task_array[0]);
