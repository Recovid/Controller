#include "tasks.h"
#include "struct.h"
#include "variables.h"

int sleepPeriodic(struct periodic_task* task)
{
  int ret = 0;
  if(xTaskGetTickCount() > (task->LastWakeTime + task->PeriodMs)) {
      ret =((xTaskGetTickCount() - (task->LastWakeTime + task->xFrequency))/task->xFrequency);
      Serialprintf("%s missed %d cycles", ret);
      //Skip the missed cycles by resetting the lastWakeTime
      task->LastWakeTime = xTaskGetTickCount();
  }
  vTaskDelayUntil( &task->LastWakeTime, task->PeriodMs);
  return ret;
}

int initTask(struct periodic_task* task)
{
  task->xFrequency = task->PeriodMs / portTICK_PERIOD_MS;
  return 0;
}


void Serialprintf( const char * format, ... )
{
  va_list args;
  va_start (args, format);
  struct message *msg_ptr = (struct message*) malloc(sizeof(struct message));
  vsnprintf( msg_ptr->text, SIZE_OF_TEXT_MESSAGE, format, args);
  int ret = xQueueSend( xQueueMessage, &(msg_ptr), ( TickType_t ) 0 );
  if(ret != pdPASS) {
    //Message queue is full
    //What should we do ?
    //at least clean up
    free(msg_ptr);
  }
  va_end (args);
}

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
  { TaskSerialSending,            1,    "SerialSend", 0, 0, 0},
};

size_t size_task_array = sizeof(task_array) / sizeof(task_array[0]);
