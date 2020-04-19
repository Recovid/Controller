#ifndef TASK_RECOVID_H 
#define TASK_RECOVID_H

#include <FreeRTOS.h>
#include <queue.h>

struct periodic_task {
  void (*task)(void*);
  const int PeriodMs; //1khz
  const char* name;
  const int priority;
  TickType_t LastWakeTime; //Last wakeup time
  TickType_t xFrequency;
};

extern size_t size_task_array;
extern struct periodic_task task_array[];

#define NB_MAX_MESSAGE 10
extern QueueHandle_t xQueueMessage;

int initTask(struct periodic_task* task);

int sleepPeriodic(struct periodic_task* task);
void Serialprintf( const char * format, ... );

void TaskMessageManagement(void* param);
void TaskRespirationCycle(void* param);

#endif //TASK_RECOVID_H
