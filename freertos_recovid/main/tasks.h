#ifndef TASK_H 
#define TASK_H

#include <STM32FreeRTOS.h>

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


int initTask(struct periodic_task* task);

int sleepPeriodic(struct periodic_task* task);

#endif TASK_H
