#include "platform.h"

#include <stdio.h>
#include <string.h>


//Low-level include
#include "lowlevel/include/lowlevel.h"
// FreeRTOS Include
#include <FreeRTOS.h>
#include <task.h>



#define PRIORITY_LOW            8
#define PRIORITY_BELOW_NORMAL   16
#define PRIORITY_NORMAL         24
#define PRIORITY_ABOVE_NORMAL   32
#define PRIORITY_HIGH           40




void controllerTask(void *argument);
void breathingTask(void *argument);
void monitoringTask(void *argument);



int main()
{

  if(!init_hardware()) {
      // Unable to initialize hardware.
      // There is nothing we can do
      while(true);
  }

  controller_init();
  breathing_init();
  monitoring_init();


  TaskHandle_t breathingTaskHandle;
  TaskHandle_t monitoringTaskHandle;
  TaskHandle_t controllerTaskHandle;

  if(xTaskCreate(breathingTask, "Breathing" , 256, NULL, PRIORITY_HIGH, &breathingTaskHandle) != pdPASS) {
    return -1;
  }
  if(xTaskCreate(monitoringTask, "Monitoring", 256, NULL, PRIORITY_ABOVE_NORMAL, &monitoringTaskHandle) != pdPASS) {
    return -1;
  }
  if(xTaskCreate(controllerTask, "Controller", 256, NULL, PRIORITY_NORMAL, &controllerTaskHandle) != pdPASS) {
    return -1;
  }

  vTaskSuspend(breathingTaskHandle);
  vTaskSuspend(monitoringTaskHandle);

  // start scheduler
  vTaskStartScheduler();
    
  // We should never get here
  while(true);
}

void startControllerTask(void *argument) {
  controller_run();
  // We should never get here
  while(true);
}
void startBreathingTask(void *argument) {
  breathing_run();
  // We should never get here
  while(true);
}
void startMonitoringTask(void *argument) {
  monitoring_run();
  // We should never get here
  while(true);
}

