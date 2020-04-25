#include "recovid.h"
#include "controller.h"
#include "breathing.h"
#include "monitoring.h"
#include "hmi.h"
#include "lowlevel.h"




#define PRIORITY_LOW            8
#define PRIORITY_BELOW_NORMAL   16
#define PRIORITY_NORMAL         24
#define PRIORITY_ABOVE_NORMAL   32
#define PRIORITY_HIGH           40


TaskHandle_t breathingTaskHandle;
TaskHandle_t monitoringTaskHandle;
TaskHandle_t controllerTaskHandle;
TaskHandle_t hmiTaskHandle;

EventGroupHandle_t eventFlags;

#ifdef DEBUG
SemaphoreHandle_t dbgMutex;
#endif


int main()
{

  if(!init_hardware()) {
      // Unable to initialize hardware.
      // There is nothing we can do
      while(true);
  }

#ifdef DEBUG
  dbgMutex= xSemaphoreCreateBinary();
  xSemaphoreGive(dbgMutex);
  printf("Starting Recovid\n");
#endif

  eventFlags = xEventGroupCreate();
  
  xTaskCreate(breathing_run , "Breathing" , BREATHING_TASK_STACK_SIZE , NULL, BREATHING_TASK_PRIORITY , &breathingTaskHandle);
  xTaskCreate(monitoring_run, "Monitoring", MONITORING_TASK_STACK_SIZE, NULL, MONITORING_TASK_PRIORITY, &monitoringTaskHandle);
  xTaskCreate(controller_run, "Controller", CONTROLLER_TASK_STACK_SIZE, NULL, CONTROLLER_TASK_PRIORITY, &controllerTaskHandle);
  xTaskCreate(hmi_run       , "HMI"       , HMI_TASK_STACK_SIZE       , NULL, HMI_TASK_PRIORITY       , &hmiTaskHandle);

  // start scheduler
  vTaskStartScheduler();
    
  // We should never get here
  while(true);
}


