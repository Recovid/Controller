#include "config.h"
#include "controller.h"
#include "breathing.h"
#include "monitoring.h"
#include "hmi.h"



TaskHandle_t breathingTaskHandle;
TaskHandle_t monitoringTaskHandle;
TaskHandle_t controllerTaskHandle;
TaskHandle_t hmiTaskHandle;

EventGroupHandle_t ctrlEventFlags;

EventGroupHandle_t brthCycleState;

#ifdef DEBUG
SemaphoreHandle_t dbgMutex;
#endif


int main(int argc char *argv )
{

  if(!init_hardware()) {
      // Unable to initialize hardware.
      // There is nothing we can do
      while(true);
  }

#ifdef DEBUG
  dbgMutex= xSemaphoreCreateBinary();
  xSemaphoreGive(dbgMutex);
  dbg_printf("Starting Recovid\n");
#endif

  ctrlEventFlags = xEventGroupCreate();

  brthCycleState = xEventGroupCreate();
  
  if(xTaskCreate(breathing_run , "Breathing" , BREATHING_TASK_STACK_SIZE , NULL, BREATHING_TASK_PRIORITY , &breathingTaskHandle) != pdTRUE) {
    HardFault_Handler();
  }
  if(xTaskCreate(monitoring_run, "Monitoring", MONITORING_TASK_STACK_SIZE, NULL, MONITORING_TASK_PRIORITY, &monitoringTaskHandle) != pdTRUE) {
    HardFault_Handler();
  }
  if(xTaskCreate(controller_run, "Controller", CONTROLLER_TASK_STACK_SIZE, NULL, CONTROLLER_TASK_PRIORITY, &controllerTaskHandle)  != pdTRUE) {
    HardFault_Handler();
  }
  if(xTaskCreate(hmi_run       , "HMI"       , HMI_TASK_STACK_SIZE       , NULL, HMI_TASK_PRIORITY       , &hmiTaskHandle) != pdTRUE) {
    HardFault_Handler();
  }

  // start scheduler
  vTaskStartScheduler();
    
  // We should never get here
  while(true);
}


