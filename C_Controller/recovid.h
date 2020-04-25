#ifndef __RECOVID_H__
#define __RECOVID_H__


#include "common.h"

#include <FreeRTOS.h>
#include <task.h>
#include <event_groups.h>



#define TASK_PRIORITY_LOW            8
#define TASK_PRIORITY_BELOW_NORMAL   16
#define TASK_PRIORITY_NORMAL         24
#define TASK_PRIORITY_ABOVE_NORMAL   32
#define TASK_PRIORITY_HIGH           40



#define BREATHING_TASK_PRIORITY     (TASK_PRIORITY_HIGH)
#define MONITORING_TASK_PRIORITY    (TASK_PRIORITY_ABOVE_NORMAL)
#define CONTROLLER_TASK_PRIORITY    (TASK_PRIORITY_NORMAL)

#define BREATHING_TASK_STACK_SIZE   (256)
#define MONITORING_TASK_STACK_SIZE  (256)
#define CONTROLLER_TASK_STACK_SIZE  (256)

extern TaskHandle_t breathingTaskHandle;
extern TaskHandle_t monitoringTaskHandle;
extern TaskHandle_t controllerTaskHandle;


#define RUN_BREATHING_FLAG   (1 << 0)
#define RUN_MONITORING_FLAG  (1 << 1)

extern EventGroupHandle_t controlFlags;


#endif