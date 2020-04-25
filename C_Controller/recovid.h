#ifndef __RECOVID_H__
#define __RECOVID_H__


#include "common.h"
#include "config.h"
#include "lowlevel.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <event_groups.h>



#define TASK_PRIORITY_LOW            8
#define TASK_PRIORITY_BELOW_NORMAL   16
#define TASK_PRIORITY_NORMAL         24
#define TASK_PRIORITY_ABOVE_NORMAL   32
#define TASK_PRIORITY_HIGH           40


#define BREATHING_TASK_PRIORITY     (TASK_PRIORITY_HIGH)
#define MONITORING_TASK_PRIORITY    (TASK_PRIORITY_ABOVE_NORMAL)
#define CONTROLLER_TASK_PRIORITY    (TASK_PRIORITY_NORMAL)
#define HMI_TASK_PRIORITY           (TASK_PRIORITY_BELOW_NORMAL)

#define BREATHING_TASK_STACK_SIZE   (256)
#define MONITORING_TASK_STACK_SIZE  (256)
#define CONTROLLER_TASK_STACK_SIZE  (512)
#define HMI_TASK_STACK_SIZE         (1024)

extern TaskHandle_t breathingTaskHandle;
extern TaskHandle_t monitoringTaskHandle;
extern TaskHandle_t controllerTaskHandle;
extern TaskHandle_t hmiTaskHandle;

#define BREATHING_RUN_FLAG        (1 << 0)
#define MONITORING_RUN_FLAG       (1 << 1)
#define HMI_RUN_FLAG              (1 << 2)

#define BREATHING_STOPPED_FLAG    (1 << 4)
#define MONITORING_STOPPED_FLAG   (1 << 5)
#define HMI_STOPPED_FLAG          (1 << 6)



extern EventGroupHandle_t eventFlags;


#ifndef NDEBUG
#define DEBUG
extern SemaphoreHandle_t dbgMutex;

#define DEBUG_CONTROLLER
//#define DEBUG_BREATHING
//#define DEBUG_MONITOR
#define DEBUG_HMI


#define dbg_print(_fmt)       { xSemaphoreTake(dbgMutex,  portMAX_DELAY); write(_fmt); xSemaphoreGive(dbgMutex); }
#define dbg_printf(_fmt,...)  { xSemaphoreTake(dbgMutex,  portMAX_DELAY); printf(_fmt, ##__VA_ARGS__); xSemaphoreGive(dbgMutex); }

#ifdef DEBUG_HMI
#define hmi_print(_fmt)       dbg_print(_fmt)
#define hmi_printf(_fmt,...)  dbg_printf(_fmt,##__VA_ARGS__)
#else
#define hmi_print(_fmt)
#define hmi_printf(_fmt,...)
#endif

#ifdef DEBUG_CONTROLLER
#define ctrl_print(_fmt)       dbg_print(_fmt)
#define ctrl_printf(_fmt,...)  dbg_printf(_fmt,##__VA_ARGS__)
#else
#define ctrl_print(_fmt)
#define ctrl_printf(_fmt,...)
#endif

#ifdef DEBUG_MONITOR
#define mntr_print(_fmt)       dbg_print(_fmt)
#define mntr_printf(_fmt,...)  dbg_printf(_fmt,##__VA_ARGS__)
#else
#define mntr_print(_fmt)
#define mntr_printf(_fmt,...)
#endif

#ifdef DEBUG_BREATHING
#define brth_print(_fmt)       dbg_print(_fmt)
#define brth_printf(_fmt,...)  dbg_printf(_fmt,##__VA_ARGS__)
#else
#define brth_print(_fmt)
#define brth_printf(_fmt,...)
#endif

#endif




#endif