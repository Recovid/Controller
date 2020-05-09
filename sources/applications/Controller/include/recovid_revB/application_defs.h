#ifndef __APPLICATION_DEFS_H__
#define __APPLICATION_DEFS_H__

#include "platform_defs.h"


#define TASK_PRIORITY_LOW            8
#define TASK_PRIORITY_BELOW_NORMAL   16
#define TASK_PRIORITY_NORMAL         24
#define TASK_PRIORITY_ABOVE_NORMAL   32
#define TASK_PRIORITY_HIGH           40
#define TASK_PRIORITY_CRITICAL       48


#define BREATHING_TASK_PRIORITY     (TASK_PRIORITY_HIGH)
#define MONITORING_TASK_PRIORITY    (TASK_PRIORITY_CRITICAL)
#define CONTROLLER_TASK_PRIORITY    (TASK_PRIORITY_NORMAL)
#define HMI_TASK_PRIORITY           (TASK_PRIORITY_BELOW_NORMAL)
#define TIMER_TASK_PRIORITY         (TASK_PRIORITY_HIGH+1)



// platform specific FreeRTOS stack size
#define FREERTOS_HEAP_SIZE          (14*1024)    // in bytes
#define BREATHING_TASK_STACK_SIZE   (512)       // in word
#define MONITORING_TASK_STACK_SIZE  (512)       // in word
#define CONTROLLER_TASK_STACK_SIZE  (512)       // in word
#define HMI_TASK_STACK_SIZE         (512)       // in word
#define TIMER_TASK_STACK_SIZE       (512)        // in word
#define MINIMAL_STACK_SIZE          (512)        // in word  

#define TOTAL_STACK_SIZE            (BREATHING_TASK_STACK_SIZE + \
                                     MONITORING_TASK_STACK_SIZE + \
                                     CONTROLLER_TASK_STACK_SIZE + \
                                     HMI_TASK_STACK_SIZE + \
                                     TIMER_TASK_STACK_SIZE + \
                                     2*MINIMAL_STACK_SIZE )

#if ( (FREERTOS_HEAP_SIZE/4) < (TOTAL_STACK_SIZE) )
#error Not enough heap to create all tasks and events
#endif



#define MOTOR_HOME_SPEED                      (225.0)   // degree/s
#define MOTOR_RELEASE_SPEED                   (300.0)   // degree/s
#define MOTOR_HOME_STEP_US                    ((uint32_t)((360.0*1000000.0)/(MOTOR_STEPS_PER_REV*MOTOR_HOME_SPEED)))      // us/step
#define MOTOR_RELEASE_STEP_US                 ((uint32_t)((360.0*1000000.0)/(MOTOR_STEPS_PER_REV*MOTOR_RELEASE_SPEED)))   // us/step


#endif