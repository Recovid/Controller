#ifndef __PLATFORM_DEFS_H__
#define __PLATFORM_DEFS_H__


#define TASK_PRIORITY_LOW            8
#define TASK_PRIORITY_BELOW_NORMAL   16
#define TASK_PRIORITY_NORMAL         24
#define TASK_PRIORITY_ABOVE_NORMAL   32
#define TASK_PRIORITY_HIGH           40
#define TASK_PRIORITY_CRITICAL       48



#define PLATFORM_BREATHING_TASK_PRIORITY     (TASK_PRIORITY_HIGH)
#define PLATFORM_MONITORING_TASK_PRIORITY    (TASK_PRIORITY_CRITICAL)
#define PLATFORM_CONTROLLER_TASK_PRIORITY    (TASK_PRIORITY_NORMAL)
#define PLATFORM_HMI_TASK_PRIORITY           (TASK_PRIORITY_BELOW_NORMAL)
#define PLATFORM_TIMER_TASK_PRIORITY         (TASK_PRIORITY_HIGH+1)



// platform specific FreeRTOS stack size
#define PLATFORM_FREERTOS_HEAP_SIZE          (64*1024)    // in bytes
#define PLATFORM_BREATHING_TASK_STACK_SIZE   (1024)       // in word
#define PLATFORM_MONITORING_TASK_STACK_SIZE  (1024)       // in word
#define PLATFORM_CONTROLLER_TASK_STACK_SIZE  (1024)       // in word
#define PLATFORM_HMI_TASK_STACK_SIZE         (1024)       // in word
#define PLATFORM_TIMER_TASK_STACK_SIZE       (256)        // in word
#define PLATFORM_MINIMAL_STACK_SIZE          (128)        // in word  

#define TOTAL_STACK_SIZE            (PLATFORM_BREATHING_TASK_STACK_SIZE + \
                                     PLATFORM_MONITORING_TASK_STACK_SIZE + \
                                     PLATFORM_CONTROLLER_TASK_STACK_SIZE + \
                                     PLATFORM_HMI_TASK_STACK_SIZE + \
                                     PLATFORM_TIMER_TASK_STACK_SIZE + \
                                     2*PLATFORM_MINIMAL_STACK_SIZE )

#if ( (PLATFORM_FREERTOS_HEAP_SIZE/4) < (TOTAL_STACK_SIZE) )
#error Not enough heap to create all tasks and events
#endif





// BAVU MOTOR platform specific defines

#define MOTOR_STEPS_PER_REV                   (1000)  
#define MOTOR_MAX_REV                         (1.25)   
#define MOTOR_MAX_STEPS                       (MOTOR_STEPS_PER_REV*MOTOR_MAX_REV)


#define MOTOR_HOME_SPEED                      (225.0)   // degree/s
#define MOTOR_RELEASE_SPEED                   (300.0)   // degree/s

#define MOTOR_HOME_STEP_US                    (360.0*1000000/(MOTOR_STEPS_PER_REV*MOTOR_HOME_SPEED))      // us/step
#define MOTOR_RELEASE_STEP_US                 (360.0*1000000/(MOTOR_STEPS_PER_REV*MOTOR_RELEASE_SPEED))   // us/step


// MOTOR PEP platform specific defines

#define MOTOR_PEP_PEP_TO_MM_FACTOR            (1.7)   // PEP to mm factor

#endif
