#ifndef __PLATFORM_DEFS_H__
#define __PLATFORM_DEFS_H__

// platform specific FreeRTOS stack size
#define PLATFORM_FREERTOS_HEAP_SIZE          (16*1024)    // in bytes
#define PLATFORM_BREATHING_TASK_STACK_SIZE   (1024)       // in word
#define PLATFORM_MONITORING_TASK_STACK_SIZE  (768)        // in word
#define PLATFORM_CONTROLLER_TASK_STACK_SIZE  (768)        // in word
#define PLATFORM_HMI_TASK_STACK_SIZE         (1024)       // in word
#define PLATFORM_MINIMAL_STACK_SIZE          (128)        // in word  (for IDLE task)

#if ( (PLATFORM_FREERTOS_HEAP_SIZE/4) < (PLATFORM_BREATHING_TASK_STACK_SIZE)+(PLATFORM_MONITORING_TASK_STACK_SIZE)+(PLATFORM_CONTROLLER_TASK_STACK_SIZE)+(PLATFORM_HMI_TASK_STACK_SIZE)+ (2*PLATFORM_MINIMAL_STACK_SIZE) )
#error Not enough heap to create all tasks and events
#endif


// BAVU MOTOR platform specific defines

#define MOTOR_STEPS_PER_REV                   (4000)  
#define MOTOR_MAX_REV                         (1.2)   
#define MOTOR_MAX_STEPS                       ((uint32_t)(MOTOR_STEPS_PER_REV*MOTOR_MAX_REV))

#define MOTOR_HOME_STEP_US                    (400)
#define MOTOR_RELEASE_STEP_US                 (300)


// MOTOR PEP platform specific defines

#define MOTOR_PEP_PEP_TO_MM_FACTOR            (1.7)   // PEP to mm factor


#endif
