#ifndef __PLATFORM_DEFS_H__
#define __PLATFORM_DEFS_H__


#define PLATFORM_BREATHING_TASK_STACK_SIZE   (1024)
#define PLATFORM_MONITORING_TASK_STACK_SIZE  (768)
#define PLATFORM_CONTROLLER_TASK_STACK_SIZE  (768)
#define PLATFORM_HMI_TASK_STACK_SIZE         (1024)


#define MOTOR_STEPS_PER_REV                   (1000)  
#define MOTOR_MAX_REV                         (4.8)   
#define MOTOR_MAX_STEPS                       ((uint32_t)(MOTOR_STEPS_PER_REV*MOTOR_MAX_REV))

#define MOTOR_HOME_STEP_US                    (400)
#define MOTOR_RELEASE_STEP_US                 (300)


#endif
