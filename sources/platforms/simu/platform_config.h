#ifndef __PLATFORM_CONFIG_H__
#define __PLATFORM_CONFIG_H__


#define BREATHING_TASK_STACK_SIZE   (1024)
#define MONITORING_TASK_STACK_SIZE  (768)
#define CONTROLLER_TASK_STACK_SIZE  (768)
#define HMI_TASK_STACK_SIZE         (1024)

#define MOTOR_CORRECTION_USTEPS (                                   1   )
#define MOTOR_STEP_TIME_INIT    (      400* ( MOTOR_CORRECTION_USTEPS)  )
#define MOTOR_ACC_STEPS         (        2* ( MOTOR_CORRECTION_USTEPS)  )
#define MOTOR_ACC_COEF          ( MOTOR_STEP_TIME_INIT/ MOTOR_ACC_STEPS )
#define MOTOR_RELEASE_STEP_US   (      300* ( MOTOR_CORRECTION_USTEPS)  )
#define MOTOR_HOME_STEP_US      (      400* ( MOTOR_CORRECTION_USTEPS)  )
#define MAX_MOTOR_STEPS         (     4800/ ( MOTOR_CORRECTION_USTEPS)  )

#define MAX_PEP_SAMPLES				  (10)

#endif
