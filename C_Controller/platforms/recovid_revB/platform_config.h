#ifndef __PLATFORM_CONFIG_H__
#define __PLATFORM_CONFIG_H__

#define MAX_MOTOR_STEPS         (                                 4800)
#define MOTOR_STEP_TIME_INIT    (                                  300)
#define MOTOR_ACC_STEPS         (                                   50)
#define MOTOR_ACC_COEF          (MOTOR_STEP_TIME_INIT/ MOTOR_ACC_STEPS)
#endif
