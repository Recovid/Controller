#ifndef __PLATFORM_CONFIG_H__
#define __PLATFORM_CONFIG_H__

#define MAX_MOTOR_STEPS         (                                 4800)
#define MOTOR_STEP_TIME_INIT    (                                  400)
#define MOTOR_ACC_STEPS         (                                    2)
#define MOTOR_ACC_COEF          (MOTOR_STEP_TIME_INIT/ MOTOR_ACC_STEPS)
#endif
