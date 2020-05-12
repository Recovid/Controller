#ifndef __PLATFORM_DEFS_H__
#define __PLATFORM_DEFS_H__


// BAVU MOTOR platform specific defines

#define MOTOR_STEPS_PER_REV                   (1000)  
#define MOTOR_MAX_REV                         (1.25)   
#define MOTOR_MAX_STEPS                       (1250) // ((uint32_t)(MOTOR_STEPS_PER_REV*MOTOR_MAX_REV)) <-- non integer define thus compilation gives a warning: variably modified 'g_motor_steps_us' at file scope


#define MOTOR_MAX_SPEED                       (900.0)   // degree/s
#define MOTOR_MIN_STEP_US                     ((uint32_t)((360.0*1000000.0)/(MOTOR_STEPS_PER_REV*MOTOR_MAX_SPEED)))      // us/step



// MOTOR PEP platform specific defines

#define MOTOR_PEP_PEP_TO_MM_FACTOR            (1.7)   // PEP to mm factor


#endif
