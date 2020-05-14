#ifndef __PLATFORM_DEFS_H__
#define __PLATFORM_DEFS_H__


// BAVU MOTOR platform specific defines

#define MOTOR_STEPS_PER_REV                   (1000)  
#define MOTOR_MAX_REV                         (1.25)   
#define MOTOR_MAX_STEPS                       (1250) // ((uint32_t)(MOTOR_STEPS_PER_REV*MOTOR_MAX_REV)) <-- non integer define thus compilation gives a warning: variably modified 'g_motor_steps_us' at file scope


#define MOTOR_STEP_US(dps)                    ((uint32_t)((360.0*1000000.0)/(MOTOR_STEPS_PER_REV*dps)))      // us/step

#define MOTOR_MAX_SPEED                       (1440.0)   // degree/s
#define MOTOR_MIN_STEP_US                     MOTOR_STEP_US(MOTOR_MAX_SPEED)



// MOTOR PEP platform specific defines

#define MOTOR_PEP_STEPS_PER_MM              (200*8*1)     // steps_per_rev*microstepping*thread_mm_per_rev
#define MOTOR_PEP_PEP_TO_MM_FACTOR          (1.1)   // PEP to mm factor
#define MOTOR_PEP_MAX_cmH2O                 (10)    // Max PEP cmH2O
#define MOTOR_PEP_MAX_STEPS                 ((int32_t)(MOTOR_PEP_MAX_cmH2O*1000*MOTOR_PEP_PEP_TO_MM_FACTOR) * MOTOR_PEP_STEPS_PER_MM)

#endif
