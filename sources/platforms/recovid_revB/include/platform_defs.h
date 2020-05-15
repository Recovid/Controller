#ifndef __PLATFORM_DEFS_H__
#define __PLATFORM_DEFS_H__


// BAVU MOTOR platform specific defines

#define MOTOR_STEPS_PER_REV                   (1000)  
#define MOTOR_MAX_REV                         (1.25)   
#define MOTOR_MAX_STEPS                       (1250) // ((uint32_t)(MOTOR_STEPS_PER_REV*MOTOR_MAX_REV)) <-- non integer define thus compilation gives a warning: variably modified 'g_motor_steps_us' at file scope


#define MOTOR_STEP_US(rpm)                    ((uint32_t)((1000000.0*60.0)/(MOTOR_STEPS_PER_REV*rpm)))      // us/step

#define MOTOR_MAX_RPM                         (240.0)   // revolution per minutes
#define MOTOR_MIN_STEP_US                     MOTOR_STEP_US(MOTOR_MAX_RPM)



// MOTOR PEP platform specific defines

#define MOTOR_PEP_MAX_SPEED                 (4)             // mm/s
#define MOTOR_PEP_HOME_SPEED                (4)             // mm/s

#define MOTOR_PEP_STEPS_PER_mm              (200*8*1)       // steps_per_rev*microstepping*thread_mm_per_rev
#define MOTOR_PEP_MAX_mm                    (170)           // Max motor PEP distance in mm

#define MOTOR_PEP_mmH2O_TO_mm_FACTOR        (1.1)           // mmH2O to mm factor
#define MOTOR_PEP_MAX_STEPS                 ((int32_t)(MOTOR_PEP_MAX_mm) * MOTOR_PEP_STEPS_PER_mm)

#endif
