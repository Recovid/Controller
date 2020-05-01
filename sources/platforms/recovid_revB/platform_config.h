#ifndef __PLATFORM_CONFIG_H__
#define __PLATFORM_CONFIG_H__


#define BREATHING_TASK_STACK_SIZE   (1024)
#define MONITORING_TASK_STACK_SIZE  (768)
#define CONTROLLER_TASK_STACK_SIZE  (768)
#define HMI_TASK_STACK_SIZE         (1024)





#define MOTOR_PULSE_WIDTH_US                10
#define MOTOR_PRESS_DIR                     GPIO_PIN_RESET
#define MOTOR_RELEASE_DIR                   GPIO_PIN_SET

#define MOTOR_CORRECTION_USTEPS (                                                                 1)
#define MOTOR_STEP_TIME_INIT    (                                  400/ ( MOTOR_CORRECTION_USTEPS) )
#define MOTOR_ACC_STEPS         (                                    2/ ( MOTOR_CORRECTION_USTEPS) )
#define MOTOR_ACC_COEF          (MOTOR_STEP_TIME_INIT/ MOTOR_ACC_STEPS/ ( MOTOR_CORRECTION_USTEPS) )
#define MOTOR_RELEASE_STEP_US   (                                  300/ ( MOTOR_CORRECTION_USTEPS) )
#define MOTOR_HOME_STEP_US      (                                  400/ ( MOTOR_CORRECTION_USTEPS) )
#define MAX_MOTOR_STEPS         (                                 4800/ ( MOTOR_CORRECTION_USTEPS) )



#define PEP_STEPS_PER_MM          (200*8)       // steps*microstepping
#define PEP_MAX_SPEED             (4)           // mm/s
#define PEP_HOME_SPEED            (0.6)         // mm/s

#define PEP_DIR_INC	              GPIO_PIN_SET
#define PEP_DIR_DEC	              GPIO_PIN_RESET


#define PEP_VALVE_INHALE          GPIO_PIN_SET
#define PEP_VALVE_EXHALE          GPIO_PIN_RESET






#define HMI_TX_BUFFER_SIZE            (4096)
#define HMI_TX_DMA_BUFFER_SIZE        (512)
#define HMI_TX_SYNC_TIMEOUT_MS        (2000)
#define HMI_RX_BUFFER_SIZE            (1024)
#define HMI_RX_DMA_BUFFER_SIZE        (512)
#define HMI_RX_LINE_TIMEOUT           (30)

#define MAX_PEP_SAMPLES				  (10)

#endif
