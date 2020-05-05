/**
  ******************************************************************************
  * @file           : Recovid_revB bsp file
  * @brief          : Initializes the board peripherals
  *                   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RECOVID_REVB_H__
#define __RECOVID_REVB_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <stdint.h>


extern I2C_HandleTypeDef hi2c1;             // Sensors I2C
extern DMA_HandleTypeDef hdma_i2c1_rx;      // Sensors DMA RX
extern DMA_HandleTypeDef hdma_i2c1_rt;      // Sensors DMA RX
extern TIM_HandleTypeDef htim7;             // Timer us for volume integration


extern TIM_HandleTypeDef htim2;             // BAVU Motor steps timer
extern DMA_HandleTypeDef hdma_tim2_up;     // BAVU Motor DMA UP

extern TIM_HandleTypeDef htim3;             // PEP Motor steps timer
extern DMA_HandleTypeDef hdma_tim3_ch4_up;  // PEP Motor DMA UP

extern UART_HandleTypeDef huart4;           // HMI uart
extern DMA_HandleTypeDef  hdma_uart4_rx;   // HMI uart DMA rx
extern DMA_HandleTypeDef  hdma_uart4_tx;   // HMI uart DMA tx

extern UART_HandleTypeDef huart2;           // Dbg uart


// System IRQ priorities

#define MOTOR_TIM_IRQ_PRIORITY              (5)
#define MOTOR_LIMIT_SW_IRQ_PRIORITY         (3)

#define PEP_TIM_IRQ_PRIORITY                (0)    

#define SENSORS_I2C_EV_IRQ_PRIORITY         (0)
#define SENSORS_I2C_ER_IRQ_PRIORITY         (0)

#define PEP_HOME_EXTI_IRQ_PRIORITY          (1)

#define HMI_UART_DMA_TX_IRQ_PRIORITY        (1)
#define HMI_UART_DMA_RX_IRQ_PRIORITY        (2)
#define HMI_UART_IRQ_PRIORITY               (2)


// System Peripheral's configuration

// I2C Sensors
#define ADDR_SPD610 	                      ((uint16_t)(0x40 <<1))
#define ADDR_NPA700B 	                      ((uint16_t)(0x68 <<1))
#define ADDR_BMP280 	                      ((uint16_t)(0x76 <<1))


#define sensors_i2c                         hi2c1

#define SENSOR_SCL_Pin                      GPIO_PIN_8
#define SENSOR_SCL_GPIO_Port                GPIOB
#define SENSOR_SDA_Pin                      GPIO_PIN_9
#define SENSOR_SDA_GPIO_Port                GPIOB

// Timer us for flow integration.
#define timer_us                            htim7       



// BAVU Motor
#define motor_tim                           htim2
#define MOTOR_TIM_CHANNEL                   TIM_CHANNEL_1

#define MOTOR_PULSE_WIDTH_US                (10)
#define MOTOR_PRESS_DIR                     GPIO_PIN_RESET
#define MOTOR_RELEASE_DIR                   GPIO_PIN_SET



#define MOTOR_INDEXPULSE_Pin                GPIO_PIN_15
#define MOTOR_INDEXPULSE_GPIO_Port          GPIOB
#define MOTOR_PWM_Pin                       GPIO_PIN_8
#define MOTOR_PWM_GPIO_Port                 GPIOA
#define MOTOR_ACTIVE_Pin                    GPIO_PIN_10
#define MOTOR_ACTIVE_GPIO_Port              GPIOA
#define MOTOR_ACTIVE_EXTI_IRQn              EXTI15_10_IRQn
#define MOTOR_LIMIT_SW_A_Pin                GPIO_PIN_11
#define MOTOR_LIMIT_SW_A_GPIO_Port          GPIOA
#define MOTOR_LIMIT_SW_A_EXTI_IRQn          EXTI15_10_IRQn
#define MOTOR_LIMIT_SW_B_Pin                GPIO_PIN_12
#define MOTOR_LIMIT_SW_B_GPIO_Port          GPIOA
#define MOTOR_LIMIT_SW_B_EXTI_IRQn          EXTI15_10_IRQn
#define MOTOR_STEP_Pin                      GPIO_PIN_15
#define MOTOR_STEP_GPIO_Port                GPIOA
#define MOTOR_DIR_Pin                       GPIO_PIN_12
#define MOTOR_DIR_GPIO_Port                 GPIOC
#define MOTOR_ENA_Pin                       GPIO_PIN_2
#define MOTOR_ENA_GPIO_Port                 GPIOD

void motor_limit_sw_A_irq();
void motor_limit_sw_B_irq();
void motor_active_irq();



// PEP Motor
#define pep_tim                             htim3
#define PEP_TIM_CHANNEL                     TIM_CHANNEL_4

#define PEP_PULSE_WIDTH_US                  10
#define PEP_STEPS_PER_MM                    (200*8*1)     // steps_per_rev*microstepping*thread_mm_per_rev
#define PEP_MAX_SPEED                       (4)           // mm/s
#define PEP_HOME_SPEED                      (0.6)         // mm/s

#define PEP_DIR_INC	                        GPIO_PIN_SET
#define PEP_DIR_DEC	                        GPIO_PIN_RESET



#define PEP_HOME_Pin                        GPIO_PIN_3
#define PEP_HOME_GPIO_Port                  GPIOC
#define PEP_HOME_EXTI_IRQn                  EXTI3_IRQn


#define PEP_nSLEEP_Pin                      GPIO_PIN_5
#define PEP_nSLEEP_GPIO_Port                GPIOC
#define PEP_CONFIG_Pin                      GPIO_PIN_0
#define PEP_CONFIG_GPIO_Port                GPIOB
#define PEP_nFAULT_Pin                      GPIO_PIN_1
#define PEP_nFAULT_GPIO_Port                GPIOB
#define PEP_nFAULT_EXTI_IRQn                EXTI1_IRQn
#define PEP_nENBL_Pin                       GPIO_PIN_10
#define PEP_nENBL_GPIO_Port                 GPIOB
#define PEP_DIR_Pin                         GPIO_PIN_11
#define PEP_DIR_GPIO_Port                   GPIOB
#define PEP_MODE0_Pin                       GPIO_PIN_13
#define PEP_MODE0_GPIO_Port                 GPIOB
#define PEP_MODE1_Pin                       GPIO_PIN_14
#define PEP_MODE1_GPIO_Port                 GPIOB
#define PEP_STEP_Pin                        GPIO_PIN_9
#define PEP_STEP_GPIO_Port                  GPIOC




void pep_home_irq();
void pep_nfault_irq();



// PEP Valve
#define PEP_VALVE_Pin                       GPIO_PIN_0
#define PEP_VALVE_GPIO_Port                 GPIOC

#define PEP_VALVE_INHALE                    GPIO_PIN_SET
#define PEP_VALVE_EXHALE                    GPIO_PIN_RESET



// Buzzer
#define BUZZER_LOW_Pin                      GPIO_PIN_4
#define BUZZER_LOW_GPIO_Port                GPIOA
#define BUZZER_MEDIUM_Pin                   GPIO_PIN_6
#define BUZZER_MEDIUM_GPIO_Port             GPIOA
#define BUZZER_HIGH_Pin                     GPIO_PIN_7
#define BUZZER_HIGH_GPIO_Port               GPIOA


// Lights
#define MAT_LED_GREEN_Pin                   GPIO_PIN_6
#define MAT_LED_GREEN_GPIO_Port             GPIOC
#define MAT_LED_ORANGE_Pin                  GPIO_PIN_7
#define MAT_LED_ORANGE_GPIO_Port            GPIOC
#define MAT_LED_RED_Pin                     GPIO_PIN_8
#define MAT_LED_RED_GPIO_Port               GPIOC


// UPS
#define TAMPON_FULL_Pin                     GPIO_PIN_2
#define TAMPON_FULL_GPIO_Port               GPIOC
#define TAMPON_FULL_EXTI_IRQn               EXTI2_TSC_IRQn

#define TAMPON_FAIL_Pin                     GPIO_PIN_5
#define TAMPON_FAIL_GPIO_Port               GPIOB
#define TAMPON_FAIL_EXTI_IRQn               EXTI9_5_IRQn

void ups_fail_irq();
void ups_full_irq();


// Battery
#define BATT_FAULT_Pin                      GPIO_PIN_7
#define BATT_FAULT_GPIO_Port                GPIOB
#define BATT_FAULT_EXTI_IRQn                EXTI9_5_IRQn

void batt_fault_irq();

// FailSafe
#define FS_Enabled_Pin                      GPIO_PIN_4
#define FS_Enabled_GPIO_Port                GPIOC
#define FS_Enabled_EXTI_IRQn                EXTI4_IRQn

void fs_enabled_irq();


// Fan
#define FAN_ENABLE_Pin                      GPIO_PIN_9
#define FAN_ENABLE_GPIO_Port                GPIOA


// Rpi
#define Enable_P5V_Rpi_Pin                  GPIO_PIN_2
#define Enable_P5V_Rpi_GPIO_Port            GPIOB


// Nucleo Button and led
#define NUCLEO_BTN_Pin                      GPIO_PIN_13
#define NUCLEO_BTN_GPIO_Port                GPIOC
#define NUCLEO_LED_Pin                      GPIO_PIN_5
#define NUCLEO_LED_GPIO_Port                GPIOA

// HMI uart DMA rx
#define hmi_uart                            huart4
#define hmi_dma_rx                          hdma_uart4_rx
#define hmi_dma_tx                          hdma_uart4_tx
#define HMI_TX_Pin                          GPIO_PIN_10
#define HMI_TX_GPIO_Port                    GPIOC
#define HMI_RX_Pin                          GPIO_PIN_11
#define HMI_RX_GPIO_Port                    GPIOC
#define HMI_UART_IRQn                       UART4_IRQn
#define HMI_DMA_CHANNEL_TX                  DMA2_Channel5
#define HMI_DMA_CHANNEL_TX_IRQn             DMA2_Channel5_IRQn
#define HMI_DMA_CHANNEL_RX                  DMA2_Channel3
#define HMI_DMA_CHANNEL_RX_IRQn             DMA2_Channel3_IRQn


// Debug serial
#define dbg_uart                            huart2
#define DBG_TX_Pin                          GPIO_PIN_2
#define DBG_TX_GPIO_Port                    GPIOA
#define DBG_RX_Pin                          GPIO_PIN_3
#define DBG_RX_GPIO_Port                    GPIOA
#define DBG_UART_IRQn                       USART2_IRQn


// Programmer
#define SWDIO_Pin                           GPIO_PIN_13
#define SWDIO_GPIO_Port                     GPIOA
#define SWDCLK_Pin                          GPIO_PIN_14
#define SWDCLK_GPIO_Port                    GPIOA

// SWO ITM Trace
#define SWO_Pin                             GPIO_PIN_3
#define SWO_GPIO_Port                       GPIOB



void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
