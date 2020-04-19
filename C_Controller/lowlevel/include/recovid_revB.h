/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RECOVID_REVB_H
#define __RECOVID_REVB_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

  extern I2C_HandleTypeDef hi2c1;           // Sensors I2C
  extern DMA_HandleTypeDef hdma_i2c1_rx;    // Sensors DMA RX

  extern TIM_HandleTypeDef htim2;           // BAVU Motor steps timer
  extern DMA_HandleTypeDef hdma_tim2_up;    // BAVU Motor DMA UP

  extern TIM_HandleTypeDef htim3;           // PEP Motor steps timer
  extern DMA_HandleTypeDef hdma_tim3_ch4_up;// PEP Motor DMA UP

  extern UART_HandleTypeDef huart2;



// I2C Sensors

#define ADDR_SPD610 	((uint16_t)(0x40 <<1))
#define ADDR_NPA700B 	((uint16_t)(0x76 <<1))

#define sensors_i2c   hi2c1

#define SENSOR_SCL_Pin GPIO_PIN_8
#define SENSOR_SCL_GPIO_Port GPIOB
#define SENSOR_SDA_Pin GPIO_PIN_9
#define SENSOR_SDA_GPIO_Port GPIOB



// BAVU Motor
#define motor_tim     htim2
#define MOTOR_TIM_CHANNEL     TIM_CHANNEL_1

#define MOTOR_PULSE_WIDTH_US 10

#define MOTOR_PRESS_DIR     GPIO_PIN_RESET
#define MOTOR_RELEASE_DIR   GPIO_PIN_SET

#define MOTOR_RELEASE_STEP_US   200



#define MOTOR_INDEXPULSE_Pin GPIO_PIN_15
#define MOTOR_INDEXPULSE_GPIO_Port GPIOB
#define MOTOR_PWM_Pin GPIO_PIN_8
#define MOTOR_PWM_GPIO_Port GPIOA
#define MOTOR_ACTIVE_Pin GPIO_PIN_10
#define MOTOR_ACTIVE_GPIO_Port GPIOA
#define MOTOR_ACTIVE_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR_LIMIT_SW_A_Pin GPIO_PIN_11
#define MOTOR_LIMIT_SW_A_GPIO_Port GPIOA
#define MOTOR_LIMIT_SW_A_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR_LIMIT_SW_B_Pin GPIO_PIN_12
#define MOTOR_LIMIT_SW_B_GPIO_Port GPIOA
#define MOTOR_LIMIT_SW_B_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR_STEP_Pin GPIO_PIN_15
#define MOTOR_STEP_GPIO_Port GPIOA
#define MOTOR_DIR_Pin GPIO_PIN_12
#define MOTOR_DIR_GPIO_Port GPIOC
#define MOTOR_ENA_Pin GPIO_PIN_2
#define MOTOR_ENA_GPIO_Port GPIOD

void motor_limit_sw_A_irq();
void motor_limit_sw_B_irq();
void motor_active_irq();



// PEP Motor
#define pep_tim        htim3
#define PEP_TIM_CHANNEL     TIM_CHANNEL_4
#define PEP_PULSE_WIDTH_US 10

#define PEP_HOME_Pin GPIO_PIN_3
#define PEP_HOME_GPIO_Port GPIOC
#define PEP_HOME_EXTI_IRQn EXTI3_IRQn

#define PEP_nSLEEP_Pin GPIO_PIN_5
#define PEP_nSLEEP_GPIO_Port GPIOC
#define PEP_CONFIG_Pin GPIO_PIN_0
#define PEP_CONFIG_GPIO_Port GPIOB
#define PEP_nFAULT_Pin GPIO_PIN_1
#define PEP_nFAULT_GPIO_Port GPIOB
#define PEP_nFAULT_EXTI_IRQn EXTI1_IRQn
#define PEP_nENBL_Pin GPIO_PIN_10
#define PEP_nENBL_GPIO_Port GPIOB
#define PEP_DIR_Pin GPIO_PIN_11
#define PEP_DIR_GPIO_Port GPIOB
#define PEP_MODE0_Pin GPIO_PIN_13
#define PEP_MODE0_GPIO_Port GPIOB
#define PEP_MODE1_Pin GPIO_PIN_14
#define PEP_MODE1_GPIO_Port GPIOB
#define PEP_STEP_Pin GPIO_PIN_9
#define PEP_STEP_GPIO_Port GPIOC

#define PEP_STEPS_PER_MM  (200*8/2)
#define PEP_MAX_SPEED     (8)         //  mm/s
#define PEP_HOME_SPEED    (1.25)      //  mm/s

#define PEP_DIR_INC	      GPIO_PIN_SET
#define PEP_DIR_DEC	      GPIO_PIN_RESET



void pep_home_irq();
void pep_nfault_irq();



// PEP Valve
#define PEP_VALVE_INHALE GPIO_PIN_SET
#define PEP_VALVE_EXHALE GPIO_PIN_RESET

#define PEP_VALVE_Pin GPIO_PIN_0
#define PEP_VALVE_GPIO_Port GPIOC


// Buzzer
#define BUZZER_LOW_Pin GPIO_PIN_4
#define BUZZER_LOW_GPIO_Port GPIOA
#define BUZZER_MEDIUM_Pin GPIO_PIN_6
#define BUZZER_MEDIUM_GPIO_Port GPIOA
#define BUZZER_HIGH_Pin GPIO_PIN_7
#define BUZZER_HIGH_GPIO_Port GPIOA


// Lights
#define MAT_LED_GREEN_Pin GPIO_PIN_6
#define MAT_LED_GREEN_GPIO_Port GPIOC
#define MAT_LED_ORANGE_Pin GPIO_PIN_7
#define MAT_LED_ORANGE_GPIO_Port GPIOC
#define MAT_LED_RED_Pin GPIO_PIN_8
#define MAT_LED_RED_GPIO_Port GPIOC


// UPS
#define TAMPON_FULL_Pin GPIO_PIN_2
#define TAMPON_FULL_GPIO_Port GPIOC
#define TAMPON_FULL_EXTI_IRQn EXTI2_TSC_IRQn

#define TAMPON_FAIL_Pin GPIO_PIN_5
#define TAMPON_FAIL_GPIO_Port GPIOB
#define TAMPON_FAIL_EXTI_IRQn EXTI9_5_IRQn

void ups_fail_irq();
void ups_full_irq();


// Battery
#define BATT_FAULT_Pin GPIO_PIN_7
#define BATT_FAULT_GPIO_Port GPIOB
#define BATT_FAULT_EXTI_IRQn EXTI9_5_IRQn

void batt_fault_irq();

// FailSafe
#define FS_Enabled_Pin GPIO_PIN_4
#define FS_Enabled_GPIO_Port GPIOC
#define FS_Enabled_EXTI_IRQn EXTI4_IRQn

void fs_enabled_irq();


// Fan
#define FAN_ENABLE_Pin GPIO_PIN_9
#define FAN_ENABLE_GPIO_Port GPIOA


// Rpi
#define Enable_P5V_Rpi_Pin GPIO_PIN_2
#define Enable_P5V_Rpi_GPIO_Port GPIOB


// Nucleo Button and led
#define NUCLEO_BTN_Pin GPIO_PIN_13
#define NUCLEO_BTN_GPIO_Port GPIOC
#define NUCLEO_BTN_EXTI_IRQn EXTI15_10_IRQn
#define NUCLEO_LED_Pin GPIO_PIN_5
#define NUCLEO_LED_GPIO_Port GPIOA


// Debug serial
#define DBG_TX_Pin GPIO_PIN_2
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_3
#define DBG_RX_GPIO_Port GPIOA


// Programmer
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA

// SWO ITM Trace
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB



void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
