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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_PULSE_WIDTH_US 10
#define PEP_PULSE_WIDTH_US 10
#define NUCLEO_BTN_Pin GPIO_PIN_13
#define NUCLEO_BTN_GPIO_Port GPIOC
#define NUCLEO_BTN_EXTI_IRQn EXTI15_10_IRQn
#define PEP_VALVE_Pin GPIO_PIN_0
#define PEP_VALVE_GPIO_Port GPIOC
#define TAMPON_FULL_Pin GPIO_PIN_2
#define TAMPON_FULL_GPIO_Port GPIOC
#define PEP_HOME_Pin GPIO_PIN_3
#define PEP_HOME_GPIO_Port GPIOC
#define PEP_HOME_EXTI_IRQn EXTI3_IRQn
#define DBG_TX_Pin GPIO_PIN_2
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_3
#define DBG_RX_GPIO_Port GPIOA
#define BUZZER_LOW_Pin GPIO_PIN_4
#define BUZZER_LOW_GPIO_Port GPIOA
#define NUCLEO_LED_Pin GPIO_PIN_5
#define NUCLEO_LED_GPIO_Port GPIOA
#define BUZZER_MEDIUM_Pin GPIO_PIN_6
#define BUZZER_MEDIUM_GPIO_Port GPIOA
#define BUZZER_HIGH_Pin GPIO_PIN_7
#define BUZZER_HIGH_GPIO_Port GPIOA
#define FS_Enabled_Pin GPIO_PIN_4
#define FS_Enabled_GPIO_Port GPIOC
#define PEP_nSLEEP_Pin GPIO_PIN_5
#define PEP_nSLEEP_GPIO_Port GPIOC
#define PEP_CONFIG_Pin GPIO_PIN_0
#define PEP_CONFIG_GPIO_Port GPIOB
#define PEP_nFAULT_Pin GPIO_PIN_1
#define PEP_nFAULT_GPIO_Port GPIOB
#define PEP_nFAULT_EXTI_IRQn EXTI1_IRQn
#define Enable_P5V_Rpi_Pin GPIO_PIN_2
#define Enable_P5V_Rpi_GPIO_Port GPIOB
#define PEP_nENBL_Pin GPIO_PIN_10
#define PEP_nENBL_GPIO_Port GPIOB
#define PEP_DIR_Pin GPIO_PIN_11
#define PEP_DIR_GPIO_Port GPIOB
#define PEP_MODE0_Pin GPIO_PIN_13
#define PEP_MODE0_GPIO_Port GPIOB
#define PEP_MODE1_Pin GPIO_PIN_14
#define PEP_MODE1_GPIO_Port GPIOB
#define MOTOR_INDEXPULSE_Pin GPIO_PIN_15
#define MOTOR_INDEXPULSE_GPIO_Port GPIOB
#define MAT_LED_GREEN_Pin GPIO_PIN_6
#define MAT_LED_GREEN_GPIO_Port GPIOC
#define MAT_LED_ORANGE_Pin GPIO_PIN_7
#define MAT_LED_ORANGE_GPIO_Port GPIOC
#define MAT_LED_RED_Pin GPIO_PIN_8
#define MAT_LED_RED_GPIO_Port GPIOC
#define PEP_STEP_Pin GPIO_PIN_9
#define PEP_STEP_GPIO_Port GPIOC
#define MOTOR_PWM_Pin GPIO_PIN_8
#define MOTOR_PWM_GPIO_Port GPIOA
#define FAN_ENABLE_Pin GPIO_PIN_9
#define FAN_ENABLE_GPIO_Port GPIOA
#define MOTOR_ACTIVE_Pin GPIO_PIN_10
#define MOTOR_ACTIVE_GPIO_Port GPIOA
#define MOTOR_ACTIVE_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR_LIMIT_SW_A_Pin GPIO_PIN_11
#define MOTOR_LIMIT_SW_A_GPIO_Port GPIOA
#define MOTOR_LIMIT_SW_A_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR_LIMIT_SW_B_Pin GPIO_PIN_12
#define MOTOR_LIMIT_SW_B_GPIO_Port GPIOA
#define MOTOR_LIMIT_SW_B_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define MOTOR_STEP_Pin GPIO_PIN_15
#define MOTOR_STEP_GPIO_Port GPIOA
#define IHM_TX_Pin GPIO_PIN_10
#define IHM_TX_GPIO_Port GPIOC
#define IHM_RX_Pin GPIO_PIN_11
#define IHM_RX_GPIO_Port GPIOC
#define MOTOR_DIR_Pin GPIO_PIN_12
#define MOTOR_DIR_GPIO_Port GPIOC
#define MOTOR_ENA_Pin GPIO_PIN_2
#define MOTOR_ENA_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define TAMPON_RUNNING_Pin GPIO_PIN_4
#define TAMPON_RUNNING_GPIO_Port GPIOB
#define TAMPON_FAIL_Pin GPIO_PIN_5
#define TAMPON_FAIL_GPIO_Port GPIOB
#define BATT_FAULT_Pin GPIO_PIN_7
#define BATT_FAULT_GPIO_Port GPIOB
#define SENSOR_SCL_Pin GPIO_PIN_8
#define SENSOR_SCL_GPIO_Port GPIOB
#define SENSOR_SDA_Pin GPIO_PIN_9
#define SENSOR_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
