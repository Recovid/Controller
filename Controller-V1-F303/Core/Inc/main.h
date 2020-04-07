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
#define STEP_PULSE 15
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define ROTARY2_A_Pin GPIO_PIN_0
#define ROTARY2_A_GPIO_Port GPIOC
#define ROTARY2_B_Pin GPIO_PIN_1
#define ROTARY2_B_GPIO_Port GPIOC
#define ROTARY2_BTN_Pin GPIO_PIN_2
#define ROTARY2_BTN_GPIO_Port GPIOC
#define IHM_TX_Pin GPIO_PIN_2
#define IHM_TX_GPIO_Port GPIOA
#define IHM_RX_Pin GPIO_PIN_3
#define IHM_RX_GPIO_Port GPIOA
#define MOTOR_ENA_Pin GPIO_PIN_5
#define MOTOR_ENA_GPIO_Port GPIOA
#define MOTOR_DIR_Pin GPIO_PIN_6
#define MOTOR_DIR_GPIO_Port GPIOA
#define MOTOR_STP_Pin GPIO_PIN_7
#define MOTOR_STP_GPIO_Port GPIOA
#define DBG_TX_Pin GPIO_PIN_4
#define DBG_TX_GPIO_Port GPIOC
#define DBG_RX_Pin GPIO_PIN_5
#define DBG_RX_GPIO_Port GPIOC
#define ROTARY1_B_Pin GPIO_PIN_6
#define ROTARY1_B_GPIO_Port GPIOC
#define ROTARY1_A_Pin GPIO_PIN_7
#define ROTARY1_A_GPIO_Port GPIOC
#define ROTARY1_BTN_Pin GPIO_PIN_8
#define ROTARY1_BTN_GPIO_Port GPIOC
#define LCD_SCL_Pin GPIO_PIN_9
#define LCD_SCL_GPIO_Port GPIOA
#define LCD_SDA_Pin GPIO_PIN_10
#define LCD_SDA_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define HOME_Pin GPIO_PIN_4
#define HOME_GPIO_Port GPIOB
#define HOME_EXTI_IRQn EXTI4_IRQn
#define PEEP_Pin GPIO_PIN_5
#define PEEP_GPIO_Port GPIOB
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
