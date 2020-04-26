#include "recovid_revB.h"

#include <stdbool.h>


I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
TIM_HandleTypeDef htim7;



TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_up;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch4_up;

UART_HandleTypeDef huart2;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;




static bool SystemClock_Config(void);
static bool MX_GPIO_Init(void);
static bool MX_DMA_Init(void);
static bool MX_USART2_UART_Init(void);
static bool MX_UART4_Init(void);
static bool MX_I2C1_Init(void);
static bool MX_TIM2_Init(void);
static bool MX_TIM3_Init(void);
static bool MX_TIM7_Init(void);


#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */




int init_hardware()
{
    HAL_Init();
    if(!SystemClock_Config()) return false;
    if(!MX_GPIO_Init()) return false;
    if(!MX_DMA_Init()) return false;
    if(!MX_USART2_UART_Init()) return false;
    if(!MX_UART4_Init()) return false;
    if(!MX_I2C1_Init()) return false;
    if(!MX_TIM2_Init()) return false;
    if(!MX_TIM3_Init()) return false;
    if(!MX_TIM7_Init()) return false;

	return true;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
static bool SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    return false;
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    return false;
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    return false;
  }
  return true;
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static bool MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    return false;
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    return false;
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    return false;
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
    return true;
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static bool MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (SystemCoreClock / 1000000) -1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    return false;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    return false;
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    return false;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    return false;
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    return false;
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);
    return true;

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static bool MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = (SystemCoreClock / 1000000) -1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    return false;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    return false;
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    return false;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    return false;
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PEP_PULSE_WIDTH_US;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    return false;
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
    return true;

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static bool MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = (SystemCoreClock / 1000000) -1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0xFFFF;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    return false;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    return false;
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */
  return true;
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static bool MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    return false;
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
    return true;

}


/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static bool MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    return false;
  }
 return true;
}

/** 
  * Enable DMA controller clock
  */
static bool MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 1);         // TIM2 Motor
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 1);         // TIM3 Motor PEP
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 1);         // I2C1 TX Sensors
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 1);         // I2C1 RX Sensors
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 1);         // UART HMI RX
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 5, 1);         // UART HMI TX
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

    return true;

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static bool MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PEP_VALVE_Pin|PEP_nSLEEP_Pin|MAT_LED_GREEN_Pin|MAT_LED_ORANGE_Pin 
                          |MAT_LED_RED_Pin|MOTOR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZZER_LOW_Pin|NUCLEO_LED_Pin|BUZZER_MEDIUM_Pin|BUZZER_HIGH_Pin 
                          |MOTOR_PWM_Pin|FAN_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PEP_CONFIG_Pin|PEP_nENBL_Pin|PEP_DIR_Pin 
                          |PEP_MODE0_Pin|PEP_MODE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Enable_P5V_Rpi_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NUCLEO_BTN_Pin */
  GPIO_InitStruct.Pin = NUCLEO_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NUCLEO_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PEP_VALVE_Pin PEP_nSLEEP_Pin MAT_LED_GREEN_Pin MAT_LED_ORANGE_Pin 
                           MAT_LED_RED_Pin MOTOR_DIR_Pin */
  GPIO_InitStruct.Pin = PEP_VALVE_Pin|PEP_nSLEEP_Pin|MAT_LED_GREEN_Pin|MAT_LED_ORANGE_Pin 
                          |MAT_LED_RED_Pin|MOTOR_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TAMPON_FULL_Pin FS_Enabled_Pin */
  GPIO_InitStruct.Pin = TAMPON_FULL_Pin|FS_Enabled_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PEP_HOME_Pin */
  GPIO_InitStruct.Pin = PEP_HOME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PEP_HOME_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_LOW_Pin NUCLEO_LED_Pin BUZZER_MEDIUM_Pin BUZZER_HIGH_Pin 
                           MOTOR_PWM_Pin FAN_ENABLE_Pin */
  GPIO_InitStruct.Pin = BUZZER_LOW_Pin|NUCLEO_LED_Pin|BUZZER_MEDIUM_Pin|BUZZER_HIGH_Pin 
                          |MOTOR_PWM_Pin|FAN_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PEP_CONFIG_Pin Enable_P5V_Rpi_Pin PEP_nENBL_Pin PEP_DIR_Pin 
                           PEP_MODE0_Pin PEP_MODE1_Pin */
  GPIO_InitStruct.Pin = PEP_CONFIG_Pin|Enable_P5V_Rpi_Pin|PEP_nENBL_Pin|PEP_DIR_Pin 
                          |PEP_MODE0_Pin|PEP_MODE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PEP_nFAULT_Pin*/
  GPIO_InitStruct.Pin = PEP_nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TAMPON_FAIL_Pin */
  GPIO_InitStruct.Pin = TAMPON_FAIL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BATT_FAULT_Pin */
  GPIO_InitStruct.Pin = BATT_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_INDEXPULSE_Pin */
  GPIO_InitStruct.Pin = MOTOR_INDEXPULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MOTOR_INDEXPULSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_LIMIT_SW_A_Pin MOTOR_LIMIT_SW_B_Pin */
  GPIO_InitStruct.Pin = MOTOR_LIMIT_SW_A_Pin|MOTOR_LIMIT_SW_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_ACTIVE_Pin */
  GPIO_InitStruct.Pin = MOTOR_ACTIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_ENA_Pin */
  GPIO_InitStruct.Pin = MOTOR_ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_ENA_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 2);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 2);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    return true;

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */



__weak void motor_limit_sw_A_irq() {
}
__weak void motor_limit_sw_B_irq() {
}
__weak void motor_active_irq() {
}
__weak void pep_home_irq() {
}
__weak void pep_nfault_irq() {
}
__weak void fs_enabled_irq() {
}
__weak void ups_fail_irq() {
}
__weak void ups_full_irq() {
}
__weak void batt_fault_irq() {
}


void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  switch(pin) {
    case MOTOR_LIMIT_SW_A_Pin : motor_limit_sw_A_irq(); break;
    case MOTOR_LIMIT_SW_B_Pin : motor_limit_sw_B_irq(); break;
    case MOTOR_ACTIVE_Pin     : motor_active_irq(); break;
    case PEP_HOME_Pin         : pep_home_irq(); break;
    case PEP_nFAULT_Pin       : pep_nfault_irq(); break;
    case FS_Enabled_Pin       : fs_enabled_irq(); break;
    case TAMPON_FAIL_Pin      : ups_fail_irq(); break;
    case TAMPON_FULL_Pin      : ups_full_irq(); break;
    case BATT_FAULT_Pin       : batt_fault_irq(); break;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}


/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
__attribute__((used)) PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the huart2 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
//	ITM_SendChar(ch);
	return ch;
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
