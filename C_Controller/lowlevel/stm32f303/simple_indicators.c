#include "simple_indicators.h"

#include "lowlevel.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_gpio.h"

#define LED_OnNucleo_Pin GPIO_PIN_5
#define LED_OnNucleo_GPIO_Port GPIOA

#define LED_Mat_GPIO_Port GPIOC
#define LED_Mat_Red_Pin GPIO_PIN_8
#define LED_Mat_Green_Pin GPIO_PIN_6
#define LED_Mat_Orange_Pin GPIO_PIN_7

#define Buzzer_GPIO_Port GPIOA
#define Buzzer_Low_Pin GPIO_PIN_4
#define Buzzer_Medium_Pin GPIO_PIN_6
#define Buzzer_High_Pin GPIO_PIN_7


bool init_indicators() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Configure On-board Nucleo Led
    HAL_GPIO_WritePin(LED_OnNucleo_GPIO_Port, LED_OnNucleo_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED_OnNucleo_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_OnNucleo_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(LED_Mat_GPIO_Port, LED_Mat_Red_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED_Mat_Red_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_Mat_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(LED_Mat_GPIO_Port, LED_Mat_Green_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED_Mat_Green_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_Mat_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(LED_Mat_GPIO_Port, LED_Mat_Orange_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED_Mat_Orange_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_Mat_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Low_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = Buzzer_Low_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Medium_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = Buzzer_Medium_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_High_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = Buzzer_High_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);


	return true;
}

bool light_nucleo(OnOff val) {
    HAL_GPIO_WritePin(LED_OnNucleo_GPIO_Port, LED_OnNucleo_Pin, val == On ? GPIO_PIN_RESET : GPIO_PIN_SET);
    return true;
}

bool light_green(OnOff val) {
    HAL_GPIO_WritePin(LED_Mat_GPIO_Port, LED_Mat_Green_Pin, val == On ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}


bool light_yellow(OnOff val) {
    HAL_GPIO_WritePin(LED_Mat_GPIO_Port, LED_Mat_Orange_Pin, val == On ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

bool light_red(OnOff val) {
    HAL_GPIO_WritePin(LED_Mat_GPIO_Port, LED_Mat_Red_Pin, val == On ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}


bool buzzer_medium(OnOff val) {
	buzzer_low(val);
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Medium_Pin, val == On ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

bool buzzer_high(OnOff val) {
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_High_Pin, val == On ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

bool buzzer_low(OnOff val) {
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Low_Pin, val == On ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}
