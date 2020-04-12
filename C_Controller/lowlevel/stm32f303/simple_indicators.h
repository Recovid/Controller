#include "simple_indicators.h"

#include "stm32f3xx_hal.h"

#define LED_OnNucleo_Pin GPIO_PIN_3
#define LED_OnNucleo_GPIO_Port GPIOB


void init_indicators() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure On-board Nucleo Led
    HAL_GPIO_WritePin(LED_OnNucleo_GPIO_Port, LED_OnNucleo_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED_OnNucleo_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_OnNucleo_GPIO_Port, &GPIO_InitStruct);
}

bool light_nucleo(OnOff val) {
    HAL_GPIO_WritePin(LED_OnNucleo_GPIO_Port, LED_OnNucleo_Pin, val == On ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

bool light_yellow(OnOff val) {
    // TODO:
    // HAL_GPIO_WritePin(LED_OnNucleo_GPIO_Port, LED_OnNucleo_Pin, val == On ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

bool light_red(OnOff val) {
    // TODO:
    // HAL_GPIO_WritePin(LED_OnNucleo_GPIO_Port, LED_OnNucleo_Pin, val == On ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

bool buzzer(OnOff val) {
    // TODO:
    // HAL_GPIO_WritePin(LED_OnNucleo_GPIO_Port, LED_OnNucleo_Pin, val == On ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}
