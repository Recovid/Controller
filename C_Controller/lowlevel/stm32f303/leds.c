#include "leds.h"

#include "stm32f3xx_hal.h"

#define LED_OnNucleo_Pin GPIO_PIN_5
#define LED_OnNucleo_GPIO_Port GPIOA


void leds_init() {
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


void led_onnucleo_toggle() {
    HAL_GPIO_TogglePin(LED_OnNucleo_GPIO_Port, LED_OnNucleo_Pin);
}
void led_onnucleo_set(int status) {
    HAL_GPIO_WritePin(LED_OnNucleo_GPIO_Port, LED_OnNucleo_Pin,
        (status == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}
