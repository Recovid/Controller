#include "simple_indicators.h"

#include <stdio.h>

#ifndef WIN32
bool blink() {
    init_indicators();
    int state = 0;
    while(1) {
        state = (state+1) % 2;
        light_nucleo((++state) == 0 ? On : Off);
// FIXME: does vTaskDelay work correctly on stm32 ?
#ifdef stm32f303
        HAL_Delay(500);
#else
        vTaskDelay(500);
#endif
    }
    return true;
}
#endif

bool init_indicators() { return true; /* nothing to do */ }


void printf_led(char const* name, OnOff val) {
    printf("set %s %s\n", name, val == On ? "on" : "off");
}

bool light_nucleo(OnOff new) {
    printf_led("nucleo", new);
    return true;
}

bool light_yellow(OnOff new) {
    printf_led("yellow", new);
    return true;
}

bool light_red(OnOff new) {
    printf_led("red", new);
    return true;
}

bool buzzer(OnOff new) {
    printf_led("buzzer", new);
    return true;
}
