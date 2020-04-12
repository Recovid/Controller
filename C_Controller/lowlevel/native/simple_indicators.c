#include "simple_indicators.h"

#include <stdio.h>

void init_indicators() { }


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
