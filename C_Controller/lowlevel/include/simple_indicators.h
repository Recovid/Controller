#pragma once

#include <stdbool.h>

#ifndef WIN32
bool blink();
#endif

typedef enum { On, Off } OnOff;

void init_indicators();

bool light_nucleo(OnOff); // on nucleo led
bool light_yellow(OnOff); // 4m visible leds
bool light_red   (OnOff); // 4m visible leds
bool light_green (OnOff); // 4m visible leds
bool buzzer      (OnOff); // onboard
