#include "leds.h"

void leds_init() {
}

static led_onnucleo_state = 0;

void led_onnucleo_printf() {
    printf("led_on_nucleo %s\n", led_onnucleo_state == 0 ? "off" : "on");
}

void led_onnucleo_toggle() {
    led_onnucleo_state = (led_onnucleo_state + 1) % 2;
    led_onnucleo_printf();
}

void led_onnucleo_set(int status) {
    led_onnucleo_state = status % 2;
    led_onnucleo_printf();
}
