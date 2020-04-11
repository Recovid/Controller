#include "i2c.h"
#include "leds.h"

int init_hardware()
{
	leds_init();
	return i2c_init();
}
