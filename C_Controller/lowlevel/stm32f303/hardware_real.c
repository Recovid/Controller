#include "i2c.h"
#include "simple_indicators.h"

int init_hardware()
{
    HAL_Init();
    SystemClock_Config();
	init_indicators();
	return i2c_init();
}
