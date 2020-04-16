#include "stm32f3xx.h"

#include "i2c.h"
#include "simple_indicators.h"

void SystemClock_Config(void);

int init_hardware()
{
    HAL_Init();
    SystemClock_Config();
	init_indicators();
	return i2c_init();
}
