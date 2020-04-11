#include "stm32f3xx_hal.h"
#include "i2c.h"

#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>

typedef enum i2c_mode_t
{
    I2C_READ,
    I2C_WRITE = 0,
}i2c_mode_t;


int i2c_init() {
	return 0;
}


void i2c_printf(i2c_mode_t mode, uint8_t address, const unsigned char * data, uint16_t data_size) {
    printf("i2c_%s @%"PRIx8"  to data %p:"PRIu16"\n", (mode ==I2C_READ) ? "READ" : "WRITE", address, data, data_size);
}

int i2c_read_data(uint8_t address, unsigned char * data, uint16_t data_size)
{
	i2c_printf(I2C_READ, address, data, data_size);
	return 0;
}

int i2c_write_data(uint8_t address, const unsigned char * data, uint16_t data_size)
{
	i2c_printf(I2C_WRITE, address, data, data_size);
	return 0;
}
