#ifndef I2C__H
#define I2C__H

#include <stdint.h>
int i2c_read_data(uint8_t address, unsigned char * data, uint16_t data_size);
int i2c_write_data(uint8_t address, const unsigned char * data, uint16_t data_size);
int i2c_init();


#endif // I2C__H
