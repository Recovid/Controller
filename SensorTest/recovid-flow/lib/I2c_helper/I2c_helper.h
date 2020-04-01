//
// Created by sheidman on 30/03/2020.
//

#ifndef RECOVID_FLOW_I2C_HELPER_H
#define RECOVID_FLOW_I2C_HELPER_H


#include <cstdint>

class I2c_helper
{
public:
    static int8_t i2c_write(uint8_t addr, const uint8_t* data, uint8_t count, bool appendCrc=false);
    static int8_t i2c_read(uint8_t addr, uint8_t* data, uint8_t count);
    static uint8_t crc8(const uint8_t* data, uint8_t len);
};


#endif //RECOVID_FLOW_I2C_HELPER_H
