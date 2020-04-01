//
// Created by sheidman on 30/03/2020.
//

#include "I2c_helper.h"
#include <Wire.h>

int8_t I2c_helper::i2c_read(uint8_t addr, uint8_t* data, uint8_t count)
{
    Wire.requestFrom(addr, count);
    if (Wire.available() != count) {
        return -1;
    }
    for (int i = 0; i < count; ++i) {
        data[i] = Wire.read();
    }
    return 0;
}

int8_t I2c_helper::i2c_write(uint8_t addr, const uint8_t* data, uint8_t count, bool appendCrc)
{
    Wire.beginTransmission(addr);
    for (int i = 0; i < count; ++i) {
        if (Wire.write(data[i]) != 1) {
            return 1;
        }
    }
    if (appendCrc) {
        uint8_t crc = crc8(data, count);
        if (Wire.write(crc) != 1) {
            return 2;
        }
    }

    if (Wire.endTransmission() != 0) {
        return 3;
    }
    return 0;
}

uint8_t I2c_helper::crc8(const uint8_t* data, uint8_t len)
{
    // adapted from SHT21 sample code from http://www.sensirion.com/en/products/humidity-temperature/download-center/

    uint8_t crc = 0xff;
    uint8_t byteCtr;
    for (byteCtr = 0; byteCtr < len; ++byteCtr) {
        crc ^= (data[byteCtr]);
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}