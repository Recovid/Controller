//
// Created by sheidman on 30/03/2020.
//

#include "SDP610-500pa.h"
#include "I2c_helper.h"


SDP610_500pa::SDP610_500pa(int i2c_address){
    this->i2c_address = i2c_address;
}

bool SDP610_500pa::init() {
    float data;
    return this->getMeasurement(&data) == 0;
}

// Operate a read on the i2c bus.
// Gives measured pressure in cmH2O
// Returns
//     O : normal operation
//     1 : could'nt write command on i2c bus
//     2 : could'nt read command on i2c bus
// TODO : check CRC
int SDP610_500pa::getMeasurement(float* data_out) {
    const uint8_t CMD_LEN = 1;
    uint8_t cmd[CMD_LEN] = { 0xF1 };
    const uint8_t DATA_LEN = 3;
    uint8_t data[DATA_LEN] = { 0 };
    if ( I2c_helper::i2c_write(this->i2c_address, cmd, CMD_LEN) != 0) {
        return 1;
    }
//    delay(1);
    if (I2c_helper::i2c_read(i2c_address, (uint8_t*)data, DATA_LEN) != 0) {
        return 2;
    }
    int16_t dp_raw   = (int16_t)data[0] << 8 | data[1];
    float dp = float(dp_raw)/60.;
    *data_out = dp;
    return 0;
}