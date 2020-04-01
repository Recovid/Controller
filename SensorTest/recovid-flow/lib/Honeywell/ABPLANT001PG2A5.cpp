//
// Created by sheidman on 30/03/2020.
//

#include "ABPLANT001PG2A5.h"

#include "I2c_helper.h"

ABPLANT001PG2A5::ABPLANT001PG2A5(uint8 i2c_address) {
    this->i2c_address = i2c_address;
}

bool ABPLANT001PG2A5::init() {
    uint16_t data;
    return this->getRawMeasurement(&data) == 0;
}

// Operate a read on the i2c bus.
// // Gives measured pressure in raw counts
//  Returns
//     O : normal operation
//     1 : could'nt read data on i2c bus
//     2 : device is in command mode
//     3 : data is stale (already been fetched since last measurement cycle)
//     4 : device is faulty (diagnostic condition)
int ABPLANT001PG2A5::getRawMeasurement(uint16 *data_out) {
    const uint8_t DATA_LEN = 2;
    uint8_t data[DATA_LEN] = { 0 };
    if (I2c_helper::i2c_read(this->i2c_address, (uint8_t*)data, DATA_LEN) != 0) {
        return 1;
    }
    uint8_t status = (data[0] & 0b11000000) >> 6;
    if(status == 1) {
        return 2;
    } else if (status == 2) {
        return 3;
    } else if (status == 3) {
        return 4;
    }
    int16_t p_raw   = (int16_t)data[0] << 8 | data[1];
    p_raw &= 0b0011111111111111;
    *data_out = p_raw;
    return 0;
}

// Operate a read on the i2c bus.
// Gives measured pressure in cmH2O
// Returns
//     O : normal operation
//     1 : could'nt read data on i2c bus
//     2 : device is in command mode
//     3 : data is stale (already been fetched since last measurement cycle)
//     4 : device is faulty (diagnostic condition)
int ABPLANT001PG2A5::getMeasurement(float *data_out) {
    uint16_t Praw;
    int ret = this->getRawMeasurement(&Praw);
    if(ret != 0) { return ret;}
    float PcmH2O = 70.307 * ((float(Praw) - 1638.)/13107.);
    *data_out = PcmH2O;
    return 0;
}