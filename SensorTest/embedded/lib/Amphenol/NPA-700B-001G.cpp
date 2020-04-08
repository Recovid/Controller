//
// Created by sheidman on 30/03/2020.
//

#include "NPA-700B-001G.h"
#include "I2c_helper.h"

NPA_700B_001G::NPA_700B_001G(uint8_t i2c_address) {
    this->i2c_address = i2c_address;
}

bool NPA_700B_001G::init() {
    uint16_t data;
    return this->getRawMeasurement(&data) == 0;
}

// Operate a read on the i2c bus.
// // Gives measured pressure in raw counts
//  Returns
//     O : normal operation
//     1 : could'nt read data on i2c bus
//     2 : device error
int NPA_700B_001G::getRawMeasurement(uint16_t *data_out) {
    const uint8_t DATA_LEN = 2;
    uint8_t data[DATA_LEN] = { 0 };
    if (I2c_helper::i2c_read(this->i2c_address, (uint8_t*)data, DATA_LEN) != 0) {
        return 1;
    }
    uint8_t status = (data[0] & 0b11000000) >> 6;
    if(status != 0) {
        return 2;
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
//     2 : device error
int NPA_700B_001G::getMeasurement(float *data_out) {
    uint16_t Praw;
    int ret = this->getRawMeasurement(&Praw);
    if(ret != 0) { return ret;}
    float PcmH2O = 70.307 * ((float(Praw) - 1638.)/13107.);
//    float PcmH2O = Praw * 0.005364 - 8.78;
    *data_out = PcmH2O;
    return 0;
}