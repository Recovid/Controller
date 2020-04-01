//
// Created by sheidman on 30/03/2020.
// Designed for Amphenol NPA_700B_001G-700B-001G
//

#ifndef RECOVID_FLOW_NPA_700B_001G_H
#define RECOVID_FLOW_NPA_700B_001G_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#include "WConstants.h"
#endif

class NPA_700B_001G {
public:
    explicit NPA_700B_001G(uint8 i2c_address);
    bool init();
    int getRawMeasurement(uint16_t * data_out);  // returns measures pressure in raw counts
    int getMeasurement(float* data_out);  // returns measures pressure in cmH2O
private:
    uint8_t i2c_address = 0;
};


#endif //RECOVID_FLOW_NPA_700B_001G_H
