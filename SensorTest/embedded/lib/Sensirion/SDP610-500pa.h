//
// Created by sheidman on 30/03/2020.
//

#ifndef RECOVID_FLOW_SDP610_500PA_H
#define RECOVID_FLOW_SDP610_500PA_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
  #include "pins_arduino.h"
  #include "WConstants.h"
#endif

class SDP610_500pa {
public:
    explicit SDP610_500pa(int i2c_address);
    bool init();
    int getMeasurement(float* data_out);  // returns measures pressure in Pa
private:
    uint8_t i2c_address = 0;
};


#endif //RECOVID_FLOW_SDP610_500PA_H
