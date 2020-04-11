//
// Created by sheidman on 30/03/2020.
//

#include "Arduino.h"
#include <Wire.h>
#include "SDP610-500pa.h"
#include "NPA-700B-001G.h"
#include "ABPLANT001PG2A5.h"

#define MOVING_AVG_SIZE 3

SDP610_500pa sdp610(0x40);
NPA_700B_001G npa700G(0x28);
//ABPLANT001PG2A5 abp(0x28);
float data_p_diff;
float data_p;
float data_p_diff_ftrd;
float data_p1_ftrd;
float data_p2_ftrd;

void scanI2C();
unsigned long start_time = millis();

void setup() {
    Wire.begin();
    Serial.begin(115200);
    delay(100);
    if(!sdp610.init()) {
        while (1) {
            Serial.println("unable to init SDP610_500pa");
            delay(1000);
        }
    }
    if(!npa700G.init()) {
        while (1) {
            Serial.println("unable to init ABPLANT001PG2A5");
            delay(1000);
        }
    }
    Serial.println("---START---");
}

void loop() {

    sdp610.getMeasurement(&data_p_diff);
//    npa700G.getMeasurement(&data_p);
    npa700G.getMeasurement(&data_p);

    Serial.print('>');
    Serial.write((const uint8_t*)&data_p_diff, 4);
    Serial.write((const uint8_t*)&data_p, 4);
    delay(80);
}

void scanI2C() {
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address<16)
                Serial.print("0");
            Serial.print(address,HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (error==4)
        {
            Serial.print("Unknown error at address 0x");
            if (address<16)
                Serial.print("0");
            Serial.println(address,HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}
