//
// Created by Mano on 28/03/2020.
//

#ifndef CONTROLLER_SENSOR_H
#define CONTROLLER_SENSOR_H

#define SFM3300


#define BME280
//#define MS5611
//#define BMP280

#ifdef SFM3300
#include <sfm3000wedo.h> //https://github.com/dwerne/Sensirion_SFM3000_arduino
// SFM3300 Flow meter parameters
#define SFM3300_offset 32000 // Offset for the sensor
#define SFM3300_scale 142.8 // Scale factor for Air and N2 is 140.0, O2 is 142.8
#define SFM3300_address 0x40  //I2X address of the sensor

SFM3000wedo measflow(SFM3300_address);
#endif

#ifdef BME280
#include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  // PAtmo BME280 sensor parameters
  #define BME_SCK 13
  #define BME_MISO 12
  #define BME_MOSI 11
  #define BME_CS 10

  #define SEALEVELPRESSURE_HPA (1013.25)
  Adafruit_BME280   bme;
#endif

#ifdef MS5611
#include <MS5611.h> //https://github.com/jarzebski/Arduino-MS5611

  MS5611 ms5611;
  double referencePressure;
#endif

#ifdef BMP280
#include <Adafruit_BMP280.h> //https://github.com/adafruit/Adafruit_BMP280_Library

  #define BMP_SCK  (13)
  #define BMP_MISO (12)
  #define BMP_MOSI (11)
  #define BMP_CS   (10)

  Adafruit_BMP280 bmp; // I2C
  //Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
  //Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
#endif

#endif //CONTROLLER_SENSOR_H
