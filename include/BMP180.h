#pragma once

#include <I2C.h>
#include <math.h>

class BMP180 {
private:
  class Calibration {
  public:
    static const unsigned char REGISTER_START_ADDRESS = 0xAA;
    static const unsigned char CALIBRATION_DATA_SIZE = 22;
    short ac1, ac2, ac3, b1, b2, mb, mc, md;
    unsigned short ac4, ac5, ac6;
    Calibration();
    bool insertData(const unsigned char *buffer, int size);
  };

  Calibration calibration;
  I2C::Config config;
  short oversampling;

  I2C *i2c;
  void (*sleep)(unsigned int ms);

  long computeB5(long UT);

public:
  static const unsigned char BMP180_I2CADDR = 0x77;
  static const unsigned char BMP180_ULTRALOWPOWER = 0;
  static const unsigned char BMP180_STANDARD = 1;
  static const unsigned char BMP180_HIGHRES = 2;
  static const unsigned char BMP180_ULTRAHIGHRES = 3;
  static const unsigned char BMP180_CONTROL = 0xF4;
  static const unsigned char BMP180_TEMPDATA = 0xF6;
  static const unsigned char BMP180_PRESSUREDATA = 0xF6;
  static const unsigned char BMP180_READTEMPCMD = 0x2E;
  static const unsigned char BMP180_READPRESSURECMD = 0x34;
  static constexpr float BMP180_STD_ATMOSPHERE = 1013.25;
  static constexpr double BMP180_STD_TEMP_VARIATION = 288.15 / 0.0065;
  static const unsigned int BMP180_MAX_I2C_FREQUENCY = 340000;

  BMP180(I2C *i2c = nullptr, void (*sleep)(unsigned int ms) = 0,
         unsigned char address = BMP180_I2CADDR,
         unsigned long frequency = BMP180_MAX_I2C_FREQUENCY,
         short oversampling = BMP180_ULTRAHIGHRES);
  bool init();
  float readTemperature();
  long readPressure();
  long readSealevelPressure(float altitude_meters = 0);
  float readAltitude(float sealevelPressure = BMP180_STD_ATMOSPHERE);
  long readRawTemperature();
  long readRawPressure();
};