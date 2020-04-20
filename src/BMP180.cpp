#include "BMP180.h"

BMP180::Calibration::Calibration() {
  ac1 = 0;
  ac2 = 0;
  ac3 = 0;
  b1 = 0;
  b2 = 0;
  mb = 0;
  mc = 0;
  md = 0;
  ac4 = 0;
  ac5 = 0;
  ac6 = 0;
}

bool BMP180::Calibration::insertData(const unsigned char *buffer, int size) {
  if (size != CALIBRATION_DATA_SIZE) {
    return false;
  }

  // Check if read calibration data is valid
  for (int i = 0; i < size; i += 2) {
    short data = buffer[i] << 8 | buffer[i + 1];
    if (data == 0 || data == 0xffff) {
      return false;
    }
  }

  // Populate calibration variables
  ac1 = buffer[0] << 8 | buffer[1];
  ac2 = buffer[2] << 8 | buffer[3];
  ac3 = buffer[4] << 8 | buffer[5];
  ac4 = buffer[6] << 8 | buffer[7];
  ac5 = buffer[8] << 8 | buffer[9];
  ac6 = buffer[10] << 8 | buffer[11];
  b1 = buffer[12] << 8 | buffer[13];
  b2 = buffer[14] << 8 | buffer[15];
  mb = buffer[16] << 8 | buffer[17];
  mc = buffer[18] << 8 | buffer[19];
  md = buffer[20] << 8 | buffer[21];

  return true;
}

BMP180::BMP180(I2C *i2c, void (*sleep)(unsigned int ms), unsigned char address,
               unsigned long frequency, short oversampling) {
  this->i2c = i2c;
  this->config.address = address;
  this->config.frequency = frequency;
  this->sleep = sleep;
  this->oversampling = oversampling;
}

bool BMP180::init() {
  if (!i2c->init()) {
    return false;
  }

  // Read calibration registers from the BMP180 module
  unsigned char buffer[Calibration::CALIBRATION_DATA_SIZE];
  char cmd[] = {(char)Calibration::REGISTER_START_ADDRESS};
  i2c->send(config, cmd, sizeof(cmd));
  if (!i2c->receive(config, (char *)buffer,
                    Calibration::CALIBRATION_DATA_SIZE)) {
    return false;
  }

  // Construct the calibration data
  if (!calibration.insertData(buffer, Calibration::CALIBRATION_DATA_SIZE)) {
    return false;
  }

  return true;
}

long BMP180::computeB5(long UT) {
  long x1 = 0;
  long x2 = 0;
  x1 = (UT - calibration.ac6) * calibration.ac5 / 32768;
  x2 = calibration.mc * 2048 / (x1 + calibration.md);
  return x1 + x2;
}

float BMP180::readTemperature() {
  long temp = (computeB5(readRawTemperature()) + 8) / 16;

  return temp * 0.1;
}

long BMP180::readPressure() {
  long UT, UP, B3, B5, B6, X1, X2, X3, p;
  unsigned long B4, B7;

  UT = readRawTemperature();
  UP = readRawPressure();
  B5 = computeB5(UT);

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = (calibration.b2 * (B6 * B6 / 4096)) / 2048;
  X2 = calibration.ac2 * B6 / 2048;
  X3 = X1 + X2;
  B3 = (((calibration.ac1 * 4 + X3) << oversampling) + 2) / 4;

  X1 = calibration.ac3 * B6 / 8192;
  X2 = (calibration.b1 * (B6 * B6 / 4096)) / 65536;
  X3 = ((X1 + X2) + 2) / 4;
  B4 = calibration.ac4 * (unsigned long)(X3 + 32768) / 32768;
  B7 = ((unsigned long)UP - B3) * (50000 >> oversampling);

  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p / 256) * (p / 256);
  X1 = (X1 * 3038) / 65536;
  X2 = (-7357 * p) / 65536;

  p = p + (X1 + X2 + 3791) / 16;

  return p * 0.01;
}

long BMP180::readSealevelPressure(float altitude_meters) {
  return readPressure() /
         pow(1.0 - altitude_meters / BMP180_STD_TEMP_VARIATION, 5.255);
}

float BMP180::readAltitude(float sealevelPressure) {
  return BMP180_STD_TEMP_VARIATION *
         (1.0 - pow(readPressure() / sealevelPressure, 1 / 5.255));
}

long BMP180::readRawTemperature() {
  unsigned char cmd[3] = {BMP180_CONTROL, BMP180_READTEMPCMD, BMP180_TEMPDATA};
  i2c->send(config, (char *)cmd, sizeof(cmd));
  sleep(5);
  unsigned char data[2] = {0, 0};
  i2c->receive(config, (char *)data, sizeof(data));
  return data[0] << 8 | data[1];
}

long BMP180::readRawPressure() {
  unsigned char cmd[3] = {BMP180_CONTROL,
                          (unsigned char)(BMP180_READPRESSURECMD | (oversampling << 6)),
                          BMP180_PRESSUREDATA};
  i2c->send(config, (char *)cmd, sizeof(cmd));

  switch (oversampling) {
  case BMP180_ULTRALOWPOWER:
    sleep(5);
    break;
  case BMP180_STANDARD:
    sleep(8);
    break;
  case BMP180_HIGHRES:
    sleep(13);
    break;
  default:
    sleep(26);
  }

  unsigned char data[3] = {0, 0, 0};
  i2c->receive(config, (char *)data, sizeof(data));
  long result = (data[0] << 16 | data[1] << 8 | data[2]) >> (8 - oversampling);

  return result;
}