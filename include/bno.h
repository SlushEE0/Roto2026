#ifndef __BNO_H
#define __BNO_H

#include <Arduino.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <SPI.h>
#include <utils.h>

class BNO {
private:
  BNO08x imu;

  // SPI connection pins
  int cs_pin;
  int int_pin;
  int rst_pin;

  // Connection status
  bool isConnected;
  bool sensorsEnabled;

  RotationQuat rotationQuat;
  RotationEuler rotationEuler;

  void updateSensorData();
  void enableReports();

public:
  BNO();
  ~BNO();

  // Connection and initialization
  bool
  connect(int csPin, int intPin, int rstPin, unsigned long spiSpeed = 8000000);
  void disconnect();
  bool isReady();

  // Main update function
  bool update();

  // Getters for filtered data
  RotationEuler getRotation();
  RotationQuat getRotationQuat();

  // Calibration
  void tare();
};

#endif
