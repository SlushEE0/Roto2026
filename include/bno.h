#ifndef __BNO_H
#define __BNO_H

#include <Arduino.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <SPI.h>
#include <MadgwickAHRS.h>
#include <utils.h>

// Structs for raw sensor data
struct AccelData {
  float x; // m/s^2
  float y;
  float z;
};

struct GyroData {
  float x; // rad/s
  float y;
  float z;
};

class BNO {
public:
  BNO();
  ~BNO();

  bool
  connect(int csPin, int intPin, int rstPin, unsigned long spiSpeed = 800000);
  void disconnect();
  bool isReady();
  bool update();
  void tare();

  RotationEuler getRotationEuler();
  RotationQuat getRotationQuat();
  AccelData getAccelData();
  GyroData getGyroData();

private:
  void init();
  void enableSensors();
  void updateSensorData();

  void setRotation(RotationEuler rotation);
  void setRotation(RotationQuat rotation);

  int cs_pin;
  int int_pin;
  int rst_pin;

  bool isConnected;
  bool sensorsEnabled;

  AccelData accel;
  GyroData gyro;
  RotationQuat rotationQuat;
  RotationEuler rotationEuler;

  BNO08x imu;
  Madgwick filter;
};

#endif
