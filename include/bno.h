#pragma once

#include <SparkFun_BNO08x_Arduino_Library.h>
#include <kalman.h>

class BNO {
    public:
  BNO();
  ~BNO();

  bool connect(PinName       sda,
               PinName       scl,
               PinName       intPin,
               PinName       rstPin,
               unsigned long freq = 350000);
  void disconnect();
  bool isReady();
  void update();
  void tare();

  Rotation *getRotation();
  AccelData getAccelData();
  GyroData  getGyroData();

    private:
  void init();
  void enableSensors();
  void updateSensorData();

  void setRotation(Rotation *rotation);

  int cs_pin;
  int int_pin;
  int rst_pin;

  bool isConnected;
  bool sensorsEnabled;

  AccelData accel;
  GyroData  gyro;
  Rotation  rotation;

  BNO08x bno;
  Kalman filter;
};
