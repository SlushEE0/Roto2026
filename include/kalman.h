#pragma once

#include "utils.h"

class Kalman {
public:
  Kalman();

  void updateWheels(double *yaw);
  void updateIMU(RotationEuler *rotation);

  RotationEuler getRotationEuler();
  RotationQuat getRotationQuat();

  // Optional external measurement (not used in current file but kept for API)
  // void setMeasurementQuat(const RotationQuat &z);
};