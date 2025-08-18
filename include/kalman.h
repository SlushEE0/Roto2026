#ifndef __KALMAN_H
#define __KALMAN_H

#include "utils.h"

class Kalman {
public:
  Kalman();

  void update(GyroData *gyroData, AccelData *accelData);

  RotationEuler getRotationEuler();
  RotationQuat getRotationQuat();

  // Optional external measurement (not used in current file but kept for API)
  // void setMeasurementQuat(const RotationQuat &z);
};

#endif