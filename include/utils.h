#pragma once

#include <Arduino.h>
#include <config.h>

#define G_MS2 9.80665

typedef struct {
  double w, i, j, k;
} RotationQuat;

// rads
typedef struct {
  double x, // roll
    y, // pitch
    z; // yaw
} RotationEuler;

RotationEuler cnv_quatToEuler(RotationQuat *q);
RotationQuat cnv_eulerToQuat(RotationEuler *e);

class Rotation {
public:
  RotationQuat rotation;

  RotationEuler getEuler();
  RotationQuat getQuat();

  void setQuat(RotationQuat *quat);
  void setEuler(RotationEuler *euler);
};

// m/s^2
typedef struct {
  double x, y, z;
} AccelData;

// rad/s
typedef struct {
  double x, y, z;
} GyroData;

static const double MMsPerStep = STEPS_PER_REV / (WHEEL_DIAMETER_MM * PI);

inline long cnv_stepsToMM(long steps);

inline long cnv_MMToSteps(long mm);

void normalizeQuat(RotationQuat &q);

bool clamp(long *value, long min, long max);