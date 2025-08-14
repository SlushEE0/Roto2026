#ifndef __UTILS_H
#define __UTILS_H

#include <Arduino.h>
#include <config.h>

typedef struct {
  float w, i, j, k;
} RotationQuat;

typedef struct {
  float x, y, z;
} RotationEuler;

static const double MMsPerStep = STEPS_PER_REV / (WHEEL_DIAMETER_MM * PI);

inline long cnv_stepsToMM(long steps);

inline long cnv_MMToSteps(long mm);

RotationEuler cnv_quatToEuler(RotationQuat q);

bool clamp(long *value, long min, long max);

#endif