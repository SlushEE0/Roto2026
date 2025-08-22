#pragma once

#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38

#define Y_STEP_PIN 60
#define Y_DIR_PIN 61
#define Y_ENABLE_PIN 56

#define Z_STEP_PIN 46
#define Z_DIR_PIN 48
#define Z_ENABLE_PIN 62

#define E0_STEP_PIN 26
#define E0_DIR_PIN 28
#define E0_ENABLE_PIN 24

#define MICROSTEPS 16
#define STEPS_PER_REV (200 * MICROSTEPS)

#define WHEEL_DIAMETER_MM 50

#define MOTOR_MAX_ACCEL 36000 // steps/sec^2
#define MOTOR_MAX_SPEED 28000  // steps/sec