#pragma once

// ======================= MKS GEN L v1.0 ======================= //

// #define X_STEP_PIN 54
// #define X_DIR_PIN 55
// #define X_ENABLE_PIN 38

// #define Y_STEP_PIN 60
// #define Y_DIR_PIN 61
// #define Y_ENABLE_PIN 56

// #define Z_STEP_PIN 46
// #define Z_DIR_PIN 48
// #define Z_ENABLE_PIN 62

// #define E0_STEP_PIN 26
// #define E0_DIR_PIN 28
// #define E0_ENABLE_PIN 24

// ======================= CREALITY V4.2.2 ======================= //

#define X_STEP_PIN PC_2
#define X_DIR_PIN PB_9
#define X_ENABLE_PIN PC_3

#define Y_STEP_PIN PB_8
#define Y_DIR_PIN PB_7
#define Y_ENABLE_PIN PC_3

#define Z_STEP_PIN PB_6
#define Z_DIR_PIN PB_5
#define Z_ENABLE_PIN PC_3

#define E0_STEP_PIN PB_4
#define E0_DIR_PIN PB_3
#define E0_ENABLE_PIN PC_3

// =============================================================== //

#define BNO_INT_PIN PB_2
#define BNO_RST_PIN PC_6
#define BNO_SDA_PIN PB_11
#define BNO_SCL_PIN PB_10

#define BNO_REPORT_INTERVAL_MS 10 // 100 Hz

#define MICROSTEPS 16
#define STEPS_PER_REV (200 * MICROSTEPS)

#define DT_TRACK_WIDTH_CM 16
#define WHEEL_DIAMETER_CM 7.3025

#define MOTOR_MAX_ACCEL 36000 // steps/sec^2
#define MOTOR_MAX_SPEED 20000 // steps/sec