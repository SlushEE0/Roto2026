#include <Arduino.h>
// #include <motor.h>
#include "utils.h"
#include <bno.h>

#include <config.h>

// SPI pins for BNO08x
const int CS_PIN = 53; // CS (Chip Select)
const int INT_PIN = 33; // Interrupt pin
const int RST_PIN = 31; // Reset pin

// Motor stepper_r;
// Motor stepper_l;
BNO imu;

void setup() {
  Serial.begin(250000);
  while (!Serial)
    delay(10);

  Serial.println("[INIT] Roto2026 Starting");
  Serial.println("=========================");

  // Initialize motors
  // stepper_r.connect(Y_DIR_PIN, Y_STEP_PIN, Y_ENABLE_PIN, false);
  // stepper_l.connect(Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN, false);

  // Initialize IMU
  while (!imu.connect(CS_PIN, INT_PIN, RST_PIN)) {
    Serial.println("[INIT] Retrying BNO connection...");
    delay(2000);
  }

  Serial.println("[INIT] All systems ready");
  delay(1000);
}

void loop() {
  long prevUS = micros();
  imu.update();
  RotationEuler rotation = imu.getRotationEuler();
  long deltaUS = micros() - prevUS;

  Serial.print("[DATA]: X: ");
  Serial.print(rotation.x);
  Serial.print(", Y: ");
  Serial.print(rotation.y);
  Serial.print(", Z: ");
  Serial.print(rotation.z);
  Serial.print(", US: ");
  Serial.println(deltaUS);

  delay(100);
}