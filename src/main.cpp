#include <Arduino.h>
#include <ArduinoLog.h>
#include <stepper.h>
#include <bno.h>
#include <utils.h>

#include <config.h>

// SPI pins for BNO08x
const int CS_PIN = 53; // CS (Chip Select)
const int INT_PIN = 33; // Interrupt pin
const int RST_PIN = 31; // Reset pin

Stepper stepper_r(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
Stepper stepper_l(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
BNO imu;

void setup() {
  Serial.begin(256000);
  while(!Serial)
    delay(10);

  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  Log.noticeln("[INIT] Roto2026 Starting");
  Log.noticeln("=========================");

  // Initialize IMU
  while (!imu.connect(CS_PIN, INT_PIN, RST_PIN)) {
    Log.infoln("[INIT] Retrying BNO connection...");
    delay(2000);
  }

  Log.infoln("[INIT] All systems ready");
  delay(1000);

  // stepper_r.setSpeed(4000);
  // stepper_l.setSpeed(4000);

  // stepper_r.move(32000);
  // stepper_l.move(32000);
}

void loop() {
  Log.infoln(F(">stepper_r_steps:%l" CR), stepper_r.currentPosition());
}

// void loop() {
//   long prevUS = micros();
//   imu.update();
//   RotationEuler rotation = imu.getRotationEuler();
//   long deltaUS = micros() - prevUS;

//   Serial.print("[DATA]: X: ");
//   Serial.print(rotation.x);
//   Serial.print(", Y: ");
//   Serial.print(rotation.y);
//   Serial.print(", Z: ");
//   Serial.print(rotation.z);
//   Serial.print(", US: ");
//   Serial.println(deltaUS);

//   delay(100);
// }