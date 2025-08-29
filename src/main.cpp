#include <Arduino.h>
#include <stepper.h>
#include <bno.h>
#include <utils.h>

#include <config.h>

// SPI pins for BNO08x
const int CS_PIN = 53; // CS (Chip Select)
const int INT_PIN = 33; // Interrupt pin
const int RST_PIN = 31; // Reset pin

BNO imu;

// void setup() {
//   // Log.begin(LOG_LEVEL_VERBOSE, &Serial11);
//   // Log.setPrefix(printPrefix);

//   // Log.noticeln("[INIT] Roto2026 Starting");
//   // Log.noticeln("=========================");

//   // while (!imu.connect(CS_PIN, INT_PIN, RST_PIN)) {
//   //   Log.infoln("[INIT] Retrying BNO connection...");
//   //   delay(2000);
//   // }

//   // Log.infoln("[INIT] All systems ready");
//   // delay(1000);
// }

// Create stepper objects
Stepper stepperX(X_STEP_PIN, X_DIR_PIN, TIM2, X_ENABLE_PIN);

// Stepper stepperZ(Z_STEP_PIN, Z_DIR_PIN, &timer3);

void setup() {
  Serial1.begin(250000);
  while (!Serial1)
    ;

  stepperX.setMaxSpeed(2000); // 40k steps/s
  stepperX.setAcceleration(500); // steps/s^2

  stepperX.setSCurve(true);

  // Example moves
  stepperX.moveTo(5000); // absolute move
}

void loop() {
  if (stepperX.isRunning()) {
    Serial1.printf("$%d;\n", stepperX.currentPosition());
  }
}