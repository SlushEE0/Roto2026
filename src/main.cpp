#include <Arduino.h>
#include <bno.h>
#include <stepper.h>
#include <utils.h>

#include <config.h>


// SPI pins for BNO08x
const int CS_PIN  = 53; // CS (Chip Select)
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

Stepper stepper_r(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, TIM2);
// Stepper stepper_l(Z_STEP_PIN, Z_DIR_PIN, &timer3);

void setup() {
  Serial1.begin(250000);
  while (!Serial1);

  stepper_r.setSpeed(2000);
  stepper_r.setAccel(500);

  // stepper_r.setSCurve(true);

  stepper_r.moveTo(3200);

  delay(2000);
}

void loop() {
  Serial1.printf("Steps: %d, target: %d, enabled: %s\n",
                 stepper_r.getSteps(),
                 stepper_r.getTargetSteps(),
                 stepper_r.getIsEnabled() ? "true" : "false");

  if (stepper_r.isAtTargetSteps()) { stepper_r.moveTo(-stepper_r.getSteps()); }
  delay(100);
}