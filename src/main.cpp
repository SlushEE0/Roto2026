#include <Arduino.h>
#include <bno.h>
#include <stepper.h>
#include <utils.h>

#include <config.h>

HardwareTimer fastTimer(TIM3);

BNO imu;

Stepper stepper_r(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, TIM1);
Stepper stepper_l(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, TIM2);

void fastLoop() { imu.update(); }

void setup() {
  Serial1.begin(250000);
  while (!Serial1);

  Serial1.println("[INIT] Roto2026 Starting");
  Serial1.println("========================");

  stepper_r.setSpeed(2000);
  stepper_r.setAccel(500);
  // stepper_r.setSCurve(true);

  while (!imu.connect(BNO_SDA_PIN, BNO_SCL_PIN, BNO_INT_PIN, BNO_RST_PIN)) {
    Serial1.println("[INIT] Retrying BNO connection...");
    delay(800);
  }

  Serial1.println("[INIT] Setting up fast loop");

  fastTimer.setOverflow(20000, MICROSEC_FORMAT); // 100 us
  fastTimer.attachInterrupt(fastLoop);
  fastTimer.resume();

  Serial1.println("[INIT] All systems ready");
  delay(1000);
}

void loop() {
  Serial1.print("[IMU] Yaw: ");
  Serial1.print(imu.getRotation()->getYawDegs());
  Serial1.println(" deg");

  delay(100);
}