#include <Arduino.h>
#include <bno.h>
#include <drivetrain.h>
#include <kalman.h>
#include <stepper.h>
#include <utils.h>

#include <config.h>

HardwareTimer fastLoopTimer(TIM1);

HardwareTimer stepperTimerR(TIM2);
HardwareTimer stepperTimerL(TIM4);

BNO imu;

Stepper stepper_r(&stepperTimerR, E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, true);
Stepper stepper_l(&stepperTimerL, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, false);

Kalman            kalmanFilter;
DifferentialDrive drivetrain(stepper_l, stepper_r, &kalmanFilter, &imu);

static unsigned long lastUpdateMicros = 0;

void fastLoop() {
  imu.update();
  drivetrain.update();
}

void setup() {
  Serial1.begin(230400);
  while (!Serial1);

  Serial1.println("[INIT] Roto2026 Starting");
  Serial1.println("========================");

  stepper_r.begin(1000000UL);
  stepper_l.begin(1000000UL);

  while (!imu.connect(BNO_SDA_PIN, BNO_SCL_PIN, BNO_INT_PIN, BNO_RST_PIN)) {
    Serial1.println("[INIT] Retrying BNO connection...");
    delay(800);
  }

  Serial1.println("[INIT] Setting up fast loop");

  fastLoopTimer.setOverflow(12000, MICROSEC_FORMAT); // 12 ms
  fastLoopTimer.attachInterrupt(fastLoop);
  fastLoopTimer.resume();

  Serial1.println("[INIT] All systems ready");
  delay(1000);
}

void loop() {
  drivetrain.driveDistance(90, 40, 80);

  Pose pose = drivetrain.getPose();
  Serial1.printf("Pose X:%0.2f Y:%0.2f Yaw:%0.2f deg | Mode:%d\n",
                 pose.x,
                 pose.y,
                 pose.rot.getYawDegs(),
                 static_cast<int>(drivetrain.mode()));

  delay(21000);
}