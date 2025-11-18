#include <Arduino.h>
#include <bno.h>
#include <drivetrain.h>
#include <kalman.h>
#include <stepper.h>
#include <utils.h>

#include <config.h>

HardwareTimer fastLoopTimer(TIM1);

HardwareTimer stepperTimerR(TIM2);
HardwareTimer stepperTimerL(TIM3);

BNO imu;

Stepper stepper_r(&stepperTimerR, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, true);
Stepper stepper_l(&stepperTimerL, E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, false);

Kalman            kalmanFilter;
DifferentialDrive drivetrain(stepper_l, stepper_r, &kalmanFilter, &imu);

void fastLoop() {
  imu.update();
  drivetrain.update();
}

void setup() {
  Serial1.begin(250000);
  while (!Serial1);

  Serial1.println("[INIT] Roto2026 Starting");
  Serial1.println("========================");

  stepper_r.begin(1000000UL);
  stepper_l.begin(1000000UL);
  stepper_r.setMaxSpeed(MOTOR_MAX_SPEED);
  stepper_l.setMaxSpeed(MOTOR_MAX_SPEED);

  while (!imu.connect(BNO_SDA_PIN, BNO_SCL_PIN, BNO_INT_PIN, BNO_RST_PIN)) {
    Serial1.println("[INIT] Retrying BNO connection...");
    delay(800);
  }

  Serial1.println("[INIT] Setting up fast loop");

  fastLoopTimer.setOverflow(12000, MICROSEC_FORMAT); // 12 ms
  fastLoopTimer.attachInterrupt(fastLoop);
  fastLoopTimer.resume();

  Serial1.println("[INIT] All systems ready");
  delay(500);

  drivetrain.resetPose();
  drivetrain.moveToPose(Pose(100.0, 0, Rotation::kZero()), 25.0, 60.0);
}

void loop() {
  Pose pose = drivetrain.getPose();
  Serial1.print("Pose X:");
  Serial1.print(pose.x, 2);
  Serial1.print(" Y:");
  Serial1.print(pose.y, 2);
  Serial1.print(" Yaw:");
  Serial1.print(pose.rot.getYawDegs(), 2);
  Serial1.print(" deg | Mode:");
  Serial1.println(static_cast<int>(drivetrain.mode()));

  delay(30);
}