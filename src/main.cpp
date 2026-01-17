#include <Arduino.h>
#include <bno.h>
#include <drivetrain.h>
#include <odometry.h>
#include <stepper.h>
#include <utils.h>

#include <config.h>
#include <dead_reckoning.h>

HardwareTimer fastLoopTimer(TIM1);

HardwareTimer stepperTimerR(TIM2);
HardwareTimer stepperTimerL(TIM3);

BNO imu;

Stepper stepper_r(&stepperTimerR, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, true);
Stepper
  stepper_l(&stepperTimerL, E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, false);

Odometry odometry;
// DifferentialDrive drivetrain(stepper_l, stepper_r, &odometry, &imu);
DeadReckoningDrivetrain drivetrain(stepper_l, stepper_r, &imu);

void fastLoop() {
  imu.update();
  drivetrain.update();
}

void setup() {
  Serial1.begin(250000);
  while (!Serial1)
    ;

  Serial1.println("[INIT] Roto2026 Starting");
  Serial1.println("========================");

  stepper_r.begin(1000000UL);
  stepper_l.begin(1000000UL);
  stepper_r.setMaxSpeed(MOTOR_MAX_SPEED);
  stepper_l.setMaxSpeed(MOTOR_MAX_SPEED);

  for (int i = 0; i < 4; i++) {
    if (imu.connect(BNO_SDA_PIN, BNO_SCL_PIN, BNO_INT_PIN, BNO_RST_PIN)) {
      Serial1.println("[INIT] Connected to BNO085");
      break;
    }
    Serial1.println("[INIT] Retrying BNO connection...");
    delay(500);
  }

  Serial1.println("[INIT] Setting up fast loop");

  fastLoopTimer.setOverflow(12000, MICROSEC_FORMAT); // 12 ms
  fastLoopTimer.attachInterrupt(fastLoop);
  fastLoopTimer.resume();

  Serial1.println("[INIT] All systems ready");
  delay(200);

  // drivetrain.resetPose();

  // Example: Move to absolute poses using EKF odometry feedback
  // queueMoveToPose(targetPose, linearSpeed_cm/s, turnSpeed_deg/s)
  // drivetrain.queueMoveToPose(
  //   Pose(34.0f, 0.0f, Rotation(0, 0, 90.0f)), 60.0f, 120.0f); // Forward 34 cm
  // drivetrain.queueMoveToPose(Pose(34.0f, 48.0f, Rotation::kZero()),
  //                            60.0f,
  //                            120.0f); // Left 48 cm (90° turn + drive)
  // drivetrain.queueMoveToPose(
  //   Pose(130.0f, 48.0f, Rotation::kZero()), 60.0f, 120.0f); // Forward 96 cm
  // drivetrain.queueMoveToPose(
  //   Pose(250.0f, -50.0f, Rotation::kZero()), 60.0f, 120.0f); // Diagonal move

  drivetrain.queueDriveStraight(100.0f, 60.0f); // Forward 100 cm
  drivetrain.queueTurn(90.0f, 90.0f);            // Turn 90 degrees
  drivetrain.queueDriveStraight(50.0f, 60.0f);  // Forward 50 cm
  drivetrain.queueTurn(-90.0f, 90.0f);          // Turn -90 degrees
  drivetrain.queueDriveStraight(100.0f, 60.0f); // Forward 100 cm
  drivetrain.queueTurn(180.0f, 90.0f);          // Turn 180 degrees
  drivetrain.queueDriveStraight(150.0f, 60.0f); // Forward 150 cm
  drivetrain.queueTurn(-90.0f, 90.0f);          // Turn -90 degrees
  drivetrain.queueDriveStraight(75.0f, 60.0f);  // Forward 75 cm
  drivetrain.queueTurn(-90.0f, 90.0f);          // Turn -90 degrees
  drivetrain.queueDriveStraight(50.0f, 60.0f);  // Forward 50 cm
  drivetrain.queueTurn(90.0f, 90.0f);           // Turn 90 degrees
}

void loop() {
  // Pose pose = drivetrain.getPose();
  // Serial1.print("Pose X:");
  // Serial1.print(pose.x, 2);
  // Serial1.print(", Y:");
  // Serial1.print(pose.y, 2);
  // Serial1.print(", Yaw:");
  // Serial1.print(pose.rot.getYawDegs(), 2);
  // Serial1.print(" deg | Busy:");
  // Serial1.print(drivetrain.isBusy() ? "YES" : "NO");

  // // Print drivetrain state
  // Serial1.print(" | Exec:");
  // Serial1.print(drivetrain.isExecuting() ? "YES" : "NO");
  // Serial1.print(" | Cmd:");
  // switch (drivetrain.getCurrentCommandType()) {
  //   case DrivetrainCommandType::Idle:
  //     Serial1.print("Idle");
  //     break;
  //   case DrivetrainCommandType::TurnDegrees:
  //     Serial1.print("Turn");
  //     break;
  //   case DrivetrainCommandType::MoveToPose:
  //     Serial1.print("MoveToPose");
  //     break;
  //   case DrivetrainCommandType::FollowTrajectory:
  //     Serial1.print("FollowTraj");
  //     break;
  // }
  // Serial1.print(" | SubState:");
  // Serial1.print(drivetrain.getSubStateName());
  // Serial1.print(" | Queue:");
  // Serial1.println(drivetrain.getQueueCount());

  // delay(30);
}