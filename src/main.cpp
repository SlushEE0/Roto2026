#include <Arduino.h>

#include <config.h>
#include <drivetrain.h>
#include <stepper.h>
#include <bno.h>

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------

// Both motors share one hardware timer running at kStepTimerHz (200 kHz).
HardwareTimer stepTimer(TIM2);

// Steppers – pins from config.h.
// invertDir=true  for the right motor so that positive velocity = forward.
Stepper stepper_r(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, /*invertDir=*/true);
Stepper stepper_l(E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, /*invertDir=*/false);

BNO imu;

DifferentialDrive drivetrain(stepper_l, stepper_r, &imu);

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void stepISR() {
  stepper_r.tick();
  stepper_l.tick();
}

void setup() {
  Serial1.begin(250000);
  while (!Serial1)
    ;

  Serial1.println("[INIT] Roto2026 starting");
  Serial1.println("=========================");

  // ── Stepper pin init ────────────────────────────────────────────────────
  stepper_r.begin();
  stepper_l.begin();
  stepper_r.setMaxSpeed(MOTOR_MAX_SPEED);
  stepper_l.setMaxSpeed(MOTOR_MAX_SPEED);
  stepper_r.setAcceleration(MOTOR_MAX_ACCEL);
  stepper_l.setAcceleration(MOTOR_MAX_ACCEL);

  // ── Shared step timer ───────────────────────────────────────────────────
  stepTimer.setOverflow(5, MICROSEC_FORMAT); // 5 µs → 200 kHz tick rate
  stepTimer.attachInterrupt(stepISR);
  stepTimer.resume();

  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  Serial1.println("[INIT] Step timer running at 200 kHz (priority 2)");

  // ── IMU ─────────────────────────────────────────────────────────────────
  // for (int a = 0; a < 4; ++a) {
  //   if (imu.connect(BNO_SDA_PIN, BNO_SCL_PIN, BNO_INT_PIN, BNO_RST_PIN)) {
  //     Serial1.println("[INIT] BNO085 connected");
  //     break;
  //   }
  //   Serial1.println("[INIT] BNO085 retry...");
  //   delay(500);
  // }

  Serial1.println("[INIT] All systems ready");
  delay(200);

  // ── Example: drive a square in 10 seconds ───────────────────────────────
  // Queue moves without specifying speeds, then let setTimeTarget calculate
  // the speeds needed to finish the whole sequence in the given time.
  drivetrain.queueDrive(50.0f);    // forward 50 cm
  drivetrain.queueTurn(107.0f);     // turn 90° left
  drivetrain.queueDrive(50.0f);    // forward 50 cm
  drivetrain.queueTurn(107.0f);     // turn 90° left
  drivetrain.queueDrive(50.0f);    // forward 50 cm
  drivetrain.queueTurn(107.0f);     // turn 90° left
  drivetrain.setTimeTarget(20.0f); // complete everything in 20 seconds
  drivetrain.queueDrive(50.0f);    // forward 50 cm
}

void logData() {
  Serial1.print("Busy:");
  Serial1.print(drivetrain.isBusy() ? "Y" : "N");
  Serial1.print(" Q:");
  Serial1.print(drivetrain.getQueueCount());
  Serial1.print(" L:");
  Serial1.print(stepper_l.currentSpeed(), 0);
  Serial1.print(" R:");
  Serial1.print(stepper_r.currentSpeed(), 0);

  if (imu.isReady()) {
    Serial1.print(" Yaw:");
    Serial1.print(imu.getRotation()->getYawDegs(), 1);
    Serial1.print(" P:");
    Serial1.print(imu.getRotation()->getPitchDegs(), 1);
    Serial1.print(" R:");
    Serial1.print(imu.getRotation()->getRollDegs(), 1);
    GyroData g = imu.getGyroData();
    Serial1.print(" Gyro:");
    Serial1.print(g.x, 2);
    Serial1.print(",");
    Serial1.print(g.y, 2);
    Serial1.print(",");
    Serial1.print(g.z, 2);
    AccelData a = imu.getAccelData();
    Serial1.print(" Acc:");
    Serial1.print(a.x, 1);
    Serial1.print(",");
    Serial1.print(a.y, 1);
    Serial1.print(",");
    Serial1.print(a.z, 1);
  } else {
    Serial1.print(" IMU:N/A");
  }

  Serial1.println();
}

void loop() {
  imu.update();
  drivetrain.update();

  // 10Hz logger
  static uint32_t lastPrintMs = 0;
  const uint32_t now = millis();

  if (now - lastPrintMs >= 100) {
    logData();
    lastPrintMs = now;
  }
}
