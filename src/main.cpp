#include <Arduino.h>

#include <config.h>
#include <dead_reckoning.h>
#include <stepper.h>
#include <bno.h>

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------

// Both motors share one hardware timer running at kStepTimerHz (200 kHz).
HardwareTimer stepTimer(TIM2);

// Steppers – pins from config.h.
// invertDir=true  for the right motor so that positive velocity = forward.
Stepper stepper_r(X_STEP_PIN,  X_DIR_PIN,  X_ENABLE_PIN, /*invertDir=*/true);
Stepper stepper_l(E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, /*invertDir=*/false);

BNO imu;

DeadReckoningDrivetrain drivetrain(stepper_l, stepper_r, &imu);

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

// Step ISR – called at 200 kHz.  Tick both motors; order doesn't matter.
void stepISR() {
  stepper_r.tick();
  stepper_l.tick();
}

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------

void setup() {
  Serial1.begin(250000);
  while (!Serial1);

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
  // One timer, one ISR, both motors.  No per-motor timer reprogramming.
  stepTimer.setOverflow(5, MICROSEC_FORMAT); // 5 µs → 200 kHz tick rate
  stepTimer.attachInterrupt(stepISR);
  stepTimer.resume();

  // Give the step timer the highest NVIC priority so it is never delayed
  // by any other interrupt.
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  Serial1.println("[INIT] Step timer running at 200 kHz (priority 0)");

  // ── IMU ─────────────────────────────────────────────────────────────────
  for (int attempt = 0; attempt < 4; ++attempt) {
    if (imu.connect(BNO_SDA_PIN, BNO_SCL_PIN, BNO_INT_PIN, BNO_RST_PIN)) {
      Serial1.println("[INIT] BNO085 connected");
      break;
    }
    Serial1.println("[INIT] BNO085 retry...");
    delay(500);
  }

  Serial1.println("[INIT] All systems ready");
  delay(200);

  // ── Queue a simple demo path ─────────────────────────────────────────────
  // Speeds: linear in cm/s, angular in deg/s.
  drivetrain.queueDriveStraight(100.0f, 60.0f); // Forward 100 cm
  drivetrain.queueTurn(90.0f,  90.0f);           // Right 90°
  drivetrain.queueDriveStraight(50.0f,  60.0f);  // Forward 50 cm
  drivetrain.queueTurn(-90.0f, 90.0f);           // Left 90°
  drivetrain.queueDriveStraight(100.0f, 60.0f);  // Forward 100 cm
  drivetrain.queueTurn(180.0f, 90.0f);           // U-turn
  drivetrain.queueDriveStraight(150.0f, 60.0f);  // Forward 150 cm
  drivetrain.queueTurn(-90.0f, 90.0f);
  drivetrain.queueDriveStraight(75.0f,  60.0f);
  drivetrain.queueTurn(-90.0f, 90.0f);
  drivetrain.queueDriveStraight(50.0f,  60.0f);
  drivetrain.queueTurn(90.0f,  90.0f);
}

// ---------------------------------------------------------------------------
// loop – IMU + drivetrain update every iteration; debug print at 10 Hz.
// ---------------------------------------------------------------------------

void loop() {
  // ── Control loop – runs every iteration of loop() ─────────────────────
  imu.update();
  drivetrain.update();

  // ── Debug output at 10 Hz ─────────────────────────────────────────────
  static uint32_t lastPrintMs = 0;
  const  uint32_t now         = millis();

  if (now - lastPrintMs >= 100) {
    lastPrintMs = now;

    Serial1.print("Busy:");
    Serial1.print(drivetrain.isBusy() ? "Y " : "N ");
    Serial1.print("Queue:");
    Serial1.print(drivetrain.getQueueCount());
    Serial1.print("  L:");
    Serial1.print(stepper_l.currentSpeed(), 0);
    Serial1.print(" sps  R:");
    Serial1.print(stepper_r.currentSpeed(), 0);
    Serial1.println(" sps");

    // IMU data
    Rotation *rot = imu.getRotation();
    GyroData  g   = imu.getGyroData();
    AccelData a   = imu.getAccelData();

    Serial1.print("  IMU Yaw:");
    Serial1.print(rot->getYawDegs(), 1);
    Serial1.print(" Pitch:");
    Serial1.print(rot->getPitchDegs(), 1);
    Serial1.print(" Roll:");
    Serial1.print(rot->getRollDegs(), 1);
    Serial1.println(" deg");

    Serial1.print("  Gyro X:");
    Serial1.print(g.x, 3);
    Serial1.print(" Y:");
    Serial1.print(g.y, 3);
    Serial1.print(" Z:");
    Serial1.print(g.z, 3);
    Serial1.println(" rad/s");

    Serial1.print("  Accel X:");
    Serial1.print(a.x, 2);
    Serial1.print(" Y:");
    Serial1.print(a.y, 2);
    Serial1.print(" Z:");
    Serial1.print(a.z, 2);
    Serial1.println(" m/s2");
  }
}
