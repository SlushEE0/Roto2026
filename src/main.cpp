#include <Arduino.h>

#include <config.h>
#include <dead_reckoning.h>
#include <stepper.h>
#include <bno.h>

// ---------------------------------------------------------------------------
// Hardware objects
// ---------------------------------------------------------------------------

// Both motors share one hardware timer running at kStepTimerHz (200 kHz).
// Using TIM2 keeps TIM1 free for the fast control-loop timer below.
HardwareTimer stepTimer(TIM2);
HardwareTimer fastLoopTimer(TIM1);

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

// Flag set by TIM1 ISR; the actual work runs in loop() so the heavy I2C
// and control-math never block the 200 kHz step timer.
volatile bool controlLoopReady = false;

// TIM1 ISR – just raises the flag; keeps ISR < 1 µs.
void fastLoopISR() {
  controlLoopReady = true;
}

// Actual control-loop work (called from loop()).
void fastLoop() {
  imu.update();
  drivetrain.update();
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
  // by the control-loop timer or any other interrupt.
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

  // ── Fast control-loop timer ─────────────────────────────────────────────
  // The ISR only sets a flag; the heavy work (I2C + math) runs in loop()
  // so it cannot block the 200 kHz step timer.
  fastLoopTimer.setOverflow(12000, MICROSEC_FORMAT); // 12 ms = ~83 Hz
  fastLoopTimer.attachInterrupt(fastLoopISR);
  fastLoopTimer.resume();

  // Control-loop timer needs a lower priority than the step timer.
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 6, 0);
  Serial1.println("[INIT] Control loop running at ~83 Hz (priority 6)");

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
// loop – runs at whatever rate the scheduler allows (~kHz).
// Print debug info at 10 Hz so it doesn't flood the serial port.
// ---------------------------------------------------------------------------

void loop() {
  // ── Control loop – runs at ~83 Hz, driven by TIM1 flag ────────────────
  if (controlLoopReady) {
    controlLoopReady = false;
    fastLoop();
  }

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
  }
}
