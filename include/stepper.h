#pragma once

#include <Arduino.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Stepper – fixed-rate tick-driven step generator
//
// A single HardwareTimer (configured in main.cpp) fires an ISR at
// kStepTimerHz.  User code calls Stepper::tick() for every motor from
// that ISR.  Step timing uses an integer countdown; floating-point ramp
// arithmetic only runs once per step event (≤ kStepTimerHz/2 per second
// per motor), never on every timer tick.
//
// Why a fixed-rate timer instead of a variable-period one?
//   Reprogramming a timer's ARR/PSC registers inside an ISR (the old
//   approach) introduces hardware-pipeline jitter and is fragile at high
//   step rates.  A fixed tick removes all of that; both motors share the
//   same hardware timer and have deterministic, jitter-free step pulses.
//
// Hardware wiring (A4988 / compatible):
//   STEP – rising edge advances one micro-step
//   DIR  – HIGH = positive direction (invertDir flag flips this)
//   EN   – active-LOW; HIGH turns the driver off
// ---------------------------------------------------------------------------

// Tick rate – must match the HardwareTimer period in main.cpp.
// 200 kHz → 5 µs / tick.  Actual max step rate = kStepTimerHz / (kPulseTicks + 1)
// ≈ 66.7 kHz, well above the A4988 practical limit of ~50 kHz.
static constexpr uint32_t kStepTimerHz = 200000UL;

// STEP pulse width in ticks.  2 × 5 µs = 10 µs.  A4988 needs only 1 µs minimum;
// 10 µs gives a comfortable margin against propagation delays.
static constexpr uint32_t kPulseTicks = 2;

class Stepper {
public:
  enum class Mode : uint8_t { Idle = 0, Velocity, Position };

  Stepper(uint8_t stepPin,
          uint8_t dirPin,
          uint8_t enablePin,
          bool    invertDir = false);

  // ── Setup (call before starting the shared timer) ────────────────────────
  void begin();

  // ── Configuration ─────────────────────────────────────────────────────────
  void setMaxSpeed(float stepsPerSec);
  void setAcceleration(float stepsPerSec2); // 0 = instant (no ramp)

  // ── Motion commands ───────────────────────────────────────────────────────
  // Signed velocity; positive = positive direction.
  void commandVelocity(float stepsPerSec);

  // Absolute / relative position with optional speed cap (0 → use maxSpeed).
  void commandPosition(int32_t target, float maxSpeed = 0.0f);
  void commandRelative(int32_t delta,  float maxSpeed = 0.0f);

  // Immediate hard stop; optionally disable the driver (removes holding torque).
  void stop(bool disableDriver = false);

  // ── State queries ──────────────────────────────────────────────────────────
  Mode    mode()             const { return _mode; }
  bool    isBusy()           const { return _mode != Mode::Idle; }
  int32_t currentPosition()  const { return _position; }
  float   currentSpeed()     const; // unsigned magnitude, steps/s
  float   commandedVelocity()const { return _cmdVelocity; }
  float   maxSpeed()         const { return _maxSpeed; }
  float   acceleration()     const { return _accel; }

  // ── Called from the shared HardwareTimer ISR – not for user code ──────────
  void tick();

private:
  void     doEnable();
  void     applyDir(int8_t dir);
  uint32_t speedToTps(float stepsPerSec) const; // steps/s → ticks/step

  // Sentinel: "not moving / fully decelerated".
  static constexpr uint32_t kTpsStop = 0x7FFFFFFFul;

  // ── Pins ──────────────────────────────────────────────────────────────────
  const uint8_t _stepPin;
  const uint8_t _dirPin;
  const uint8_t _enablePin;
  const bool    _invertDir;

  // ── Limits ────────────────────────────────────────────────────────────────
  float _maxSpeed; // steps/s
  float _accel;    // steps/s²

  // ── ISR state (write with interrupts disabled, or only from tick()) ───────

  volatile Mode _mode;

  // Step counter – monotonically tracks physical position.
  volatile int32_t _position;

  // Absolute target (Position mode only).
  volatile int32_t _target;

  // Current speed as ticks-per-step.  Smaller = faster.  kTpsStop = stopped.
  volatile uint32_t _tps;

  // Target ticks-per-step set by the latest command.
  volatile uint32_t _targetTps;

  // Actual current speed in steps/s – updated every step event.
  // Used for ramp math to avoid re-deriving from the quantized integer _tps,
  // which would cause large deceleration errors at high step rates.
  volatile float _speedSps;

  // Mirror of _accel used inside tick() to avoid reading a non-volatile float.
  volatile float _accelISR;

  // Ticks remaining until the next rising edge.
  volatile uint32_t _counter;

  // Non-zero while STEP pin is HIGH; counts down to the falling edge.
  volatile uint32_t _pulseCnt;

  // Current step direction (+1 or -1).
  volatile int8_t _dir;

  // Signed commanded velocity – kept for commandedVelocity() and the
  // near-zero check inside tick() for Velocity mode.
  volatile float _cmdVelocity;
};
