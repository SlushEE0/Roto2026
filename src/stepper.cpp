#include "stepper.h"

#include <math.h>

// ---------------------------------------------------------------------------
// Implementation notes
// ---------------------------------------------------------------------------
// The step generator runs as a state machine clocked by a fixed-rate ISR
// (kStepTimerHz, typically 200 kHz).  Every tick(), two things can happen:
//
//   1. Falling-edge phase (pulseCnt > 0):
//        Count down the pulse width.  When pulseCnt reaches 0, pull STEP
//        LOW, advance the position counter, then compute the next ticks-per-
//        step value via the velocity ramp (one float multiply and possibly
//        one sqrtf – fast enough on Cortex-M3 at 72 MHz since it runs at
//        most once per step, not once per tick).
//
//   2. Countdown phase (pulseCnt == 0):
//        Decrement _counter.  When it reaches 0, assert STEP HIGH and arm
//        the pulse-width countdown.
//
// The velocity ramp uses the Austin / Eiderman recurrence:
//   • Starting from rest: first interval  c₀ = √(2 / a) × kStepTimerHz
//   • Each subsequent step: Δv = a / v_current  (= a × Δt)
//   • Position-mode deceleration: look ahead using  d = v² / (2a)  and cap
//     the target speed so the motor arrives at the target with v ≈ 0.
// ---------------------------------------------------------------------------

namespace {
  constexpr float kVelEpsilon = 1e-3f; // steps/s below which we consider stopped
}

// ── Constructor ──────────────────────────────────────────────────────────────

Stepper::Stepper(uint8_t stepPin,
                 uint8_t dirPin,
                 uint8_t enablePin,
                 bool    invertDir)
  : _stepPin(stepPin),
    _dirPin(dirPin),
    _enablePin(enablePin),
    _invertDir(invertDir),
    _maxSpeed(10000.0f),
    _accel(0.0f),
    _mode(Mode::Idle),
    _position(0),
    _target(0),
    _tps(kTpsStop),
    _targetTps(kTpsStop),
    _speedSps(0.0f),
    _accelISR(0.0f),
    _counter(kTpsStop),
    _pulseCnt(0),
    _dir(1),
    _cmdVelocity(0.0f) {}

// ── Setup ────────────────────────────────────────────────────────────────────

void Stepper::begin() {
  pinMode(_stepPin,   OUTPUT);
  pinMode(_dirPin,    OUTPUT);
  pinMode(_enablePin, OUTPUT);
  digitalWrite(_stepPin,   LOW);
  digitalWrite(_dirPin,    LOW);
  digitalWrite(_enablePin, HIGH); // driver disabled at startup
}

// ── Configuration ─────────────────────────────────────────────────────────────

void Stepper::setMaxSpeed(float stepsPerSec) {
  if (stepsPerSec > 0.0f) _maxSpeed = stepsPerSec;
}

void Stepper::setAcceleration(float stepsPerSec2) {
  _accel = (stepsPerSec2 > 0.0f) ? stepsPerSec2 : 0.0f;
  noInterrupts();
  _accelISR = _accel;
  interrupts();
}

// ── Helpers ───────────────────────────────────────────────────────────────────

float Stepper::currentSpeed() const {
  return _speedSps;
}

uint32_t Stepper::speedToTps(float stepsPerSec) const {
  if (stepsPerSec < kVelEpsilon) return kTpsStop;
  // Minimum ticks/step: pulse width + 1 off-tick.
  float minTps = (float)(kPulseTicks + 1u);
  float tps    = (float)kStepTimerHz / stepsPerSec;
  if (tps < minTps) tps = minTps;
  return (uint32_t)tps;
}

void Stepper::doEnable() {
  digitalWrite(_enablePin, LOW);
}

void Stepper::applyDir(int8_t dir) {
  _dir = (dir >= 0) ? 1 : -1;
  bool high = (_dir > 0);
  digitalWrite(_dirPin, _invertDir ? !high : high);
}

// ── Motion commands ───────────────────────────────────────────────────────────

void Stepper::commandVelocity(float stepsPerSec) {
  if (stepsPerSec >  _maxSpeed) stepsPerSec =  _maxSpeed;
  if (stepsPerSec < -_maxSpeed) stepsPerSec = -_maxSpeed;

  if (fabsf(stepsPerSec) < kVelEpsilon) {
    stop(false);
    return;
  }

  doEnable();

  int8_t   dir    = (stepsPerSec > 0.0f) ? 1 : -1;
  uint32_t newTps = speedToTps(fabsf(stepsPerSec));

  noInterrupts();
  const bool wasIdle = (_mode == Mode::Idle);
  _cmdVelocity = stepsPerSec;
  _targetTps   = newTps;
  _mode        = Mode::Velocity;
  applyDir(dir);

  if (wasIdle) {
    // Starting from rest: schedule the first step using the Austin first-step
    // formula c₀ = √(2/a) × kStepTimerHz.  Fall back to the target interval
    // when acceleration is disabled.
    _tps     = kTpsStop; // will be updated on the first falling edge
    _pulseCnt = 0;
    _counter  = (_accel > 0.0f)
                  ? (uint32_t)((float)kStepTimerHz * sqrtf(2.0f / _accel))
                  : newTps;
  }
  // If already running: tick() will ramp _tps toward _targetTps naturally.
  interrupts();
}

void Stepper::commandPosition(int32_t target, float maxSpeed) {
  if (maxSpeed <= 0.0f || maxSpeed > _maxSpeed) maxSpeed = _maxSpeed;

  int32_t cur;
  noInterrupts();
  cur = _position;
  interrupts();

  const int32_t delta = target - cur;
  if (delta == 0) {
    stop(false);
    return;
  }

  doEnable();

  const int8_t   dir    = (delta > 0) ? 1 : -1;
  const uint32_t newTps = speedToTps(maxSpeed);

  noInterrupts();
  const bool wasIdle = (_mode == Mode::Idle);
  _cmdVelocity = maxSpeed * (float)dir;
  _target      = target;
  _targetTps   = newTps;
  _mode        = Mode::Position;
  applyDir(dir);

  if (wasIdle) {
    _tps      = kTpsStop;
    _pulseCnt = 0;
    _counter  = (_accel > 0.0f)
                  ? (uint32_t)((float)kStepTimerHz * sqrtf(2.0f / _accel))
                  : newTps;
  }
  interrupts();
}

void Stepper::commandRelative(int32_t delta, float maxSpeed) {
  int32_t cur;
  noInterrupts();
  cur = _position;
  interrupts();
  commandPosition(cur + delta, maxSpeed);
}

void Stepper::stop(bool disableDriver) {
  noInterrupts();
  _mode        = Mode::Idle;
  _cmdVelocity = 0.0f;
  _target      = _position;
  _tps         = kTpsStop;
  _targetTps   = kTpsStop;
  _speedSps    = 0.0f;
  _pulseCnt    = 0;
  _counter     = kTpsStop;
  interrupts();

  digitalWrite(_stepPin, LOW);
  if (disableDriver) digitalWrite(_enablePin, HIGH);
}

// ── ISR tick ─────────────────────────────────────────────────────────────────
// Called at kStepTimerHz.  Keep this fast: integer ops on every call,
// float only on the (infrequent) step falling-edge event.

void Stepper::tick() {
  if (_mode == Mode::Idle) return;

  // ── Falling-edge phase: STEP pin is still HIGH ──────────────────────────
  if (_pulseCnt > 0) {
    if (--_pulseCnt == 0) {
      digitalWrite(_stepPin, LOW);
      _position += _dir;

      // Position mode: check arrival.
      if (_mode == Mode::Position) {
        if (_position == _target) {
          _mode        = Mode::Idle;
          _cmdVelocity = 0.0f;
          _speedSps    = 0.0f;
          _tps         = kTpsStop;
          return;
        }
        // Correct direction if we somehow overshot.
        const int32_t rem  = _target - _position;
        const int8_t  want = (rem > 0) ? 1 : -1;
        if (want != _dir) applyDir(want);
      }

      // ── Velocity ramp (float – runs once per step, not once per tick) ────
      const float v_cur = _speedSps;  // exact float, not re-derived from _tps
      float v_tgt = (float)kStepTimerHz / (float)_targetTps;

      // Position-mode deceleration look-ahead:
      // Reduce v_tgt so the motor brakes to a stop exactly at _target.
      if (_mode == Mode::Position && _accelISR > 0.0f) {
        const int32_t rem        = abs(_target - _position);
        const float   decelSteps = (v_cur * v_cur) / (2.0f * _accelISR);
        if ((float)rem <= decelSteps) {
          const float v_brake = sqrtf(2.0f * _accelISR * (float)rem);
          if (v_brake < v_tgt) v_tgt = v_brake;
        }
      }

      // Ramp current speed toward v_tgt.
      float v;
      if (_accelISR > 0.0f) {
        // Δv = a × Δt ≈ a / v_cur (time per step ≈ 1 / v_cur).
        // At rest (v_cur ≈ 0) use the Austin/Eiderman first-step formula:
        //   c₀ = √(2/a) · kStepTimerHz  →  Δv = √(2a).
        const float dv   = (v_cur <= kVelEpsilon) ? sqrtf(2.0f * _accelISR)
                                                   : (_accelISR / v_cur);
        const float diff = v_tgt - v_cur;
        v = v_cur + (fabsf(diff) <= dv ? diff : copysignf(dv, diff));
      } else {
        v = v_tgt; // No ramp: instant speed change.
      }

      if (v < kVelEpsilon) {
        // Fully decelerated to a stop.
        _mode        = Mode::Idle;
        _cmdVelocity = 0.0f;
        _speedSps    = 0.0f;
        _tps         = kTpsStop;
        return;
      }

      _speedSps = v;
      _tps = (uint32_t)((float)kStepTimerHz / v);

      // Schedule the next rising edge.  Subtract the pulse width already
      // "spent" so the total period (rising-to-rising) equals _tps ticks.
      // The 1-tick minimum is a safety net; speedToTps() already enforces
      // _tps >= kPulseTicks + 1 for all normally commanded speeds.
      _counter = (_tps > kPulseTicks) ? (_tps - kPulseTicks) : 1u;
    }
    return; // Pin still HIGH – nothing else to do this tick.
  }

  // ── Countdown phase: waiting for the next rising edge ───────────────────
  if (--_counter == 0) {
    // Velocity mode: bail out if a stop was requested between steps.
    if (_mode == Mode::Velocity && fabsf(_cmdVelocity) < kVelEpsilon) {
      _mode = Mode::Idle;
      return;
    }

    digitalWrite(_stepPin, HIGH);
    _pulseCnt = kPulseTicks;
  }
}
