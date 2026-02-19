#include "stepper.h"

#include <math.h>

#include <config.h>

// POST-USC
/*
* this is probably the issue
* - HELLA vibrations at ANY speed, could be torque, microstepping, etc.
* - maybe switch back to bresnham?
* - obtain an oscilloscope?
* - lowk nuke the codebase neel
*/


namespace {
constexpr uint32_t kStepPulseWidthUs = 3;
constexpr float    kSpeedEpsilon     = 1e-3f;
constexpr uint32_t kPrimeDelayUs     = 5;
constexpr uint32_t kMinOffTimeUs     = 1;
} // namespace

Stepper::Stepper(HardwareTimer *timer,
                 uint8_t        stepPin,
                 uint8_t        dirPin,
                 uint8_t        enablePin,
                 bool           invertDir)
  : _timer(timer),
    _stepPin(stepPin),
    _dirPin(dirPin),
    _enablePin(enablePin),
    _invertDir(invertDir),
    _currentPosition(0),
    _targetPosition(0),
    _commandVelocity(0.0f),
    _maxSpeed(
      static_cast<float>(MOTOR_MAX_SPEED > 0 ? MOTOR_MAX_SPEED : 20000)),
    _acceleration(0.0f),
    _stepIntervalUs(0),
    _currentSpeedStepsPerSec(0.0f),
    _running(false),
    _stepPinIsHigh(false),
    _directionSign(1),
    _mode(static_cast<uint8_t>(Mode::Idle)) {}

void Stepper::begin(uint32_t /*timerFreqHz*/) {
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_enablePin, OUTPUT);
  digitalWrite(_stepPin, LOW);
  digitalWrite(_dirPin, LOW);
  digitalWrite(_enablePin, HIGH);

  // HardwareTimer::setOverflow(MICROSEC_FORMAT) manages the prescaler internally.
  // We must NOT manually set prescaler before this call as it would conflict.
  _timer->pause();
  _timer->setOverflow(1000, MICROSEC_FORMAT); // placeholder; reprogrammed each step
  _timer->attachInterrupt([this]() { this->handleTimerInterrupt(); });
  _timer->pause();

  stop(true);
}

void Stepper::enable() { digitalWrite(_enablePin, LOW); }

void Stepper::disable() { stop(true); }

void Stepper::stop(bool disableDriver) {
  noInterrupts();
  _running                  = false;
  _mode                     = static_cast<uint8_t>(Mode::Idle);
  _commandVelocity          = 0.0f;
  _targetPosition           = _currentPosition;
  _stepPinIsHigh            = false;
  _currentSpeedStepsPerSec  = 0.0f;
  interrupts();

  _timer->pause();
  digitalWrite(_stepPin, LOW);
  if (disableDriver) { digitalWrite(_enablePin, HIGH); }
}

void Stepper::setMaxSpeed(float stepsPerSecond) {
  if (stepsPerSecond > 0.0f) { _maxSpeed = stepsPerSecond; }
}

void Stepper::commandRelative(int32_t stepDelta, float speedStepsPerSec) {
  int32_t current;
  noInterrupts();
  current = _currentPosition;
  interrupts();
  commandPosition(current + stepDelta, speedStepsPerSec);
}

void Stepper::commandPosition(int32_t targetPosition, float speedStepsPerSec) {
  if (speedStepsPerSec <= 0.0f) {
    speedStepsPerSec = (_maxSpeed > 0.0f) ? _maxSpeed : fabsf(speedStepsPerSec);
    if (speedStepsPerSec <= 0.0f) speedStepsPerSec = 1000.0f;
  }
  if (_maxSpeed > 0.0f && speedStepsPerSec > _maxSpeed) {
    speedStepsPerSec = _maxSpeed;
  }

  enable();

  uint32_t interval = intervalFromSpeed(speedStepsPerSec);

  noInterrupts();
  _targetPosition = targetPosition;
  int32_t delta   = _targetPosition - _currentPosition;

  if (delta == 0) {
    _mode            = static_cast<uint8_t>(Mode::Idle);
    _commandVelocity = 0.0f;
    _running         = false;
    interrupts();
    _timer->pause();
    digitalWrite(_stepPin, LOW);
    return;
  }

  int8_t direction = (delta > 0) ? 1 : -1;
  bool   start = !_running || (_mode != static_cast<uint8_t>(Mode::Position));

  _mode            = static_cast<uint8_t>(Mode::Position);
  _commandVelocity = speedStepsPerSec * static_cast<float>(direction);
  _stepIntervalUs  = interval;
  _running         = true;
  if (start) {
    _stepPinIsHigh           = false;
    _currentSpeedStepsPerSec = 0.0f;
  }
  applyDirection(direction);
  interrupts();

  if (start) { primeTimer(kPrimeDelayUs); }
}

void Stepper::commandVelocity(float stepsPerSecond) {
  if (_maxSpeed > 0.0f) {
    if (stepsPerSecond > _maxSpeed) stepsPerSecond = _maxSpeed;
    if (stepsPerSecond < -_maxSpeed) stepsPerSecond = -_maxSpeed;
  }

  if (fabsf(stepsPerSecond) < kSpeedEpsilon) {
    stop(false);
    return;
  }

  enable();

  float    magnitude = fabsf(stepsPerSecond);
  uint32_t interval  = intervalFromSpeed(magnitude);
  int8_t   direction = (stepsPerSecond > 0.0f) ? 1 : -1;

  noInterrupts();
  bool start = !_running || (_mode != static_cast<uint8_t>(Mode::Velocity));
  _mode      = static_cast<uint8_t>(Mode::Velocity);
  _commandVelocity = stepsPerSecond;
  _stepIntervalUs  = interval;
  _running         = true;
  if (start) {
    _stepPinIsHigh           = false;
    _currentSpeedStepsPerSec = 0.0f;
  }
  applyDirection(direction);
  interrupts();

  if (start) { primeTimer(kPrimeDelayUs); }
}

uint32_t Stepper::intervalFromSpeed(float stepsPerSecond) const {
  if (stepsPerSecond < 1.0f) stepsPerSecond = 1.0f;
  float interval = 1000000.0f / stepsPerSecond;
  float minUs    = static_cast<float>(kStepPulseWidthUs + 1);
  if (interval < minUs) interval = minUs;
  return static_cast<uint32_t>(interval);
}

void Stepper::applyDirection(int8_t dir) {
  _directionSign = (dir >= 0) ? 1 : -1;
  bool dirHigh   = (_directionSign > 0);
  digitalWrite(_dirPin, _invertDir ? !dirHigh : dirHigh);
}

void Stepper::primeTimer(uint32_t periodUs) {
  if (periodUs == 0) periodUs = 1;
  _timer->pause();
  _timer->setCount(0);
  _timer->setOverflow(periodUs, MICROSEC_FORMAT);
  _timer->refresh();
  _timer->resume();
}

void Stepper::hardStopFromISR() {
  _running                 = false;
  _mode                    = static_cast<uint8_t>(Mode::Idle);
  _commandVelocity         = 0.0f;
  _targetPosition          = _currentPosition;
  _stepPinIsHigh           = false;
  _currentSpeedStepsPerSec = 0.0f;
  _timer->pause();
  digitalWrite(_stepPin, LOW);
}

void Stepper::handleTimerInterrupt() {
  if (!_running) {
    _timer->pause();
    digitalWrite(_stepPin, LOW);
    _stepPinIsHigh = false;
    return;
  }

  if (!_stepPinIsHigh) {
    // Rising edge: assert step pulse
    digitalWrite(_stepPin, HIGH);
    _stepPinIsHigh = true;
    primeTimer(kStepPulseWidthUs);
    return;
  }

  // Falling edge: step pulse complete — count the step
  digitalWrite(_stepPin, LOW);
  _stepPinIsHigh = false;
  _currentPosition += _directionSign;

  // --- Mode-specific position / direction checks ---
  if (_mode == static_cast<uint8_t>(Mode::Position)) {
    if (_currentPosition == _targetPosition) {
      hardStopFromISR();
      return;
    }
    int32_t remaining = _targetPosition - _currentPosition;
    int8_t  desired   = (remaining > 0) ? 1 : -1;
    if (desired != _directionSign) { applyDirection(desired); }

  } else if (_mode == static_cast<uint8_t>(Mode::Velocity)) {
    float commanded = _commandVelocity;
    if (fabsf(commanded) < kSpeedEpsilon) {
      hardStopFromISR();
      return;
    }
    int8_t desired = (commanded > 0.0f) ? 1 : -1;
    if (desired != _directionSign) { applyDirection(desired); }

  } else {
    hardStopFromISR();
    return;
  }

  // --- Compute the commanded (target) speed magnitude ---
  float cmdSpeed = fabsf(_commandVelocity);

  // For position mode: apply deceleration look-ahead so we arrive at the
  // target position with zero velocity instead of slamming into it.
  if (_mode == static_cast<uint8_t>(Mode::Position) && _acceleration > 0.0f) {
    int32_t remaining  = abs(_targetPosition - _currentPosition);
    float   v          = _currentSpeedStepsPerSec;
    // Distance (in steps) needed to brake from v to 0 at _acceleration.
    float   decelSteps = (v * v) / (2.0f * _acceleration);
    if (static_cast<float>(remaining) <= decelSteps) {
      // Target speed that brings us to a stop exactly at the target.
      float decelV = sqrtf(2.0f * _acceleration * static_cast<float>(remaining));
      if (decelV < cmdSpeed) { cmdSpeed = decelV; }
    }
  }

  // --- Acceleration ramp: smoothly move current speed toward cmdSpeed ---
  float v = _currentSpeedStepsPerSec;
  if (_acceleration > 0.0f) {
    float dv;
    if (v < 1.0f) {
      // Starting from rest: first-step speed using Austin/Eiderman formula.
      // This corresponds to the time needed to travel 1 step from rest.
      dv = sqrtf(2.0f * _acceleration);
    } else {
      // Normal operation: dv = a * dt, where dt ≈ 1/v (seconds per step).
      dv = _acceleration / v;
    }
    float diff = cmdSpeed - v;
    v += (fabsf(diff) <= dv) ? diff : (diff > 0.0f ? dv : -dv);
  } else {
    // No ramp configured: instant speed change (legacy behaviour).
    v = cmdSpeed;
  }

  // If the ramp has brought us to a full stop, finish.
  if (v < kSpeedEpsilon) {
    hardStopFromISR();
    return;
  }

  _currentSpeedStepsPerSec = v;

  // Schedule the off-time before the next rising edge.
  uint32_t nextIntervalUs = static_cast<uint32_t>(1e6f / v);
  uint32_t offTime        = (nextIntervalUs > kStepPulseWidthUs)
                               ? (nextIntervalUs - kStepPulseWidthUs)
                               : kMinOffTimeUs;
  primeTimer(offTime);
}
