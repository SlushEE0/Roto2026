#include "stepper.h"
#include <math.h>

#define STEP_PULSE_WIDTH_US 2

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
    _currentSpeed(0.0f),
    _maxSpeed(0.0f),
    _acceleration(0.0f),
    _stepIntervalUs(0),
    _running(false),
    _stepPinIsHigh(false),
    _directionSign(1) {}

void Stepper::begin(uint32_t timerFreqHz) {
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_enablePin, OUTPUT);
  digitalWrite(_stepPin, LOW);
  disable();

  if (timerFreqHz == 0) timerFreqHz = 1000000UL;
  uint32_t prescale = (SystemCoreClock / timerFreqHz) - 1;
  _timer->setPrescaleFactor(prescale);
  _timer->setOverflow(1000, MICROSEC_FORMAT);
  _timer->attachInterrupt([this]() { this->handleTimerInterrupt(); });
  _timer->pause();

  if (_maxSpeed <= 0.0f) _maxSpeed = 1000.0f;
  if (_acceleration <= 0.0f) _acceleration = 1000.0f;
}

void Stepper::enable() { digitalWrite(_enablePin, LOW); }

void Stepper::disable() {
  _running = false;
  _timer->pause();
  digitalWrite(_stepPin, LOW);
  digitalWrite(_enablePin, HIGH);
}

void Stepper::setMaxSpeed(float stepsPerSecond) {
  if (stepsPerSecond <= 0.0f) return;
  _maxSpeed = stepsPerSecond;
}

void Stepper::setAcceleration(float stepsPerSecondSquared) {
  if (stepsPerSecondSquared <= 0.0f) return;
  _acceleration = stepsPerSecondSquared;
}

void Stepper::setTarget(int32_t targetPosition) {
  _targetPosition = targetPosition;
  if (_running) return;
  moveTo(targetPosition,
         static_cast<int32_t>(_maxSpeed),
         static_cast<int32_t>(_acceleration));
}

void Stepper::moveTo(int32_t targetPosition,
                     int32_t maxSpeedStepsPerSec,
                     int32_t accelStepsPerSec2) {
  if (maxSpeedStepsPerSec != 0) {
    _maxSpeed = static_cast<float>(abs(maxSpeedStepsPerSec));
  }
  if (_maxSpeed <= 0.0f) _maxSpeed = 1000.0f;

  if (accelStepsPerSec2 != 0) {
    _acceleration = static_cast<float>(abs(accelStepsPerSec2));
  }
  if (_acceleration <= 0.0f) _acceleration = _maxSpeed * 2.0f;

  _targetPosition = targetPosition;

  int32_t delta = _targetPosition - _currentPosition;
  if (delta == 0) {
    _running      = false;
    _currentSpeed = 0.0f;
    _timer->pause();
    digitalWrite(_stepPin, LOW);
    return;
  }

  enable();

  if (!_running) {
    _stepPinIsHigh = false;
    _currentSpeed  = 0.0f;
    applyDirection(delta > 0 ? 1 : -1);

    if (_acceleration > 0.0f) {
      float firstInterval = sqrtf(2.0f / _acceleration) * 1000000.0f;
      _stepIntervalUs     = static_cast<uint32_t>(firstInterval);
    } else {
      _stepIntervalUs = static_cast<uint32_t>(1000000.0f / _maxSpeed);
    }

    if (_stepIntervalUs < (STEP_PULSE_WIDTH_US + 2)) {
      _stepIntervalUs = STEP_PULSE_WIDTH_US + 2;
    }

    _running = true;
    primeTimer(5);
  }
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

bool Stepper::planNextStep() {
  int32_t delta = _targetPosition - _currentPosition;
  if (delta == 0) {
    _currentSpeed = 0.0f;
    return false;
  }

  int8_t desiredDir = (delta > 0) ? 1 : -1;

  if (_currentSpeed == 0.0f) { applyDirection(desiredDir); }

  float speedMag = fabsf(_currentSpeed);
  float dt       = (_stepIntervalUs > 0)
                     ? (static_cast<float>(_stepIntervalUs) * 1e-6f)
                     : ((_acceleration > 0.0f) ? sqrtf(2.0f / _acceleration)
                                               : (1.0f / _maxSpeed));

  if (_acceleration <= 0.0f) {
    speedMag = _maxSpeed;
  } else {
    float  stepsRemaining = static_cast<float>(abs(delta));
    float  stepsToStop    = (speedMag * speedMag) / (2.0f * _acceleration);
    int8_t speedDir =
      (_currentSpeed > 0.0f) ? 1 : (_currentSpeed < 0.0f ? -1 : 0);

    if (speedDir != 0 && speedDir != desiredDir) {
      speedMag -= _acceleration * dt;
      if (speedMag < 0.0f) speedMag = 0.0f;
      if (speedMag == 0.0f) { applyDirection(desiredDir); }
    } else {
      if (stepsToStop >= stepsRemaining) {
        speedMag -= _acceleration * dt;
        if (speedMag < 0.0f) speedMag = 0.0f;
      } else if (speedMag < _maxSpeed) {
        speedMag += _acceleration * dt;
        if (speedMag > _maxSpeed) speedMag = _maxSpeed;
      }
    }
  }

  if (speedMag < 0.5f) { speedMag = 0.5f; }

  _currentSpeed = speedMag * static_cast<float>(_directionSign);

  float interval = 1000000.0f / speedMag;
  if (interval < (STEP_PULSE_WIDTH_US + 2)) {
    interval = STEP_PULSE_WIDTH_US + 2;
  }
  _stepIntervalUs = static_cast<uint32_t>(interval);
  return true;
}

void Stepper::handleTimerInterrupt() {
  if (!_running) {
    _timer->pause();
    digitalWrite(_stepPin, LOW);
    return;
  }

  if (!_stepPinIsHigh) {
    digitalWrite(_stepPin, HIGH);
    _stepPinIsHigh = true;
    primeTimer(STEP_PULSE_WIDTH_US);
    return;
  }

  digitalWrite(_stepPin, LOW);
  _stepPinIsHigh = false;
  _currentPosition += _directionSign;

  if (!planNextStep()) {
    _running      = false;
    _currentSpeed = 0.0f;
    _timer->pause();
    return;
  }

  uint32_t offTime = (_stepIntervalUs > STEP_PULSE_WIDTH_US)
                       ? (_stepIntervalUs - STEP_PULSE_WIDTH_US)
                       : 1;
  primeTimer(offTime);
}