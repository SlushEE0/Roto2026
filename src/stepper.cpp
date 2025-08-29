#include "Stepper.h"

// small global map: index 1..4 maps TIM1..TIM4 to Stepper*
static Stepper *g_timerOwners[5] = { nullptr,
                                     nullptr,
                                     nullptr,
                                     nullptr,
                                     nullptr };

// forward callbacks (attachInterrupt wants a plain function)
static void hwTimer_cb_tim1() {
  if (g_timerOwners[1])
    g_timerOwners[1]->onTimerISR();
}

static void hwTimer_cb_tim2() {
  if (g_timerOwners[2])
    g_timerOwners[2]->onTimerISR();
}

static void hwTimer_cb_tim3() {
  if (g_timerOwners[3])
    g_timerOwners[3]->onTimerISR();
}

static void hwTimer_cb_tim4() {
  if (g_timerOwners[4])
    g_timerOwners[4]->onTimerISR();
}

int Stepper::timerIndex(TIM_TypeDef *tim) {
  if (tim == TIM1)
    return 1;
  if (tim == TIM2)
    return 2;
  if (tim == TIM3)
    return 3;
  if (tim == TIM4)
    return 4;
  return -1;
}

Stepper::Stepper(uint8_t stepPin,
                 uint8_t dirPin,
                 TIM_TypeDef *timer,
                 int8_t enablePin,
                 bool enableActiveHigh,
                 uint32_t pulse_us) :
  _stepPin(stepPin),
  _dirPin(dirPin),
  _enablePin(enablePin),
  _enableActiveHigh(enableActiveHigh),
  _reverseDir(false),
  _timInstance(timer),
  _pulse_us(pulse_us) {
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  digitalWrite(_stepPin, LOW);

  if (_enablePin >= 0) {
    pinMode(_enablePin, OUTPUT);
    digitalWrite(_enablePin, !_enableActiveHigh); // start disabled
  }

  _hwTimer = new HardwareTimer(_timInstance);

  int idx = timerIndex(_timInstance);
  if (idx < 1 || idx > 4) {
    // unsupported timer instance
    // keep _hwTimer null-check in rest of code
    return;
  }
  g_timerOwners[idx] = this;

  // attach the proper plain callback for this timer
  if (idx == 1)
    _hwTimer->attachInterrupt(hwTimer_cb_tim1);
  if (idx == 2)
    _hwTimer->attachInterrupt(hwTimer_cb_tim2);
  if (idx == 3)
    _hwTimer->attachInterrupt(hwTimer_cb_tim3);
  if (idx == 4)
    _hwTimer->attachInterrupt(hwTimer_cb_tim4);

  // default small overflow so ISR won't be very slow to start; pause
  // immediately
  _hwTimer->setOverflow(1000, MICROSEC_FORMAT);
  _hwTimer->pause();
}

void Stepper::setMaxSpeed(double s) {
  _maxSpeed = s;
}

void Stepper::setAcceleration(double a) {
  _accel = a;
}

void Stepper::setSCurve(bool on) {
  _sCurve = on;
}

void Stepper::setReverse(bool rev) {
  _reverseDir = rev;
}

void Stepper::setPulseWidthUs(uint32_t p) {
  _pulse_us = max<uint32_t>(1, p);
}

void Stepper::move(long steps) {
  moveTo(_currentPos + steps);
}

void Stepper::moveTo(long position) {
  _targetPos = position;
  _running = true;
  _phase = 0;

  // enable driver if present
  if (_enablePin >= 0)
    digitalWrite(_enablePin, _enableActiveHigh);

  // prepare first period: compute a sensible starting period
  computeNewPeriod();
  // schedule first interrupt to start the first step after the (period - pulse)
  // time
  uint32_t remainder =
    (_period_us > _pulse_us) ? (_period_us - _pulse_us) : _min_period_us;
  _hwTimer->setOverflow(remainder, MICROSEC_FORMAT);
  _hwTimer->resume();
}

void Stepper::stop() {
  // graceful: set target to current so we ramp to zero
  _targetPos = _currentPos;
}

void Stepper::startTimer() {
  if (_hwTimer)
    _hwTimer->resume();
}

void Stepper::stopTimer() {
  if (_hwTimer)
    _hwTimer->pause();
}

// update _currentSpeed towards _targetSpeed and compute a new _period_us
void Stepper::computeNewPeriod() {
  long remaining = _targetPos - _currentPos;
  if (remaining == 0) {
    _targetSpeed = 0.0f;
  } else {
    _targetSpeed = (remaining > 0) ? _maxSpeed : -_maxSpeed;
  }

  // estimate dt from last period (avoid 0)
  double dt = max<double>(_last_period_us / 1e6f, 1e-6f);

  double accelStep = _accel * dt;
  if (_sCurve)
    accelStep *=
      (1.0 - expf(-fabs(_currentSpeed) * _jerkFactor / max(1.0, _maxSpeed)));

  if (_currentSpeed < _targetSpeed)
    _currentSpeed = min(_currentSpeed + accelStep, _targetSpeed);
  else if (_currentSpeed > _targetSpeed)
    _currentSpeed = max(_currentSpeed - accelStep, _targetSpeed);

  // if still essentially stopped but we need to move, kick-start with a tiny
  // speed so period isn't infinite
  if (fabs(_currentSpeed) < 1e-4f && _targetSpeed != 0.0f) {
    _currentSpeed =
      (_targetSpeed > 0) ? min(1.0, _accel * dt) : max(-1.0, -_accel * dt);
  }

  if (fabs(_currentSpeed) < 1e-6f) {
    // stopped => set a long period that will effectively pause steps
    _period_us = 1000000; // 1 second
  } else {
    double f = fabs(_currentSpeed); // steps/sec
    uint32_t p_us = (uint32_t)max<double>(_min_period_us, (1e6f / f));
    _period_us = p_us;
  }
}

// The timer callback (runs in IRQ context)
void Stepper::onTimerISR() {
  if (!_running) {
    // nothing to do
    _hwTimer->pause();
    return;
  }

  // Phase 0 => start step (raise pin), Phase 1 => end pulse (lower pin +
  // advance)
  if (_phase == 0) {
    // if we are at the target already, finish
    if (_currentPos == _targetPos) {
      _running = false;
      _hwTimer->pause();
      if (_enablePin >= 0)
        digitalWrite(_enablePin, !_enableActiveHigh); // auto-disable
      return;
    }

    // set direction according to remaining distance (more robust than using
    // _currentSpeed sign)
    long remaining = _targetPos - _currentPos;
    bool dirHigh = (remaining > 0) ? true : false;
    if (_reverseDir)
      dirHigh = !dirHigh;
    digitalWrite(_dirPin, dirHigh ? HIGH : LOW);

    // raise step pin
    digitalWrite(_stepPin, HIGH);

    // next interrupt: end pulse after pulse_us
    _phase = 1;
    _hwTimer->setOverflow(_pulse_us, MICROSEC_FORMAT);
    return;
  }

  // phase == 1: finish pulse
  digitalWrite(_stepPin, LOW);

  // update position (use direction pin as authority)
  bool dirHigh = (digitalRead(_dirPin) == HIGH);
  bool movedPositive = _reverseDir ? !dirHigh : dirHigh;
  if (movedPositive)
    _currentPos++;
  else
    _currentPos--;

  // store last period and recompute speed/period for next step
  _last_period_us = _period_us;
  computeNewPeriod();

  // if reached destination, stop and auto disable
  if (_currentPos == _targetPos && fabs(_currentSpeed) < 1e-3f) {
    _running = false;
    _hwTimer->pause();
    if (_enablePin >= 0)
      digitalWrite(_enablePin, !_enableActiveHigh); // auto-disable
    return;
  }

  // schedule remainder of the period (period - pulse)
  uint32_t remainder =
    (_period_us > _pulse_us) ? (_period_us - _pulse_us) : _min_period_us;
  _hwTimer->setOverflow(remainder, MICROSEC_FORMAT);
  _phase = 0;
}
