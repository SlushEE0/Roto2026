#include "Stepper.h"

#define MAX_STEPPERS 6
static Stepper *steppers[MAX_STEPPERS];
static uint8_t stepperCount = 0;

ISR(TIMER1_COMPA_vect) {
  for (uint8_t i = 0; i < stepperCount; i++) {
    steppers[i]->stepService();
  }
}

Stepper::Stepper(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin) :
  _stepPin(stepPin),
  _dirPin(dirPin),
  _enablePin(enablePin) {
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  if (_enablePin != 255)
    pinMode(_enablePin, OUTPUT);

  if (stepperCount < MAX_STEPPERS) {
    steppers[stepperCount++] = this;
  }

  // init ISR once
  static bool timerInit = false;
  if (!timerInit) {
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 16; // compare every 1µs @16MHz
    TCCR1B |= (1 << WGM12); // CTC mode
    TCCR1B |= (1 << CS10); // prescaler = 1
    TIMSK1 |= (1 << OCIE1A); // enable compare interrupt
    sei();
    timerInit = true;
  }
}

void Stepper::setSpeed(long stepsPerSec) {
  if (stepsPerSec > 0) {
    _stepInterval = 1000000L / stepsPerSec;
  }
}

void Stepper::moveTo(long absolute) {
  _targetPos = absolute;
}

void Stepper::move(long relative) {
  _targetPos = _currentPos + relative;
}

long Stepper::currentPosition() {
  return _currentPos;
}

bool Stepper::isBusy() {
  return _currentPos != _targetPos;
}

void Stepper::enable() {
  if (_enablePin != 255)
    digitalWrite(_enablePin, LOW);
}

void Stepper::disable() {
  if (_enablePin != 255)
    digitalWrite(_enablePin, HIGH);
}

void Stepper::stepService() {
  if (_enablePin != 255) {
    if (_currentPos != _targetPos)
      enable();
    else
      disable();
  }

  unsigned long now = micros();
  if (_currentPos == _targetPos)
    return;

  if ((now - _lastStepMicros) >= _stepInterval) {
    _dir = (_targetPos > _currentPos);
    digitalWrite(_dirPin, _dir ? HIGH : LOW);

    // step pulse
    digitalWrite(_stepPin, HIGH);
    digitalWrite(_stepPin, LOW);

    _currentPos += (_dir ? 1 : -1);
    _lastStepMicros = now;
  }
}
