#include <Arduino.h>
#include <FlexyStepper.h>
#include <utils.h>
#include <config.h>
#include <motor.h>

Motor::Motor() :
  pin_DIR(0),
  pin_STEP(0),
  pin_ENABLE(0),
  target_steps(0),
  position_steps(0),
  isAlwaysEnabled(false),
  isEnabled(false),
  isReversed(false) {
  // FlexyStepper doesn't need engine initialization
}

void Motor::connect(int dirPin, int stepPin, int enablePin, bool reversed) {
  pin_DIR = dirPin;
  pin_STEP = stepPin;
  pin_ENABLE = enablePin;
  this->isReversed = reversed;

  // Connect stepper to pins
  stepper.connectToPins(stepPin, dirPin);

  // Set up enable pin manually since FlexyStepper doesn't have setEnablePin
  if (enablePin >= 0) {
    pinMode(pin_ENABLE, OUTPUT);
  }

  disable();

  setAccel(accel);
  setSpeed(speed);

  Serial.print("Stepper: ");
  Serial.print(pin_STEP);
  Serial.println(" (step_pin) initialized");
}

Motor::~Motor() {
  disable();
}

long Motor::getMovePosition(long relativeSteps) {
  const long multiplier = isReversed ? -1 : 1;
  return position_steps + (relativeSteps * multiplier);
}

void Motor::enable() {
  digitalWrite(pin_ENABLE, LOW);
  isEnabled = true;
}

void Motor::disable() {
  digitalWrite(pin_ENABLE, HIGH);
  isEnabled = false;
}

void Motor::moveAbsolute_blocking(long steps) {
  Serial.print("Moving to absolute position: ");
  Serial.println((long)steps);

  long adjustedSteps = isReversed ? -steps : steps;
  stepper.moveToPositionInSteps(adjustedSteps);
  updateData();
}

void Motor::moveRelative_blocking(long steps) {
  long adjustedSteps = isReversed ? -steps : steps;
  Serial.println(adjustedSteps);
  enable();
  stepper.moveRelativeInSteps(adjustedSteps);
  disable();
  updateData();
}

void Motor::moveAbsolute_nonblocking(long steps) {
  long adjustedSteps = isReversed ? -steps : steps;
  stepper.setTargetPositionInSteps(adjustedSteps);
  updateData();
}

void Motor::moveRelative_nonblocking(long steps) {
  long currentPos = stepper.getCurrentPositionInSteps();
  long adjustedSteps = isReversed ? -steps : steps;
  stepper.setTargetPositionInSteps(currentPos + adjustedSteps);
  updateData();
}

void Motor::constrainedMove(long steps,
                            long accel,
                            long speed,
                            StepperMoveType moveType) {
  setAccel(accel);
  setSpeed(speed);

  switch (moveType) {
    case StepperMoveType::ABSOLUTE_BLOCKING:
      moveAbsolute_blocking(steps);
      break;
    case StepperMoveType::RELATIVE_BLOCKING:
      moveRelative_blocking(steps);
      break;
    case StepperMoveType::ABSOLUTE_NONBLOCKING:
      moveAbsolute_nonblocking(steps);
      break;
    case StepperMoveType::RELATIVE_NONBLOCKING:
      moveRelative_nonblocking(steps);
  }
}

// @param newSpeed steps/sec
void Motor::setSpeed(long newSpeed) {
  long setSpeed = newSpeed;
  bool clamped = clamp(&setSpeed, 1, MOTOR_MAX_SPEED);

  if (clamped) {
    Serial.print("[Motor] - WARN: Speed '");
    Serial.print(newSpeed);
    Serial.print("' clamped to '");
    Serial.print(setSpeed);
    Serial.println("'");
  }

  speed = setSpeed;
  stepper.setSpeedInStepsPerSecond(speed);
}

// @param newAccel steps/sec^2
void Motor::setAccel(long newAccel) {
  long setAccel = newAccel;
  bool clamped = clamp(&setAccel, 1, MOTOR_MAX_ACCEL);

  if (clamped) {
    Serial.print("[Motor] - WARN: Acceleration '");
    Serial.print(newAccel);
    Serial.print("' clamped to '");
    Serial.print(setAccel);
    Serial.println("'");
  }

  accel = setAccel;
  stepper.setAccelerationInStepsPerSecondPerSecond(accel);
}

void Motor::setReversed(bool isReversed) {
  this->isReversed = isReversed;
}

void Motor::setAlwaysEnabled(bool alwaysEnabled) {
  this->isAlwaysEnabled = alwaysEnabled;

  // FlexyStepper doesn't have setAutoEnable, we handle it manually
  if (alwaysEnabled) {
    enable();
  }
}

FlexyStepper *Motor::getStepper() {
  return &stepper;
}

long Motor::getPositionSteps() {
  return stepper.getCurrentPositionInSteps();
}

void Motor::updateData() {
  // target_steps = stepper.getT();
  position_steps = stepper.getCurrentPositionInSteps();
}

void Motor::processMovement() {
  // This needs to be called regularly for non-blocking movements to work
  stepper.processMovement();
}

bool Motor::isMovementComplete() {
  return stepper.motionComplete();
}

void Motor::emergencyStop() {
  updateData();
}