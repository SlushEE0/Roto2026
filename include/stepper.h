#pragma once
#include <FastAccelStepper.h>

#include <config.h>

class StepperMotor {
public:
  StepperMotor(FastAccelStepperEngine &engine,
               uint8_t stepPin,
               uint8_t dirPin,
               uint8_t enPin) :
    stepPin(stepPin),
    dirPin(dirPin),
    engine(engine),
    enPin(enPin),
    stepper(nullptr),
    currentPosition(0) {
    init();
  }

  void init() {
    stepper = engine.stepperConnectToPin(stepPin);
    if (stepper) {
      stepper->setDirectionPin(dirPin);
      stepper->setEnablePin(enPin); // disable enable pin if not used
      stepper->setAutoEnable(true);
      setSpeedAccel(10000, 5000); // default speed/accel
    }
  }

  void moveTo(long targetPosition) {
    if (!stepper)
      return;
    stepper->moveTo(targetPosition);
    currentPosition = targetPosition;
  }

  void move(long relativePosition) {
    if (!stepper)
      return;
    long target = currentPosition + relativePosition;
    moveTo(target);
  }

  long distanceToGo() {
    return abs(stepper->targetPos() - stepper->getCurrentPosition());
  }

  long getCurrentPosition() {
    if (!stepper)
      return 0;
    return stepper->getCurrentPosition();
  }

  void setSpeed(uint32_t speed) { stepper->setSpeedInTicks(speed); }

  void setAccel(uint32_t accel) { stepper->setAcceleration(accel); }

  void setSpeedAccel(uint32_t speed, uint32_t accel) {
    if (!stepper)
      return;
    setSpeed(speed);
    setAccel(accel);
  }

private:
  uint8_t stepPin, dirPin, enPin;
  FastAccelStepperEngine &engine;
  FastAccelStepper *stepper;
  long currentPosition;
};
