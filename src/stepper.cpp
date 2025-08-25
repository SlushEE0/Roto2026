// #pragma once
// #include <FastAccelStepper.h>

// // Define step and direction pins for MKS Gen L v1.0
// #define X_STEP_PIN 54   // X step
// #define X_DIR_PIN 55    // X direction
// #define Z_STEP_PIN 46   // Z step
// #define Z_DIR_PIN 48    // Z direction

// class StepperMotor {
// public:
//     StepperMotor(FastAccelStepperEngine& engine, uint8_t stepPin, uint8_t dirPin)
//         : stepPin(stepPin), dirPin(dirPin), engine(engine), stepper(nullptr), currentPosition(0) {}

//     void init() {
//         engine.init();
//         stepper = engine.stepperConnectToPin(stepPin);
//         if (stepper) {
//             stepper->setDirectionPin(dirPin);
//             stepper->setEnablePin(0xFF); // disable enable pin if not used
//             stepper->setAutoEnable(false);
//             stepper->setSpeedAcceleration(10000, 10000); // default speed/accel
//         }
//     }

//     void moveTo(long targetPosition) {
//         if (!stepper) return;
//         stepper->moveTo(targetPosition);
//         currentPosition = targetPosition;
//     }

//     void move(long relativePosition) {
//         if (!stepper) return;
//         long target = currentPosition + relativePosition;
//         moveTo(target);
//     }

//     long getSteps() {
//         if (!stepper) return 0;
//         return stepper->getCurrentPosition();
//     }

//     void setSpeedAcceleration(uint32_t speed, uint32_t accel) {
//         if (!stepper) return;
//         stepper->setSpeedAcceleration(speed, accel);
//     }

// private:
//     uint8_t stepPin, dirPin;
//     FastAccelStepperEngine& engine;
//     FastAccelStepper* stepper;
//     long currentPosition;
// };

// // Global engine
// FastAccelStepperEngine engine = FastAccelStepperEngine();

// // Define your steppers
// StepperMotor stepperX(engine, X_STEP_PIN, X_DIR_PIN);
// StepperMotor stepperZ(engine, Z_STEP_PIN, Z_DIR_PIN);
