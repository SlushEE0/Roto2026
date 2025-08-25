#include <Arduino.h>
#include <ArduinoLog.h>
#include <stepper.h>
#include <bno.h>
#include <utils.h>

#include <config.h>

// SPI pins for BNO08x
const int CS_PIN = 53; // CS (Chip Select)
const int INT_PIN = 33; // Interrupt pin
const int RST_PIN = 31; // Reset pin

// Global engine
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// StepperMotor stepper_r(engine, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
// StepperMotor stepper_l(engine, Z_STEP_PIN, Z_DIR_PIN, Y_ENABLE_PIN);
BNO imu;

void printPrefix(Print *_logOutput, int logLevel) {
  _logOutput->print("");
};

void setup() {
  Serial.begin(256000);
  while (!Serial)
    delay(10);

  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.setPrefix(printPrefix);

  Log.noticeln("[INIT] Roto2026 Starting");
  Log.noticeln("=========================");

  // Initialize IMU
  while (!imu.connect(CS_PIN, INT_PIN, RST_PIN)) {
    Log.infoln("[INIT] Retrying BNO connection...");
    delay(2000);
  }

  Log.infoln("[INIT] All systems ready");
  delay(1000);

  engine.init();
  stepper = engine.stepperConnectToPin(X_STEP_PIN);
  stepper->setDirectionPin(X_DIR_PIN);
  stepper->setEnablePin(X_ENABLE_PIN);
  stepper->setAutoEnable(true);

  stepper->setSpeedInHz(500); // 500 steps/s
  stepper->setAcceleration(100); // 100 steps/s²

  // stepper_r.setSpeed(5000);
  // stepper_l.setSpeed(5000);

  // stepper_r.moveTo(10000);
  // stepper_l.moveTo(-5000);
  delay(1000);
  stepper->moveTo(10000);
}

void loop() {
  // Serial.print(">stepper_r_steps:");
  // Serial.println(stepper_r.getCurrentPosition());

  // if (stepper_r.distanceToGo() == 0) {
  //   stepper_r.moveTo(-stepper_r.getCurrentPosition());
  // }

  if (stepper->isStopping()) {
    stepper->moveTo(-stepper->targetPos());
  }
}

// void loop() {
//   long prevUS = micros();
//   imu.update();
//   RotationEuler rotation = imu.getRotationEuler();
//   long deltaUS = micros() - prevUS;

//   Serial.print("[DATA]: X: ");
//   Serial.print(rotation.x);
//   Serial.print(", Y: ");
//   Serial.print(rotation.y);
//   Serial.print(", Z: ");
//   Serial.print(rotation.z);
//   Serial.print(", US: ");
//   Serial.println(deltaUS);

//   delay(100);
// }