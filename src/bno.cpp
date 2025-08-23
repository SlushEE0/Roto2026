#include <bno.h>

static constexpr uint16_t REPORT_INTERVAL_MS = 10; // 100 Hz

BNO::BNO() {
  isConnected = false;
  sensorsEnabled = false;

  accel = { 0.0, 0.0, 0.0 };
  gyro = { 0.0, 0.0, 0.0 };
  rotation = { 0.0, 0.0, 0.0 };
}

BNO::~BNO() {
  disconnect();
}

bool BNO::connect(int csPin, int intPin, int rstPin, unsigned long spiSpeed) {
  cs_pin = csPin;
  int_pin = intPin;
  rst_pin = rstPin;

  Serial.println("[BNO] Connecting to IMU...");

  if (!bno.beginSPI(cs_pin, int_pin, rst_pin, spiSpeed, SPI)) {
    Serial.println("[BNO] Failed to connect to BNO08x");
    return false;
  }
  isConnected = true;
  Serial.println("[BNO] Connected successfully");

  init();
  return true;
}

void BNO::init() {
  enableSensors(); // calibrated accel + gyro, 100 Hz
  tare();
}

void BNO::tare() {
  Serial.println("[BNO] Tare completed (not implemented ytet)");
}

void BNO::disconnect() {
  isConnected = false;
  sensorsEnabled = false;
}

bool BNO::isReady() {
  return isConnected && sensorsEnabled;
}

void BNO::enableSensors() {
  const int MAX_RETRIES = 10;
  bool successAccel = false, successGyro = false;

  Serial.print("[BNO] Enabling calibrated Accel+Gyro @ ");
  Serial.print(REPORT_INTERVAL_MS);
  Serial.println(" ms...");

  for (int i = 0; i < MAX_RETRIES; i++) {
    // Calibrated reports (NOT raw)
    if (bno.enableRotationVector(REPORT_INTERVAL_MS))
      successAccel = true;

    if (successAccel && successGyro)
      break;
    delay(100);
  }

  sensorsEnabled = successAccel && successGyro;
  Serial.println(sensorsEnabled ? "  SUCCESS" : "  FAILED");

  delay(100); // let sensors settle
}

void BNO::updateSensorData() {
  // Drain all pending reports
  if (bno.getSensorEvent()) {
    uint8_t id = bno.getSensorEventID();

    // // Calibrated accelerometer → m/s^2
    // if (id == SENSOR_REPORTID_ACCELEROMETER ||
    //     id == SENSOR_REPORTID_RAW_ACCELEROMETER) {
    //   accel.x = bno.getAccelX();
    //   accel.y = bno.getAccelY();
    //   accel.z = bno.getAccelZ();
    // }
    // // Calibrated gyro → radians/s
    // else if (id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED ||
    //          id == SENSOR_REPORTID_RAW_GYROSCOPE) {
    //   gyro.x = bno.getGyroX();
    //   gyro.y = bno.getGyroY();
    //   gyro.z = bno.getGyroZ();
    // }
    if (id == SENSOR_REPORTID_ROTATION_VECTOR) {
      rotation.setRollRads(bno.getRoll());
      rotation.setPitchRads(bno.getPitch());
      rotation.setYawRads(bno.getYaw());
    }
  }
}

void BNO::update() {
  if (!isReady()) {
    Serial.println("[BNO] Not ready - isConnected: " + String(isConnected) +
                   ", sensorsEnabled: " + String(sensorsEnabled));
    return;
  }

  if (bno.wasReset() && millis() > 2000) {
    Serial.println("[BNO] Reset detected, re-enabling sensors");
    enableSensors();
    return;
  }

  updateSensorData();
}

void BNO::setRotation(Rotation *rotation) {
  rotation = rotation;
}

Rotation *BNO::getRotation() {
  return &rotation;
}

AccelData BNO::getAccelData() {
  return accel;
}

GyroData BNO::getGyroData() {
  return gyro;
}