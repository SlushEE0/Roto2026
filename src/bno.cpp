#include "bno.h"

BNO::BNO() {
  isConnected = false;
  sensorsEnabled = false;

  rotationQuat = { 1.0f, 0.0f, 0.0f, 0.0f }; // w, i, j, k
  rotationEuler = { 0.0f, 0.0f, 0.0f }; // x, y, z
}

BNO::~BNO() {
  disconnect();
}

bool BNO::connect(int csPin, int intPin, int rstPin, unsigned long spiSpeed) {
  cs_pin = csPin;
  int_pin = intPin;
  rst_pin = rstPin;

  Serial.println("[BNO] Connecting to IMU...");

  if (!imu.beginSPI(cs_pin, int_pin, rst_pin, spiSpeed, SPI)) {
    Serial.println("[BNO] Failed to connect to BNO08x");
    return false;
  }

  isConnected = true;
  Serial.println("[BNO] Connected successfully");

  enableReports();
  tare();

  return true;
}

void BNO::enableReports() {
  const int MAX_RETRIES = 10;

  // Enable Game Rotation Vector at 100Hz
  Serial.print("[BNO] Enabling Game Rotation Vector...");
  bool success = false;
  for (int i = 0; i < MAX_RETRIES; i++) {
    if (imu.enableGameRotationVector(10)) { // 10ms = 100Hz
      Serial.println(" SUCCESS");
      success = true;
      break;
    }
    Serial.print(".");
    delay(100);
  }

  sensorsEnabled = success;
  delay(100); // Allow sensors to stabilize
}

void BNO::disconnect() {
  isConnected = false;
  sensorsEnabled = false;
}

bool BNO::isReady() {
  return isConnected && sensorsEnabled;
}

void BNO::updateSensorData() {
  if (imu.getSensorEvent()) {
    // Get Game Rotation Vector data (quaternion without magnetometer)
    if (imu.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      RotationQuat quat;

      quat.i = imu.getQuatI();
      quat.j = imu.getQuatJ();
      quat.k = imu.getQuatK();
      quat.w = imu.getQuatReal();

      RotationEuler euler = cnv_quatToEuler(quat);

      rotationQuat = quat;
      rotationEuler = euler;
    }
  }
}

bool BNO::update() {
  if (!isReady()) {
    Serial.println("[BNO] Not ready - isConnected: " + String(isConnected) +
                   ", sensorsEnabled: " + String(sensorsEnabled));
    return false;
  }

  // Check for IMU reset
  if (imu.wasReset()) {
    Serial.println("[BNO] Reset detected, re-enabling reports");
    enableReports();
    return false;
  }

  updateSensorData();

  return true;
}

RotationEuler BNO::getRotation() {
  return rotationEuler;
}

RotationQuat BNO::getRotationQuat() {
  return rotationQuat;
}

void BNO::tare() {
  if (isReady()) {
    imu.tareNow();

    Serial.println("[BNO] Tare completed");
  }
}
