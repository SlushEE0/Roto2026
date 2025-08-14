#include <bno.h>

static constexpr uint16_t REPORT_INTERVAL_MS = 10; // 100 Hz
static constexpr float G_TO_MS2 = 9.80665f;

BNO::BNO() {
  isConnected = false;
  sensorsEnabled = false;

  accel = { 0.0f, 0.0f, 0.0f };
  gyro = { 0.0f, 0.0f, 0.0f };
  rotationQuat = { 1.0f, 0.0f, 0.0f, 0.0f };
  rotationEuler = { 0.0f, 0.0f, 0.0f };
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

  init();
  return true;
}

void BNO::disconnect() {
  isConnected = false;
  sensorsEnabled = false;
}

bool BNO::isReady() {
  return isConnected && sensorsEnabled;
}

void BNO::init() {
  enableSensors(); // calibrated accel + gyro, 100 Hz
  tare();
}

void BNO::enableSensors() {
  const int MAX_RETRIES = 10;
  bool successAccel = false, successGyro = false;

  Serial.print("[BNO] Enabling calibrated Accel+Gyro @ ");
  Serial.print(REPORT_INTERVAL_MS);
  Serial.println(" ms...");

  for (int i = 0; i < MAX_RETRIES; i++) {
    // Calibrated reports (NOT raw)
    if (imu.enableAccelerometer(REPORT_INTERVAL_MS))
      successAccel = true;
    if (imu.enableGyro(REPORT_INTERVAL_MS))
      successGyro = true;
    if (successAccel && successGyro)
      break;
    delay(100);
  }

  sensorsEnabled = successAccel && successGyro;
  Serial.println(sensorsEnabled ? "  SUCCESS" : "  FAILED");

  filter.begin(1000.0f / REPORT_INTERVAL_MS);

  delay(100); // let sensors settle
}

void BNO::updateSensorData() {
  // Drain all pending reports
  if (imu.getSensorEvent()) {
    uint8_t id = imu.getSensorEventID();

    // Calibrated accelerometer → m/s^2
    if (id == SENSOR_REPORTID_RAW_ACCELEROMETER ||
        id == SENSOR_REPORTID_RAW_ACCELEROMETER) {
      float ax_ms2 = imu.getAccelX();
      float ay_ms2 = imu.getAccelY();
      float az_ms2 = imu.getAccelZ();
      // Convert to g for Madgwick
      accel.x = ax_ms2 / G_TO_MS2;
      accel.y = ay_ms2 / G_TO_MS2;
      accel.z = az_ms2 / G_TO_MS2;
    }
    // Calibrated gyro → radians/s
    else if (id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED ||
             id == SENSOR_REPORTID_RAW_GYROSCOPE) {
      gyro.x = imu.getGyroX() * (180.0f / PI);
      gyro.y = imu.getGyroY() * (180.0f / PI);
      gyro.z = imu.getGyroZ() * (180.0f / PI);
    }
  }
}

bool BNO::update() {
  if (!isReady()) {
    Serial.println("[BNO] Not ready - isConnected: " + String(isConnected) +
                   ", sensorsEnabled: " + String(sensorsEnabled));
    return false;
  }

  if (imu.wasReset() && millis() > 2000) {
    Serial.println("[BNO] Reset detected, re-enabling sensors");
    enableSensors();
    return false;
  }

  updateSensorData();

  // Feed Madgwick: gyro in rad/s, accel in g
  filter.updateIMU(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);

  RotationEuler filtered = {
    .x = filter.getRoll(), // degrees
    .y = filter.getPitch(), // degrees
    .z = filter.getYaw() // degrees
  };
  setRotation(filtered);
  return true;
}

void BNO::setRotation(RotationEuler rotation) {
  rotationEuler = rotation;
  rotationQuat = cnv_eulerToQuat(&rotation);
}

void BNO::setRotation(RotationQuat rotation) { /* optional */ }

RotationEuler BNO::getRotationEuler() {
  return rotationEuler;
}

RotationQuat BNO::getRotationQuat() {
  return rotationQuat;
}

AccelData BNO::getAccelData() {
  return accel;
}

GyroData BNO::getGyroData() {
  return gyro;
}

void BNO::tare() {
  // Reset orientation estimate. (Re-seeds internal timing too.)
  filter.begin(1000.0f / REPORT_INTERVAL_MS);
  Serial.println("[BNO] Tare completed");
}
