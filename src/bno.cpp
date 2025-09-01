#include <SPI.h>
#include <bno.h>

static constexpr uint16_t REPORT_INTERVAL_MS = 10; // 100 Hz

BNO::BNO() {
  isConnected    = false;
  sensorsEnabled = false;

  accel    = {0.0, 0.0, 0.0};
  gyro     = {0.0, 0.0, 0.0};
  rotation = {0.0, 0.0, 0.0};
}

BNO::~BNO() { disconnect(); }

bool BNO::connect(PinName       sda,
                  PinName       scl,
                  PinName       intPin,
                  PinName       rstPin,
                  unsigned long freq) {
  int_pin = intPin;
  rst_pin = rstPin;

  Serial1.println("[BNO] Connecting to IMU...");

  bno.enableDebugging(Serial1);

  Wire.setSDA(sda);
  Wire.setSCL(scl);
  // Wire.setClock(freq); // <-- crashes for some reason

  Wire.begin();

  if (!bno.begin(0x4B, Wire, int_pin, rst_pin)) {
    Serial1.println("[BNO] Failed to connect to BNO08x");
    return false;
  }

  Serial1.println("[BNO] Connected successfully");
  isConnected = true;

  init();
  return true;
}

void BNO::init() {
  enableSensors(); // calibrated accel + gyro, 100 Hz
  tare();
}

void BNO::tare() {
  Serial1.println("[BNO] Tare completed (not implemented ytet)");
}

void BNO::disconnect() {
  isConnected    = false;
  sensorsEnabled = false;
}

bool BNO::isReady() { return isConnected; }

void BNO::enableSensors() {
  const int MAX_RETRIES = 10;

  Serial1.print("[BNO] Enabling Rotation Vector @ ");
  Serial1.print(REPORT_INTERVAL_MS);
  Serial1.println(" ms...");

  for (int i = 0; i < MAX_RETRIES; i++) {
    static int successCount = 0;
    // Enable rotation vector (includes accelerometer, gyroscope, and
    // magnetometer fusion)
    if (bno.enableGameRotationVector(REPORT_INTERVAL_MS)) {
      Serial1.println("[BNO] Game Rotation Vector enabled");
      successCount++;
    }
    if (bno.enableAccelerometer(REPORT_INTERVAL_MS)) {
      Serial1.println("[BNO] Accelerometer enabled");
      successCount++;
    }
    if (bno.enableGyro(REPORT_INTERVAL_MS)) {
      Serial1.println("[BNO] Gyroscope enabled");
      successCount++;
    }

    if (successCount >= 3) {
      sensorsEnabled = true;
      break;
    }

    Serial1.println("[BNO] Retry...");
    delay(100);
  }

  Serial1.println(sensorsEnabled ? "[BNO] SUCCESS" : "[BNO] FAILED");

  delay(100); // let sensors settle
}

void BNO::updateSensorData() {
  // Drain all pending reports
  if (bno.getSensorEvent()) {
    uint8_t id = bno.getSensorEventID();

    // Calibrated accelerometer → m/s^2
    if (id == SENSOR_REPORTID_ACCELEROMETER ||
        id == SENSOR_REPORTID_RAW_ACCELEROMETER) {
      accel.x = bno.getAccelX();
      accel.y = bno.getAccelY();
      accel.z = bno.getAccelZ();
    }
    // Calibrated gyro → radians/s
    else if (id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED ||
             id == SENSOR_REPORTID_RAW_GYROSCOPE) {
      gyro.x = bno.getGyroX();
      gyro.y = bno.getGyroY();
      gyro.z = bno.getGyroZ();
    }
    if (id == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      rotation.setRollRads(bno.getRoll());
      rotation.setPitchRads(bno.getPitch());
      rotation.setYawRads(bno.getYaw());
    }
  }
}

void BNO::update() {
  if (!isReady()) {
    Serial1.println("[BNO] Not ready");
    return;
  }

  if (bno.wasReset() && millis() > 2000) {
    Serial1.println("[BNO] Reset detected, re-enabling sensors");
    enableSensors();
    return;
  }

  updateSensorData();
}

void BNO::setRotation(Rotation *rotation) { rotation = rotation; }

Rotation *BNO::getRotation() { return &rotation; }
AccelData BNO::getAccelData() { return accel; }
GyroData  BNO::getGyroData() { return gyro; }