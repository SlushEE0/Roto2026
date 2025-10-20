#pragma once

#include <SPI.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <utils.h>

#include <config.h>

class BNO {
    private:
  int cs_pin;
  int int_pin;
  int rst_pin;

  bool isConnected;
  bool sensorsEnabled;

  AccelData accel;
  GyroData  gyro;
  Rotation  rotation;

  BNO08x bno;

    public:
  BNO() {
    isConnected    = false;
    sensorsEnabled = false;

    accel    = {0.0, 0.0, 0.0};
    gyro     = {0.0, 0.0, 0.0};
    rotation = {0.0, 0.0, 0.0};
  }

  ~BNO() { disconnect(); }

  void setRotation(Rotation *rotation) { rotation = rotation; }

  Rotation *getRotation() { return &rotation; }
  AccelData getAccelData() { return accel; }
  GyroData  getGyroData() { return gyro; }
  bool      isReady() { return isConnected; }

  bool connect(PinName       sda,
               PinName       scl,
               PinName       intPin,
               PinName       rstPin,
               unsigned long freq = 350000) {
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

  void disconnect() {
    isConnected    = false;
    sensorsEnabled = false;
  }

  void update() {
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

  void tare() {
    Serial1.println("[BNO] Taring...");
    bno.tareNow(true, SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);
    bno.saveTare();
    Serial1.println("[BNO] Tare completed");
  }

  void calibrate() {
    Serial1.println("[BNO] Calibrating...");
    Serial1.println("[BNO] Gyro Calibration (do not move device)");
    bno.setCalibrationConfig(SH2_CAL_GYRO);
    delay(2500);
    bno.setCalibrationConfig(SH2_CAL_ACCEL);
    Serial1.println("[BNO] Accel Calibration (move to 6 unique positions)");
    for (int i = 0; i < 6; i++) {
      Serial1.print("[BNO] Position ");
      Serial1.print(i + 1);
      Serial1.println("/6");

      delay(1100);
    }
    Serial1.println("[BNO] Finished, saving...");

    bno.setCalibrationConfig(0);
    bno.saveCalibration();

    Serial1.printf("Accel Accuracy: %d, Gyro Accuracy: %d, Mag Accuracy: %d\n",
                   bno.getAccelAccuracy(),
                   bno.getGyroAccuracy(),
                   bno.getMagAccuracy());

    Serial1.println("[BNO] Calibration completed");
  }

    protected:
  void init() {
    enableSensors(); // calibrated accel + gyro, 100 Hz
    tare();
    // calibrate();
  }

  void enableSensors() {
    const int MAX_RETRIES = 10;

    Serial1.print("[BNO] Enabling Rotation Vector @ ");
    Serial1.print(BNO_REPORT_INTERVAL_MS);
    Serial1.println(" ms...");

    for (int i = 0; i < MAX_RETRIES; i++) {
      static int successCount = 0;
      // Enable rotation vector (includes accelerometer, gyroscope, and
      // magnetometer fusion)
      if (bno.enableGameRotationVector(BNO_REPORT_INTERVAL_MS)) {
        Serial1.println("[BNO] Game Rotation Vector enabled");
        successCount++;
      }
      if (bno.enableAccelerometer(BNO_REPORT_INTERVAL_MS)) {
        Serial1.println("[BNO] Accelerometer enabled");
        successCount++;
      }
      if (bno.enableGyro(BNO_REPORT_INTERVAL_MS)) {
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

  void updateSensorData() {
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
};
