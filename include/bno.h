#pragma once

#include <SPI.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <MadgwickAHRS.h>
#include <stepper.h>
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
  GyroData gyro;
  Rotation rotation;
  float yawOffset;
  bool yawOffsetInitialized;

  BNO08x bno;

  // Madgwick AHRS filter – fuses raw accel + gyro into a smooth
  // orientation estimate, rejecting the sporadic BNO Game Rotation
  // Vector spikes that cause turn jitter.
  Madgwick _madgwick;
  bool _madgwickReady; // true once we've received both accel & gyro
  bool _gotAccelThisCycle;
  bool _gotGyroThisCycle;
  uint32_t _lastMadgwickUs; // for accurate dt

public:
  BNO() {
    isConnected = false;
    sensorsEnabled = false;

    accel = {0.0f, 0.0f, 0.0f};
    gyro = {0.0f, 0.0f, 0.0f};
    rotation = {0.0f, 0.0f, 0.0f};
    yawOffsetInitialized = false;
    yawOffset = 0.0f;

    _madgwickReady = false;
    _gotAccelThisCycle = false;
    _gotGyroThisCycle = false;
    _lastMadgwickUs = 0;
  }

  ~BNO() {
    disconnect();
  }

  void setRotation(Rotation* rotation) {
    rotation = rotation;
  }

  Rotation* getRotation() {
    return &rotation;
  }
  AccelData getAccelData() {
    return accel;
  }
  GyroData getGyroData() {
    return gyro;
  }
  bool isReady() {
    return isConnected;
  }

  bool connect(PinName sda,
               PinName scl,
               PinName intPin,
               PinName rstPin,
               unsigned long freq = 350000) {
    int_pin = intPin;
    rst_pin = rstPin;

    Serial1.println("[BNO] Connecting to IMU...");

    // bno.enableDebugging(Serial1);  // disabled – floods Serial1 and stalls
    // I2C

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
    isConnected = false;
    sensorsEnabled = false;
  }

  void update() {
    if (!isReady()) {
      Serial1.println("[BNO] Not ready");
      return;
    }

    if (bno.wasReset() && millis() > 2000) {
      // Debounce: don't re-init more than once per second
      static unsigned long lastResetHandled = 0;
      if (millis() - lastResetHandled > 1000) {
        lastResetHandled = millis();
        Serial1.println("[BNO] Reset detected, re-enabling sensors");
        enableSensors();
      }
      return;
    }

    updateSensorData();
  }

  void tare() {
    Serial1.println("[BNO] Taring...");
    bno.tareNow(true, SH2_TARE_BASIS_GAMING_ROTATION_VECTOR);
    bno.saveTare();
    yawOffsetInitialized = false; // next update will re-zero yaw
    Serial1.println("[BNO] Tare completed");
  }

  // ── Calibration for Game Rotation Vector ──────────────────────────────
  // Follows the official BNO080/085 Sensor Calibration Procedure
  // (CEVA/Hillcrest document 1000-4044 Rev 1.3).
  //
  // For Game Rotation Vector (gyro + accel, NO magnetometer):
  //   1. Enable dynamic calibration for accel + gyro simultaneously
  //   2. Enable Game Rotation Vector output (already done in enableSensors)
  //   3. Gyro calibration: hold device stationary for ~3 seconds
  //   4. Accel calibration: place device in 4-6 unique orientations,
  //      hold each for ~1 second ("cube method")
  //   5. Save DCD to flash
  //
  // Steppers run during calibration so the BNO learns the real
  // vibration / EMI environment it will operate in.
  //
  // This method BLOCKS.  Call once from setup(), then comment out
  // once calibration is saved to the BNO's flash (it persists across
  // power cycles via DCD auto-save).
  //
  // @param accelOrientations  Number of distinct orientations to prompt
  //                           the user for during accel calibration (4-6).
  //                           For a flat robot, 1 is usually enough.
  void calibrateForGameRotationVector(Stepper& leftStepper,
                                      Stepper& rightStepper,
                                      float driveSpeed = 5000.0f,
                                      uint8_t accelOrientations = 1,
                                      uint32_t logIntervalMs = 500) {
    if (!isReady()) {
      Serial1.println("[CAL] ERROR: IMU not connected");
      return;
    }

    Serial1.println(
      "[CAL] ====================================================");
    Serial1.println("[CAL]  BNO085 Game Rotation Vector Calibration");
    Serial1.println("[CAL]  Procedure per CEVA/Hillcrest 1000-4044 Rev 1.3");
    Serial1.println(
      "[CAL] ====================================================");
    Serial1.println("[CAL] Uses gyro + accel only (no magnetometer)");
    Serial1.print("[CAL] Stepper speed: ");
    Serial1.print(driveSpeed, 0);
    Serial1.println(" stp/s");
    Serial1.print("[CAL] Accel orientations requested: ");
    Serial1.println(accelOrientations);
    Serial1.println("[CAL]");
    Serial1.println("[CAL] Starting in 2 seconds...");
    Serial1.flush();
    delay(2000);

    // ═══════════════════════════════════════════════════════════════════════
    // Step 1: Enable dynamic calibration for accel + gyro simultaneously
    //         Per the document: "Enable dynamic calibration for
    //         accelerometer, gyroscope [, and magnetometer]"
    //         We skip mag since Game Rotation Vector doesn't use it.
    // ═══════════════════════════════════════════════════════════════════════
    Serial1.println("[CAL] Enabling dynamic calibration (accel + gyro)...");
    Serial1.flush();
    bno.setCalibrationConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO);
    drainResetFlag();
    Serial1.println("[CAL] Dynamic calibration enabled");
    Serial1.flush();

    // Ensure Game Rotation Vector is enabled (should already be)
    bno.enableGameRotationVector(BNO_REPORT_INTERVAL_MS);
    delay(50);

    // Start steppers so the BNO experiences real operating vibration
    leftStepper.commandVelocity(driveSpeed);
    rightStepper.commandVelocity(driveSpeed);
    Serial1.println("[CAL] Steppers running");
    Serial1.flush();

    uint8_t bestGyroAcc = 0;
    uint8_t bestAccelAcc = 0;

    // ═══════════════════════════════════════════════════════════════════════
    // Step 2: Gyro calibration — hold device STATIONARY for ~3 seconds
    //         Per the document: "Set the device down on a stationary
    //         surface for ~2-3 seconds to calibrate the gyroscope"
    // ═══════════════════════════════════════════════════════════════════════
    Serial1.println("[CAL]");
    Serial1.println("[CAL] ── Phase 1: GYRO CALIBRATION ──");
    Serial1.println("[CAL] Keep robot COMPLETELY STILL for 5 seconds");
    Serial1.println("[CAL] (wheels off ground or locked)");
    Serial1.flush();

    const uint32_t GYRO_PHASE_MS = 5000; // 5s (doc says 2-3, extra margin)
    uint32_t phaseStart = millis();
    uint32_t phaseEnd = phaseStart + GYRO_PHASE_MS;
    uint32_t lastLog = 0;
    bool gyroCalibrated = false;

    while (millis() < phaseEnd) {
      update();

      uint8_t ga = bno.getGyroAccuracy();
      uint8_t aa = bno.getAccelAccuracy();
      if (ga > bestGyroAcc)
        bestGyroAcc = ga;
      if (aa > bestAccelAcc)
        bestAccelAcc = aa;

      uint32_t now = millis();
      if (now - lastLog >= logIntervalMs) {
        lastLog = now;
        float elapsed = (float)(now - phaseStart) / 1000.0f;
        Serial1.print("[CAL] t=");
        Serial1.print(elapsed, 1);
        Serial1.print("s  GyroAcc=");
        Serial1.print(ga);
        Serial1.print("/3  AccelAcc=");
        Serial1.print(aa);
        Serial1.print("/3  Gyro(rad/s): ");
        Serial1.print(gyro.x, 4);
        Serial1.print(",");
        Serial1.print(gyro.y, 4);
        Serial1.print(",");
        Serial1.println(gyro.z, 4);
      }

      if (ga >= 3 && !gyroCalibrated) {
        Serial1.println("[CAL] >>> Gyro accuracy 3/3 reached! <<<");
        gyroCalibrated = true;
      }

      delay(10);
    }

    // Intermediate save — preserve gyro calibration
    Serial1.println("[CAL] Saving gyro calibration to flash (DCD)...");
    Serial1.flush();
    bno.saveCalibration();
    delay(200);
    Serial1.print("[CAL] Gyro phase complete.  Best accuracy: ");
    Serial1.print(bestGyroAcc);
    Serial1.println("/3");
    Serial1.flush();

    // ═══════════════════════════════════════════════════════════════════════
    // Step 3: Accel calibration — "cube method"
    //         Per the document: "The accelerometer will be calibrated
    //         after the device is moved into 4-6 unique orientations
    //         and held in each orientation for ~1 second."
    //
    //         For a flat-running robot we only do 1 orientation by
    //         default (normal operating position), which is usually
    //         enough for accel accuracy 2-3.  Set accelOrientations
    //         higher if better calibration is needed.
    // ═══════════════════════════════════════════════════════════════════════
    Serial1.println("[CAL]");
    Serial1.println("[CAL] ── Phase 2: ACCEL CALIBRATION (cube method) ──");
    Serial1.print("[CAL] Will prompt for ");
    Serial1.print(accelOrientations);
    Serial1.println(" orientation(s)");
    Serial1.println("[CAL] Hold each orientation STILL for ~3 seconds");
    Serial1.flush();

    // Switch steppers to opposite direction for varied vibration
    leftStepper.commandVelocity(-driveSpeed);
    rightStepper.commandVelocity(driveSpeed);

    for (uint8_t orient = 1; orient <= accelOrientations; orient++) {
      Serial1.print("[CAL] --- Orientation ");
      Serial1.print(orient);
      Serial1.print("/");
      Serial1.print(accelOrientations);
      Serial1.println(" ---");

      if (accelOrientations > 1) {
        // Give user time to reposition the robot
        Serial1.println("[CAL] Reposition robot now... holding 3s in:");
        for (int countdown = 3; countdown > 0; countdown--) {
          Serial1.print("[CAL]   ");
          Serial1.print(countdown);
          Serial1.println("...");
          Serial1.flush();
          delay(1000);
        }
      }

      // Hold still for 3 seconds (doc says 1s per orientation, extra margin)
      Serial1.println("[CAL] Holding still...");
      Serial1.flush();

      const uint32_t HOLD_MS = 3000;
      uint32_t holdStart = millis();
      uint32_t holdEnd = holdStart + HOLD_MS;
      lastLog = 0;

      while (millis() < holdEnd) {
        update();

        uint8_t ga = bno.getGyroAccuracy();
        uint8_t aa = bno.getAccelAccuracy();
        if (ga > bestGyroAcc)
          bestGyroAcc = ga;
        if (aa > bestAccelAcc)
          bestAccelAcc = aa;

        uint32_t now = millis();
        if (now - lastLog >= logIntervalMs) {
          lastLog = now;
          float elapsed = (float)(now - holdStart) / 1000.0f;
          Serial1.print("[CAL]   t=");
          Serial1.print(elapsed, 1);
          Serial1.print("s  AccelAcc=");
          Serial1.print(aa);
          Serial1.print("/3  GyroAcc=");
          Serial1.print(ga);
          Serial1.println("/3");
        }

        delay(10);
      }

      // Save after each orientation
      bno.saveCalibration();
      Serial1.print("[CAL] Orientation ");
      Serial1.print(orient);
      Serial1.print(" done.  AccelAcc=");
      Serial1.print(bno.getAccelAccuracy());
      Serial1.println("/3");
      Serial1.flush();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Step 4: Stop steppers and quiet settling phase
    // ═══════════════════════════════════════════════════════════════════════
    leftStepper.commandVelocity(0);
    rightStepper.commandVelocity(0);
    Serial1.println("[CAL]");
    Serial1.println("[CAL] Steppers stopped.  Quiet settling (2s)...");
    Serial1.flush();

    uint32_t quietEnd = millis() + 2000;
    while (millis() < quietEnd) {
      update();
      delay(10);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Step 5: Stop dynamic calibration & final DCD save
    //         Per the document: "run the Save DCD Now command, which
    //         will save this calibration data into flash"
    // ═══════════════════════════════════════════════════════════════════════
    bno.setCalibrationConfig(0); // stop dynamic calibration
    delay(50);
    bno.saveCalibration();
    delay(200);

    // ═══════════════════════════════════════════════════════════════════════
    // Step 6: Report
    // ═══════════════════════════════════════════════════════════════════════
    // Pull one more sensor reading to get final accuracy
    update();
    uint8_t finalGyro = bno.getGyroAccuracy();
    uint8_t finalAccel = bno.getAccelAccuracy();

    Serial1.println(
      "[CAL] ====================================================");
    Serial1.println("[CAL]  CALIBRATION RESULTS");
    Serial1.println(
      "[CAL] ====================================================");
    Serial1.print("[CAL] Gyro  accuracy: ");
    Serial1.print(finalGyro);
    Serial1.print("/3  (best seen: ");
    Serial1.print(bestGyroAcc);
    Serial1.println("/3)");
    Serial1.print("[CAL] Accel accuracy: ");
    Serial1.print(finalAccel);
    Serial1.print("/3  (best seen: ");
    Serial1.print(bestAccelAcc);
    Serial1.println("/3)");

    if (finalGyro >= 2 && finalAccel >= 2) {
      Serial1.println("[CAL] PASS — calibration saved to BNO flash (DCD)");
      Serial1.println("[CAL] You can now comment out the calibration call.");
    } else {
      Serial1.println("[CAL] WARN — accuracy low, consider re-running:");
      if (finalGyro < 2)
        Serial1.println("[CAL]   Gyro: ensure robot is completely still");
      if (finalAccel < 2)
        Serial1.println("[CAL]   Accel: try more orientations (cube method)");
    }
    Serial1.println(
      "[CAL] ====================================================");
    Serial1.flush();
  }

protected:
  // Drain any pending reset flag from the BNO after a config change.
  // Without this, the next update() call sees wasReset()==true and
  // re-runs enableSensors() in a loop, starving the calibration polling.
  void drainResetFlag() {
    delay(100);     // let the BNO process the config command
    bno.wasReset(); // read & clear the reset flag
    delay(50);
    // If it was truly reset, re-enable our sensor reports once.
    if (bno.wasReset()) {
      Serial1.println("[CAL] BNO reset detected, re-enabling sensors");
      enableSensors();
      bno.wasReset(); // clear flag again
    }
  }

  void init() {
    enableSensors(); // calibrated accel + gyro, 100 Hz

    // Madgwick filter sample rate matches BNO report rate
    float sampleHz = 1000.0f / (float)BNO_REPORT_INTERVAL_MS;
    _madgwick.begin(sampleHz);
    _madgwickReady = false;
    _gotAccelThisCycle = false;
    _gotGyroThisCycle = false;
    _lastMadgwickUs = micros();

    tare();
    // calibrate();
  }

  void enableSensors() {
    const int MAX_RETRIES = 10;

    Serial1.print("[BNO] Enabling sensors (Game RV + Accel + Gyro, no mag) @ ");
    Serial1.print(BNO_REPORT_INTERVAL_MS);
    Serial1.println(" ms...");

    for (int i = 0; i < MAX_RETRIES; i++) {
      int successCount = 0;

      // Game Rotation Vector = gyro + accel fusion (NO magnetometer).
      // This is immune to the magnetic field from stepper drivers.
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
    yawOffsetInitialized = false;
  }

  void updateSensorData() {
    // Drain pending reports so readings stay fresh, but cap iterations
    // to avoid blocking loop() when the BNO queues reports faster
    // than we consume them (3 sensors × 100 Hz).
    // 10 iterations is enough to stay current without starving serial/loop.
    for (int evtCount = 0; evtCount < 10 && bno.getSensorEvent(); ++evtCount) {
      uint8_t id = bno.getSensorEventID();

      // Calibrated accelerometer → m/s²
      if (id == SENSOR_REPORTID_ACCELEROMETER ||
          id == SENSOR_REPORTID_RAW_ACCELEROMETER) {
        accel.x = (float)bno.getAccelX();
        accel.y = (float)bno.getAccelY();
        accel.z = (float)bno.getAccelZ();
        _gotAccelThisCycle = true;
      }
      // Calibrated gyro → radians/s
      else if (id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED ||
               id == SENSOR_REPORTID_RAW_GYROSCOPE) {
        gyro.x = (float)bno.getGyroX();
        gyro.y = (float)bno.getGyroY();
        gyro.z = (float)bno.getGyroZ();
        _gotGyroThisCycle = true;
      }
      // We still read Game Rotation Vector (keeps BNO calibration alive)
      // but we no longer use it for heading.
      // if (id == SENSOR_REPORTID_GAME_ROTATION_VECTOR) { /* ignored */ }
    }

    // Once we have both fresh accel and gyro, run the Madgwick filter
    if (_gotAccelThisCycle && _gotGyroThisCycle) {
      _gotAccelThisCycle = false;
      _gotGyroThisCycle = false;

      // Compute actual dt for the filter
      uint32_t nowUs = micros();
      float dt = (float)(nowUs - _lastMadgwickUs) * 1e-6f;
      _lastMadgwickUs = nowUs;

      // Clamp dt to avoid huge jumps on first call or after stalls
      if (dt <= 0.0f || dt > 0.5f)
        dt = (float)BNO_REPORT_INTERVAL_MS * 0.001f;

      // Madgwick expects gyro in deg/s; BNO gives rad/s → convert
      float gxDeg = gyro.x * (180.0f / PI_F);
      float gyDeg = gyro.y * (180.0f / PI_F);
      float gzDeg = gyro.z * (180.0f / PI_F);

      // Update the filter's internal sample rate to match real dt
      _madgwick.begin(1.0f / dt);

      // Feed accel (m/s²) and gyro (deg/s) — Madgwick normalizes accel
      // internally so units don't matter as long as they're consistent.
      _madgwick.updateIMU(gxDeg, gyDeg, gzDeg, accel.x, accel.y, accel.z);

      // Extract filtered Euler angles (Madgwick returns degrees)
      float filteredRollDeg = _madgwick.getRoll();
      float filteredPitchDeg = _madgwick.getPitch();
      float filteredYawDeg = _madgwick.getYaw() - 180.0f; // library adds 180°

      // Convert to radians
      float filteredYawRad = filteredYawDeg * (PI_F / 180.0f);

      // Apply yaw offset (tare)
      if (!yawOffsetInitialized) {
        yawOffset = filteredYawRad;
        yawOffsetInitialized = true;
      }

      rotation.setRollRads(filteredRollDeg * (PI_F / 180.0f));
      rotation.setPitchRads(filteredPitchDeg * (PI_F / 180.0f));
      rotation.setYawRads(filteredYawRad - yawOffset);

      _madgwickReady = true;
    }
  }
};
