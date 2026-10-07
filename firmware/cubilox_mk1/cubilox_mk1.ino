// ============================================================
// Cubilox Mk1 - self-balancing reaction wheel cube
// ESP32, MPU-6500 IMU board (driven with the MPU6050_light library), three brushless
// motors with reaction wheels.
// Balances on an edge (motor X) and on a vertex (all three motors).
//
// Files:
//   config.h         pins, gains, limits, shared state
//   cubilox_mk1.ino  setup, filters, state machine, controllers
//   calibration.ino  gyro calibration, edge/vertex calibration, pose detection
//   commands.ino     command interface over USB / Bluetooth
//   hardware.ino     motors, encoders, battery, buzzer, motor test
// ============================================================
#include <Wire.h>
#include "BluetoothSerial.h"
#include "config.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled in the board package. Check Tools -> Partition Scheme.
#endif

void setup() {
  // Larger TX buffer so log lines never block the control loop (must be set before begin()).
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  SerialBT.begin(BT_DEVICE_NAME);
  loadParams();
  loadCalibration();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(21, 22);
  ledcAttach(PWM_X, PWM_FREQ, PWM_RES);
  ledcAttach(PWM_Y, PWM_FREQ, PWM_RES);
  ledcAttach(PWM_Z, PWM_FREQ, PWM_RES);

  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(DIR_X, OUTPUT); pinMode(ENC_X_A, INPUT); pinMode(ENC_X_B, INPUT);
  pinMode(DIR_Y, OUTPUT); pinMode(ENC_Y_A, INPUT); pinMode(ENC_Y_B, INPUT);
  pinMode(DIR_Z, OUTPUT); pinMode(ENC_Z_A, INPUT); pinMode(ENC_Z_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_X_A), ENC_X_READ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_X_B), ENC_X_READ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_Y_A), ENC_Y_READ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_Y_B), ENC_Y_READ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_Z_A), ENC_Z_READ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_Z_B), ENC_Z_READ, CHANGE);

  motorStopAll();

  logLine("=== %s ===", FIRMWARE_NAME);
  if (mpu.begin() != 0) {
    logLine("ERROR: IMU not found - check the I2C wiring.");
    while (true) { beep(1, 1000, 0); delay(1000); }
  }
  beep(1, 100, 0);

  calibrateGyro();

  updateBattery();
  printBattery();
  printCalibration();
  logLine("Ready. Place the cube on its edge or vertex - it starts automatically. Send 'help' for commands.");
  beep(1, 500, 0);
}

void loop() {
  handleCommands();
  mpu.update();

  long currentT = millis();
  float dt = (currentT - previousT_pid) / 1000.0;

  float ax = mpu.getAccX(), ay = mpu.getAccY(), az = mpu.getAccZ();
  float accMag = sqrt(ax * ax + ay * ay + az * az);
  bool isResting = (fabs(accMag - 1.0) < ACC_MAG_THRESHOLD);
  bool isRestingVertex = (fabs(accMag - 1.0) < VERTEX_ACC_MAG_THRESHOLD);

  // ---------- Edge filter ----------
  // Note: dt is the time since the last CONTROL step, not since the last filter
  // step. The edge gains are tuned with this behaviour - do not change it.
  float correctedAccX = mpu.getAngleX() - ANGLE_OFFSET_edge;
  float gyroRateX = mpu.getGyroX();
  if (dt > 0 && dt < 0.1) {
    cfAngle_edge = CF_ALPHA * (cfAngle_edge + gyroRateX * dt) + (1.0 - CF_ALPHA) * correctedAccX;
  }

  // ---------- Vertex filter: 3D tilt error ----------
  // r = calibrated gravity direction at the vertex (the vertical axis),
  // n = current gravity direction. For small angles, n x r is the rotation
  // error in rad. The order matters: n x r has the same sign as the integrated
  // gyro (MPU6050_light: positive rotation about X => accY increases).
  float gyroRateY = mpu.getGyroY();
  float gyroRateZ = mpu.getGyroZ();

  float refMag = sqrt(refAccX_vertex * refAccX_vertex +
                      refAccY_vertex * refAccY_vertex +
                      refAccZ_vertex * refAccZ_vertex);

  float accErrX = 0.0, accErrY = 0.0, accErrZ = 0.0;
  float accErrAngleDeg = 0.0;
  float rx = 0.0, ry = 0.0, rz = 0.0;

  if (accMag > 0.2 && refMag > 0.2) {
    float nx = ax / accMag, ny = ay / accMag, nz = az / accMag;
    rx = refAccX_vertex / refMag;
    ry = refAccY_vertex / refMag;
    rz = refAccZ_vertex / refMag;

    accErrX = (ny * rz - nz * ry) * 57.2957795;
    accErrY = (nz * rx - nx * rz) * 57.2957795;
    accErrZ = (nx * ry - ny * rx) * 57.2957795;

    float crossMag = sqrt(sq(ry * nz - rz * ny) + sq(rz * nx - rx * nz) + sq(rx * ny - ry * nx));
    float dot = constrain(rx * nx + ry * ny + rz * nz, -1.0, 1.0);
    accErrAngleDeg = atan2(crossMag, dot) * 57.2957795;
  }

  // Complementary filter on all three axes: the gyro gives the fast motion, the
  // accelerometer slowly corrects the drift (and is disturbed by wheel accelerations,
  // hence the long time constant). Uses its own timestamp (time since last filter step).
  unsigned long nowUs = micros();
  float dtV = (nowUs - previousT_vertexFilterUs) * 1e-6;
  previousT_vertexFilterUs = nowUs;
  if (dtV > 0 && dtV < 0.1) {
    float a = VERTEX_CF_TAU / (VERTEX_CF_TAU + dtV);
    cfVertexAngleX = a * (cfVertexAngleX + gyroRateX * dtV) + (1.0 - a) * accErrX;
    cfVertexAngleY = a * (cfVertexAngleY + gyroRateY * dtV) + (1.0 - a) * accErrY;
    cfVertexAngleZ = a * (cfVertexAngleZ + gyroRateZ * dtV) + (1.0 - a) * accErrZ;

    // Rotation about the vertical axis (yaw) does not matter for balancing and is
    // never corrected by the accelerometer -> keep only the tilt part.
    float along = cfVertexAngleX * rx + cfVertexAngleY * ry + cfVertexAngleZ * rz;
    cfVertexAngleX -= along * rx;
    cfVertexAngleY -= along * ry;
    cfVertexAngleZ -= along * rz;
  }

  float vertexErrAngleDeg = sqrt(sq(cfVertexAngleX) + sq(cfVertexAngleY) + sq(cfVertexAngleZ));

  // ---------- Wheel speeds: 20 ms window, scaled to counts per 100 ms ----------
  const unsigned long ENCODER_WINDOW_MS = 20;
  if (currentT - previousT_enc >= ENCODER_WINDOW_MS) {
    motorSpeed_X = enc_count_X * 5; enc_count_X = 0;
    motorSpeed_Y = enc_count_Y * 5; enc_count_Y = 0;
    motorSpeed_Z = enc_count_Z * 5; enc_count_Z = 0;
    previousT_enc = currentT;
  }

  Pose pose = detectPose(ax, ay, az);
  float edgeAngle = cfAngle_edge - edgeTrim;

  if (state == RUNNING_EDGE) {
    // ================= EDGE BALANCING =================
    if (abs(edgeAngle) > EDGE_STOP_ANGLE_DEG || !isResting) {
      motorStopAll();
      state = WAITING;
      stableStart = 0;
      logLine("Edge balancing stopped after %.1f s.", (currentT - rampStartT) / 1000.0);
      beep(1, 200, 0);
      previousT_pid = currentT;
      return;
    }
    if (currentT - previousT_pid >= 5) {
      if (rampFactor < 1.0) {
        rampFactor = constrain((float)(currentT - rampStartT) / EDGE_RAMP_MS, 0.0, 1.0);
      }
      maxLoopGapMs = max(maxLoopGapMs, (unsigned long)(currentT - previousT_pid));
      float output = (eK1 * edgeAngle + eK2 * gyroRateX + eK3 * motorSpeed_X) * rampFactor;
      motorControl_X((int)output);

      // At equilibrium eK1*angle + eK3*wheel ~ 0. If the wheel keeps turning
      // positive, the setpoint is too large -> lower the trim slowly (and vice versa).
      if (EDGE_AUTOTRIM && rampFactor >= 1.0 && dt < 0.1) {
        edgeTrim = constrain(edgeTrim - EDGE_TRIM_RATE * motorSpeed_X * dt,
                             -EDGE_TRIM_LIMIT, EDGE_TRIM_LIMIT);
      }
      if (debugOn) {
        static long lastEdgeLog = 0;
        if (currentT - lastEdgeLog >= 500) {
          lastEdgeLog = currentT;
          logLine("EDGE t=%ld angle=%.2f wheel=%d trim=%.3f loop=%lu",
                  currentT - rampStartT, edgeAngle, motorSpeed_X, edgeTrim, maxLoopGapMs);
          maxLoopGapMs = 0;
        }
      } else {
        long beat = (currentT - rampStartT) / (long)HEARTBEAT_MS;
        if (beat > heartbeatCount) {
          heartbeatCount = beat;
          logLine("Edge balancing for %ld s", (currentT - rampStartT) / 1000);
        }
      }
      previousT_pid = currentT;
    }

  } else if (state == RUNNING_VERTEX) {
    // ================= VERTEX BALANCING =================
    // Count shocks, but only stop on a sustained one.
    static int shockCount = 0;
    if (!isRestingVertex) {
      if (accBadStart == 0) { accBadStart = currentT; shockCount++; }
    } else {
      accBadStart = 0;
    }
    bool accStop = (accBadStart != 0 && currentT - accBadStart > VERTEX_ACC_STOP_MS);

    if (vertexErrAngleDeg > VERTEX_STOP_ANGLE_DEG || accStop
        || (satStart != 0 && currentT - satStart > SATURATION_STOP_MS)) {
      motorStopAll();
      state = WAITING;
      stableStart = 0;
      satStart = 0;
      accBadStart = 0;
      const char *cause = accStop ? "sustained shock"
                        : (vertexErrAngleDeg > VERTEX_STOP_ANGLE_DEG) ? "tilt angle too large"
                        : "motors saturated";
      logLine("Vertex balancing stopped after %.1f s (%s).", (currentT - rampStartT) / 1000.0, cause);
      if (debugOn) logLine("STOP angle=%.2f accMag=%.2f shocks=%d", vertexErrAngleDeg, accMag, shockCount);
      shockCount = 0;
      beep(1, 200, 0);
      previousT_pid = currentT;
      return;
    }
    if (currentT - previousT_pid >= 5) {
      maxLoopGapMs = max(maxLoopGapMs, (unsigned long)(currentT - previousT_pid));
      if (rampFactor < 1.0) {
        rampFactor = constrain((float)(currentT - rampStartT) / VERTEX_RAMP_MS, 0.0, 1.0);
      }

      // Everything in the cube frame: t = torque command for the wheels,
      // h = wheel momentum (encoder with mounting direction). Both are split into
      // a tilt part (perpendicular to r) and a yaw part (along r) and controlled
      // separately: there is no gravity about the vertical axis, so feeding wheel
      // momentum back there would be positive feedback.
      float hX = MOTOR_SIGN_X * WHEEL_SIGN_X * motorSpeed_X;
      float hY = MOTOR_SIGN_Y * WHEEL_SIGN_Y * motorSpeed_Y;
      float hZ = MOTOR_SIGN_Z * WHEEL_SIGN_Z * motorSpeed_Z;
      float hYaw = hX * rx + hY * ry + hZ * rz;
      float gYaw = gyroRateX * rx + gyroRateY * ry + gyroRateZ * rz;
      float hTx = hX - hYaw * rx, hTy = hY - hYaw * ry, hTz = hZ - hYaw * rz;
      float gTx = gyroRateX - gYaw * rx, gTy = gyroRateY - gYaw * ry, gTz = gyroRateZ - gYaw * rz;

      // Auto-trim: at equilibrium vK1*(angle - trim) + vK3*momentum ~ 0.
      // Remaining tilt momentum moves the trim slowly (like on the edge).
      if (rampFactor >= 1.0 && dt < 0.1) {
        vTrimX -= VERTEX_TRIM_RATE * hTx * dt;
        vTrimY -= VERTEX_TRIM_RATE * hTy * dt;
        vTrimZ -= VERTEX_TRIM_RATE * hTz * dt;
        float along = vTrimX * rx + vTrimY * ry + vTrimZ * rz;  // tilt part only
        vTrimX -= along * rx; vTrimY -= along * ry; vTrimZ -= along * rz;
        float trimMag = sqrt(sq(vTrimX) + sq(vTrimY) + sq(vTrimZ));
        if (trimMag > VERTEX_TRIM_LIMIT) {
          float k = VERTEX_TRIM_LIMIT / trimMag;
          vTrimX *= k; vTrimY *= k; vTrimZ *= k;
        }
      }

      float yawTorque = vYawD * gYaw - vYawW * hYaw;
      float tX = vK1 * (cfVertexAngleX - vTrimX) + vK2 * gTx + vK3 * hTx + yawTorque * rx;
      float tY = vK1 * (cfVertexAngleY - vTrimY) + vK2 * gTy + vK3 * hTy + yawTorque * ry;
      float tZ = vK1 * (cfVertexAngleZ - vTrimZ) + vK2 * gTz + vK3 * hTz + yawTorque * rz;

      float outX = MOTOR_SIGN_X * tX * rampFactor;
      float outY = MOTOR_SIGN_Y * tY * rampFactor;
      float outZ = MOTOR_SIGN_Z * tZ * rampFactor;

      // Saturation: the controller asks for more than the motors can deliver.
      bool saturated = abs(outX) > 255 || abs(outY) > 255 || abs(outZ) > 255;
      if (saturated) {
        if (satStart == 0) satStart = currentT;
      } else {
        satStart = 0;
      }

      // Motors first, logging afterwards: a log line never delays a motor command.
      motorControl_X((int)outX);
      motorControl_Y((int)outY);
      motorControl_Z((int)outZ);

      if (debugOn) {
        // One line every 50 ms (= one Bluetooth packet). loop = longest control gap in ms.
        static long lastRunLog = 0;
        if (currentT - lastRunLog >= 50) {
          lastRunLog = currentT;
          logLine("RUN t=%ld e=%.1f/%.1f/%.1f g=%.0f/%.0f/%.0f w=%d/%d/%d o=%.0f/%.0f/%.0f "
                  "yaw=%.0f tilt=%.0f trim=%.2f/%.2f/%.2f loop=%lu",
                  currentT - rampStartT, cfVertexAngleX, cfVertexAngleY, cfVertexAngleZ,
                  gyroRateX, gyroRateY, gyroRateZ, motorSpeed_X, motorSpeed_Y, motorSpeed_Z,
                  outX, outY, outZ, hYaw, sqrt(sq(hTx) + sq(hTy) + sq(hTz)),
                  vTrimX, vTrimY, vTrimZ, maxLoopGapMs);
          maxLoopGapMs = 0;
        }
      } else {
        long beat = (currentT - rampStartT) / (long)HEARTBEAT_MS;
        if (beat > heartbeatCount) {
          heartbeatCount = beat;
          logLine("Vertex balancing for %ld s (tilt %.1f deg)", (currentT - rampStartT) / 1000, vertexErrAngleDeg);
        }
      }
      previousT_pid = currentT;
    }

  } else {
    // ================= WAITING =================
    // Battery is only measured while idle (the voltage sags under load).
    static long lastBatT = 0, lastBatWarnT = -30000;
    if (currentT - lastBatT >= 1000) {
      lastBatT = currentT;
      updateBattery();
      if (batteryConnected() && batteryV < BAT_WARN_V && currentT - lastBatWarnT >= 30000) {
        lastBatWarnT = currentT;
        logLine("WARNING: battery low (%.2f V) - please charge soon!", batteryV);
        beep(3, 60, 60);
      }
    }
    bool batteryEmpty = batteryConnected() && batteryV < BAT_MIN_V;

    bool readyEdge = (pose == POSE_EDGE) && (abs(edgeAngle) < EDGE_START_ANGLE_DEG) && isResting;
    bool readyVertex = (pose == POSE_VERTEX) && (accErrAngleDeg < VERTEX_START_ANGLE_DEG) && isResting;

    if ((readyEdge || readyVertex) && batteryEmpty) {
      stableStart = 0;
      if (currentT - previousT_print >= 2000) {
        logLine("No start: battery empty (%.2f V < %.1f V) - please charge.", batteryV, BAT_MIN_V);
        beep(1, 400, 0);
        previousT_print = currentT;
      }
    } else if (readyEdge || readyVertex) {
      if (stableStart == 0) stableStart = currentT;
      if (currentT - stableStart >= STABLE_HOLD_MS) {
        digitalWrite(BRAKE_PIN, HIGH);   // release the brake
        rampStartT = currentT;
        rampFactor = 0.0;
        maxLoopGapMs = 0;
        heartbeatCount = 0;
        state = readyEdge ? RUNNING_EDGE : RUNNING_VERTEX;
        if (batteryConnected()) {
          logLine("%s balancing started (battery %.2f V).", readyEdge ? "Edge" : "Vertex", batteryV);
        } else {
          logLine("%s balancing started.", readyEdge ? "Edge" : "Vertex");
        }
        beep(2, 50, 50);
      }
    } else {
      stableStart = 0;
      if (debugOn && currentT - previousT_print >= 500) {
        logLine("IDLE pose=%s acc=%.2f/%.2f/%.2f edge=%.2f vertex=%.2f deg battery=%.2f V",
                pose == POSE_EDGE ? "edge" : pose == POSE_VERTEX ? "vertex" : "none",
                ax, ay, az, cfAngle_edge, accErrAngleDeg, batteryV);
        previousT_print = currentT;
      }
    }
    previousT_pid = currentT;
  }
}
