// ============================================================
// Cubilox Mk1 - hardware: logging, battery, buzzer, motors, encoders, motor test
// ============================================================

// Formats a whole line and sends it as ONE packet (see DPRINT note in config.h).
void logLine(const char *fmt, ...) {
  char buf[220];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf, sizeof(buf) - 2, fmt, args);
  va_end(args);
  if (n < 0) return;
  if (n > (int)sizeof(buf) - 3) n = sizeof(buf) - 3;
  buf[n++] = '\r';
  buf[n++] = '\n';
  Serial.write((const uint8_t *)buf, n);
  SerialBT.write((const uint8_t *)buf, n);
}

// ============================================================
// BATTERY
// ============================================================
float readBatteryPinV() {
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) sum += analogReadMilliVolts(VBAT_PIN);
  return sum / 16.0 / 1000.0;
}

void updateBattery() {
  float v = readBatteryPinV() * VBAT_FACTOR;
  batteryV = (batteryV <= 0) ? v : 0.8 * batteryV + 0.2 * v;
}

// Below 1 V the divider is most likely not connected -> ignore battery checks.
bool batteryConnected() { return batteryV > 1.0; }

// ============================================================
// BUZZER
// ============================================================
void beep(int count, int onMs, int offMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(offMs);
  }
}

// ============================================================
// MOTORS (speed -255..255, sign = direction)
// ============================================================
void motorControl(int pwmPin, int dirPin, int sp) {
  sp = constrain(sp, -255, 255);
  digitalWrite(dirPin, sp >= 0 ? LOW : HIGH);
  ledcWrite(pwmPin, 255 - abs(sp));
}
void motorControl_X(int sp) { motorControl(PWM_X, DIR_X, sp); }
void motorControl_Y(int sp) { motorControl(PWM_Y, DIR_Y, sp); }
void motorControl_Z(int sp) { motorControl(PWM_Z, DIR_Z, sp); }

// Stops all motors and engages the brake.
void motorStopAll() {
  motorControl_X(0);
  motorControl_Y(0);
  motorControl_Z(0);
  digitalWrite(BRAKE_PIN, LOW);
}

// ============================================================
// ENCODER INTERRUPTS (quadrature decoding)
// ============================================================
void IRAM_ATTR ENC_X_READ() {
  static int st = 0;
  st = (st << 2 | (digitalRead(ENC_X_A) << 1) | digitalRead(ENC_X_B)) & 0x0f;
  if (st == 0x02 || st == 0x0d || st == 0x04 || st == 0x0b) enc_count_X++;
  else if (st == 0x01 || st == 0x0e || st == 0x08 || st == 0x07) enc_count_X--;
}
void IRAM_ATTR ENC_Y_READ() {
  static int st = 0;
  st = (st << 2 | (digitalRead(ENC_Y_A) << 1) | digitalRead(ENC_Y_B)) & 0x0f;
  if (st == 0x02 || st == 0x0d || st == 0x04 || st == 0x0b) enc_count_Y++;
  else if (st == 0x01 || st == 0x0e || st == 0x08 || st == 0x07) enc_count_Y--;
}
void IRAM_ATTR ENC_Z_READ() {
  static int st = 0;
  st = (st << 2 | (digitalRead(ENC_Z_A) << 1) | digitalRead(ENC_Z_B)) & 0x0f;
  if (st == 0x02 || st == 0x0d || st == 0x04 || st == 0x0b) enc_count_Z++;
  else if (st == 0x01 || st == 0x0e || st == 0x08 || st == 0x07) enc_count_Z--;
}

// ============================================================
// MOTOR TEST ("motortest" command)
// Short pulse on each motor while the cube is held loosely by hand. Measures the
// cube's reaction (gyro) and the wheel's reaction (encoder) and recommends the
// MOTOR_SIGN / WHEEL_SIGN values for config.h.
// ============================================================
void motorControlAxis(int axis, int sp) {
  if (axis == 0) motorControl_X(sp);
  else if (axis == 1) motorControl_Y(sp);
  else motorControl_Z(sp);
}

int readAndResetEnc(int axis) {
  int c;
  noInterrupts();
  if (axis == 0) { c = enc_count_X; enc_count_X = 0; }
  else if (axis == 1) { c = enc_count_Y; enc_count_Y = 0; }
  else { c = enc_count_Z; enc_count_Z = 0; }
  interrupts();
  return c;
}

void runMotorTest() {
  const char *names[3] = {"X", "Y", "Z"};
  float gyroMean[3][3];   // [motor][gyro axis]
  int encCounts[3];

  logLine("=== MOTOR TEST: hold the cube loosely in your hand. Starting in 3 s ===");
  delay(3000);

  for (int m = 0; m < 3; m++) {
    beep(1, 50, 0);
    delay(500);
    float sum[3] = {0, 0, 0};
    int n = 0;
    digitalWrite(BRAKE_PIN, HIGH);
    readAndResetEnc(m);
    unsigned long t0 = millis();
    motorControlAxis(m, MOTOR_TEST_PWM);
    while (millis() - t0 < MOTOR_TEST_PULSE_MS) {
      mpu.update();
      sum[0] += mpu.getGyroX(); sum[1] += mpu.getGyroY(); sum[2] += mpu.getGyroZ();
      n++;
      delay(2);
    }
    encCounts[m] = readAndResetEnc(m);
    motorStopAll();
    for (int k = 0; k < 3; k++) gyroMean[m][k] = sum[k] / max(n, 1);

    logLine("Motor %s: gyro X/Y/Z = %.2f / %.2f / %.2f deg/s | encoder = %d",
            names[m], gyroMean[m][0], gyroMean[m][1], gyroMean[m][2], encCounts[m]);
    delay(2500);  // let the wheel spin down
  }

  // Evaluation relative to motor X.
  for (int m = 0; m < 3; m++) {
    float own = gyroMean[m][m];
    float other = max(fabs(gyroMean[m][(m + 1) % 3]), fabs(gyroMean[m][(m + 2) % 3]));
    if (fabs(own) < other) {
      logLine("WARNING: motor %s turns the cube more about ANOTHER axis -> check wiring/axes!", names[m]);
    }
  }
  float gX = gyroMean[0][0] >= 0 ? 1.0 : -1.0;
  float eX = encCounts[0] >= 0 ? 1.0 : -1.0;
  for (int m = 0; m < 3; m++) {
    float g = gyroMean[m][m] >= 0 ? 1.0 : -1.0;
    float e = encCounts[m] >= 0 ? 1.0 : -1.0;
    logLine("Recommended: MOTOR_SIGN_%s = %.0f   WHEEL_SIGN_%s = %.0f", names[m], gX * g, names[m], eX * e);
  }
  logLine("Encoder counts should be similar (~380); a clearly lower value = weak motor or friction.");
  logLine("=== MOTOR TEST DONE ===");
}
