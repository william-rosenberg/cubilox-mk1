// ============================================================
// Cubilox Mk1 - calibration
//
// Gyro bias: measured at every start (the cube only has to lie still, the
//   surface does not need to be level).
// Poses: the balance points of the edge and the vertex are measured ONCE with
//   the "edge" / "vertex" commands while the cube is balanced by hand, and are
//   stored in flash together with the accelerometer offsets. Because these
//   offsets are never recomputed, the stored poses stay valid after every
//   restart or power loss.
//   Balancing by hand finds the real balance point (center of mass above the
//   pivot), which a geometrically perfect fixture does not.
// ============================================================

static const char *CALIB_NS = "cuberef";  // flash namespace (not affected by "reset")

void loadCalibration() {
  prefs.begin(CALIB_NS, true);
  accOffX = prefs.getFloat("ox", 0); accOffY = prefs.getFloat("oy", 0); accOffZ = prefs.getFloat("oz", 0);
  vertexCalibrated = prefs.getBool("ok", false);
  if (vertexCalibrated) {
    refAccX_vertex = prefs.getFloat("rx", 0); refAccY_vertex = prefs.getFloat("ry", 0);
    refAccZ_vertex = prefs.getFloat("rz", 0);
  }
  edgeCalibrated = prefs.getBool("eok", false);
  if (edgeCalibrated) {
    refAccX_edge = prefs.getFloat("ex", 0); refAccY_edge = prefs.getFloat("ey", 0);
    refAccZ_edge = prefs.getFloat("ez", 0); ANGLE_OFFSET_edge = prefs.getFloat("ea", 0);
  }
  prefs.end();
}

void saveCalibration() {
  prefs.begin(CALIB_NS, false);
  prefs.putFloat("ox", mpu.getAccXoffset()); prefs.putFloat("oy", mpu.getAccYoffset());
  prefs.putFloat("oz", mpu.getAccZoffset());
  prefs.putBool("ok", vertexCalibrated);
  prefs.putFloat("rx", refAccX_vertex); prefs.putFloat("ry", refAccY_vertex); prefs.putFloat("rz", refAccZ_vertex);
  prefs.putBool("eok", edgeCalibrated);
  prefs.putFloat("ex", refAccX_edge); prefs.putFloat("ey", refAccY_edge); prefs.putFloat("ez", refAccZ_edge);
  prefs.putFloat("ea", ANGLE_OFFSET_edge);
  prefs.end();
}

// Called once in setup(): the cube must lie still for about 3 s.
void calibrateGyro() {
  logLine("Keep the cube still (gyro calibration)...");
  delay(1000);
  mpu.calcOffsets(true, false);                   // gyro only
  mpu.setAccOffsets(accOffX, accOffY, accOffZ);   // accelerometer offsets from flash
  long warmup = millis();
  while (millis() - warmup < 1000) { mpu.update(); delay(10); }  // let the library's angle filter settle
  beep(2, 100, 100);
}

// Angle between a measured pose and the ideal one:
// vertex = a body diagonal (±1, ±1, ±1)/sqrt(3), edge = a diagonal in the Y/Z plane
// (motor X must be the axis along the edge).
float poseErrorDeg(bool vertex, float x, float y, float z) {
  float m = sqrt(x * x + y * y + z * z);
  if (m < 0.2) return 180.0;
  x /= m; y /= m; z /= m;
  float c = vertex ? (fabs(x) + fabs(y) + fabs(z)) / sqrt(3.0) : (fabs(y) + fabs(z)) / sqrt(2.0);
  return acos(constrain(c, -1.0, 1.0)) * 57.2957795;
}

void printCalibration() {
  if (edgeCalibrated) {
    logLine("Edge:   calibrated (%.2f deg from the ideal edge)",
            poseErrorDeg(false, refAccX_edge, refAccY_edge, refAccZ_edge));
  } else {
    logLine("Edge:   NOT calibrated - send 'edge'");
  }
  if (vertexCalibrated) {
    logLine("Vertex: calibrated (%.2f deg from the cube diagonal)",
            poseErrorDeg(true, refAccX_vertex, refAccY_vertex, refAccZ_vertex));
  } else {
    logLine("Vertex: NOT calibrated - send 'vertex'");
  }
}

// "edge" / "vertex" command: the user balances the cube by hand, then the
// gravity vector is averaged and stored as the new balance point.
void calibratePose(bool vertex) {
  const char *name = vertex ? "vertex" : "edge";
  logLine("Balance the cube on its %s by hand. Measuring starts in %lu s...",
          name, CALIB_SETTLE_MS / 1000);
  beep(1, 300, 0);

  unsigned long t0 = millis();
  unsigned long nextTick = 1000;
  while (millis() - t0 < CALIB_SETTLE_MS) {
    mpu.update();
    if (millis() - t0 >= nextTick) {   // short tick every second
      beep(1, 15, 0);
      nextTick += 1000;
    }
    delay(5);
  }

  logLine("Measuring - hold still (%lu s)...", CALIB_AVERAGE_MS / 1000);
  beep(1, 150, 0);
  double s[3] = {0, 0, 0}, sumSq[3] = {0, 0, 0}, sumAngle = 0;
  int n = 0;
  t0 = millis();
  while (millis() - t0 < CALIB_AVERAGE_MS) {
    mpu.update();
    float a[3] = {mpu.getAccX(), mpu.getAccY(), mpu.getAccZ()};
    for (int i = 0; i < 3; i++) { s[i] += a[i]; sumSq[i] += a[i] * a[i]; }
    sumAngle += mpu.getAngleX();
    n++;
    delay(10);
  }

  float mean[3], noise = 0;
  for (int i = 0; i < 3; i++) {
    mean[i] = s[i] / n;
    noise = max(noise, (float)sqrt(max(0.0, sumSq[i] / n - mean[i] * mean[i])));
  }
  float err = poseErrorDeg(vertex, mean[0], mean[1], mean[2]);

  if (noise > CALIB_MAX_NOISE_G) {
    logLine("Calibration rejected: the cube moved too much (noise %.3f g). Please try again.", noise);
    beep(1, 800, 0);
    return;
  }
  if (err > CALIB_MAX_POSE_ERR_DEG) {
    logLine("Calibration rejected: %.1f deg away from a %s%s. Please try again.", err,
            vertex ? "vertex" : "edge", vertex ? "" : " along motor X");
    beep(1, 800, 0);
    return;
  }

  if (vertex) {
    refAccX_vertex = mean[0]; refAccY_vertex = mean[1]; refAccZ_vertex = mean[2];
    vertexCalibrated = true;
    cfVertexAngleX = cfVertexAngleY = cfVertexAngleZ = 0.0;
    vTrimX = vTrimY = vTrimZ = 0.0;
  } else {
    refAccX_edge = mean[0]; refAccY_edge = mean[1]; refAccZ_edge = mean[2];
    ANGLE_OFFSET_edge = sumAngle / n;
    edgeCalibrated = true;
    cfAngle_edge = 0.0;
    edgeTrim = 0.0;
  }
  saveCalibration();
  logLine("%s calibrated and saved (%.2f deg from the ideal pose, noise %.3f g).",
          vertex ? "Vertex" : "Edge", err, noise);
  beep(2, 150, 150);
}

// Which calibrated pose is the cube closest to (raw acceleration in g)?
Pose detectPose(float ax, float ay, float az) {
  float dEdge = edgeCalibrated
      ? sqrt(sq(ax - refAccX_edge) + sq(ay - refAccY_edge) + sq(az - refAccZ_edge)) : 99.0;
  float dVertex = vertexCalibrated
      ? sqrt(sq(ax - refAccX_vertex) + sq(ay - refAccY_vertex) + sq(az - refAccZ_vertex)) : 99.0;

  if (dEdge < POSE_MATCH_TOLERANCE && dEdge <= dVertex) return POSE_EDGE;
  if (dVertex < POSE_MATCH_TOLERANCE && dVertex < dEdge) return POSE_VERTEX;
  return POSE_NONE;
}
