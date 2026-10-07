// ============================================================
// Cubilox Mk1 - command interface (USB serial or Bluetooth "Cubilox_Mk1")
// One command per line:
//   help            list of commands
//   status          gains, calibration, battery
//   edge / vertex   calibrate the balance point (balance the cube by hand)
//   vk1 95          set a gain (vk1 alone shows it), see paramNames below
//   save / reset    store gains in flash / restore the defaults from config.h
//   debug 0/1       detailed log lines off/on
//   motortest       short pulse on every motor (hold the cube loosely)
//   vbatcal 12.6    optional: fine-tune the battery reading to a multimeter value
// ============================================================

const char *paramNames[] = {"vk1", "vk2", "vk3", "vyd", "vyw", "vtrim",
                            "ek1", "ek2", "ek3", "trim"};
float *paramValues[] = {&vK1, &vK2, &vK3, &vYawD, &vYawW, &VERTEX_TRIM_RATE,
                        &eK1, &eK2, &eK3, &EDGE_TRIM_RATE};
const int NUM_PARAMS = sizeof(paramNames) / sizeof(paramNames[0]);
float paramDefaults[NUM_PARAMS];

char usbBuf[32], btBuf[32];
int usbLen = 0, btLen = 0;

void printParams() {
  char line[200];
  int len = snprintf(line, sizeof(line), "Gains:");
  for (int i = 0; i < NUM_PARAMS && len < (int)sizeof(line); i++) {
    len += snprintf(line + len, sizeof(line) - len, " %s=%g", paramNames[i], *paramValues[i]);
  }
  logLine("%s", line);
}

void printBattery() {
  if (batteryConnected()) logLine("Battery: %.2f V", batteryV);
  else logLine("Battery: not connected (pin %.3f V)", readBatteryPinV());
}

void printStatus() {
  printParams();
  printCalibration();
  printBattery();
  logLine("Debug output: %s", debugOn ? "on" : "off");
}

// Remember the defaults from config.h, then load stored values.
void loadParams() {
  for (int i = 0; i < NUM_PARAMS; i++) paramDefaults[i] = *paramValues[i];
  prefs.begin("cube", true);
  for (int i = 0; i < NUM_PARAMS; i++) {
    *paramValues[i] = prefs.getFloat(paramNames[i], *paramValues[i]);
  }
  prefs.end();
  prefs.begin("cubebat", true);   // own namespace, survives "reset"
  VBAT_FACTOR = prefs.getFloat("vbatf", VBAT_FACTOR);
  prefs.end();
}

bool requireIdle() {
  if (state == WAITING) return true;
  logLine("Only possible while the cube is not balancing.");
  return false;
}

void processCommand(char *cmd) {
  for (char *c = cmd; *c; c++) *c = tolower(*c);
  char name[16];
  float val;
  int n = sscanf(cmd, "%15s %f", name, &val);
  if (n < 1) return;

  if (!strcmp(name, "help") || !strcmp(name, "?")) {
    logLine("Commands: status | edge | vertex | save | reset | debug 0/1 | motortest | vbatcal <V>");
    logLine("Gains: <name> <value> to set, <name> to show. Names: vk1 vk2 vk3 vyd vyw vtrim (vertex),");
    logLine("       ek1 ek2 ek3 trim (edge). Changes are lost on restart unless you send 'save'.");
  } else if (!strcmp(name, "status") || !strcmp(name, "p")) {
    printStatus();
  } else if (!strcmp(name, "edge") || !strcmp(name, "vertex")) {
    if (requireIdle()) calibratePose(!strcmp(name, "vertex"));
  } else if (!strcmp(name, "debug")) {
    if (n == 2) debugOn = (val != 0);
    logLine("Debug output: %s", debugOn ? "on" : "off");
  } else if (!strcmp(name, "motortest")) {
    if (requireIdle()) runMotorTest();
  } else if (!strcmp(name, "vbatcal")) {
    if (!requireIdle()) return;
    float pinV = readBatteryPinV();
    if (n < 2 || val < 6 || val > 20) { logLine("Please give the measured voltage, e.g. 'vbatcal 12.6'"); return; }
    if (pinV < 0.1) { logLine("No signal on the battery pin (%.3f V) - check the wiring.", pinV); return; }
    VBAT_FACTOR = val / pinV;
    batteryV = val;
    prefs.begin("cubebat", false);
    prefs.putFloat("vbatf", VBAT_FACTOR);
    prefs.end();
    logLine("Battery reading calibrated: factor %.3f, saved.", VBAT_FACTOR);
  } else if (!strcmp(name, "save") || !strcmp(name, "reset")) {
    if (!requireIdle()) return;   // writing flash takes a few ms
    prefs.begin("cube", false);
    if (!strcmp(name, "save")) {
      for (int i = 0; i < NUM_PARAMS; i++) prefs.putFloat(paramNames[i], *paramValues[i]);
      logLine("Gains saved.");
    } else {
      prefs.clear();
      for (int i = 0; i < NUM_PARAMS; i++) *paramValues[i] = paramDefaults[i];
      logLine("Gains reset to the defaults.");
    }
    prefs.end();
    printParams();
  } else {
    for (int i = 0; i < NUM_PARAMS; i++) {
      if (!strcmp(name, paramNames[i])) {
        if (n == 2) *paramValues[i] = val;
        logLine("%s=%g", paramNames[i], *paramValues[i]);
        return;
      }
    }
    logLine("Unknown command: %s  (send 'help')", name);
  }
}

void pollStream(Stream &s, char *buf, int &len) {
  while (s.available()) {
    char c = s.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = 0; processCommand(buf); len = 0; }
    } else if (len < 31) {
      buf[len++] = c;
    }
  }
}

void handleCommands() {
  pollStream(Serial, usbBuf, usbLen);
  pollStream(SerialBT, btBuf, btLen);
}
