// ============================================================
// Cubilox Mk1 - configuration: pins, controller gains, shared state
// ============================================================
#pragma once
#include <MPU6050_light.h>  // also works with the MPU-6500 board used here (compatible registers)
#include <Preferences.h>

#define FIRMWARE_NAME "Cubilox Mk1"
#define BT_DEVICE_NAME "Cubilox_Mk1"

// ============================================================
// PINS
// Each motor is named after the IMU axis it spins around:
// motor X turns the cube about sensor X, motor Y about Y, motor Z about Z.
// ============================================================
#define PWM_X       32
#define DIR_X       4
#define ENC_X_A     35
#define ENC_X_B     33

#define PWM_Y       14
#define DIR_Y       13
#define ENC_Y_A     36
#define ENC_Y_B     19

#define PWM_Z       25
#define DIR_Z       27
#define ENC_Z_A     34
#define ENC_Z_B     18

#define BRAKE_PIN   26   // HIGH = motors free, LOW = brake
#define BUZZER_PIN  23
#define VBAT_PIN    39   // battery voltage divider (33k / 10k)

#define PWM_FREQ    20000
#define PWM_RES     8    // PWM is inverted by the motor drivers: duty = 255 - |speed|

// ============================================================
// CONTROLLER GAINS (all adjustable at runtime, see commands.ino)
// ============================================================
// Edge controller (motor X only): out = eK1*angle + eK2*gyro + eK3*wheel
float eK1 = 70.0;   // tilt angle [deg]
float eK2 = 8.0;    // angular rate [deg/s]
float eK3 = 0.35;   // wheel speed [encoder units]

// Edge auto-trim: if the wheel keeps spinning one way, the balance point is
// slightly off. The setpoint is shifted slowly until the wheel rests on average.
#define EDGE_AUTOTRIM true
float EDGE_TRIM_RATE = 0.001;        // deg per (encoder unit * s)
const float EDGE_TRIM_LIMIT = 3.0;   // max. shift [deg]

// Vertex controller, computed in the cube frame and split into
//  - tilt part (perpendicular to the vertical axis r): vK1, vK2, vK3
//  - yaw part (along r, no gravity there): vYawD, vYawW
float vK1 = 90.0;    // tilt angle [deg]
float vK2 = 9.5;     // tilt rate [deg/s]
float vK3 = 0.30;    // wheel momentum in tilt direction
float vYawD = 9.5;   // damping of the cube's rotation about the vertical axis [deg/s]
float vYawW = 0.15;  // removes wheel momentum about the vertical axis (> 0, otherwise positive feedback)

// Vertex auto-trim (like the edge trim, as a tilt vector). 0 = off.
float VERTEX_TRIM_RATE = 0.0;
const float VERTEX_TRIM_LIMIT = 3.0; // [deg]
float vTrimX = 0.0, vTrimY = 0.0, vTrimZ = 0.0;

// Mounting directions of the motors relative to motor X (measured with "motortest"):
// MOTOR_SIGN for torque, WHEEL_SIGN for the encoder direction.
const float MOTOR_SIGN_X = 1.0, MOTOR_SIGN_Y = -1.0, MOTOR_SIGN_Z = -1.0;
const float WHEEL_SIGN_X = 1.0, WHEEL_SIGN_Y = 1.0, WHEEL_SIGN_Z = 1.0;
const int MOTOR_TEST_PWM = 150;
const unsigned long MOTOR_TEST_PULSE_MS = 200;

// ============================================================
// LIMITS AND TIMING
// ============================================================
// Edge
const float EDGE_STOP_ANGLE_DEG = 15.0;
const float EDGE_START_ANGLE_DEG = 1.0;
const int   EDGE_RAMP_MS = 800;
const float CF_ALPHA = 0.98;             // edge complementary filter
const float ACC_MAG_THRESHOLD = 0.3;     // |acc| - 1 g above this = cube is moving

// Vertex
const float VERTEX_STOP_ANGLE_DEG = 25.0;
const float VERTEX_START_ANGLE_DEG = 1.5;
const int   VERTEX_RAMP_MS = 150;
const float VERTEX_CF_TAU = 0.5;         // [s] gyro/acc complementary filter time constant
// Fast wheel reversals cause short shocks > 2 g although the cube is fine,
// so only a sustained shock stops the vertex balance.
const float VERTEX_ACC_MAG_THRESHOLD = 1.2;
const unsigned long VERTEX_ACC_STOP_MS = 150;
const unsigned long SATURATION_STOP_MS = 200; // motors at their limit for this long = stop

const unsigned long STABLE_HOLD_MS = 400;     // pose must be held this long before start
const float POSE_MATCH_TOLERANCE = 0.25;      // [g] distance to a calibrated pose

// Pose calibration ("edge" / "vertex" commands)
const unsigned long CALIB_SETTLE_MS  = 7000;  // time to get the cube balanced by hand
const unsigned long CALIB_AVERAGE_MS = 3000;  // averaging window
const float CALIB_MAX_NOISE_G = 0.05;         // std. deviation above this = cube moved
const float CALIB_MAX_POSE_ERR_DEG = 12.0;    // plausibility check against the ideal pose

// Status message while balancing (normal mode)
const unsigned long HEARTBEAT_MS = 10000;

// ============================================================
// BATTERY (3S LiPo, 12.6 V full)
// Divider R1 = 33k (battery+ -> pin), R2 = 10k (pin -> GND): V_bat = V_pin * 4.3.
// "vbatcal <V>" can fine-tune the factor with a multimeter reading.
// ============================================================
float VBAT_FACTOR = (33.0 + 10.0) / 10.0;
const float BAT_WARN_V = 11.1;    // 3.7 V/cell: warning
const float BAT_MIN_V  = 10.5;    // 3.5 V/cell: no start (protects the battery)
float batteryV = 0.0;             // smoothed, only measured while idle

// ============================================================
// STATE
// ============================================================
enum SystemState { WAITING, RUNNING_EDGE, RUNNING_VERTEX };
SystemState state = WAITING;

enum Pose { POSE_NONE, POSE_EDGE, POSE_VERTEX };

bool debugOn = false;             // "debug 1": detailed log lines
float rampFactor = 0.0;
unsigned long stableStart = 0;
long previousT_pid = 0, previousT_print = 0, rampStartT = 0;
unsigned long maxLoopGapMs = 0;   // longest gap between two control steps (should be 5-6 ms)
long heartbeatCount = 0;          // status messages sent during the current run

// Calibrated poses (acceleration vector in g), stored in flash
bool edgeCalibrated = false, vertexCalibrated = false;
float refAccX_edge = 0, refAccY_edge = 0, refAccZ_edge = 0;
float refAccX_vertex = 0, refAccY_vertex = 0, refAccZ_vertex = 0;
float ANGLE_OFFSET_edge = 0.0;
// Accelerometer offsets belonging to the stored poses. They are never recomputed,
// so the stored poses stay valid after every restart.
float accOffX = 0, accOffY = 0, accOffZ = 0;

// Edge filter
float cfAngle_edge = 0.0;
float edgeTrim = 0.0;             // learned balance point shift (kept after a fall)

// Vertex filter
unsigned long previousT_vertexFilterUs = 0;
float cfVertexAngleX = 0.0, cfVertexAngleY = 0.0, cfVertexAngleZ = 0.0;
long accBadStart = 0;
long satStart = 0;

MPU6050 mpu(Wire);
BluetoothSerial SerialBT;
Preferences prefs;

volatile int enc_count_X = 0, enc_count_Y = 0, enc_count_Z = 0;
int16_t motorSpeed_X = 0, motorSpeed_Y = 0, motorSpeed_Z = 0;
long previousT_enc = 0;

// Declared by hand: Arduino's automatic prototypes are unreliable for IRAM_ATTR functions.
void IRAM_ATTR ENC_X_READ();
void IRAM_ATTR ENC_Y_READ();
void IRAM_ATTR ENC_Z_READ();

// Output to USB and Bluetooth. With a connected Bluetooth client every print call
// becomes one packet (queue of 32, up to 1 s blocking when full). Use DPRINT only
// for rare messages; regular lines always go through logLine() = one line, one packet.
#define DPRINT(...)   { Serial.print(__VA_ARGS__);   SerialBT.print(__VA_ARGS__); }
#define DPRINTLN(...) { Serial.println(__VA_ARGS__); SerialBT.println(__VA_ARGS__); }
void logLine(const char *fmt, ...);
