#include <Wire.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

// =====================================================
//          FIX Arduino auto-prototypes issues
// =====================================================
struct JointCtrl {
  float th = 0, om = 0, th_prev = 0;
  float ei = 0;
  float ref = 0;
  float Kp = 0, Kd = 0, Ki = 0;
  float vel_lim = 0;
};

// Prototypes (declare BEFORE Arduino generates its own)
float computeVelCmd(JointCtrl &J);
void updateJointStates();
void sendTelemetry(float z_mm);

// =====================================================
//                 PYBOT-COMPAT CONFIG
// =====================================================

// ----- Geometry (from PyBot Configuration.h) -----
static const float L1 = 91.61f;
static const float L2 = 105.92f;

// Axis definition (from PyBot Configuration.h):
// #define ROBOT_AXIS_DEFINITION -90*GRAD2RAD
static const float AXIS_DEF_RAD = -90.0f * (PI / 180.0f);

// Joint limits (from PyBot Configuration.h)
static const float J1_MIN = -120.0f;
static const float J1_MAX =  120.0f;
static const float J2_MIN = -140.0f;
static const float J2_MAX =  140.0f;

static const float Z_MIN_MM = 0.0f;
static const float Z_MAX_MM = 120.0f;

// ----- I2C / TCA9548A / AS5600 -----
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;
static const uint8_t TCA_ADDR = 0x70;
static const uint8_t AS5600_ADDR = 0x36;
static const uint8_t CH_J1 = 0;
static const uint8_t CH_J2 = 1;

// ----- Stepper pins (عدّل حسب توصيلك) -----
static const int EN_PIN  = 25;  // Enable for drivers (إن وجد)
static const int J1_STEP = 26;
static const int J1_DIR  = 27;
static const int J2_STEP = 32;
static const int J2_DIR  = 33;
static const int Z_STEP  = 18;
static const int Z_DIR   = 19;

// ----- Endstops pins (Active-Low) -----
static const int ES_J1 = 34;
static const int ES_J2 = 35;
static const int ES_Z  = 39;
static inline bool ES_PRESSED(int pin) { return digitalRead(pin) == LOW; }

// ----- Servos (End Effector) -----
static const int SERVO_ROT_PIN  = 16;  // rotation servo signal
static const int SERVO_GRIP_PIN = 17;  // gripper servo signal
static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2500;
static int ROT_SERVO_DEG  = 90;  // 0..180
static int GRIP_SERVO_DEG = 90;  // 0..180

Servo servoRot;
Servo servoGrip;

// ----- Steps calibration (غيرها حسب ميكانيكك) -----
// PyBot reference: 40 steps/deg, 64.713 steps/deg, 396 steps/mm
static float J1_STEPS_PER_DEG = 40.0f;
static float J2_STEPS_PER_DEG = 64.713f;
static float Z_STEPS_PER_MM   = 396.0f;

// Speeds (محافظ للعزم)
static const float J1_MAX_SPEED_STEPS = 2000;
static const float J2_MAX_SPEED_STEPS = 2200;
static const float Z_MAX_SPEED_STEPS  = 2000;

static const float J1_ACC_STEPS = 3000;
static const float J2_ACC_STEPS = 3500;
static const float Z_ACC_STEPS  = 3000;

// Homing directions (جرّب -1 أو +1 لكل محور)
static int HOME_DIR_J1 = -1;
static int HOME_DIR_J2 = -1;
static int HOME_DIR_Z  = -1;

static const float HOME_SPEED_STEPS = 400;
static const long  HOME_BACKOFF_STEPS = 250;

// Control loop
static const uint32_t CTRL_HZ = 200;
static const float Ts = 1.0f / CTRL_HZ;

// Closed-loop velocity limit (deg/s)
static const float J1_VEL_LIMIT_DEG_S = 25.0f;
static const float J2_VEL_LIMIT_DEG_S = 35.0f;

// anti-windup
static const float EI_LIMIT = 30.0f;

// Default gains
static float J1_Kp = 8.0f,  J1_Kd = 2.5f, J1_Ki = 0.8f;
static float J2_Kp = 10.0f, J2_Kd = 2.0f, J2_Ki = 1.2f;

// =====================================================
//                    INTERNALS
// =====================================================

AccelStepper stJ1(AccelStepper::DRIVER, J1_STEP, J1_DIR);
AccelStepper stJ2(AccelStepper::DRIVER, J2_STEP, J2_DIR);
AccelStepper stZ (AccelStepper::DRIVER, Z_STEP , Z_DIR);

// Encoder zeros in motor-deg
float zeroMotorDegJ1 = 0.0f;
float zeroMotorDegJ2 = 0.0f;

// Encoder on motor shaft => need gear ratio motor/joint
static float ENC_R1 = 10.65f;
static float ENC_R2 = 18.90f;

JointCtrl J1, J2;
float z_mm_cmd = 0.0f;

static inline float wrapDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

bool tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  return Wire.endTransmission() == 0;
}

uint16_t as5600_readRaw() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E); // RAW ANGLE hi
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom((int)AS5600_ADDR, 2);
  if (Wire.available() < 2) return 0;
  uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
  return raw & 0x0FFF;
}

static inline float rawToMotorDeg(uint16_t raw) {
  return (raw * 360.0f) / 4096.0f;
}

float readJointDeg_J1() {
  tcaSelect(CH_J1);
  float m = rawToMotorDeg(as5600_readRaw());
  m = wrapDeg(m - zeroMotorDegJ1);
  return m / ENC_R1;
}
float readJointDeg_J2() {
  tcaSelect(CH_J2);
  float m = rawToMotorDeg(as5600_readRaw());
  m = wrapDeg(m - zeroMotorDegJ2);
  return m / ENC_R2;
}

static inline float distance2D(float x, float y) { return sqrtf(x*x + y*y); }
static inline float lawOfCosines(float a, float b, float c) {
  float v = (a*a + b*b - c*c) / (2.0f * a * b);
  if (v >  1.0f) v =  1.0f;
  if (v < -1.0f) v = -1.0f;
  return acosf(v);
}

// elbow: 0 normal, 1 inverse elbow solution (per PyBot)
bool IK_PyBot(float x, float y, uint8_t elbow, float &A1_deg, float &A2_deg) {
  if (elbow == 1) x = -x;
  float dist = distance2D(x, y);

  float maxReach = (L1 + L2);
  if (dist > maxReach) dist = maxReach - 0.001f;

  float D1 = atan2f(y, x);
  float D2 = lawOfCosines(dist, L1, L2);

  A1_deg = (D1 + D2 + AXIS_DEF_RAD) * 180.0f / PI;
  A2_deg = (lawOfCosines(L1, L2, dist) * 180.0f / PI) - 180.0f;

  if (elbow == 1) { A1_deg = -A1_deg; A2_deg = -A2_deg; }
  return true;
}

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ---------- Telemetry ----------
void sendTelemetry(float z_mm) {
  Serial.print("TLM,");
  Serial.print(J1.th, 3); Serial.print(",");
  Serial.print(J2.th, 3); Serial.print(",");
  Serial.print(J1.om, 3); Serial.print(",");
  Serial.print(J2.om, 3); Serial.print(",");
  Serial.print(z_mm, 2);  Serial.print(",");
  Serial.print(ES_PRESSED(ES_J1) ? 1 : 0); Serial.print(",");
  Serial.print(ES_PRESSED(ES_J2) ? 1 : 0); Serial.print(",");
  Serial.print(ES_PRESSED(ES_Z)  ? 1 : 0); Serial.print(",");
  Serial.print(ROT_SERVO_DEG); Serial.print(",");
  Serial.print(GRIP_SERVO_DEG);
  Serial.print("\n");
}

void updateJointStates() {
  float th1 = readJointDeg_J1();
  float th2 = readJointDeg_J2();
  float d1 = wrapDeg(th1 - J1.th_prev);
  float d2 = wrapDeg(th2 - J2.th_prev);
  J1.om = d1 / Ts;
  J2.om = d2 / Ts;
  J1.th = th1; J2.th = th2;
  J1.th_prev = th1; J2.th_prev = th2;
}

float computeVelCmd(JointCtrl &J) {
  float e = wrapDeg(J.ref - J.th);
  J.ei += e * Ts;
  if (J.ei > EI_LIMIT)  J.ei = EI_LIMIT;
  if (J.ei < -EI_LIMIT) J.ei = -EI_LIMIT;

  float u = (J.Kp * e) - (J.Kd * J.om) + (J.Ki * J.ei);
  if (u > J.vel_lim)  u = J.vel_lim;
  if (u < -J.vel_lim) u = -J.vel_lim;
  return u; // deg/s
}

// ---------- Homing ----------
void homeAxis(AccelStepper &st, int esPin, int dir, const char *name) {
  st.setAcceleration(1500);
  st.setMaxSpeed(HOME_SPEED_STEPS);

  while (!ES_PRESSED(esPin)) {
    st.setSpeed(dir * HOME_SPEED_STEPS);
    st.runSpeed();
  }
  st.setSpeed(0);
  delay(150);

  st.move(-dir * HOME_BACKOFF_STEPS);
  while (st.distanceToGo() != 0) st.run();
  delay(100);

  while (!ES_PRESSED(esPin)) {
    st.setSpeed(dir * (HOME_SPEED_STEPS * 0.4f));
    st.runSpeed();
  }
  st.setSpeed(0);
  delay(150);

  st.setCurrentPosition(0);
  Serial.print("OK,HOME_"); Serial.println(name);
}

void doZero() {
  tcaSelect(CH_J1); zeroMotorDegJ1 = rawToMotorDeg(as5600_readRaw());
  tcaSelect(CH_J2); zeroMotorDegJ2 = rawToMotorDeg(as5600_readRaw());
  J1.ei = 0; J2.ei = 0;
  Serial.println("OK,ZERO");
}

void doHomeAll() {
  homeAxis(stZ,  ES_Z,  HOME_DIR_Z,  "Z");
  homeAxis(stJ2, ES_J2, HOME_DIR_J2, "J2");
  homeAxis(stJ1, ES_J1, HOME_DIR_J1, "J1");
  doZero();
  Serial.println("OK,HOME_ALL");
}

// ---------- Serial ----------
String line;

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;

  if (s == "PING") { Serial.println("OK,PONG"); return; }
  if (s == "ZERO") { doZero(); return; }
  if (s == "HOME") { doHomeAll(); return; }

  // SERVO,rot_deg,grip_deg
  if (s.startsWith("SERVO,")) {
    int p1 = s.indexOf(',', 6);
    if (p1 < 0) { Serial.println("ERR,SERVO_FMT"); return; }
    int rot = s.substring(6, p1).toInt();
    int grp = s.substring(p1 + 1).toInt();
    rot = constrain(rot, 0, 180);
    grp = constrain(grp, 0, 180);
    ROT_SERVO_DEG = rot;
    GRIP_SERVO_DEG = grp;
    servoRot.write(ROT_SERVO_DEG);
    servoGrip.write(GRIP_SERVO_DEG);
    Serial.println("OK,SERVO");
    return;
  }

  // CMD,j1_deg,j2_deg,z_mm
  if (s.startsWith("CMD,")) {
    int p1 = s.indexOf(',', 4);
    int p2 = s.indexOf(',', p1 + 1);
    if (p1 < 0 || p2 < 0) return;

    float j1 = s.substring(4, p1).toFloat();
    float j2 = s.substring(p1 + 1, p2).toFloat();
    float z  = s.substring(p2 + 1).toFloat();

    j1 = clampf(j1, J1_MIN, J1_MAX);
    j2 = clampf(j2, J2_MIN, J2_MAX);
    z  = clampf(z,  Z_MIN_MM, Z_MAX_MM);

    J1.ref = j1;
    J2.ref = j2;
    z_mm_cmd = z;
    stZ.moveTo(lround(z_mm_cmd * Z_STEPS_PER_MM));

    Serial.println("OK,CMD");
    return;
  }

  // IK,x,y,z,elbow
  if (s.startsWith("IK,")) {
    int p1 = s.indexOf(',', 3);
    int p2 = s.indexOf(',', p1 + 1);
    int p3 = s.indexOf(',', p2 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0) return;

    float x = s.substring(3, p1).toFloat();
    float y = s.substring(p1 + 1, p2).toFloat();
    float z = s.substring(p2 + 1, p3).toFloat();
    uint8_t elbow = (uint8_t)s.substring(p3 + 1).toInt();

    float a1, a2;
    IK_PyBot(x, y, elbow, a1, a2);

    a1 = clampf(a1, J1_MIN, J1_MAX);
    a2 = clampf(a2, J2_MIN, J2_MAX);
    z  = clampf(z,  Z_MIN_MM, Z_MAX_MM);

    J1.ref = a1;
    J2.ref = a2;
    z_mm_cmd = z;
    stZ.moveTo(lround(z_mm_cmd * Z_STEPS_PER_MM));

    Serial.println("OK,IK");
    return;
  }

  // GAIN,axis,Kp,Kd,Ki
  if (s.startsWith("GAIN,")) {
    int p1 = s.indexOf(',', 5);
    int p2 = s.indexOf(',', p1 + 1);
    int p3 = s.indexOf(',', p2 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0) return;

    int axis = s.substring(5, p1).toInt();
    float Kp = s.substring(p1 + 1, p2).toFloat();
    float Kd = s.substring(p2 + 1, p3).toFloat();
    float Ki = s.substring(p3 + 1).toFloat();

    if (axis == 1) { J1.Kp = Kp; J1.Kd = Kd; J1.Ki = Ki; Serial.println("OK,GAIN1"); return; }
    if (axis == 2) { J2.Kp = Kp; J2.Kd = Kd; J2.Ki = Ki; Serial.println("OK,GAIN2"); return; }
    Serial.println("ERR,GAIN_AXIS");
    return;
  }

  // CONF,ENC_R1,ENC_R2,J1stepsDeg,J2stepsDeg,ZstepsMm
  if (s.startsWith("CONF,")) {
    int p1 = s.indexOf(',', 5);
    int p2 = s.indexOf(',', p1 + 1);
    int p3 = s.indexOf(',', p2 + 1);
    int p4 = s.indexOf(',', p3 + 1);
    int p5 = s.indexOf(',', p4 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0 || p5 < 0) { Serial.println("ERR,CONF_FMT"); return; }

    ENC_R1 = s.substring(5, p1).toFloat();
    ENC_R2 = s.substring(p1 + 1, p2).toFloat();
    J1_STEPS_PER_DEG = s.substring(p2 + 1, p3).toFloat();
    J2_STEPS_PER_DEG = s.substring(p3 + 1, p4).toFloat();
    Z_STEPS_PER_MM   = s.substring(p4 + 1, p5).toFloat();

    Serial.print("OK,CONF,");
    Serial.print(ENC_R1,3); Serial.print(",");
    Serial.print(ENC_R2,3); Serial.print(",");
    Serial.print(J1_STEPS_PER_DEG,3); Serial.print(",");
    Serial.print(J2_STEPS_PER_DEG,3); Serial.print(",");
    Serial.println(Z_STEPS_PER_MM,3);
    return;
  }

  Serial.println("ERR,UNKNOWN_CMD");
}

void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // LOW عادة يعني Enable، إذا عندك العكس خلّيها HIGH

  pinMode(ES_J1, INPUT);
  pinMode(ES_J2, INPUT);
  pinMode(ES_Z,  INPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  stJ1.setMaxSpeed(J1_MAX_SPEED_STEPS);
  stJ2.setMaxSpeed(J2_MAX_SPEED_STEPS);
  stZ.setMaxSpeed(Z_MAX_SPEED_STEPS);

  stJ1.setAcceleration(J1_ACC_STEPS);
  stJ2.setAcceleration(J2_ACC_STEPS);
  stZ.setAcceleration(Z_ACC_STEPS);

  // Servos
  servoRot.setPeriodHertz(50);
  servoGrip.setPeriodHertz(50);
  servoRot.attach(SERVO_ROT_PIN,  SERVO_MIN_US, SERVO_MAX_US);
  servoGrip.attach(SERVO_GRIP_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoRot.write(ROT_SERVO_DEG);
  servoGrip.write(GRIP_SERVO_DEG);

  // Controller init
  J1.Kp = J1_Kp; J1.Kd = J1_Kd; J1.Ki = J1_Ki; J1.vel_lim = J1_VEL_LIMIT_DEG_S;
  J2.Kp = J2_Kp; J2.Kd = J2_Kd; J2.Ki = J2_Ki; J2.vel_lim = J2_VEL_LIMIT_DEG_S;

  doZero();
  Serial.println("READY");
}

uint32_t lastCtrl = 0;
uint32_t lastTlm  = 0;

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') { handleLine(line); line = ""; }
    else line += c;
  }

  uint32_t now = millis();
  if (now - lastCtrl >= (1000 / CTRL_HZ)) {
    lastCtrl = now;

    updateJointStates();
    float v1_deg_s = computeVelCmd(J1);
    float v2_deg_s = computeVelCmd(J2);

    stJ1.setSpeed(v1_deg_s * J1_STEPS_PER_DEG);
    stJ2.setSpeed(v2_deg_s * J2_STEPS_PER_DEG);

    stJ1.runSpeed();
    stJ2.runSpeed();
    stZ.run();

    if (now - lastTlm >= 50) {
      lastTlm = now;
      float zmm = stZ.currentPosition() / Z_STEPS_PER_MM;
      sendTelemetry(zmm);
    }
  }
}