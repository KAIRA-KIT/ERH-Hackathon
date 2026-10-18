/*
  RIGID v2 — tendon arm + mecanum base (Seeed XIAO ESP32-C3)
  Control over USB serial (115200) or Bluetooth LE (Nordic UART, device name "RIGID").
  Calibration and settings are saved in flash, so the robot boots ready.

  ---------------- ARM ----------------
  servo_k [deg] = G1*b1*cos(phi1 - alpha_k) + (tip tendon ? G2*b2*cos(phi2 - alpha_k) : 0) + lock
  alpha_k = 43.6 + 60*k deg. Tendons 0,2,4 end mid-arm, 1,3,5 at the tip.

    S b1 phi1 [b2 phi2]   go to shape (deg)          L / R   lock / release
    A a0..a5              raw angles from zero (deg) X       STOP: motors off + arm torque off
    C                     centre all servos (2048)   E       arm torque back on
    P                     status
  Calibration (saved in flash):
    J k deg               jog servo k by deg (+ = wind in) — use to make each line just taut
    Q [soft]              SOFT-TENSION: all six wind in together at a low torque limit (default 80/1000),
                          so every line pulls equally and the arm hangs itself straight. Tap/shake it, then Z.
    Z                     SAVE ZERO: current positions become zero (restores the normal torque limit)
    Y                     telemetry line for the controller page: Y a0..a5 l0..l5 (deg from zero, load)
    H                     auto-home: wind each line until load rises, then save zero
    M id0 .. id5          servo bus IDs for tendon k = 0..5
    W 1|-1                wind direction
    G g1 g2               gains (deg servo per rad bend), default 202 115
    T n                   torque limit 50..1000 (default 300 ~ 0.6 A)
    I old new             change a servo's bus ID (only that servo on the bus!)
    K                     print settings      F  factory reset settings

  ---------------- BASE ----------------
    D vx vy w             drive, each -100..100 (vx forward, vy left, w counter-clockwise)
                          motors stop by themselves 400 ms after the last D command
    N s0 s1 s2 s3         motor direction fix (+1/-1) for FL FR RL RR, saved
    V pct                 max drive speed 10..100, saved

  WIRING (check!)
    Servo bus adapter: adapter TX -> D7 (GPIO20), adapter RX -> D6 (GPIO21), common GND, 12 V to servos.
    Motors: two L298N boards, ENA/ENB jumpers ON (enabled), PWM goes to the IN pins:
       FL: IN1 D0 (GPIO2)  IN2 D1 (GPIO3)      FR: IN3 D2 (GPIO4)  IN4 D3 (GPIO5)     <- L298N #1
       RL: IN1 D4 (GPIO6)  IN2 D5 (GPIO7)      RR: IN3 D8 (GPIO8)  IN4 D10 (GPIO10)   <- L298N #2
    L298N GND to XIAO GND. D9 is left free (BOOT button).
*/
#include <SCServo.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <stdarg.h>

// ---------------- pins
#define SERVO_RX 20   // D7
#define SERVO_TX 21   // D6
const int MOTOR_PIN[4][2] = {{2, 3}, {4, 5}, {6, 7}, {8, 10}};   // FL, FR, RL, RR : {IN_a, IN_b}
const int PWM_FREQ = 1000, PWM_BITS = 8;
const uint32_t DRIVE_TIMEOUT_MS = 400;

// ---------------- fixed model constants
const float STEPS_PER_DEG = 4096.0f / 360.0f;
const float ALPHA0_DEG = 43.6f;
const float B1_MAX_DEG = 140, B2_MAX_DEG = 160;
const float SERVO_LIMIT_DEG = 180;        // single-turn travel each way from zero
const float LOCK_WIND_DEG = 25;
const float MAX_SPEED_DEG_S = 400;
const int HOME_LOAD = 120;

// ---------------- settings (saved in flash)
struct Cfg {
  uint8_t id[6];
  int8_t windSign;
  float g1, g2;
  int torqueLimit;
  int16_t zero[6];      // raw servo steps at zero
  bool zeroValid;
  int8_t motorSign[4];
  int maxDrive;         // percent
} cfg;

void cfgDefaults() {
  for (int k = 0; k < 6; k++) { cfg.id[k] = k + 1; cfg.zero[k] = 2048; }
  cfg.windSign = 1; cfg.g1 = 202; cfg.g2 = 115; cfg.torqueLimit = 300; cfg.zeroValid = false;
  for (int m = 0; m < 4; m++) cfg.motorSign[m] = 1;
  cfg.maxDrive = 70;
}
Preferences prefs;
void cfgSave() { prefs.begin("rigid", false); prefs.putBytes("cfg", &cfg, sizeof(cfg)); prefs.end(); }
void cfgLoad() {
  cfgDefaults();
  prefs.begin("rigid", true);
  if (prefs.getBytesLength("cfg") == sizeof(cfg)) prefs.getBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
}

// ---------------- output to USB + BLE
#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
BLECharacteristic *txChar = nullptr;
volatile bool bleConnected = false;
volatile bool linkLost = false;
char rxRing[512];
volatile uint16_t rxHead = 0, rxTail = 0;

void outRaw(const char *msg) {
  Serial.print(msg);
  if (bleConnected && txChar) {
    size_t n = strlen(msg);
    for (size_t i = 0; i < n; i += 20) {
      size_t len = min((size_t)20, n - i);
      txChar->setValue((uint8_t *)(msg + i), len);
      txChar->notify();
      delay(4);
    }
  }
}
void out(const char *fmt, ...) {
  char buf[220]; va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  outRaw(buf);
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override { bleConnected = true; }
  void onDisconnect(BLEServer *s) override { bleConnected = false; linkLost = true; BLEDevice::startAdvertising(); }
};
class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    auto v = c->getValue();
    for (size_t i = 0; i < v.length(); i++) {
      uint16_t nx = (rxHead + 1) % sizeof(rxRing);
      if (nx == rxTail) break;
      rxRing[rxHead] = (char)v[i]; rxHead = nx;
    }
  }
};
void bleBegin() {
  BLEDevice::init("RIGID");
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new ServerCB());
  BLEService *svc = srv->createService(NUS_SERVICE);
  txChar = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());
  BLECharacteristic *rx = svc->createCharacteristic(NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// ================= ARM =================
SMS_STS st;
float curDeg[6] = {0};
float lockOffset = 0;
float shape[4] = {0, 0, 0, 0};
bool softMode = false;

float alphaRad(int k) { return (ALPHA0_DEG + 60.0f * k) * DEG_TO_RAD; }
bool endsAtTip(int k) { return k % 2 == 1; }
s16 stepsFor(int k, float deg) { return (s16)lroundf(cfg.zero[k] + cfg.windSign * deg * STEPS_PER_DEG); }
float degNow(int k) { int p = st.ReadPos(cfg.id[k]); return p < 0 ? NAN : cfg.windSign * (p - cfg.zero[k]) / STEPS_PER_DEG; }

void shapeToServos(const float s[4], float lock, float o[6]) {
  float b1 = s[0] * DEG_TO_RAD, p1 = s[1] * DEG_TO_RAD, b2 = s[2] * DEG_TO_RAD, p2 = s[3] * DEG_TO_RAD;
  for (int k = 0; k < 6; k++) {
    float a = alphaRad(k), th = cfg.g1 * b1 * cosf(p1 - a);
    if (endsAtTip(k)) th += cfg.g2 * b2 * cosf(p2 - a);
    o[k] = th + lock;
  }
}
float fitTravel(float s[4], float lock) {
  float o[6]; shapeToServos(s, lock, o);
  float worst = 0; for (int k = 0; k < 6; k++) worst = max(worst, fabsf(o[k]));
  if (worst <= SERVO_LIMIT_DEG) return 1.0f;
  float scale = constrain((SERVO_LIMIT_DEG - fabsf(lock)) / (worst - fabsf(lock)), 0.0f, 1.0f);
  s[0] *= scale; s[2] *= scale; return scale;
}
void moveTo(const float target[6]) {
  u8 ids[6]; s16 pos[6]; u16 spd[6]; u8 acc[6];
  float maxD = 0; for (int k = 0; k < 6; k++) maxD = max(maxD, fabsf(target[k] - curDeg[k]));
  if (maxD < 0.3f) return;
  float T = maxD / MAX_SPEED_DEG_S;
  for (int k = 0; k < 6; k++) {
    float t = constrain(target[k], -SERVO_LIMIT_DEG, SERVO_LIMIT_DEG);
    ids[k] = cfg.id[k]; pos[k] = stepsFor(k, t);
    spd[k] = (u16)max(20.0f, fabsf(t - curDeg[k]) / T * STEPS_PER_DEG);
    acc[k] = 50; curDeg[k] = t;
  }
  st.SyncWritePosEx(ids, 6, pos, spd, acc);
}
void applyShape() {
  float s[4] = {constrain(shape[0], 0.0f, B1_MAX_DEG), shape[1], constrain(shape[2], 0.0f, B2_MAX_DEG), shape[3]};
  float scale = fitTravel(s, lockOffset);
  float t[6]; shapeToServos(s, lockOffset, t); moveTo(t);
  if (scale < 0.999f) out("WARN travel limit: bend scaled to %.0f%%\n", scale * 100);
  out("OK shape %.1f %.1f %.1f %.1f lock=%.0f\n", s[0], s[1], s[2], s[3], lockOffset);
}
void servosInit() {
  for (int k = 0; k < 6; k++) {
    int id = cfg.id[k];
    if (st.Ping(id) < 0) { out("ERR no servo at ID %d (tendon %d)\n", id, k); continue; }
    st.writeWord(id, SMS_STS_TORQUE_LIMIT_L, cfg.torqueLimit);
    st.EnableTorque(id, 1);
    float d = degNow(k); curDeg[k] = isnan(d) ? 0 : d;
  }
}
void softTension(int soft) {
  soft = constrain(soft, 20, 300);
  for (int k = 0; k < 6; k++) {
    int id = cfg.id[k], p = st.ReadPos(id);
    if (p < 0) { out("ERR no servo at ID %d\n", id); continue; }
    st.writeWord(id, SMS_STS_TORQUE_LIMIT_L, soft);
    st.EnableTorque(id, 1);
    s16 goal = (s16)constrain(p + cfg.windSign * 90 * STEPS_PER_DEG, 0, 4095);   // aim past taut: the low torque limit stops it
    st.WritePosEx(id, goal, 300, 30);
  }
  softMode = true;
  out("OK soft tension %d on all six. Let it settle (tap/shake the arm), then Z to save zero\n", soft);
}
void telemetry() {
  char buf[200]; int n = snprintf(buf, sizeof(buf), "Y");
  for (int k = 0; k < 6; k++) { float d = degNow(k); n += snprintf(buf + n, sizeof(buf) - n, " %.1f", isnan(d) ? 0.0f : d); }
  for (int k = 0; k < 6; k++) n += snprintf(buf + n, sizeof(buf) - n, " %d", st.ReadLoad(cfg.id[k]));
  snprintf(buf + n, sizeof(buf) - n, "\n");
  outRaw(buf);
}
void saveZeroHere() {
  for (int k = 0; k < 6; k++) { int p = st.ReadPos(cfg.id[k]); if (p >= 0) cfg.zero[k] = p; curDeg[k] = 0; }
  cfg.zeroValid = true; lockOffset = 0; for (int i = 0; i < 4; i++) shape[i] = 0;
  cfgSave();
  for (int k = 0; k < 6; k++) {                               // hold here at normal torque
    st.WritePosEx(cfg.id[k], cfg.zero[k], 300, 30);
    st.writeWord(cfg.id[k], SMS_STS_TORQUE_LIMIT_L, cfg.torqueLimit);
  }
  softMode = false;
  out("OK zero saved: %d %d %d %d %d %d\n", cfg.zero[0], cfg.zero[1], cfg.zero[2], cfg.zero[3], cfg.zero[4], cfg.zero[5]);
}
void autoHome() {
  out("Homing: arm must hang straight and still\n");
  for (int k = 0; k < 6; k++) {
    int id = cfg.id[k]; float p = st.ReadPos(id);
    if (p < 0) { out("ERR no servo at ID %d\n", id); continue; }
    bool taut = false;
    for (int i = 0; i < 180 && !taut; i++) {               // up to 360 deg in 2 deg steps
      p += cfg.windSign * 2 * STEPS_PER_DEG;
      st.WritePosEx(id, (s16)p, 600, 50); delay(25);
      taut = abs(st.ReadLoad(id)) > HOME_LOAD;
    }
    out("  tendon %d %s\n", k, taut ? "taut" : "NOT taut (check line)");
  }
  saveZeroHere();
}
void status() {
  for (int k = 0; k < 6; k++) {
    int id = cfg.id[k];
    out("k%d id%d cmd=%.0f act=%.0f load=%d\n", k, id, curDeg[k], degNow(k), st.ReadLoad(id));
  }
}
void printCfg() {
  out("ids %d %d %d %d %d %d | wind %d | gains %.0f %.0f | torque %d\n", cfg.id[0], cfg.id[1], cfg.id[2], cfg.id[3], cfg.id[4], cfg.id[5], cfg.windSign, cfg.g1, cfg.g2, cfg.torqueLimit);
  out("zero %s %d %d %d %d %d %d\n", cfg.zeroValid ? "saved" : "NOT SET", cfg.zero[0], cfg.zero[1], cfg.zero[2], cfg.zero[3], cfg.zero[4], cfg.zero[5]);
  out("motors %d %d %d %d | max drive %d%%\n", cfg.motorSign[0], cfg.motorSign[1], cfg.motorSign[2], cfg.motorSign[3], cfg.maxDrive);
}

// ================= BASE (mecanum) =================
int motorDir[4] = {0, 0, 0, 0};     // which IN pin carries PWM: 0 none, 1 = pin a, -1 = pin b
uint32_t lastDrive = 0;
bool driving = false;

void motorSet(int m, float v) {             // v -1..1
  v = constrain(v * cfg.motorSign[m], -1.0f, 1.0f);
  int duty = (int)(fabsf(v) * 255);
  int dir = duty < 8 ? 0 : (v > 0 ? 1 : -1);
  int pa = MOTOR_PIN[m][0], pb = MOTOR_PIN[m][1];
  if (dir != motorDir[m]) {                 // C3 has only 6 PWM channels: move the channel to the active pin
    if (motorDir[m] == 1) ledcDetach(pa);
    if (motorDir[m] == -1) ledcDetach(pb);
    pinMode(pa, OUTPUT); pinMode(pb, OUTPUT); digitalWrite(pa, LOW); digitalWrite(pb, LOW);
    if (dir == 1) ledcAttachChannel(pa, PWM_FREQ, PWM_BITS, m);
    if (dir == -1) ledcAttachChannel(pb, PWM_FREQ, PWM_BITS, m);
    motorDir[m] = dir;
  }
  if (dir == 1) ledcWrite(pa, duty);
  if (dir == -1) ledcWrite(pb, duty);
}
void driveStop() { for (int m = 0; m < 4; m++) motorSet(m, 0); driving = false; }
void drive(float vx, float vy, float w) {   // inputs -100..100
  float s = cfg.maxDrive / 100.0f / 100.0f;
  vx *= s; vy *= s; w *= s;
  float fl = vx - vy - w, fr = vx + vy + w, rl = vx + vy - w, rr = vx - vy + w;
  float mx = max(max(fabsf(fl), fabsf(fr)), max(fabsf(rl), fabsf(rr)));
  if (mx > 1) { fl /= mx; fr /= mx; rl /= mx; rr /= mx; }
  motorSet(0, fl); motorSet(1, fr); motorSet(2, rl); motorSet(3, rr);
  driving = mx > 0.01f; lastDrive = millis();
}

// ================= commands =================
void handleLine(String line) {
  line.trim();
  if (!line.length()) return;
  char c = toupper(line[0]);
  float v[6] = {0};
  int n = sscanf(line.c_str() + 1, "%f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]);
  switch (c) {
    // ---- base
    case 'D': if (n == 3) drive(v[0], v[1], v[2]); else driveStop(); break;     // no reply: sent many times a second
    case 'N': if (n == 4) { for (int m = 0; m < 4; m++) cfg.motorSign[m] = v[m] < 0 ? -1 : 1; cfgSave(); out("OK motor signs saved\n"); } break;
    case 'V': if (n == 1) { cfg.maxDrive = constrain((int)v[0], 10, 100); cfgSave(); out("OK max drive %d%%\n", cfg.maxDrive); } break;
    // ---- arm
    case 'S': if (softMode) { out("ERR in soft-tension mode: press Z first\n"); break; }
              shape[0] = v[0]; shape[1] = v[1]; shape[2] = n >= 4 ? v[2] : 0; shape[3] = n >= 4 ? v[3] : 0; applyShape(); break;
    case 'A': if (n == 6) { moveTo(v); out("OK raw\n"); } else out("ERR A needs 6 angles\n"); break;
    case 'L': lockOffset = LOCK_WIND_DEG; applyShape(); break;
    case 'R': lockOffset = 0; applyShape(); break;
    case 'C': for (int k = 0; k < 6; k++) st.WritePosEx(cfg.id[k], 2048, 800, 50); out("OK centred at 2048. Tie the lines on, jog each taut (J), then Z\n"); break;
    case 'J': if (n == 2 && v[0] >= 0 && v[0] < 6) {
                int k = (int)v[0], p = st.ReadPos(cfg.id[k]);
                if (p < 0) { out("ERR servo %d not answering\n", k); break; }
                s16 np = (s16)lroundf(p + cfg.windSign * v[1] * STEPS_PER_DEG);
                st.WritePosEx(cfg.id[k], np, 600, 50); delay(150);
                out("OK jog k%d -> %d steps, load %d\n", k, (int)np, st.ReadLoad(cfg.id[k]));
              } else out("ERR J needs servo 0..5 and degrees\n"); break;
    case 'Q': softTension(n >= 1 ? (int)v[0] : 80); break;
    case 'Z': saveZeroHere(); break;
    case 'Y': telemetry(); break;
    case 'H': autoHome(); break;
    case 'P': status(); break;
    case 'X': driveStop(); for (int k = 0; k < 6; k++) st.EnableTorque(cfg.id[k], 0); out("OK STOP: motors off, arm torque off (E = torque on)\n"); break;
    case 'E': servosInit(); out("OK arm torque on\n"); break;
    // ---- settings
    case 'M': if (n == 6) { for (int k = 0; k < 6; k++) cfg.id[k] = (uint8_t)v[k]; cfgSave(); servosInit(); out("OK servo IDs saved\n"); } else out("ERR M needs 6 IDs\n"); break;
    case 'W': if (n == 1) { cfg.windSign = v[0] < 0 ? -1 : 1; cfgSave(); out("OK wind %d (re-do zero)\n", cfg.windSign); } break;
    case 'G': if (n == 2) { cfg.g1 = v[0]; cfg.g2 = v[1]; cfgSave(); out("OK gains %.0f %.0f\n", cfg.g1, cfg.g2); } break;
    case 'T': if (n == 1) { cfg.torqueLimit = constrain((int)v[0], 50, 1000); cfgSave(); servosInit(); out("OK torque limit %d\n", cfg.torqueLimit); } break;
    case 'I': if (n == 2) {
                int o = (int)v[0], nw = (int)v[1];
                if (st.Ping(o) < 0) { out("ERR no servo at ID %d\n", o); break; }
                st.unLockEprom(o); st.writeByte(o, SMS_STS_ID, nw); st.LockEprom(nw);
                if (st.Ping(nw) >= 0) out("OK servo ID %d -> %d\n", o, nw); else out("ERR ID change %d -> %d not confirmed\n", o, nw);
              } break;
    case 'K': printCfg(); break;
    case 'F': cfgDefaults(); cfgSave(); out("OK settings reset to defaults\n"); printCfg(); break;
    default: out("ERR unknown command '%c'\n", c);
  }
}

void setup() {
  Serial.begin(115200);
  for (int m = 0; m < 4; m++) { pinMode(MOTOR_PIN[m][0], OUTPUT); pinMode(MOTOR_PIN[m][1], OUTPUT); digitalWrite(MOTOR_PIN[m][0], LOW); digitalWrite(MOTOR_PIN[m][1], LOW); }
  cfgLoad();
  bleBegin();
  Serial1.begin(1000000, SERIAL_8N1, SERVO_RX, SERVO_TX);
  st.pSerial = &Serial1;
  delay(500);
  servosInit();
  out("RIGID v2.1 ready. %s\n", cfg.zeroValid ? "Zero loaded from flash." : "No zero saved yet: C, tie lines, J to tension, then Z.");
}

String usbRx, bleLine;
void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (usbRx.length()) handleLine(usbRx); usbRx = ""; } else usbRx += ch;
  }
  while (rxTail != rxHead) {
    char ch = rxRing[rxTail]; rxTail = (rxTail + 1) % sizeof(rxRing);
    if (ch == '\n' || ch == '\r') { if (bleLine.length()) handleLine(bleLine); bleLine = ""; } else bleLine += ch;
  }
  if (linkLost) { linkLost = false; driveStop(); }                         // Bluetooth dropped
  if (driving && millis() - lastDrive > DRIVE_TIMEOUT_MS) driveStop();    // key released / link stalled
  delay(2);
}
