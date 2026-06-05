/**
 * Robot Car - SM-ESP32 + L298N + Gas Sensor (IO36)
 * Control source: ThingsBoard Shared Attributes
 *
 * Modes:
 * - manual: XY control (x, y, duration timeout)
 * - auto  : scheduled mission - go to target, scan gas 360, return to (0,0)
 *
 * LittleFS stores: auto schedule (target, run_at_epoch, enabled)
 * NTP only used to get current time - NOT cached.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <LittleFS.h>
#include <time.h>
#include <sys/time.h>

// -------------------------------------------------------------------
// WiFi + ThingsBoard
// -------------------------------------------------------------------
const char* WIFI_SSID     = "TDTuan";
const char* WIFI_PASSWORD = "12345678";

const char* TB_SERVER = "thingsboard.cloud";
const int   TB_PORT   = 1883;
const char* TB_TOKEN  = "8kG78bmcD7x0h1hBPk1r";

// -------------------------------------------------------------------
// Pin mapping (L298N)
// -------------------------------------------------------------------
#define PIN_ENA   32
#define PIN_IN1   33
#define PIN_IN2   25

#define PIN_ENB   14
#define PIN_IN3   26
#define PIN_IN4   27

#define PIN_GAS   36

// -------------------------------------------------------------------
// PWM config
// -------------------------------------------------------------------
#define PWM_FREQ      1000
#define PWM_RES       8
#define PWM_CH_RIGHT  0
#define PWM_CH_LEFT   1

#define MAX_PWM_DUTY   102   // 40% of 255
#define TURN_PWM_DUTY   90

// -------------------------------------------------------------------
// Timing
// -------------------------------------------------------------------
#define TELEMETRY_INTERVAL_MS    3000UL
#define ATTR_REQUEST_INTERVAL_MS 15000UL
#define TIME_SYNC_INTERVAL_MS  3600000UL
#define GAS_SCAN_SAMPLE_MS        200UL
#define GAS_ALERT_THRESHOLD        700

// -------------------------------------------------------------------
// Calibration defaults
// -------------------------------------------------------------------
float gMoveMsPerCmX = 24.0f;
float gMoveMsPerCmY = 24.0f;
uint32_t gTurn90Ms  = 420;
uint32_t gScan360Ms = 1700;

const int MAP_SIZE_CM = 300;

// -------------------------------------------------------------------
// Types
// -------------------------------------------------------------------
enum ControlMode { MODE_MANUAL = 0, MODE_AUTO = 1 };

enum AutoState {
  AUTO_DISABLED = 0,
  AUTO_WAITING_TIME,
  AUTO_MOVING_TO_TARGET,
  AUTO_SCANNING_GAS,
  AUTO_RETURNING_HOME,
  AUTO_FINISHED
};

enum Heading { HEAD_NORTH = 0, HEAD_EAST = 1, HEAD_SOUTH = 2, HEAD_WEST = 3 };

struct AutoConfig {
  bool enabled;
  int targetX;
  int targetY;
  uint32_t runAtEpoch;  // 0 = run immediately
};

struct Pose {
  int x;
  int y;
  Heading heading;
};

struct MotionStep {
  int leftDuty;
  int rightDuty;
  uint32_t durationMs;
  int deltaX;
  int deltaY;
  int deltaHeading;
};

// -------------------------------------------------------------------
// Globals
// -------------------------------------------------------------------
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

ControlMode gMode = MODE_MANUAL;
AutoState gAutoState = AUTO_DISABLED;
AutoConfig gAutoCfg = {false, 0, 0, 0};

int gCurrentX = 0;
int gCurrentY = 0;
bool gManualAutoStop = false;
unsigned long gManualStopAt = 0;

Pose gPose = {0, 0, HEAD_NORTH};
bool gMissionActive = false;

MotionStep gQueue[32];
uint8_t gQueueCount = 0;
uint8_t gQueueIndex = 0;
bool gStepRunning = false;
unsigned long gStepEndsAt = 0;

bool gScanRunning = false;
unsigned long gScanStartedAt = 0;
unsigned long gLastGasSampleAt = 0;
int gPeakGasRaw = -1;
int gPeakGasAngleDeg = 0;
bool gGasAboveThreshold = false;

unsigned long gLastTelemetry = 0;
unsigned long gLastAttrReq = 0;
unsigned long gLastTimeSync = 0;

bool gTimeValid = false;

const char* SCHEDULE_FILE = "/auto_schedule.json";

// -------------------------------------------------------------------
// Forward declarations
// -------------------------------------------------------------------
void stopMotors();
void publishTelemetry(bool force = false);
void publishEvent(const char* eventName, int gasRaw = -1, int angle = -1);
void cancelAutoMission();
void startAutoMissionNow();
void saveScheduleToFS();
void loadScheduleFromFS();

// -------------------------------------------------------------------
// LEDC compatibility
// -------------------------------------------------------------------
void initPwmPin(uint8_t pin, uint8_t channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttachChannel(pin, PWM_FREQ, PWM_RES, channel);
#else
  ledcSetup(channel, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, channel);
#endif
}

void writePwmRight(uint8_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PIN_ENA, duty);
#else
  ledcWrite(PWM_CH_RIGHT, duty);
#endif
}

void writePwmLeft(uint8_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(PIN_ENB, duty);
#else
  ledcWrite(PWM_CH_LEFT, duty);
#endif
}

// -------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------
const char* modeToText(ControlMode m) { return (m == MODE_AUTO) ? "auto" : "manual"; }

const char* autoStateToText(AutoState st) {
  switch (st) {
    case AUTO_DISABLED: return "disabled";
    case AUTO_WAITING_TIME: return "waiting_time";
    case AUTO_MOVING_TO_TARGET: return "moving_to_target";
    case AUTO_SCANNING_GAS: return "scanning_gas";
    case AUTO_RETURNING_HOME: return "returning_home";
    case AUTO_FINISHED: return "finished";
    default: return "unknown";
  }
}

int clampDuty(int duty) { return constrain(duty, -MAX_PWM_DUTY, MAX_PWM_DUTY); }

uint32_t nowEpoch() {
  time_t t = time(nullptr);
  if (t > 1700000000) { gTimeValid = true; return (uint32_t)t; }
  return 0;
}

// -------------------------------------------------------------------
// NTP (chi lay thoi gian, khong luu)
// -------------------------------------------------------------------
void syncTimeNtp(bool force = false) {
  unsigned long nowMs = millis();
  if (!force && (nowMs - gLastTimeSync < TIME_SYNC_INTERVAL_MS)) return;
  if (WiFi.status() != WL_CONNECTED) return;

  configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");

  struct tm tmInfo;
  if (getLocalTime(&tmInfo, 3000)) {
    gTimeValid = true;
    gLastTimeSync = nowMs;
    Serial.printf("[NTP] Synced, epoch=%lu\n", (unsigned long)nowEpoch());
  } else {
    Serial.println("[NTP] Sync failed");
  }
}

// -------------------------------------------------------------------
// LittleFS: luu/doc lich hen gio auto
// -------------------------------------------------------------------
void saveScheduleToFS() {
  File f = LittleFS.open(SCHEDULE_FILE, "w");
  if (!f) { Serial.println("[FS] Save failed"); return; }

  StaticJsonDocument<128> doc;
  doc["enabled"]      = gAutoCfg.enabled;
  doc["target_x"]     = gAutoCfg.targetX;
  doc["target_y"]     = gAutoCfg.targetY;
  doc["run_at_epoch"] = gAutoCfg.runAtEpoch;

  serializeJson(doc, f);
  f.close();
  Serial.printf("[FS] Schedule saved: en=%d tx=%d ty=%d epoch=%lu\n",
    gAutoCfg.enabled, gAutoCfg.targetX, gAutoCfg.targetY, (unsigned long)gAutoCfg.runAtEpoch);
}

void loadScheduleFromFS() {
  if (!LittleFS.exists(SCHEDULE_FILE)) return;

  File f = LittleFS.open(SCHEDULE_FILE, "r");
  if (!f) return;

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return;

  gAutoCfg.enabled    = doc["enabled"] | false;
  gAutoCfg.targetX    = constrain(doc["target_x"] | 0, 0, MAP_SIZE_CM);
  gAutoCfg.targetY    = constrain(doc["target_y"] | 0, 0, MAP_SIZE_CM);
  gAutoCfg.runAtEpoch = doc["run_at_epoch"] | 0;

  Serial.printf("[FS] Schedule loaded: en=%d tx=%d ty=%d epoch=%lu\n",
    gAutoCfg.enabled, gAutoCfg.targetX, gAutoCfg.targetY, (unsigned long)gAutoCfg.runAtEpoch);

  // Neu co lich hen va dang enabled thi chuyen sang auto mode de cho
  if (gAutoCfg.enabled && gAutoCfg.runAtEpoch > 0) {
    gMode = MODE_AUTO;
    gAutoState = AUTO_WAITING_TIME;
    Serial.println("[FS] Auto mode resumed from saved schedule");
  }
}

void clearScheduleFS() {
  if (LittleFS.exists(SCHEDULE_FILE)) {
    LittleFS.remove(SCHEDULE_FILE);
    Serial.println("[FS] Schedule file removed");
  }
}

// -------------------------------------------------------------------
// Motor control
// -------------------------------------------------------------------
void setMotorRightDuty(int duty) {
  duty = clampDuty(duty);
  if (duty > 0) {
    digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
    writePwmRight((uint8_t)duty);
  } else if (duty < 0) {
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH);
    writePwmRight((uint8_t)(-duty));
  } else {
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    writePwmRight(0);
  }
}

void setMotorLeftDuty(int duty) {
  duty = clampDuty(duty);
  if (duty > 0) {
    digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
    writePwmLeft((uint8_t)duty);
  } else if (duty < 0) {
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, HIGH);
    writePwmLeft((uint8_t)(-duty));
  } else {
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
    writePwmLeft(0);
  }
}

void stopMotors() {
  setMotorLeftDuty(0);
  setMotorRightDuty(0);
  gCurrentX = 0;
  gCurrentY = 0;
  gManualAutoStop = false;
}

void driveXY(int x, int y) {
  x = constrain(x, -100, 100);
  y = constrain(y, -100, 100);
  int leftDuty  = (constrain(x + y, -100, 100) * MAX_PWM_DUTY) / 100;
  int rightDuty = (constrain(x - y, -100, 100) * MAX_PWM_DUTY) / 100;
  setMotorLeftDuty(leftDuty);
  setMotorRightDuty(rightDuty);
  gCurrentX = x;
  gCurrentY = y;
}

// -------------------------------------------------------------------
// Motion queue
// -------------------------------------------------------------------
bool queuePush(int l, int r, uint32_t ms, int dx, int dy, int dh) {
  if (gQueueCount >= 32) return false;
  gQueue[gQueueCount] = {clampDuty(l), clampDuty(r), ms, dx, dy, dh};
  gQueueCount++;
  return true;
}

void queueClear() { gQueueCount = 0; gQueueIndex = 0; gStepRunning = false; }

void queueAddTurnRight90() { queuePush(TURN_PWM_DUTY, -TURN_PWM_DUTY, gTurn90Ms, 0, 0, +1); }
void queueAddTurnLeft90()  { queuePush(-TURN_PWM_DUTY, TURN_PWM_DUTY, gTurn90Ms, 0, 0, -1); }

void queueTurnTo(Heading target) {
  int diff = ((int)target - (int)gPose.heading + 4) % 4;
  if (diff == 0) return;
  if (diff == 1) queueAddTurnRight90();
  else if (diff == 3) queueAddTurnLeft90();
  else { queueAddTurnRight90(); queueAddTurnRight90(); }
}

void queueAddForwardCm(int cm) {
  if (cm <= 0) return;
  float msPerCm = (gPose.heading == HEAD_EAST || gPose.heading == HEAD_WEST) ? gMoveMsPerCmX : gMoveMsPerCmY;
  uint32_t dur = (uint32_t)(cm * msPerCm);
  if (dur < 1) dur = 1;

  int dx = 0, dy = 0;
  switch (gPose.heading) {
    case HEAD_NORTH: dy = cm; break;
    case HEAD_EAST:  dx = cm; break;
    case HEAD_SOUTH: dy = -cm; break;
    case HEAD_WEST:  dx = -cm; break;
  }
  queuePush(MAX_PWM_DUTY, MAX_PWM_DUTY, dur, dx, dy, 0);
}

void applyStepToPose(const MotionStep& st) {
  gPose.x = constrain(gPose.x + st.deltaX, 0, MAP_SIZE_CM);
  gPose.y = constrain(gPose.y + st.deltaY, 0, MAP_SIZE_CM);
  int hd = ((int)gPose.heading + st.deltaHeading + 4) % 4;
  gPose.heading = (Heading)hd;
}

bool processQueue() {
  if (!gStepRunning && gQueueIndex < gQueueCount) {
    const MotionStep& st = gQueue[gQueueIndex];
    setMotorLeftDuty(st.leftDuty);
    setMotorRightDuty(st.rightDuty);
    gStepEndsAt = millis() + st.durationMs;
    gStepRunning = true;
  }

  if (gStepRunning && (long)(millis() - gStepEndsAt) >= 0) {
    stopMotors();
    applyStepToPose(gQueue[gQueueIndex]);
    gQueueIndex++;
    gStepRunning = false;
  }

  return (!gStepRunning && gQueueIndex >= gQueueCount);
}

// -------------------------------------------------------------------
// Auto mission planner
// -------------------------------------------------------------------
void planPathToPoint(int dstX, int dstY) {
  queueClear();
  Pose saved = gPose;

  int dx = dstX - gPose.x;
  int dy = dstY - gPose.y;

  if (dy > 0) { queueTurnTo(HEAD_NORTH); gPose.heading = HEAD_NORTH; queueAddForwardCm(dy); gPose.y += dy; }
  else if (dy < 0) { queueTurnTo(HEAD_SOUTH); gPose.heading = HEAD_SOUTH; queueAddForwardCm(-dy); gPose.y += dy; }

  if (dx > 0) { queueTurnTo(HEAD_EAST); gPose.heading = HEAD_EAST; queueAddForwardCm(dx); gPose.x += dx; }
  else if (dx < 0) { queueTurnTo(HEAD_WEST); gPose.heading = HEAD_WEST; queueAddForwardCm(-dx); gPose.x += dx; }

  gPose = saved;
}

void beginGasScan() {
  gScanRunning = true;
  gScanStartedAt = millis();
  gLastGasSampleAt = 0;
  gPeakGasRaw = -1;
  gPeakGasAngleDeg = 0;
  setMotorLeftDuty(TURN_PWM_DUTY);
  setMotorRightDuty(-TURN_PWM_DUTY);
  gAutoState = AUTO_SCANNING_GAS;
  publishEvent("arrived_target", analogRead(PIN_GAS), -1);
}

void processGasScan() {
  if (!gScanRunning) return;
  unsigned long nowMs = millis();
  unsigned long elapsed = nowMs - gScanStartedAt;

  if (nowMs - gLastGasSampleAt >= GAS_SCAN_SAMPLE_MS) {
    gLastGasSampleAt = nowMs;
    int gas = analogRead(PIN_GAS);
    int angle = (int)((elapsed * 360UL) / gScan360Ms) % 360;
    if (gas > gPeakGasRaw) { gPeakGasRaw = gas; gPeakGasAngleDeg = angle; }
    publishEvent("scan_sample", gas, angle);
  }

  if (elapsed >= gScan360Ms) {
    stopMotors();
    gScanRunning = false;
    publishEvent("scan_done", gPeakGasRaw, gPeakGasAngleDeg);
    planPathToPoint(0, 0);
    gAutoState = AUTO_RETURNING_HOME;
  }
}

void startAutoMissionNow() {
  if (gMode != MODE_AUTO) return;
  gMissionActive = true;
  gAutoState = AUTO_MOVING_TO_TARGET;
  gPose = {0, 0, HEAD_NORTH};
  int tx = constrain(gAutoCfg.targetX, 0, MAP_SIZE_CM);
  int ty = constrain(gAutoCfg.targetY, 0, MAP_SIZE_CM);
  planPathToPoint(tx, ty);
  publishEvent("auto_started", analogRead(PIN_GAS), -1);
}

void cancelAutoMission() {
  gMissionActive = false;
  gScanRunning = false;
  queueClear();
  gAutoState = AUTO_DISABLED;
  stopMotors();
}

void processAutoStateMachine() {
  if (gMode != MODE_AUTO) return;
  if (!gAutoCfg.enabled) {
    if (gAutoState != AUTO_DISABLED) cancelAutoMission();
    return;
  }

  if (!gMissionActive) {
    if (gAutoCfg.runAtEpoch == 0) { startAutoMissionNow(); return; }
    gAutoState = AUTO_WAITING_TIME;
    uint32_t epoch = nowEpoch();
    if (epoch > 0 && epoch >= gAutoCfg.runAtEpoch) { startAutoMissionNow(); }
    return;
  }

  if (gAutoState == AUTO_MOVING_TO_TARGET) {
    if (processQueue()) beginGasScan();
  } else if (gAutoState == AUTO_SCANNING_GAS) {
    processGasScan();
  } else if (gAutoState == AUTO_RETURNING_HOME) {
    if (processQueue()) {
      stopMotors();
      gMissionActive = false;
      gAutoState = AUTO_FINISHED;
      gAutoCfg.enabled = false;
      clearScheduleFS();
      publishEvent("returned_home", analogRead(PIN_GAS), -1);
    }
  }
}

// -------------------------------------------------------------------
// MQTT / Attributes
// -------------------------------------------------------------------
void applyAttributeJson(JsonVariant data) {
  if (data.isNull()) return;

  // Mode
  if (data.containsKey("mode")) {
    const char* m = data["mode"] | "manual";
    if (strcmp(m, "auto") == 0) { gMode = MODE_AUTO; }
    else { gMode = MODE_MANUAL; cancelAutoMission(); }
    Serial.printf("[ATTR] mode=%s\n", modeToText(gMode));
  }

  // Stop
  if ((bool)(data["stop"] | false)) {
    stopMotors();
    cancelAutoMission();
    clearScheduleFS();
    Serial.println("[ATTR] stop");
    return;
  }

  // Calibration
  if (data.containsKey("move_ms_cm_x")) gMoveMsPerCmX = data["move_ms_cm_x"].as<float>();
  if (data.containsKey("move_ms_cm_y")) gMoveMsPerCmY = data["move_ms_cm_y"].as<float>();
  if (data.containsKey("turn_90_ms"))   gTurn90Ms = data["turn_90_ms"] | gTurn90Ms;
  if (data.containsKey("scan_360_ms"))  gScan360Ms = data["scan_360_ms"] | gScan360Ms;

  // Manual XY
  if (gMode == MODE_MANUAL && (data.containsKey("x") || data.containsKey("y"))) {
    int x = data["x"] | gCurrentX;
    int y = data["y"] | gCurrentY;
    int duration = data["duration"] | 0;
    driveXY(x, y);
    if (duration > 0) { gManualAutoStop = true; gManualStopAt = millis() + (unsigned long)duration; }
    else { gManualAutoStop = false; }
    Serial.printf("[MANUAL] x=%d y=%d dur=%d\n", x, y, duration);
  }

  // Auto config
  bool scheduleChanged = false;

  if (data.containsKey("auto_enabled")) {
    gAutoCfg.enabled = data["auto_enabled"] | false;
    if (!gAutoCfg.enabled) { cancelAutoMission(); clearScheduleFS(); }
    scheduleChanged = true;
  }
  if (data.containsKey("target_x")) {
    gAutoCfg.targetX = constrain(data["target_x"].as<int>(), 0, MAP_SIZE_CM);
    scheduleChanged = true;
  }
  if (data.containsKey("target_y")) {
    gAutoCfg.targetY = constrain(data["target_y"].as<int>(), 0, MAP_SIZE_CM);
    scheduleChanged = true;
  }
  if (data.containsKey("run_at_epoch")) {
    gAutoCfg.runAtEpoch = data["run_at_epoch"] | 0;
    scheduleChanged = true;
  }

  // Luu lich hen gio vao LittleFS moi khi co thay doi
  if (scheduleChanged && gAutoCfg.enabled) {
    saveScheduleToFS();
    if (gMode == MODE_AUTO && !gMissionActive) {
      gAutoState = AUTO_WAITING_TIME;
    }
  }

  // Start now
  if ((bool)(data["auto_start_now"] | false) && gMode == MODE_AUTO) {
    gAutoCfg.enabled = true;
    gAutoCfg.runAtEpoch = 0;
    saveScheduleToFS();
    startAutoMissionNow();
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<384> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) { Serial.println("[MQTT] JSON error"); return; }

  String topicStr = String(topic);
  if (!topicStr.startsWith("v1/devices/me/attributes")) return;

  JsonVariant data = doc.as<JsonVariant>();
  if (doc.containsKey("shared")) data = doc["shared"];

  applyAttributeJson(data);
}

void requestSharedAttributes() {
  if (!mqttClient.connected()) return;
  mqttClient.publish("v1/devices/me/attributes/request/1",
    "{\"sharedKeys\":\"mode,x,y,duration,stop,auto_enabled,target_x,target_y,run_at_epoch,auto_start_now,move_ms_cm_x,move_ms_cm_y,turn_90_ms,scan_360_ms\"}");
}

// -------------------------------------------------------------------
// Connectivity
// -------------------------------------------------------------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("\n[WiFi] Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000UL) { delay(500); Serial.print("."); }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected: " + WiFi.localIP().toString());
    syncTimeNtp(true);
  } else {
    Serial.println("\n[WiFi] Failed");
  }
}

void connectMQTT() {
  if (mqttClient.connected()) return;
  mqttClient.setServer(TB_SERVER, TB_PORT);
  mqttClient.setCallback(onMqttMessage);
  mqttClient.setKeepAlive(30);
  Serial.print("[MQTT] Connecting...");
  if (mqttClient.connect("ESP32_RobotCar", TB_TOKEN, nullptr)) {
    Serial.println(" ok");
    mqttClient.subscribe("v1/devices/me/attributes");
    mqttClient.subscribe("v1/devices/me/attributes/response/+");
    requestSharedAttributes();
    gLastAttrReq = millis();
    gLastTelemetry = 0;
  } else {
    Serial.printf(" fail rc=%d\n", mqttClient.state());
  }
}

// -------------------------------------------------------------------
// Telemetry
// -------------------------------------------------------------------
void publishEvent(const char* eventName, int gasRaw, int angle) {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<256> doc;
  doc["event"] = eventName;
  doc["mode"] = modeToText(gMode);
  doc["auto_state"] = autoStateToText(gAutoState);
  doc["epoch"] = nowEpoch();
  if (gasRaw >= 0) doc["gas_raw"] = gasRaw;
  if (angle >= 0) doc["angle_deg"] = angle;
  doc["target_x"] = gAutoCfg.targetX;
  doc["target_y"] = gAutoCfg.targetY;
  doc["est_x"] = gPose.x;
  doc["est_y"] = gPose.y;
  char buf[256];
  serializeJson(doc, buf);
  mqttClient.publish("v1/devices/me/telemetry", buf);
}

void publishTelemetry(bool force) {
  if (!mqttClient.connected()) return;
  unsigned long nowMs = millis();
  if (!force && (nowMs - gLastTelemetry < TELEMETRY_INTERVAL_MS)) return;
  gLastTelemetry = nowMs;

  int gasRaw = analogRead(PIN_GAS);
  bool gasAbove = (gasRaw > GAS_ALERT_THRESHOLD);

  // Chi phat event khi vua vuot nguong (tranh spam)
  if (gasAbove && !gGasAboveThreshold) {
    publishEvent("gas_over_threshold", gasRaw, -1);
  }
  gGasAboveThreshold = gasAbove;

  StaticJsonDocument<128> doc;
  doc["gas_raw"] = gasRaw;
  doc["est_x"]   = gPose.x;
  doc["est_y"]   = gPose.y;
  if (gasAbove) {
    doc["detect"] = true;
  }
  char buf[128];
  serializeJson(doc, buf);
  mqttClient.publish("v1/devices/me/telemetry", buf);
  Serial.printf("[TEL] gas=%d x=%d y=%d detect=%d\n", gasRaw, gPose.x, gPose.y, gasAbove ? 1 : 0);
}

// -------------------------------------------------------------------
// Setup + Loop
// -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Robot Car Booting ===");

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  initPwmPin(PIN_ENA, PWM_CH_RIGHT);
  initPwmPin(PIN_ENB, PWM_CH_LEFT);
  stopMotors();

  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS mount failed");
  } else {
    Serial.println("[FS] LittleFS mounted");
    loadScheduleFromFS();
  }

  connectWiFi();
  connectMQTT();
  Serial.println("=== Ready ===");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.loop();

  syncTimeNtp(false);

  if (mqttClient.connected() && millis() - gLastAttrReq >= ATTR_REQUEST_INTERVAL_MS) {
    gLastAttrReq = millis();
    requestSharedAttributes();
  }

  if (gMode == MODE_MANUAL && gManualAutoStop && millis() >= gManualStopAt) {
    stopMotors();
    Serial.println("[MANUAL] timeout");
  }

  processAutoStateMachine();
  publishTelemetry(false);
}
