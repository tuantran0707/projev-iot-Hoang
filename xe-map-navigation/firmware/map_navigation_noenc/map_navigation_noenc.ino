/*
 * ============================================================================
 *  XE ĐI THEO MAP (X,Y) - BẢN KHÔNG ENCODER
 *  Dùng cho: 2 motor 12V không encoder + L298N + MPU6050 + ESP32
 *
 *  Ý tưởng:
 *    - Góc quay theta lấy từ gyro Z (MPU6050).
 *    - Quãng đường ước lượng bằng tốc độ đã hiệu chuẩn theo PWM:
 *        v ~= K_SPEED_MPS * pwmCmd
 *      => dCenter = v * dt
 *
 *  Chế độ:
 *    - AUTO  : nhận target (x,y), quay hướng rồi tiến tới đích (dead-reckoning)
 *    - MANUAL: nhận lệnh forward/back/left/right/stop
 *
 *  RPC (ThingsBoard):
 *    setTarget: {"x":1.2,"y":0.8}
 *    setMode  : {"mode":"auto|manual|idle"}
 *    manual   : {"cmd":"forward|back|left|right|stop","speed":160}
 *
 *  Cảnh báo: Không encoder sẽ có drift, phù hợp map nhỏ và cần hiệu chuẩn.
 * ============================================================================
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>

void handleCommand(const char* method, JsonVariantConst params);
void publishTelemetry();
void publishEvent(const char* ev);

const char* WIFI_SSID = "iot";
const char* WIFI_PASS = "12345678";
const char* TB_HOST   = "thingsboard.cloud";
const int   TB_PORT   = 1883;
const char* TB_TOKEN  = "PASTE_DEVICE_TOKEN";

// L298N
#define M_L_EN   25
#define M_L_IN1  26
#define M_L_IN2  27
#define M_R_EN   14
#define M_R_IN1  18
#define M_R_IN2  19

// MPU6050
#define PIN_SDA  21
#define PIN_SCL  22
#define MPU_ADDR 0x68

// Hiệu chuẩn thực nghiệm (BẮT BUỘC sửa lại)
// Ví dụ: PWM=160 chạy thẳng đo được 0.32 m/s -> K = 0.32/160 = 0.002
float K_SPEED_MPS = 0.0020f;   // m/s trên mỗi 1 đơn vị PWM lệnh tiến/lùi
float POS_TOLERANCE = 0.08f;   // m
int PWM_MAX = 255;
int PWM_MIN = 95;

// PID hướng
float Kp_h = 170.0f, Ki_h = 0.0f, Kd_h = 10.0f;
// Tốc độ tiến theo khoảng cách còn lại
float Kp_d = 520.0f;

enum Mode { MODE_IDLE, MODE_AUTO, MODE_MANUAL };
volatile Mode mode = MODE_IDLE;

double poseX = 0, poseY = 0, poseTheta = 0;
double targetX = 0, targetY = 0;

int manL = 0, manR = 0;
int autoPwmCmd = 0; // lệnh tiến/lùi dùng cho ước lượng quãng đường

float gyroBiasZ = 0;
unsigned long lastOdomMs = 0;
float headErrPrev = 0, headErrInt = 0;

WiFiClient espClient;
PubSubClient mqtt(espClient);
unsigned long lastTelemetryMs = 0;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t mpuReadGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t v = (Wire.read() << 8) | Wire.read();
  return v;
}

void mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  mpuWrite(0x6B, 0x00);
  delay(50);
  mpuWrite(0x1B, 0x00);
  delay(50);

  long sum = 0;
  for (int i = 0; i < 500; i++) {
    sum += mpuReadGyroZ();
    delay(2);
  }
  gyroBiasZ = sum / 500.0f;
}

float readGyroRate() {
  float raw = mpuReadGyroZ() - gyroBiasZ;
  float degPerSec = raw / 131.0f;
  return degPerSec * PI / 180.0f;
}

void setMotorLeft(int pwm) {
  bool fwd = pwm >= 0;
  pwm = abs(pwm);
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  digitalWrite(M_L_IN1, fwd ? HIGH : LOW);
  digitalWrite(M_L_IN2, fwd ? LOW : HIGH);
  ledcWrite(0, pwm);
}

void setMotorRight(int pwm) {
  bool fwd = pwm >= 0;
  pwm = abs(pwm);
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  digitalWrite(M_R_IN1, fwd ? HIGH : LOW);
  digitalWrite(M_R_IN2, fwd ? LOW : HIGH);
  ledcWrite(1, pwm);
}

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
  autoPwmCmd = 0;
}

void updateOdometryNoEncoder() {
  unsigned long now = millis();
  float dt = (now - lastOdomMs) / 1000.0f;
  if (dt <= 0) return;
  lastOdomMs = now;

  float dTheta = readGyroRate() * dt;
  poseTheta += dTheta;
  while (poseTheta > PI) poseTheta -= 2 * PI;
  while (poseTheta < -PI) poseTheta += 2 * PI;

  float v = K_SPEED_MPS * autoPwmCmd;
  float dCenter = v * dt;
  poseX += dCenter * cos(poseTheta);
  poseY += dCenter * sin(poseTheta);
}

void runAuto() {
  double dx = targetX - poseX;
  double dy = targetY - poseY;
  double dist = sqrt(dx * dx + dy * dy);

  if (dist < POS_TOLERANCE) {
    stopMotors();
    mode = MODE_IDLE;
    publishEvent("reached");
    return;
  }

  double targetHeading = atan2(dy, dx);
  double headErr = targetHeading - poseTheta;
  while (headErr > PI) headErr -= 2 * PI;
  while (headErr < -PI) headErr += 2 * PI;

  headErrInt += headErr;
  float dErr = headErr - headErrPrev;
  headErrPrev = headErr;
  float turn = Kp_h * headErr + Ki_h * headErrInt + Kd_h * dErr;

  float forward = 0;
  if (fabs(headErr) < 0.55f) {
    forward = Kp_d * dist;
    if (forward > 170) forward = 170;
  }

  int pwmL = (int)(forward - turn);
  int pwmR = (int)(forward + turn);

  if (forward > 1) {
    if (pwmL > 0 && pwmL < PWM_MIN) pwmL = PWM_MIN;
    if (pwmL < 0 && pwmL > -PWM_MIN) pwmL = -PWM_MIN;
    if (pwmR > 0 && pwmR < PWM_MIN) pwmR = PWM_MIN;
    if (pwmR < 0 && pwmR > -PWM_MIN) pwmR = -PWM_MIN;
  }

  setMotorLeft(pwmL);
  setMotorRight(pwmR);

  // Ước lượng quãng đường dựa trên lệnh tiến trung bình
  autoPwmCmd = (abs(pwmL) + abs(pwmR)) / 2;
  if (pwmL < 0 && pwmR < 0) autoPwmCmd = -autoPwmCmd;
  if (forward < 1 && fabs(turn) > 1) autoPwmCmd = 0;
}

void handleCommand(const char* method, JsonVariantConst params) {
  if (strcmp(method, "setTarget") == 0) {
    targetX = params["x"] | poseX;
    targetY = params["y"] | poseY;
    headErrInt = 0;
    headErrPrev = 0;
    mode = MODE_AUTO;
    publishEvent("auto_start");
  } else if (strcmp(method, "setMode") == 0) {
    const char* m = params["mode"] | "idle";
    if (strcmp(m, "auto") == 0) {
      mode = MODE_AUTO;
    } else if (strcmp(m, "manual") == 0) {
      mode = MODE_MANUAL;
      manL = 0;
      manR = 0;
    } else {
      mode = MODE_IDLE;
      stopMotors();
    }
  } else if (strcmp(method, "manual") == 0) {
    mode = MODE_MANUAL;
    autoPwmCmd = 0;
    const char* c = params["cmd"] | "stop";
    int sp = params["speed"] | 160;
    if (sp > PWM_MAX) sp = PWM_MAX;
    if (sp < PWM_MIN) sp = PWM_MIN;

    if (strcmp(c, "forward") == 0) {
      manL = sp; manR = sp;
    } else if (strcmp(c, "back") == 0) {
      manL = -sp; manR = -sp;
    } else if (strcmp(c, "left") == 0) {
      manL = -sp; manR = sp;
    } else if (strcmp(c, "right") == 0) {
      manL = sp; manR = -sp;
    } else {
      manL = 0; manR = 0;
    }
  } else if (strcmp(method, "setCalib") == 0) {
    K_SPEED_MPS = params["k_speed"] | K_SPEED_MPS;
    POS_TOLERANCE = params["pos_tol"] | POS_TOLERANCE;
  } else if (strcmp(method, "setPose") == 0) {
    poseX = params["x"] | poseX;
    poseY = params["y"] | poseY;
    double deg = params["theta_deg"] | (poseTheta * 180.0 / PI);
    poseTheta = deg * PI / 180.0;
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, len)) return;
  const char* method = doc["method"] | "";
  handleCommand(method, doc["params"]);
}

void mqttReconnect() {
  while (!mqtt.connected()) {
    if (mqtt.connect("esp32-car-noenc", TB_TOKEN, NULL)) {
      mqtt.subscribe("v1/devices/me/rpc/request/+");
    } else {
      delay(1000);
    }
  }
}

void publishTelemetry() {
  StaticJsonDocument<220> doc;
  doc["x"] = round(poseX * 1000) / 1000.0;
  doc["y"] = round(poseY * 1000) / 1000.0;
  doc["theta"] = round(poseTheta * 180.0 / PI);
  doc["mode"] = (mode == MODE_AUTO) ? "auto" : (mode == MODE_MANUAL ? "manual" : "idle");
  doc["k_speed"] = K_SPEED_MPS;
  doc["no_encoder"] = true;
  char buf[220];
  size_t n = serializeJson(doc, buf);
  mqtt.publish("v1/devices/me/telemetry", buf, n);
}

void publishEvent(const char* ev) {
  StaticJsonDocument<96> doc;
  doc["event"] = ev;
  char buf[96];
  size_t n = serializeJson(doc, buf);
  mqtt.publish("v1/devices/me/telemetry", buf, n);
}

void setup() {
  Serial.begin(115200);

  pinMode(M_L_IN1, OUTPUT);
  pinMode(M_L_IN2, OUTPUT);
  pinMode(M_R_IN1, OUTPUT);
  pinMode(M_R_IN2, OUTPUT);

  ledcSetup(0, 20000, 8);
  ledcAttachPin(M_L_EN, 0);
  ledcSetup(1, 20000, 8);
  ledcAttachPin(M_R_EN, 1);
  stopMotors();

  mpuInit();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }

  mqtt.setServer(TB_HOST, TB_PORT);
  mqtt.setCallback(mqttCallback);

  lastOdomMs = millis();
}

void loop() {
  if (!mqtt.connected()) mqttReconnect();
  mqtt.loop();

  updateOdometryNoEncoder();

  switch (mode) {
    case MODE_AUTO:
      runAuto();
      break;
    case MODE_MANUAL:
      setMotorLeft(manL);
      setMotorRight(manR);
      break;
    default:
      stopMotors();
      break;
  }

  if (millis() - lastTelemetryMs > 300) {
    lastTelemetryMs = millis();
    publishTelemetry();
  }

  delay(10);
}
