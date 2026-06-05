/*
 * ============================================================================
 *  XE ĐI THEO MAP (toạ độ X,Y) — Firmware ESP32
 *  Khung tank TP101 + 2 motor giảm tốc 12V + encoder + MPU6050
 *
 *  CHỨC NĂNG:
 *    - Đọc 2 encoder (quadrature) -> quãng đường mỗi bánh
 *    - Đọc MPU6050 (gyro Z) -> heading θ (bù trượt xích)
 *    - Odometry: ước lượng vị trí (x, y, θ) theo thời gian thực
 *    - Chế độ AUTO : nhận toạ độ đích (x,y) -> PID tự lái tới nơi
 *    - Chế độ MANUAL: nhận lệnh tiến/lùi/trái/phải/dừng
 *    - WiFi -> ThingsBoard (MQTT): nhận lệnh (RPC) + gửi telemetry (x,y,θ,mode)
 *
 *  THƯ VIỆN cần cài (Arduino IDE -> Library Manager):
 *    - PubSubClient  (by Nick O'Leary)   -> MQTT
 *    - ArduinoJson   (by Benoit Blanchon) -> parse lệnh JSON
 *  Driver mặc định: L298N (2 motor). Đổi sang BTS7960 xem ghi chú phần MOTOR.
 *
 *  LƯU Ý: xe xích bị trượt khi rẽ -> heading lấy từ GYRO (chính xác hơn encoder).
 * ============================================================================
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>

// Forward declarations (tránh lỗi auto-prototype của Arduino với kiểu JsonVariantConst)
void handleCommand(const char* method, JsonVariantConst params);
void publishTelemetry();
void publishEvent(const char* ev);

// ===================== CẤU HÌNH WIFI / THINGSBOARD =========================
const char* WIFI_SSID = "iot";
const char* WIFI_PASS = "12345678";

const char* TB_HOST   = "thingsboard.cloud";
const int   TB_PORT   = 1883;
const char* TB_TOKEN  = "PASTE_DEVICE_TOKEN";   // <-- token thiết bị xe trên ThingsBoard

// ===================== CHÂN GPIO ==========================================
// --- Driver L298N (trái) ---
#define M_L_EN   25      // ENA (PWM)
#define M_L_IN1  26
#define M_L_IN2  27
// --- Driver L298N (phải) ---
#define M_R_EN   14      // ENB (PWM)
#define M_R_IN1  18
#define M_R_IN2  19
// Nếu dùng BTS7960: mỗi motor có RPWM + LPWM (2 chân PWM), bỏ IN1/IN2,
//   xuất PWM tiến vào RPWM, lùi vào LPWM. Sửa hàm setMotor() tương ứng.

// --- Encoder (quadrature A/B) — dùng chân input-only có ngắt ---
#define ENC_L_A  34
#define ENC_L_B  35
#define ENC_R_A  32
#define ENC_R_B  33

// --- MPU6050 I2C ---
#define PIN_SDA  21
#define PIN_SCL  22
#define MPU_ADDR 0x68

// ===================== THÔNG SỐ XE (ĐO/HIỆU CHỈNH THỰC TẾ) =================
const float WHEEL_DIAMETER = 0.065f;   // m, đường kính bánh/đĩa xích chủ động
const int   ENC_CPR        = 1320;     // xung/vòng TRỤC RA (gồm hộp số & x4). ĐO lại!
const float TRACK_WIDTH    = 0.17f;    // m, khoảng cách 2 dải xích (tâm-tâm)

const float DIST_PER_COUNT = (PI * WHEEL_DIAMETER) / ENC_CPR;  // m mỗi xung

// ===================== THAM SỐ ĐIỀU KHIỂN =================================
const float POS_TOLERANCE  = 0.05f;    // m — coi như tới đích khi cách < 5cm
const float HEAD_TOLERANCE = 0.10f;    // rad (~6°) — sai số góc cho phép
const int   PWM_MAX        = 255;
const int   PWM_MIN        = 90;       // PWM tối thiểu để motor quay (đo lại)

// PID hướng (góc) khi auto
float Kp_h = 180.0f, Ki_h = 0.0f, Kd_h = 12.0f;
// P khoảng cách (tốc độ tiến tỉ lệ khoảng cách còn lại)
float Kp_d = 700.0f;

// ===================== BIẾN TRẠNG THÁI ====================================
enum Mode { MODE_IDLE, MODE_AUTO, MODE_MANUAL };
volatile Mode mode = MODE_IDLE;

// Encoder counters (cập nhật trong ngắt)
volatile long encLeft  = 0;
volatile long encRight = 0;

// Pose hiện tại
double poseX = 0, poseY = 0, poseTheta = 0;   // m, m, rad

// Mục tiêu auto
double targetX = 0, targetY = 0;

// Lệnh manual
int manL = 0, manR = 0;     // -255..255

// MPU gyro
float gyroBiasZ = 0;
unsigned long lastOdomMs = 0;

// PID hướng — biến nhớ
float headErrPrev = 0, headErrInt = 0;

// ===================== MQTT ===============================================
WiFiClient   espClient;
PubSubClient mqtt(espClient);
unsigned long lastTelemetryMs = 0;

// ===================== NGẮT ENCODER =======================================
void IRAM_ATTR isrLeftA() {
  bool a = digitalRead(ENC_L_A);
  bool b = digitalRead(ENC_L_B);
  encLeft += (a == b) ? 1 : -1;
}
void IRAM_ATTR isrRightA() {
  bool a = digitalRead(ENC_R_A);
  bool b = digitalRead(ENC_R_B);
  encRight += (a == b) ? 1 : -1;
}

// ===================== MPU6050 (raw I2C) ==================================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
int16_t mpuReadGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);                 // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t v = (Wire.read() << 8) | Wire.read();
  return v;
}
void mpuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  mpuWrite(0x6B, 0x00);             // wake up
  delay(50);
  mpuWrite(0x1B, 0x00);             // gyro ±250 °/s
  delay(50);
  // Hiệu chỉnh bias gyro Z (giữ xe ĐỨNG YÊN lúc bật nguồn)
  long sum = 0;
  for (int i = 0; i < 500; i++) { sum += mpuReadGyroZ(); delay(2); }
  gyroBiasZ = sum / 500.0f;
}
// Trả tốc độ góc quanh trục Z (rad/s)
float readGyroRate() {
  float raw = mpuReadGyroZ() - gyroBiasZ;
  float degPerSec = raw / 131.0f;   // ±250 °/s -> 131 LSB/(°/s)
  return degPerSec * PI / 180.0f;
}

// ===================== ĐIỀU KHIỂN MOTOR ===================================
// pwm: -255..255 (âm = lùi)
void setMotorLeft(int pwm) {
  bool fwd = pwm >= 0; pwm = abs(pwm);
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  digitalWrite(M_L_IN1, fwd ? HIGH : LOW);
  digitalWrite(M_L_IN2, fwd ? LOW  : HIGH);
  ledcWrite(0, pwm);
}
void setMotorRight(int pwm) {
  bool fwd = pwm >= 0; pwm = abs(pwm);
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  digitalWrite(M_R_IN1, fwd ? HIGH : LOW);
  digitalWrite(M_R_IN2, fwd ? LOW  : HIGH);
  ledcWrite(1, pwm);
}
void stopMotors() { setMotorLeft(0); setMotorRight(0); }

// ===================== ODOMETRY ===========================================
void updateOdometry() {
  unsigned long now = millis();
  float dt = (now - lastOdomMs) / 1000.0f;
  if (dt <= 0) return;
  lastOdomMs = now;

  // Lấy & reset bộ đếm xung
  noInterrupts();
  long dL = encLeft;  encLeft  = 0;
  long dR = encRight; encRight = 0;
  interrupts();

  float distL = dL * DIST_PER_COUNT;
  float distR = dR * DIST_PER_COUNT;
  float dCenter = (distL + distR) / 2.0f;

  // Heading: ưu tiên GYRO (chính xác hơn với xe xích hay trượt)
  float dTheta = readGyroRate() * dt;

  poseTheta += dTheta;
  // chuẩn hoá về [-pi, pi]
  if (poseTheta >  PI) poseTheta -= 2 * PI;
  if (poseTheta < -PI) poseTheta += 2 * PI;

  poseX += dCenter * cos(poseTheta);
  poseY += dCenter * sin(poseTheta);
}

// ===================== ĐIỀU HƯỚNG AUTO (go-to-goal) =======================
void runAuto() {
  double dx = targetX - poseX;
  double dy = targetY - poseY;
  double dist = sqrt(dx * dx + dy * dy);

  if (dist < POS_TOLERANCE) {            // ĐÃ TỚI ĐÍCH
    stopMotors();
    mode = MODE_IDLE;
    publishEvent("reached");
    return;
  }

  double targetHeading = atan2(dy, dx);
  double headErr = targetHeading - poseTheta;
  while (headErr >  PI) headErr -= 2 * PI;
  while (headErr < -PI) headErr += 2 * PI;

  // PID hướng
  headErrInt += headErr;
  float dErr = headErr - headErrPrev;
  headErrPrev = headErr;
  float turn = Kp_h * headErr + Ki_h * headErrInt + Kd_h * dErr;

  // Tốc độ tiến: chỉ tiến mạnh khi đã hướng đúng
  float forward = 0;
  if (fabs(headErr) < 0.6f) {            // ~34°
    forward = Kp_d * dist;
    if (forward > 160) forward = 160;    // giới hạn tốc tiến
  }

  int pwmL = (int)(forward - turn);
  int pwmR = (int)(forward + turn);

  // Đảm bảo vượt PWM tối thiểu khi cần quay tại chỗ
  if (forward < 1 && fabs(turn) > 1) {
    if (pwmL > 0 && pwmL < PWM_MIN) pwmL = PWM_MIN;
    if (pwmL < 0 && pwmL > -PWM_MIN) pwmL = -PWM_MIN;
    if (pwmR > 0 && pwmR < PWM_MIN) pwmR = PWM_MIN;
    if (pwmR < 0 && pwmR > -PWM_MIN) pwmR = -PWM_MIN;
  }
  setMotorLeft(pwmL);
  setMotorRight(pwmR);
}

// ===================== MQTT: NHẬN LỆNH ====================================
// ThingsBoard RPC tới topic: v1/devices/me/rpc/request/{id}
// method = "setTarget"  params = {"x":1.0,"y":0.5}
// method = "manual"     params = {"cmd":"forward"|"back"|"left"|"right"|"stop"}
// method = "setMode"    params = {"mode":"auto"|"manual"|"idle"}
void handleCommand(const char* method, JsonVariantConst params) {
  if (strcmp(method, "setTarget") == 0) {
    targetX = params["x"] | poseX;
    targetY = params["y"] | poseY;
    headErrInt = 0; headErrPrev = 0;
    mode = MODE_AUTO;
    publishEvent("auto_start");
  }
  else if (strcmp(method, "setMode") == 0) {
    const char* m = params["mode"] | "idle";
    if (strcmp(m, "auto") == 0)        mode = MODE_AUTO;
    else if (strcmp(m, "manual") == 0) { mode = MODE_MANUAL; manL = manR = 0; }
    else { mode = MODE_IDLE; stopMotors(); }
  }
  else if (strcmp(method, "manual") == 0) {
    mode = MODE_MANUAL;
    const char* c = params["cmd"] | "stop";
    int sp = params["speed"] | 150;
    if      (strcmp(c, "forward") == 0) { manL =  sp; manR =  sp; }
    else if (strcmp(c, "back")    == 0) { manL = -sp; manR = -sp; }
    else if (strcmp(c, "left")    == 0) { manL = -sp; manR =  sp; }
    else if (strcmp(c, "right")   == 0) { manL =  sp; manR = -sp; }
    else                                { manL = 0;   manR = 0;   }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  // Tìm request id trong topic để trả lời
  String t = String(topic);
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, len)) return;
  const char* method = doc["method"] | "";
  handleCommand(method, doc["params"]);
}

void mqttReconnect() {
  while (!mqtt.connected()) {
    if (mqtt.connect("esp32-car", TB_TOKEN, NULL)) {
      mqtt.subscribe("v1/devices/me/rpc/request/+");
    } else {
      delay(1000);
    }
  }
}

// ===================== MQTT: GỬI TELEMETRY ================================
void publishTelemetry() {
  StaticJsonDocument<200> doc;
  doc["x"]     = round(poseX * 1000) / 1000.0;
  doc["y"]     = round(poseY * 1000) / 1000.0;
  doc["theta"] = round(poseTheta * 180.0 / PI);   // độ
  doc["mode"]  = (mode == MODE_AUTO) ? "auto" : (mode == MODE_MANUAL ? "manual" : "idle");
  char buf[200];
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

// ===================== SETUP ==============================================
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(M_L_IN1, OUTPUT); pinMode(M_L_IN2, OUTPUT);
  pinMode(M_R_IN1, OUTPUT); pinMode(M_R_IN2, OUTPUT);
  // PWM (ESP32 core 2.x). Core 3.x: dùng ledcAttach(pin, freq, res) + ledcWrite(pin, val)
  ledcSetup(0, 20000, 8); ledcAttachPin(M_L_EN, 0);
  ledcSetup(1, 20000, 8); ledcAttachPin(M_R_EN, 1);
  stopMotors();

  // Encoder
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftA,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightA, CHANGE);

  // MPU6050
  mpuInit();

  // WiFi + MQTT
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println(" OK " + WiFi.localIP().toString());
  mqtt.setServer(TB_HOST, TB_PORT);
  mqtt.setCallback(mqttCallback);

  lastOdomMs = millis();
  Serial.println("Xe map navigation san sang.");
}

// ===================== LOOP ===============================================
void loop() {
  if (!mqtt.connected()) mqttReconnect();
  mqtt.loop();

  updateOdometry();

  switch (mode) {
    case MODE_AUTO:   runAuto(); break;
    case MODE_MANUAL: setMotorLeft(manL); setMotorRight(manR); break;
    default:          stopMotors(); break;
  }

  // Gửi telemetry mỗi 300ms
  if (millis() - lastTelemetryMs > 300) {
    lastTelemetryMs = millis();
    publishTelemetry();
  }

  delay(10);
}
