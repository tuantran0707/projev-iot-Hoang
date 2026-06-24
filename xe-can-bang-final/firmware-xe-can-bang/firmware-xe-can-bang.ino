/* =============================================================================
 *  XE TỰ CÂN BẰNG 2 BÁNH  -  ESP32 + MPU6050/9250 + TB6612FNG + 2 Encoder + 2 Servo
 * -----------------------------------------------------------------------------
 *  Điều khiển: PID góc (P+D), góc cân bằng cố định trong code. *  MAP CHÂN:
 *    I2C MPU : SDA=21, SCL=22
 *    TB6612  : PWMA=25 (Motor A/PHẢI)  PWMB=26 (Motor B/TRÁI)
 *              AIN1=15  AIN2=18  BIN1=4  BIN2=5  STBY=2
 *    Encoder PHẢI : C1=14, C2=27
 *    Encoder TRÁI : C1=13, C2=33
 *    Servo   : RIGHT=19, LEFT=23   (GPIO18 dùng AIN2 -> đổi chân servo)
 * ========================================================================== */

#include <Wire.h>
#include <MPU6050_tockn.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ----------------------- WIFI / THINGSBOARD ----------------------------
#define WIFI_SSID     "iot"
#define WIFI_PASS     "12345678"
#define TB_SERVER     "thingsboard.cloud"
#define TB_PORT       1883
#define TB_TOKEN      "0nlCN9NS7yX63z6pDagW"   // dùng làm MQTT username

// Shared attribute keys nhận từ ThingsBoard:
//   drive       : tiến/lùi   (-100..100, am=lui, duong=tien). 0 = dung yen.
//   servoRight  : servo phai (0..100 -> 0..180 do)
//   servoLeft   : servo trai (0..100 -> 0..180 do)
#define TB_SHARED_KEYS "drive,servoRight,servoLeft"

// Khi drive = 100 (max) thi nghieng diem can bang them bao nhieu DO de chay toi.
// Bat dau nho (2-4 do) cho an toan; tang neu muon chay nhanh hon.
#define MAX_DRIVE_TILT 4.0

// ----------------------- MAP CHÂN ----------------------------
#define PIN_SDA 21
#define PIN_SCL 22

// Động cơ Phải (TB6612FNG - kênh A)
#define PWMA 25
#define AIN1 15
#define AIN2 18

// Động cơ Trái (TB6612FNG - kênh B)
#define PWMB 26
#define BIN1 4
#define BIN2 5
#define STBY 2

// Encoder
#define ENC_R_C1 14
#define ENC_R_C2 27
#define ENC_L_C1 13
#define ENC_L_C2 33

// Servo (GPIO18 = AIN2 nên không dùng cho servo)
#define SERVO_RIGHT_PIN 19
#define SERVO_LEFT_PIN  23

// ===================== CẤU HÌNH CÂN BẰNG (sửa trực tiếp trong code) =====================
// Trục cân bằng: 1=AngleY (xe ngả tới/lui là Ay đổi nhiều)
#define BALANCE_AXIS_Y 1

// Quy ước Ay: chúi TRƯỚC = góc âm hơn (-6,-7) | chúi SAU = ít âm hơn (-5,-4,-3)
// Mục tiêu -6.2 = hơi chúi sau xíu (ít âm hơn -6.4)
#define BALANCE_TARGET   -6.2f

#define FALL_LIMIT         28.0f

float Kp = 16.0f;
float Ki = 0.038f;    // I nhỏ: kéo dần về góc cân, chống trôi lùi
float Kd = 0.60f;

#define GYRO_LPF      0.55f
#define PWM_MAX       145
#define PWM_MAX_BACK  145
#define PWM_CATCH     165
#define PWM_MICRO     13
#define PWM_MICRO_REAR 15
#define MICRO_ERR     0.24f
#define MICRO_ZONE    0.8f
#define REAR_ERR_GAIN 1.28f
#define KI_CLAMP      15.0f
#define KI_ZONE       2.2f   // ngoài vùng này: giảm tích phân tránh windup
#define OUTPUT_SLEW   45
#define SLEW_NEAR     16
#define SLEW_CATCH    200     // lệch lớn: gần như không giới hạn slew
#define CATCH_ERR     2.0f    // |Err| >= này → chế độ bắt ngã
#define KP_BOOST_ERR  1.2f    // từ đây Kp tăng dần theo |Err|
#define KP_BOOST_RATE 0.55f

// Trim 2 bánh (đo encoder: phải nhanh hơn ~3%)
#define MOTOR_R_TRIM 0.97f
#define MOTOR_L_TRIM 1.03f
int motorSign = 1;   // ngã cả 2 phía: đổi thành -1 trong code hoặc gõ 'n'

#define MOTOR_R_SIGN  1
#define MOTOR_L_SIGN -1
#define ENC_R_SIGN 1
#define ENC_L_SIGN -1

#define DT_MS 5
// ========================================================================

MPU6050 mpu6050(Wire);
Servo servoRight;
Servo servoLeft;

WiFiClient espClient;
PubSubClient mqtt(espClient);

// Lenh dieu khien tu ThingsBoard (shared attributes)
float driveCmd      = 0.0;   // -100..100 (tien/lui) -> nghieng diem can bang
int   servoRightCmd = 90;    // goc servo phai 0..180
int   servoLeftCmd  = 90;    // goc servo trai 0..180

unsigned long lastMqttRetry = 0;
unsigned long lastTelemetry = 0;

// Biến encoder (đếm trong ngắt)
volatile long encCountR = 0;
volatile long encCountL = 0;

// Biến PID
unsigned long lastLoop = 0;
unsigned long lastPrint = 0;

bool balancing = false;
float lastPidOutput = 0.0f;
float rateFiltered = 0.0f;
float pidIntegral = 0.0f;
float gBalanceError = 0.0f;

static float balanceTarget() {
  return BALANCE_TARGET;
}

void setMotors(int speedR, int speedL);

void resetBalanceState() {
  balancing = false;
  lastPidOutput = 0.0f;
  rateFiltered = 0.0f;
  pidIntegral = 0.0f;
  setMotors(0, 0);
}

// ----------------- NGẮT ĐẾM ENCODER -----------------
void IRAM_ATTR isrEncR() {
  // Đọc kênh B để xác định chiều quay
  if (digitalRead(ENC_R_C2)) encCountR += ENC_R_SIGN;
  else                       encCountR -= ENC_R_SIGN;
}
void IRAM_ATTR isrEncL() {
  if (digitalRead(ENC_L_C2)) encCountL += ENC_L_SIGN;
  else                       encCountL -= ENC_L_SIGN;
}

// ----------------- ĐỌC GÓC CÂN BẰNG -----------------
float getBalanceAngle() {
#if BALANCE_AXIS_Y
  return mpu6050.getAngleY();
#else
  return mpu6050.getAngleX();
#endif
}

// VẬN TỐC GÓC (°/s) lấy TRỰC TIẾP từ gyro -> D-term rất mượt, không giật
float getBalanceRate() {
#if BALANCE_AXIS_Y
  return mpu6050.getGyroY();
#else
  return mpu6050.getGyroX();
#endif
}

// Serial: p/d/n (tùy chọn khi test)
void handleSerialTuning() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'n') {
    motorSign = -motorSign;
    Serial.print("motorSign="); Serial.println(motorSign);
    return;
  }
  if (c == 'p' || c == 'd') {
    float v = Serial.parseFloat();
    if (c == 'p') Kp = v;
    else          Kd = v;
    Serial.print("Kp="); Serial.print(Kp);
    Serial.print(" Kd="); Serial.println(Kd);
  }
}

// ----------------- KET NOI WIFI / THINGSBOARD MQTT -----------------
// Ap dung 1 cap shared-attribute JSON (object) vao cac bien dieu khien.
void applySharedAttributes(JsonObjectConst obj) {
  if (obj.containsKey("drive"))      driveCmd      = constrain((float)obj["drive"], -100.0, 100.0);
  if (obj.containsKey("servoRight")) servoRightCmd = map(constrain((int)obj["servoRight"], 0, 100), 0, 100, 0, 180);
  if (obj.containsKey("servoLeft"))  servoLeftCmd  = map(constrain((int)obj["servoLeft"],  0, 100), 0, 100, 0, 180);

  // Cap nhat servo ngay khi co lenh moi
  servoRight.write(servoRightCmd);
  servoLeft.write(servoLeftCmd);
}

// Callback nhan ban tin MQTT tu ThingsBoard
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, len)) return;   // JSON loi -> bo qua

  // - Topic update truc tiep: v1/devices/me/attributes      -> { "drive":.. }
  // - Topic tra loi request : .../response/+                -> { "shared": {..} }
  if (doc.containsKey("shared"))      applySharedAttributes(doc["shared"].as<JsonObjectConst>());
  else                                applySharedAttributes(doc.as<JsonObjectConst>());
}

void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);   // khong chan: kiem tra trang thai trong loop
}

// Reconnect MQTT KHONG CHAN vong can bang: chi thu lai moi 3s, timeout ngan.
void mqttEnsureConnected() {
  if (WiFi.status() != WL_CONNECTED) return;     // chua co wifi thi thoi
  if (mqtt.connected()) { mqtt.loop(); return; }

  unsigned long now = millis();
  if (now - lastMqttRetry < 3000) return;        // chua toi luc thu lai
  lastMqttRetry = now;

  mqtt.setServer(TB_SERVER, TB_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setSocketTimeout(1);                      // 1s, tranh treo lau lam nga xe

  if (mqtt.connect("esp32-xecanbang", TB_TOKEN, "")) {
    Serial.println(">> MQTT ThingsBoard: CONNECTED");
    // Nhan moi cap nhat shared attribute thoi gian thuc
    mqtt.subscribe("v1/devices/me/attributes");
    // Nhan ket qua khi request gia tri shared hien tai
    mqtt.subscribe("v1/devices/me/attributes/response/+");
    // Request gia tri shared dang co tren server (de dong bo luc khoi dong)
    mqtt.publish("v1/devices/me/attributes/request/1",
                 "{\"sharedKeys\":\"" TB_SHARED_KEYS "\"}");
  } else {
    Serial.print(">> MQTT that bai, rc="); Serial.println(mqtt.state());
  }
}

// Gui telemetry (goc, trang thai) len ThingsBoard de xem tren dashboard
void publishTelemetry(float angle) {
  if (!mqtt.connected()) return;
  char buf[96];
  snprintf(buf, sizeof(buf),
           "{\"angle\":%.2f,\"drive\":%.0f,\"balancing\":%d}",
           angle, driveCmd, balancing ? 1 : 0);
  mqtt.publish("v1/devices/me/telemetry", buf);
}

void setup() {
  Serial.begin(115200);

  Wire.begin(PIN_SDA, PIN_SCL);

  // Chân động cơ
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);   // bật TB6612
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  // PWM cho 2 motor (ESP32 core 3.x: ledcAttach(pin, freq, bit))
  ledcAttach(PWMA, 20000, 8);  // 20kHz để chạy êm, không kêu
  ledcAttach(PWMB, 20000, 8);
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);

  // Encoder. Lưu ý: 34/35 chỉ input, cần trở kéo NGOÀI.
  pinMode(ENC_R_C1, INPUT_PULLUP);
  pinMode(ENC_R_C2, INPUT_PULLUP);
  pinMode(ENC_L_C1, INPUT_PULLUP);
  pinMode(ENC_L_C2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1), isrEncR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C1), isrEncL, RISING);

  // Servo
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoRight.setPeriodHertz(50);
  servoLeft.setPeriodHertz(50);
  servoRight.attach(SERVO_RIGHT_PIN, 500, 2400);
  servoLeft.attach(SERVO_LEFT_PIN, 500, 2400);
  servoRight.write(90);
  servoLeft.write(90);

  // Ket noi WiFi (khong chan) -> sau do MQTT tu reconnect trong loop
  startWiFi();

  // IMU
  Serial.println("Dang khoi tao MPU...");
  mpu6050.begin();
  Serial.println(">>> GIU CO DINH ROBOT - dang tinh offset gyro...");
  mpu6050.calcGyroOffsets(true);
  Serial.print("San sang. Set="); Serial.print(balanceTarget(), 1);
  Serial.println(" | gan -5.5 do roi tha");

  lastLoop = millis();
}

void loop() {
  mpu6050.update();
  handleSerialTuning();   // cho phép chỉnh Kp/Ki/Kd/Set qua Serial lúc đang chạy
  mqttEnsureConnected();  // duy tri ket noi ThingsBoard (khong chan vong can bang)

  unsigned long now = millis();
  if (now - lastLoop < DT_MS) return;
  float dt = DT_MS / 1000.0f;
  lastLoop = now;

  float angle = getBalanceAngle();
  float target = balanceTarget();

  if (fabs(angle - target) > FALL_LIMIT) {
    if (balancing) resetBalanceState();
  } else {
    if (!balancing) {
      balancing = true;
      rateFiltered = getBalanceRate();
      pidIntegral = 0.0f;
    }
    float error = target - angle;
    gBalanceError = error;
    float absErr = fabsf(error);
    pidIntegral += error * dt;
    if (absErr > KI_ZONE) {
      pidIntegral *= 0.90f;
    }
    pidIntegral = constrain(pidIntegral, -KI_CLAMP, KI_CLAMP);
    float rate = getBalanceRate();
    rateFiltered += GYRO_LPF * (rate - rateFiltered);

    float kpEff = Kp;
    if (absErr > KP_BOOST_ERR) {
      kpEff *= 1.0f + KP_BOOST_RATE * (absErr - KP_BOOST_ERR);
    }
    float kdEff = Kd * (1.0f + 0.20f * min(absErr, 5.0f));

    float output = (kpEff * error) + (Ki * pidIntegral) - (kdEff * rateFiltered);
    if (error < 0.0f) output *= REAR_ERR_GAIN;
    output *= motorSign;

    float slew;
    if (absErr >= CATCH_ERR)      slew = SLEW_CATCH;
    else if (absErr < MICRO_ZONE) slew = SLEW_NEAR;
    else                          slew = OUTPUT_SLEW;

    float delta = output - lastPidOutput;
    if (delta > slew)       output = lastPidOutput + slew;
    else if (delta < -slew) output = lastPidOutput - slew;

    int pwmCap = (absErr >= CATCH_ERR) ? PWM_CATCH : PWM_MAX;
    output = constrain(output, -pwmCap, pwmCap);
    lastPidOutput = output;
    setMotors((int)output, (int)output);
  }

  if (now - lastPrint >= 2000) {
    lastPrint = now;
    Serial.print("Ang:"); Serial.print(angle, 1);
    Serial.print(" Set:"); Serial.print(target, 1);
    Serial.print(" Err:"); Serial.print(target - angle, 1);
    Serial.print(" Out:"); Serial.print(lastPidOutput, 0);
    Serial.print(" Bal:"); Serial.println(balancing ? "ON" : "OFF");
  }

  // Gui telemetry len ThingsBoard moi 1s
  if (now - lastTelemetry >= 1000) {
    lastTelemetry = now;
    publishTelemetry(angle);
  }
}

// ===================== ĐIỀU KHIỂN ĐỘNG CƠ =====================
// Nhận tốc độ CÓ DẤU: -255..255  (âm = lùi, dương = tiến)
void setMotors(int speedR, int speedL) {
  driveMotorRight((int)(speedR * MOTOR_R_TRIM));
  driveMotorLeft((int)(speedL * MOTOR_L_TRIM));
}

void driveMotorRight(int speed) {
  speed *= MOTOR_R_SIGN;
  bool forward = (speed >= 0);
  int pwm = abs(speed);
  float absErr = fabsf(gBalanceError);
  int cap = (absErr >= CATCH_ERR) ? PWM_CATCH : (forward ? PWM_MAX : PWM_MAX_BACK);
  int microMin = forward ? PWM_MICRO : PWM_MICRO_REAR;
  float microErr = MICRO_ERR;
  if (absErr < MICRO_ZONE && absErr < CATCH_ERR && pwm > 0 && pwm < microMin && absErr > microErr) {
    pwm = microMin;
  }
  pwm = constrain(pwm, 0, cap);

  if (forward) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else         { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
  ledcWrite(PWMA, pwm);
}

void driveMotorLeft(int speed) {
  speed *= MOTOR_L_SIGN;
  bool forward = (speed >= 0);
  int pwm = abs(speed);
  float absErr = fabsf(gBalanceError);
  int cap = (absErr >= CATCH_ERR) ? PWM_CATCH : (forward ? PWM_MAX : PWM_MAX_BACK);
  int microMin = forward ? PWM_MICRO : PWM_MICRO_REAR;
  float microErr = MICRO_ERR;
  if (absErr < MICRO_ZONE && absErr < CATCH_ERR && pwm > 0 && pwm < microMin && absErr > microErr) {
    pwm = microMin;
  }
  pwm = constrain(pwm, 0, cap);

  if (forward) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else         { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  ledcWrite(PWMB, pwm);
}