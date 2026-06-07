/* =============================================================================
 *  XE TỰ CÂN BẰNG 2 BÁNH  -  ESP32 + MPU6050/9250 + TB6612FNG + 2 Encoder + 2 Servo
 * -----------------------------------------------------------------------------
 *  Cấu trúc điều khiển: PID lồng nhau (cascade)
 *    - Vòng TRONG  : PID GÓC  -> giữ thân xe đứng thẳng (quan trọng nhất)
 *    - Vòng NGOÀI  : PID TỐC ĐỘ/VỊ TRÍ (từ encoder) -> chống xe trôi đi
 *
 *  MAP CHÂN:
 *    I2C MPU : SDA=21, SCL=22
 *    TB6612  : PWMA=25 AIN2=26 AIN1=27  (Motor PHẢI)
 *              PWMB=13 BIN1=14 BIN2=12  (Motor TRÁI)   STBY -> 3V3
 *    Encoder PHẢI : C1=33, C2=32
 *    Encoder TRÁI : C1=34, C2=35   (GPIO34/35 chỉ INPUT, KHÔNG có trở kéo nội
 *                                   -> phải gắn TRỞ KÉO LÊN 10k ra 3V3 cho 2 chân này)
 *    Servo   : RIGHT=18, LEFT=19
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
#define AIN2 26
#define AIN1 27

// Động cơ Trái (TB6612FNG - kênh B)
#define BIN1 14
#define BIN2 12
#define PWMB 13

// Encoder
#define ENC_R_C1 33
#define ENC_R_C2 32
#define ENC_L_C1 34
#define ENC_L_C2 35

// Servo
#define SERVO_RIGHT_PIN 18
#define SERVO_LEFT_PIN  19

// ===================== THÔNG SỐ CẦN TINH CHỈNH (TUNE) =====================
// 1) Chọn TRỤC cân bằng: trục mà GÓC thay đổi NHIỀU khi xe ngả tới/lui.
//    Cho xe nghiêng tới/lui bằng tay rồi xem Serial: trục nào nhảy mạnh thì chọn.
//      0 = dùng AngleX   |   1 = dùng AngleY
#define BALANCE_AXIS_Y 1

// 2) Góc cân bằng (điểm thăng bằng) - đo khi xe ĐỨNG THẲNG đứng yên (độ).
//    Từ log: khi giữ xe đứng cân thật, Angle ≈ 1.0 -> đặt điểm cân bằng = 1.0
//    Tinh chỉnh thêm qua Serial bằng lệnh: s1.0 / s0.5 / s1.5 ...
float ANGLE_SETPOINT = 1.0;

// 3) Ngưỡng góc ngã: |góc| vượt ngưỡng này -> tắt động cơ (xe đã đổ).
#define FALL_LIMIT 45.0

// 4) PID GÓC (vòng trong) - đây là 3 hệ số QUAN TRỌNG NHẤT cần dò:
//    Cách dò: Ki=0, Kd=0. Tăng Kp tới khi xe rung quanh thẳng đứng.
//             Tăng Kd để dập rung. Cuối cùng thêm Ki nhỏ để hết trôi góc.
float Kp = 15.0;
float Ki = 0.0;
float Kd = 2.5;

// 5) PID TỐC ĐỘ/VỊ TRÍ (vòng ngoài, từ encoder) - CHỐNG XE TRÔI/CHÚI 1 HƯỚNG.
//    >>> TẠM TẮT (=0): vì encoder PHẢI và TRÁI đang đếm NGƯỢC DẤU nhau khi xe đi cùng
//        hướng -> tổng quãng đường sai. Phải sửa dấu encoder xong mới bật lại.
//    Sau khi 2 encoder đếm cùng dấu, bật lại: Kp_pos=0.003, Kd_pos=0.030
float Kp_pos = 0.0;
float Kd_pos = 0.0;
#define POS_SIGN 1     // đảo chiều vòng ngoài nếu xe trôi nhanh hơn

// 6) Giới hạn PWM (TB6612 dùng 8-bit: 0..255).
#define PWM_MAX 255
// PWM tối thiểu để thắng ma sát/khởi động (deadband). Tăng nếu motor ì.
#define PWM_MIN 0

// 7) Đảo chiều motor.
//    MOTOR_SIGN  : đảo chiều CHUNG cả 2 bánh (nếu xe đẩy sai hướng -> ngã nhanh hơn)
//    MOTOR_R/L_SIGN: đảo chiều RIÊNG từng bánh.
//    >>> XE CỦA BẠN: 1 bánh tiến 1 bánh lùi -> đổi DẤU 1 trong 2 dòng dưới (1 <-> -1)
#define MOTOR_SIGN   -1
#define MOTOR_R_SIGN  1
#define MOTOR_L_SIGN -1

// Đảo chiều ĐẾM encoder cho khớp chiều quay thực tế (1 hoặc -1)
#define ENC_R_SIGN 1
#define ENC_L_SIGN -1

// 8) Chu kỳ vòng điều khiển (ms). 5ms = 200Hz, hợp cho cân bằng.
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
float pidIntegral = 0.0;
float lastAngle = 0.0;
unsigned long lastLoop = 0;
unsigned long lastPrint = 0;

bool balancing = false;   // cờ: chỉ chạy động cơ khi xe đủ gần thẳng đứng

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

// ----------------- CHỈNH PID REALTIME QUA SERIAL -----------------
// Gõ vào Serial Monitor (kèm Enter), ví dụ:
//   p12     -> đặt Kp = 12
//   i0.3    -> đặt Ki = 0.3
//   d2.5    -> đặt Kd = 2.5
//   s-3.5   -> đặt ANGLE_SETPOINT = -3.5 (điểm cân bằng)
//   ?       -> in các giá trị hiện tại
void handleSerialTuning() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '?') {
    Serial.print(">> Kp="); Serial.print(Kp);
    Serial.print(" Ki=");   Serial.print(Ki);
    Serial.print(" Kd=");   Serial.print(Kd);
    Serial.print(" Set=");  Serial.println(ANGLE_SETPOINT);
    return;
  }
  if (c=='p'||c=='i'||c=='d'||c=='s') {
    float v = Serial.parseFloat();
    switch (c) {
      case 'p': Kp = v; break;
      case 'i': Ki = v; pidIntegral = 0; break;
      case 'd': Kd = v; break;
      case 's': ANGLE_SETPOINT = v; break;
    }
    Serial.print(">> Cap nhat: Kp="); Serial.print(Kp);
    Serial.print(" Ki="); Serial.print(Ki);
    Serial.print(" Kd="); Serial.print(Kd);
    Serial.print(" Set="); Serial.println(ANGLE_SETPOINT);
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
  pinMode(ENC_L_C1, INPUT);   // GPIO34: không có pullup nội
  pinMode(ENC_L_C2, INPUT);   // GPIO35: không có pullup nội
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
  Serial.println(">>> San sang! Dat xe gan thang dung de bat dau can bang.");

  lastLoop = millis();
}

void loop() {
  mpu6050.update();
  handleSerialTuning();   // cho phép chỉnh Kp/Ki/Kd/Set qua Serial lúc đang chạy
  mqttEnsureConnected();  // duy tri ket noi ThingsBoard (khong chan vong can bang)

  unsigned long now = millis();
  if (now - lastLoop < DT_MS) return;   // giữ chu kỳ cố định
  float dt = (now - lastLoop) / 1000.0; // giây
  lastLoop = now;

  float angle = getBalanceAngle();

  // --- An toàn: nếu ngã quá ngưỡng thì tắt động cơ, reset tích phân ---
  if (fabs(angle - ANGLE_SETPOINT) > FALL_LIMIT) {
    balancing = false;
    pidIntegral = 0;
    setMotors(0, 0);
  } else {
    // Bật cân bằng lại khi xe được dựng gần thẳng (±3°)
    if (!balancing && fabs(angle - ANGLE_SETPOINT) < 3.0) {
      balancing = true;
      pidIntegral = 0;
      encCountR = 0;
      encCountL = 0;
    }
  }

  if (balancing) {
    // ---------- VÒNG NGOÀI: vị trí/tốc độ từ encoder ----------
    long pos = encCountR + encCountL;            // tổng quãng đường
    static long lastPos = 0;
    float vel = (pos - lastPos) / dt;            // tốc độ
    lastPos = pos;
    // Vòng ngoài đẩy nhẹ điểm đặt góc để xe không trôi
    // driveCmd (-100..100) tu ThingsBoard nghieng diem can bang de chay toi/lui
    float driveBias = (driveCmd / 100.0) * MAX_DRIVE_TILT;
    float setpoint = ANGLE_SETPOINT + driveBias + POS_SIGN * ((Kp_pos * pos) + (Kd_pos * vel));

    // ---------- VÒNG TRONG: PID GÓC ----------
    float error = setpoint - angle;
    pidIntegral += error * dt;
    pidIntegral = constrain(pidIntegral, -300, 300); // chống bão hòa tích phân
    // D-term lấy từ vận tốc góc gyro (mượt hơn nhiều so với vi phân góc)
    float rate = getBalanceRate();
    lastAngle = angle;

    float output = (Kp * error) + (Ki * pidIntegral) - (Kd * rate);
    output *= MOTOR_SIGN;
    output = constrain(output, -PWM_MAX, PWM_MAX);

    setMotors((int)output, (int)output);
  }

  // In trạng thái 5 lần/giây (không chặn vòng điều khiển)
  if (now - lastPrint >= 200) {
    lastPrint = now;
    Serial.print("Angle: ");  Serial.print(angle, 2);
    Serial.print("\tSet: ");  Serial.print(ANGLE_SETPOINT, 2);
    Serial.print("\tEncR: "); Serial.print(encCountR);
    Serial.print("\tEncL: "); Serial.print(encCountL);
    Serial.print("\tDrive: "); Serial.print(driveCmd, 0);
    Serial.print("\tWiFi: "); Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "--");
    Serial.print("\tBal: ");  Serial.println(balancing ? "ON" : "OFF");
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
  driveMotorRight(speedR);
  driveMotorLeft(speedL);
}

void driveMotorRight(int speed) {
  speed *= MOTOR_R_SIGN;
  bool forward = (speed >= 0);
  int pwm = abs(speed);
  if (pwm > 0 && pwm < PWM_MIN) pwm = PWM_MIN;   // vượt deadband
  pwm = constrain(pwm, 0, PWM_MAX);

  if (forward) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else         { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); }
  ledcWrite(PWMA, pwm);
}

void driveMotorLeft(int speed) {
  speed *= MOTOR_L_SIGN;
  bool forward = (speed >= 0);
  int pwm = abs(speed);
  if (pwm > 0 && pwm < PWM_MIN) pwm = PWM_MIN;
  pwm = constrain(pwm, 0, PWM_MAX);

  if (forward) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else         { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  ledcWrite(PWMB, pwm);
}