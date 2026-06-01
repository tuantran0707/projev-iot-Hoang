/*
 * ============================================================================
 *  Thiết bị đeo phát hiện ngã thông minh — Smart Fall Detection
 *  MCU   : ESP32-S3 (chạy được cả ESP32 thường)
 *  Sensor: MPU6050 / MPU6500 (đọc I2C thô — chung thanh ghi, không cần lib riêng)
 *  Cảnh báo: Motor rung + gửi sự kiện lên IoT server (ThingsBoard Cloud).
 *            Nút bấm để huỷ cảnh báo trong 10s (chống báo nhầm).
 *
 *  THUẬT TOÁN (rule-based): Free-fall -> Impact -> Bất động
 *  Ngưỡng mặc định lấy theo các nghiên cứu/dataset phát hiện ngã phổ biến
 *  (SisFall, MobiFall, UMAFall): free-fall ~0.5g, impact ~2.5–3g, bất động ~2s.
 * ============================================================================
 */

#include <Wire.h>

// ----------------------------- CẤU HÌNH CHÂN -------------------------------
// ⚠️ Sửa lại đúng GPIO trên board ESP32-S3 mini của bạn
#define PIN_SDA        8     // I2C SDA
#define PIN_SCL        9     // I2C SCL
#define PIN_MOTOR      4     // Chân điều khiển motor rung (qua transistor)
#define PIN_BUTTON     5     // Nút bấm huỷ cảnh báo (INPUT_PULLUP -> nối GND)
#define PIN_LED        2     // LED báo trạng thái (tuỳ chọn)

// ----------------------------- ĐỊA CHỈ MPU ---------------------------------
#define MPU_ADDR       0x68  // AD0=GND -> 0x68 ; AD0=VCC -> 0x69
#define REG_PWR_MGMT_1 0x6B
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG     0x1A
#define REG_GYRO_CFG   0x1B
#define REG_ACCEL_CFG  0x1C
#define REG_ACCEL_XOUT 0x3B
#define REG_WHO_AM_I   0x75

// ----------------------------- THÔNG SỐ ĐO ---------------------------------
// Accel range ±16g để KHÔNG bị "clip" lúc va chạm (impact có thể >8g)
//   ±16g -> 2048 LSB/g  => g = raw / 2048.0
// Gyro range ±500 dps -> 65.5 LSB/(deg/s)
const float ACC_SENS  = 2048.0f;   // LSB/g  (±16g)
const float GYR_SENS  = 65.5f;     // LSB/dps (±500)

// ----------------------- THAM SỐ THUẬT TOÁN (ngưỡng) -----------------------
// Lấy theo literature, có thể tinh chỉnh:
const float FREEFALL_G   = 0.5f;   // a < 0.5g  -> nghi rơi tự do
const float IMPACT_G     = 2.6f;   // a > 2.6g  -> nghi va chạm
const uint32_t FREEFALL_TO_IMPACT_MS = 600;   // cửa sổ free-fall -> impact
const uint32_t IMMOBILE_WINDOW_MS    = 2500;  // theo dõi bất động sau va chạm
const float IMMOBILE_STD_G = 0.12f;           // std(a) nhỏ => nằm yên
const float IMMOBILE_MEAN_LOW  = 0.80f;       // mean(a) quanh 1g (0.8–1.2)
const float IMMOBILE_MEAN_HIGH = 1.20f;
const uint32_t ALERT_TIMEOUT_MS = 10000;      // 10s cảnh báo trước khi gửi TG

const uint16_t SAMPLE_HZ = 100;               // tần số lấy mẫu
const uint16_t SAMPLE_DT_MS = 1000 / SAMPLE_HZ;

// ----------------------------- IoT SERVER (ThingsBoard) --------------------
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

const char* WIFI_SSID   = "iot";
const char* WIFI_PASS   = "12345678";
const char* TB_HOST     = "thingsboard.cloud";
const char* TB_TOKEN    = "TUGjdF8ZNvUKG2J2uOwe";   // Device access token
// Endpoint: https://thingsboard.cloud/api/v1/<TOKEN>/telemetry

// ----------------------------- TRẠNG THÁI ----------------------------------
enum State { ST_NORMAL, ST_SUSPECT_IMPACT, ST_MONITOR_IMMOBILE, ST_ALERT };
State state = ST_NORMAL;

uint32_t freefallTime = 0;
bool     freefallSeen = false;
uint32_t monitorStart = 0;
uint32_t alertStart   = 0;
uint32_t lastSample   = 0;

// Buffer thống kê trong cửa sổ theo dõi bất động
const uint16_t MAXBUF = SAMPLE_HZ * 3;   // 3 giây
float magBuf[MAXBUF];
uint16_t bufIdx = 0;
float lastMean = 1.0f, lastStd = 0.0f;   // lưu kết quả cửa sổ để gửi server

// =========================================================================
//                            HÀM ĐỌC MPU
// =========================================================================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpuRead8(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return Wire.read();
}

void mpuInit() {
  mpuWrite(REG_PWR_MGMT_1, 0x00);   // wake up
  delay(100);
  mpuWrite(REG_SMPLRT_DIV, 0x09);   // sample rate divider
  mpuWrite(REG_CONFIG,     0x03);   // DLPF ~44Hz (lọc nhiễu)
  mpuWrite(REG_GYRO_CFG,   0x08);   // ±500 dps
  mpuWrite(REG_ACCEL_CFG,  0x18);   // ±16 g
  delay(50);

  uint8_t who = mpuRead8(REG_WHO_AM_I);
  Serial.print("WHO_AM_I = 0x"); Serial.println(who, HEX);
  // MPU6050 -> 0x68 ; MPU6500 -> 0x70 ; MPU9250 -> 0x71 (giá trị tham khảo)
}

// Đọc 6 trục, trả gia tốc (g) và gyro (dps)
void mpuReadAll(float &ax, float &ay, float &az,
                float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                       // bỏ qua nhiệt độ
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = rawAx / ACC_SENS;  ay = rawAy / ACC_SENS;  az = rawAz / ACC_SENS;
  gx = rawGx / GYR_SENS;  gy = rawGy / GYR_SENS;  gz = rawGz / GYR_SENS;
}

// =========================================================================
//                            ĐIỀU KHIỂN NGOẠI VI
// =========================================================================
void motorOn()  { digitalWrite(PIN_MOTOR, HIGH); }
void motorOff() { digitalWrite(PIN_MOTOR, LOW);  }
bool buttonPressed() { return digitalRead(PIN_BUTTON) == LOW; } // PULLUP

// Kết nối WiFi (gọi 1 lần ở setup, và tự nối lại khi cần)
bool ensureWiFi(uint32_t timeoutMs = 8000) {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
    delay(200); Serial.print(".");
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

// Gửi sự kiện NGÃ lên ThingsBoard (telemetry JSON)
void sendFallToServer(float mean, float stdev) {
  if (!ensureWiFi()) {
    Serial.println("WiFi chưa kết nối -> không gửi được.");
    return;
  }
  WiFiClientSecure client;
  client.setInsecure();                 // bỏ kiểm tra chứng chỉ (đơn giản hoá)

  HTTPClient https;
  String url = String("https://") + TB_HOST + "/api/v1/" + TB_TOKEN + "/telemetry";

  String payload = String("{\"fall\":true,\"status\":\"FALL\",")
                 + "\"acc_mean\":" + String(mean, 2) + ","
                 + "\"acc_std\":"  + String(stdev, 2) + "}";

  if (https.begin(client, url)) {
    https.addHeader("Content-Type", "application/json");
    int code = https.POST(payload);
    Serial.printf("ThingsBoard POST -> HTTP %d\n", code);
    https.end();
  } else {
    Serial.println("Không mở được kết nối tới ThingsBoard.");
  }
}

// Tính mean & std của buffer độ lớn gia tốc
void bufStats(float &mean, float &stdev) {
  if (bufIdx == 0) { mean = 1.0f; stdev = 0.0f; return; }
  float s = 0;
  for (uint16_t i = 0; i < bufIdx; i++) s += magBuf[i];
  mean = s / bufIdx;
  float v = 0;
  for (uint16_t i = 0; i < bufIdx; i++) { float d = magBuf[i]-mean; v += d*d; }
  stdev = sqrt(v / bufIdx);
}

// =========================================================================
//                                SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_MOTOR, OUTPUT);  motorOff();
  pinMode(PIN_LED, OUTPUT);    digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  mpuInit();

  // Kết nối WiFi để sẵn sàng gửi ThingsBoard
  Serial.print("Đang kết nối WiFi");
  if (ensureWiFi(10000)) {
    Serial.print("WiFi OK, IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi chưa nối được (sẽ thử lại khi có cảnh báo).");
  }

  Serial.println("Fall Detection sẵn sàng.");
}

// =========================================================================
//                                 LOOP
// =========================================================================
void loop() {
  // Lấy mẫu đều theo SAMPLE_DT_MS
  if (millis() - lastSample < SAMPLE_DT_MS) return;
  lastSample = millis();

  float ax, ay, az, gx, gy, gz;
  mpuReadAll(ax, ay, az, gx, gy, gz);
  float a = sqrt(ax*ax + ay*ay + az*az);   // độ lớn gia tốc (g) — SVM

  switch (state) {

    // -------- BÌNH THƯỜNG: tìm free-fall rồi impact --------
    case ST_NORMAL: {
      if (a < FREEFALL_G) {                 // rơi tự do
        freefallSeen = true;
        freefallTime = millis();
      }
      // Impact: ưu tiên sau free-fall, nhưng impact mạnh đơn lẻ cũng xét
      bool impact = (a > IMPACT_G);
      bool afterFF = freefallSeen &&
                     (millis() - freefallTime < FREEFALL_TO_IMPACT_MS);
      if (impact && (afterFF || a > IMPACT_G + 0.5f)) {
        Serial.printf("Impact a=%.2fg -> theo dõi bất động\n", a);
        state = ST_MONITOR_IMMOBILE;
        monitorStart = millis();
        bufIdx = 0;
        freefallSeen = false;
        digitalWrite(PIN_LED, HIGH);
      }
      // reset cờ free-fall nếu quá hạn
      if (freefallSeen && millis() - freefallTime > FREEFALL_TO_IMPACT_MS)
        freefallSeen = false;
      break;
    }

    // -------- THEO DÕI BẤT ĐỘNG ~2.5s sau va chạm --------
    case ST_MONITOR_IMMOBILE: {
      if (bufIdx < MAXBUF) magBuf[bufIdx++] = a;

      if (millis() - monitorStart >= IMMOBILE_WINDOW_MS) {
        float mean, stdev;
        bufStats(mean, stdev);
        lastMean = mean; lastStd = stdev;   // lưu để gửi server
        bool immobile = (stdev < IMMOBILE_STD_G) &&
                        (mean > IMMOBILE_MEAN_LOW) &&
                        (mean < IMMOBILE_MEAN_HIGH);
        Serial.printf("Cửa sổ: mean=%.2f std=%.2f immobile=%d\n",
                      mean, stdev, immobile);

        if (immobile) {
          Serial.println("=> XÁC NHẬN NGÃ. Bắt đầu cảnh báo!");
          state = ST_ALERT;
          alertStart = millis();
        } else {
          Serial.println("=> Không phải ngã (cử động/thể thao). Bỏ qua.");
          state = ST_NORMAL;
          digitalWrite(PIN_LED, LOW);
        }
      }
      break;
    }

    // -------- CẢNH BÁO: rung 10s, chờ nút bấm --------
    case ST_ALERT: {
      // Rung nhịp: 200ms on / 200ms off
      bool on = ((millis() / 200) % 2) == 0;
      digitalWrite(PIN_MOTOR, on ? HIGH : LOW);
      digitalWrite(PIN_LED,   on ? HIGH : LOW);

      if (buttonPressed()) {                // người dùng huỷ -> ổn
        Serial.println("Người dùng tắt cảnh báo (an toàn).");
        motorOff(); digitalWrite(PIN_LED, LOW);
        state = ST_NORMAL;
        delay(300);                         // chống dội nút
        break;
      }

      if (millis() - alertStart >= ALERT_TIMEOUT_MS) {
        motorOff();
        Serial.println("Hết 10s chưa tắt -> gửi cảnh báo lên IoT server!");
        sendFallToServer(lastMean, lastStd);
        // Sau khi gửi: rung dài 1 lần rồi về bình thường
        motorOn(); delay(800); motorOff();
        state = ST_NORMAL;
        digitalWrite(PIN_LED, LOW);
      }
      break;
    }
  }
}
