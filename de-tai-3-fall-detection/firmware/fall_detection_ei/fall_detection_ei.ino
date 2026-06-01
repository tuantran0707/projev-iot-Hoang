/*
 * ============================================================================
 *  Thiết bị đeo phát hiện ngã — PHIÊN BẢN DÙNG MÔ HÌNH EDGE IMPULSE (TinyML)
 *  MCU   : ESP32 / ESP32-S3 (board CYD ESP32-2432S024)
 *  Sensor: MPU6050 / MPU6500 (đọc I2C thô)
 *  AI    : Mô hình phân loại Edge Impulse chạy on-device (run_classifier)
 *          Nhãn: "binh_thuong" / "the_thao" / "NGA"
 *  IoT   : Đẩy kết quả lên ThingsBoard Cloud qua MQTT.
 *
 *  >>> CÁCH DÙNG <<<
 *  1. Train mô hình trên https://www.edgeimpulse.com/ (accelerometer 3 trục,
 *     window 2000ms / stride 200ms, 3 nhãn ở trên).
 *  2. Export "Arduino library" và cài vào Arduino IDE (Sketch > Include Library
 *     > Add .ZIP Library). Thư viện có tên dạng <ProjectName>_inferencing.h.
 *  3. Sửa dòng #include bên dưới cho ĐÚNG tên thư viện của bạn.
 *  4. Đảm bảo EI_CLASSIFIER_RAW_SAMPLE_COUNT, _FREQUENCY khớp cấu hình train.
 * ============================================================================
 */

// >>> SỬA DÒNG NÀY theo tên project Edge Impulse của bạn <<<
#include "FallDetection_inferencing.h"   // thư viện inference của mô hình (cùng thư mục sketch)

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ----------------------------- CẤU HÌNH CHÂN -------------------------------
#define PIN_SDA        21
#define PIN_SCL        22
#define PIN_BUTTON     0      // nút BOOT (GPIO0) - huỷ cảnh báo
#define PIN_RGB_R      4      // LED RGB onboard (chung anode: BẬT = LOW)
#define PIN_RGB_G      16
#define PIN_RGB_B      17

// ----------------------------- ĐỊA CHỈ MPU ---------------------------------
#define MPU_ADDR       0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG     0x1A
#define REG_GYRO_CFG   0x1B
#define REG_ACCEL_CFG  0x1C
#define REG_ACCEL_XOUT 0x3B
#define REG_WHO_AM_I   0x75

const float ACC_SENS = 2048.0f;   // ±16g -> 2048 LSB/g

// ----------------------- THAM SỐ SUY LUẬN (AI) -----------------------------
// Số trục mỗi mẫu (ax, ay, az). Nếu train kèm gyro thì đổi thành 6.
#define AXES_PER_SAMPLE   3
// Ngưỡng xác suất để kết luận "NGÃ"
const float FALL_CONFIDENCE = 0.70f;
// Sau khi AI báo ngã, chờ xác nhận bất động trước khi cảnh báo (ms)
const uint32_t ALERT_TIMEOUT_MS = 60000;   // 60s để huỷ bằng nút BOOT

// Buffer dữ liệu cho 1 cửa sổ inference của Edge Impulse
static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static size_t feature_ix = 0;

const uint16_t SAMPLE_HZ = EI_CLASSIFIER_FREQUENCY;     // tần số train (vd 100Hz)
const uint16_t SAMPLE_DT_MS = 1000 / SAMPLE_HZ;
uint32_t lastSample = 0;

// ----------------------------- IoT (ThingsBoard) ---------------------------
const char* WIFI_SSID   = "iot";
const char* WIFI_PASS   = "12345678";
const char* TB_HOST     = "thingsboard.cloud";
const int   TB_MQTT_PORT = 1883;
const char* TB_TOKEN    = "TUGjdF8ZNvUKG2J2uOwe";
const char* TB_TELEMETRY_TOPIC = "v1/devices/me/telemetry";

WiFiClient   mqttNet;
PubSubClient mqtt(mqttNet);

// ----------------------------- TRẠNG THÁI ----------------------------------
bool inAlert = false;
uint32_t alertStart = 0;
uint32_t lastHeartbeat = 0;

// =========================================================================
//                            HÀM ĐỌC MPU
// =========================================================================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
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
  mpuWrite(REG_PWR_MGMT_1, 0x00); delay(100);
  mpuWrite(REG_SMPLRT_DIV, 0x09);
  mpuWrite(REG_CONFIG,     0x03);
  mpuWrite(REG_GYRO_CFG,   0x08);   // ±500 dps
  mpuWrite(REG_ACCEL_CFG,  0x18);   // ±16 g
  delay(50);
  uint8_t who = mpuRead8(REG_WHO_AM_I);
  Serial.print("WHO_AM_I = 0x"); Serial.println(who, HEX);
}
// Đọc gia tốc (đơn vị g)
void mpuReadAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  int16_t rx = (Wire.read() << 8) | Wire.read();
  int16_t ry = (Wire.read() << 8) | Wire.read();
  int16_t rz = (Wire.read() << 8) | Wire.read();
  ax = rx / ACC_SENS;
  ay = ry / ACC_SENS;
  az = rz / ACC_SENS;
}

// =========================================================================
//                            LED RGB (active-LOW)
// =========================================================================
void setRGB(bool r, bool g, bool b) {
  digitalWrite(PIN_RGB_R, r ? LOW : HIGH);
  digitalWrite(PIN_RGB_G, g ? LOW : HIGH);
  digitalWrite(PIN_RGB_B, b ? LOW : HIGH);
}
void ledGreen() { setRGB(false, true,  false); }
void ledRed()   { setRGB(true,  false, false); }
void ledOff()   { setRGB(false, false, false); }

// =========================================================================
//                            WiFi + MQTT
// =========================================================================
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
}
void ensureMqtt() {
  if (mqtt.connected()) return;
  mqtt.setServer(TB_HOST, TB_MQTT_PORT);
  String cid = "esp32-fall-ei-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  mqtt.connect(cid.c_str(), TB_TOKEN, nullptr);
}
void publishResult(const char* status, bool fall, float confidence) {
  ensureWiFi(); ensureMqtt();
  if (!mqtt.connected()) return;
  char payload[160];
  snprintf(payload, sizeof(payload),
           "{\"fall\":%s,\"status\":\"%s\",\"confidence\":%.2f,\"model\":\"edge-impulse\"}",
           fall ? "true" : "false", status, confidence);
  mqtt.publish(TB_TELEMETRY_TOPIC, payload);
  Serial.print("MQTT -> "); Serial.println(payload);
}

// =========================================================================
//          CALLBACK ĐỂ EDGE IMPULSE ĐỌC DỮ LIỆU TỪ BUFFER features[]
// =========================================================================
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

// =========================================================================
//                            SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  ledGreen();
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  mpuInit();

  ensureWiFi();
  ensureMqtt();

  Serial.println("Edge Impulse fall detection - san sang.");
  Serial.print("Window samples = "); Serial.println(EI_CLASSIFIER_RAW_SAMPLE_COUNT);
  Serial.print("Frame size     = "); Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
}

// =========================================================================
//                            LOOP
// =========================================================================
void loop() {
  mqtt.loop();

  // --- Nút BOOT huỷ cảnh báo ---
  if (inAlert && digitalRead(PIN_BUTTON) == LOW) {
    inAlert = false;
    ledGreen();
    publishResult("CANCELLED", false, 0.0f);
    delay(300);   // chống dội phím
  }

  // --- Trong lúc cảnh báo: nháy đỏ + đếm timeout ---
  if (inAlert) {
    // nháy đỏ
    static uint32_t blinkT = 0; static bool on = false;
    if (millis() - blinkT > 200) { blinkT = millis(); on = !on; on ? ledRed() : ledOff(); }
    if (millis() - alertStart > ALERT_TIMEOUT_MS) {
      inAlert = false;
      ledGreen();
      publishResult("FALL", true, 1.0f);   // hết 60s chưa huỷ -> gửi cảnh báo
    }
    return;   // khi đang cảnh báo thì không thu mẫu mới
  }

  // --- Thu mẫu đúng tần số train ---
  if (millis() - lastSample < SAMPLE_DT_MS) return;
  lastSample = millis();

  float ax, ay, az;
  mpuReadAccel(ax, ay, az);

  // Đưa mẫu vào buffer đặc trưng (theo đúng thứ tự trục lúc train)
  if (feature_ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    features[feature_ix++] = ax;
    features[feature_ix++] = ay;
    features[feature_ix++] = az;
  }

  // Khi đã đủ 1 cửa sổ -> chạy inference
  if (feature_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    feature_ix = 0;

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &raw_feature_get_data;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    if (err != EI_IMPULSE_OK) {
      Serial.print("run_classifier loi: "); Serial.println(err);
      return;
    }

    // Tìm nhãn có xác suất cao nhất
    float bestVal = 0.0f; const char* bestLabel = "?"; float fallProb = 0.0f;
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      float v = result.classification[i].value;
      const char* lbl = result.classification[i].label;
      if (v > bestVal) { bestVal = v; bestLabel = lbl; }
      if (strcmp(lbl, "NGA") == 0) fallProb = v;
    }

    Serial.printf("AI -> %s (%.2f) | NGA=%.2f\n", bestLabel, bestVal, fallProb);

    // Quyết định
    if (fallProb >= FALL_CONFIDENCE) {
      inAlert = true;
      alertStart = millis();
      Serial.println(">>> PHAT HIEN NGA (Edge Impulse)");
      publishResult("FALL_SUSPECT", true, fallProb);   // bắt đầu đếm 60s
    } else {
      // STILL / MOVING theo nhãn còn lại
      bool moving = (strcmp(bestLabel, "the_thao") == 0);
      publishResult(moving ? "MOVING" : "STILL", false, bestVal);
      ledGreen();
    }
  }

  // --- Heartbeat 5s (giữ kết nối + báo sống) ---
  if (millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    ensureWiFi(); ensureMqtt();
  }
}
