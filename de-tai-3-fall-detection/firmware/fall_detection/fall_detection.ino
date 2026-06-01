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
#include <TFT_eSPI.h>   // Màn hình ILI9341 (cấu hình chân trong User_Setup.h của thư viện)

TFT_eSPI tft = TFT_eSPI();   // Đối tượng điều khiển màn hình

// ----------------------------- CẤU HÌNH CHÂN -------------------------------
// Board: ESP32-2432S024 (CYD - Cheap Yellow Display 2.4")
// ⚠️ Màn hình chiếm sẵn: SPI(12/13/14) + CS15 + DC2 + BL27. KHÔNG dùng các chân này.
#define PIN_SDA        21    // I2C SDA (connector mở rộng)
#define PIN_SCL        22    // I2C SCL (connector mở rộng)
#define PIN_MOTOR      4     // Motor rung (trùng kênh Đỏ của LED RGB) - qua transistor
#define PIN_BUTTON     0     // Nut bam SW1 (nut BOOT onboard = GPIO0), INPUT_PULLUP -> an ve GND

// LED RGB onboard (CYD): chung anode -> BẬT = mức LOW, TẮT = mức HIGH
#define PIN_RGB_R      4     // Kênh Đỏ  (GPIO4)
#define PIN_RGB_G      16    // Kênh Lục  (GPIO16)
#define PIN_RGB_B      17    // Kênh Lam  (GPIO17)

// ----------------------- MÀN HÌNH TFT 2.4" (ILI9341) -----------------------
// Hiển thị SPI (ILI9341) qua thư viện TFT_eSPI.
// ⚠️ QUAN TRỌNG: TFT_eSPI đọc cấu hình chân từ User_Setup.h BÊN TRONG thư viện,
//    KHÔNG khai báo chân TFT ở đây để tránh đè macro. Chân thật của CYD S024:
//      TFT_MISO 12 / TFT_MOSI 13 / TFT_SCLK 14 / TFT_CS 15 / TFT_DC 2 / TFT_RST -1 / TFT_BL 27

// ----------------------- CẢM ỨNG ĐIỆN TRỞ (XPT2046) -----------------------
#define XPT2046_IRQ    36    // T_IRQ
#define XPT2046_MOSI   13    // T_DIN  (chung SPI với màn hình)
#define XPT2046_MISO   12    // T_OUT  (chung SPI với màn hình)
#define XPT2046_CLK    14    // T_CLK  (chung SPI với màn hình)
#define XPT2046_CS     33    // T_CS

#define SCREEN_WIDTH   240
#define SCREEN_HEIGHT  320
#define FONT_SIZE      2

// ----------------------- GIAO DIỆN DASHBOARD -------------------------------
const char* TITLE_LINE1 = "DONG HO CANH BAO";
const char* TITLE_LINE2 = "PHAT HIEN NGA";
#define HDR_H   46    // chiều cao thanh tiêu đề
#define CLK_H   40    // chiều cao vùng đồng hồ
#define FOOT_H  60    // chiều cao chân trang (gia tốc live)
uint16_t curPanelBg = TFT_BLACK;   // màu nền panel hiện tại

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
const uint32_t SETTLE_AFTER_IMPACT_MS = 1000; // bỏ qua giai đoạn rung/trượt NGAY sau va chạm
const uint32_t IMMOBILE_WINDOW_MS    = 2000;  // theo dõi bất động (sau khi đã settle)
const float IMMOBILE_STD_G = 0.20f;           // std(a) nhỏ => nằm yên (cho phép cử động NHẸ khi đã ngã)
const float IMMOBILE_MEAN_LOW  = 0.70f;       // mean(a) quanh 1g
const float IMMOBILE_MEAN_HIGH = 1.30f;

// Phân loại VẬN ĐỘNG khi bình thường (đứng yên / di chuyển - thể thao)
// Đo độ DAO ĐỘNG so với đường nền trượt (khử bias khi thiết bị nghiêng, vì |a| khi đứng yên
// có thể ≠ 1.0g). Đứng yên -> dao động nhỏ ; di chuyển liên tục -> dao động lớn.
const float ACTIVITY_ON_G  = 0.12f;  // dao động (EMA) vượt -> đang vận động
const float ACTIVITY_OFF_G = 0.06f;  // hạ xuống dưới -> đứng yên (hysteresis chống nhấp nháy)
const uint32_t ALERT_TIMEOUT_MS = 60000;      // 60s cảnh báo trước khi gửi (hoặc ấn nút để tắt)

const uint16_t SAMPLE_HZ = 100;               // tần số lấy mẫu
const uint16_t SAMPLE_DT_MS = 1000 / SAMPLE_HZ;

// ----------------------------- IoT SERVER (ThingsBoard / MQTT) -------------
#include <WiFi.h>
#include <PubSubClient.h>   // MQTT client (cài qua Library Manager: "PubSubClient" by Nick O'Leary)

const char* WIFI_SSID   = "iot";
const char* WIFI_PASS   = "12345678";
const char* TB_HOST     = "thingsboard.cloud";
const int   TB_MQTT_PORT = 1883;                    // MQTT không TLS
const char* TB_TOKEN    = "TUGjdF8ZNvUKG2J2uOwe";   // Device access token = MQTT username
const char* TB_TELEMETRY_TOPIC = "v1/devices/me/telemetry";

WiFiClient    mqttNet;
PubSubClient  mqtt(mqttNet);

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

// Phân loại vận động (chỉ dùng ở trạng thái ST_NORMAL)
float magBaseline = 1.0f; // đường nền trượt của |a| (bám theo hướng đặt/trọng lực hiện tại)
float actEMA  = 0.0f;     // trung bình trượt độ lệch so với baseline = mức vận động
bool  actInit = false;
bool  isMoving = false;   // true = đang di chuyển/thể thao ; false = đứng yên

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

// LED RGB chung anode (BẬT = LOW). r/g/b = true -> bật kênh đó
void setRGB(bool r, bool g, bool b) {
  digitalWrite(PIN_RGB_R, r ? LOW : HIGH);
  digitalWrite(PIN_RGB_G, g ? LOW : HIGH);
  digitalWrite(PIN_RGB_B, b ? LOW : HIGH);
}
void ledGreen() { setRGB(false, true,  false); } // bình thường
void ledBlue()  { setRGB(false, false, true);  } // va chạm
void ledRed()   { setRGB(true,  false, false); } // ngã
void ledOff()   { setRGB(false, false, false); }

// ---------------------- GIAO DIỆN MÀN HÌNH TFT ----------------------------
// Vẽ thanh tiêu đề (gọi 1 lần lúc khởi động)
void drawHeader() {
  int w = tft.width();
  tft.fillRect(0, 0, w, HDR_H, TFT_NAVY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setTextSize(1);
  tft.drawString(TITLE_LINE1, w / 2, 13);
  tft.setTextColor(TFT_CYAN, TFT_NAVY);
  tft.setTextSize(2);
  tft.drawString(TITLE_LINE2, w / 2, 31);
}

// Cập nhật đồng hồ thời gian thực (NTP) - vùng ngay dưới header
void drawClock() {
  int w = tft.width();
  int y0 = HDR_H;
  char buf[16] = "--:--:--";
  struct tm ti;
  if (getLocalTime(&ti, 5)) strftime(buf, sizeof(buf), "%H:%M:%S", &ti);
  tft.fillRect(0, y0, w, CLK_H, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.setTextSize(3);
  tft.drawString(buf, w / 2, y0 + CLK_H / 2);
}

// Vẽ panel trạng thái chính (giữa màn hình) với màu nền theo trạng thái
void showStatus(const char* title, const char* sub, uint16_t bg, uint16_t fg) {
  curPanelBg = bg;
  int w  = tft.width();
  int y0 = HDR_H + CLK_H;
  int h  = tft.height() - FOOT_H - y0;
  tft.fillRect(0, y0, w, h, bg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(fg, bg);
  tft.setTextSize(3);
  tft.drawString(title, w / 2, y0 + h / 2 - 18);
  if (sub && sub[0]) {
    tft.setTextSize(2);
    tft.drawString(sub, w / 2, y0 + h / 2 + 24);
  }
}

// Vẽ chân trang: gia tốc hiện tại + đỉnh trong cửa sổ
void drawLive(float a, float peak) {
  int w  = tft.width();
  int y0 = tft.height() - FOOT_H;
  tft.fillRect(0, y0, w, FOOT_H, TFT_DARKGREY);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  char l1[24], l2[24];
  snprintf(l1, sizeof(l1), "Gia toc: %.2f g", a);
  snprintf(l2, sizeof(l2), "Dinh:    %.2f g", peak);
  tft.setTextSize(2);
  tft.drawString(l1, w / 2, y0 + 18);
  tft.drawString(l2, w / 2, y0 + 42);
}

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

// Đảm bảo MQTT đã kết nối tới ThingsBoard (token làm username, password để trống)
bool ensureMqtt(uint32_t timeoutMs = 5000) {
  if (mqtt.connected()) return true;
  if (!ensureWiFi()) return false;

  mqtt.setServer(TB_HOST, TB_MQTT_PORT);
  String clientId = String("esp32-fall-") + String((uint32_t)ESP.getEfuseMac(), HEX);

  uint32_t t0 = millis();
  while (!mqtt.connected() && millis() - t0 < timeoutMs) {
    Serial.print("MQTT connecting... ");
    // ThingsBoard: username = device access token, password = rỗng
    if (mqtt.connect(clientId.c_str(), TB_TOKEN, nullptr)) {
      Serial.println("OK");
      return true;
    }
    Serial.printf("fail rc=%d, thử lại...\n", mqtt.state());
    delay(500);
  }
  return mqtt.connected();
}

// Gửi sự kiện NGÃ lên ThingsBoard qua MQTT (telemetry JSON)
void sendFallToServer(float mean, float stdev) {
  if (!ensureMqtt()) {
    Serial.println("MQTT chưa kết nối -> không gửi được.");
    return;
  }

  char payload[96];
  snprintf(payload, sizeof(payload),
           "{\"fall\":true,\"status\":\"FALL\",\"acc_mean\":%.2f,\"acc_std\":%.2f}",
           mean, stdev);

  bool ok = mqtt.publish(TB_TELEMETRY_TOPIC, payload);
  Serial.printf("MQTT publish telemetry -> %s | %s\n", ok ? "OK" : "FAIL", payload);
  mqtt.loop();   // đẩy gói đi ngay
}

// Gửi TRẠNG THÁI hiện tại lên ThingsBoard NGAY khi có thay đổi (không chờ định kỳ)
void publishState(const char* status, bool fall, float acc) {
  if (!ensureMqtt(2000)) {
    Serial.println("MQTT chưa kết nối -> bỏ qua gửi trạng thái.");
    return;
  }
  char payload[96];
  snprintf(payload, sizeof(payload),
           "{\"fall\":%s,\"status\":\"%s\",\"acc\":%.2f}",
           fall ? "true" : "false", status, acc);
  bool ok = mqtt.publish(TB_TELEMETRY_TOPIC, payload);
  Serial.printf("MQTT state -> %s | %s\n", ok ? "OK" : "FAIL", payload);
  mqtt.loop();
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
  // LED RGB onboard (chung anode): khởi tạo và đặt màu XANH LÁ (bình thường)
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  ledGreen();
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // Khởi tạo màn hình TFT
  pinMode(TFT_BL, OUTPUT);  digitalWrite(TFT_BL, HIGH);   // bật đèn nền
  tft.init();
  tft.setRotation(0);                                     // dọc 240x320 (rotation 0 phủ đầy đúng cho panel CYD; KHÔNG dùng 2 vì panel bị lệch offset)
  tft.fillScreen(TFT_BLACK);
  drawHeader();
  drawClock();
  showStatus("KHOI DONG", "Dang khoi tao...", TFT_BLACK, TFT_WHITE);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  mpuInit();

  // Kết nối WiFi để sẵn sàng gửi ThingsBoard
  Serial.print("Đang kết nối WiFi");
  if (ensureWiFi(10000)) {
    Serial.print("WiFi OK, IP: "); Serial.println(WiFi.localIP());
    configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");  // giờ VN GMT+7 qua NTP
    ensureMqtt(5000);   // kết nối MQTT luôn cho sẵn sàng
  } else {
    Serial.println("WiFi chưa nối được (sẽ thử lại khi có cảnh báo).");
  }

  Serial.println("Fall Detection sẵn sàng.");
  showStatus("BINH THUONG", "He thong san sang", TFT_DARKGREEN, TFT_WHITE);
}

// =========================================================================
//                                 LOOP
// =========================================================================
void loop() {
  // Giữ kết nối MQTT sống (xử lý keep-alive); không chặn nếu chưa nối
  if (mqtt.connected()) mqtt.loop();

  // Cập nhật đồng hồ mỗi giây
  static uint32_t lastClock = 0;
  if (millis() - lastClock >= 1000) {
    lastClock = millis();
    drawClock();
  }

  // Lấy mẫu đều theo SAMPLE_DT_MS
  if (millis() - lastSample < SAMPLE_DT_MS) return;
  lastSample = millis();

  float ax, ay, az, gx, gy, gz;
  mpuReadAll(ax, ay, az, gx, gy, gz);
  float a = sqrt(ax*ax + ay*ay + az*az);   // độ lớn gia tốc (g) — SVM

  // ---- LOG GỠ LỖI: in giá trị IMU định kỳ + đỉnh gia tốc khi lắc ----
  static uint32_t lastDbg = 0;
  static float aMax = 0, aMin = 99;
  if (a > aMax) aMax = a;
  if (a < aMin) aMin = a;
  if (millis() - lastDbg >= 300) {
    lastDbg = millis();
    Serial.printf("a=%.2fg (min=%.2f max=%.2f) | ax=%.2f ay=%.2f az=%.2f | g=%.0f,%.0f,%.0f dps | state=%d\n",
                  a, aMin, aMax, ax, ay, az, gx, gy, gz, (int)state);
    // Cập nhật chân trang gia tốc live
    drawLive(a, aMax);
    aMax = 0; aMin = 99;   // reset cửa sổ đỉnh mỗi 300ms
  }

  // ---- PHÂN LOẠI VẬN ĐỘNG (đứng yên / di chuyển - thể thao) ----
  // baseline = EMA chậm của |a| (bám theo trọng lực dù thiết bị nghiêng);
  // mức vận động = EMA nhanh của |a - baseline| -> tự khử bias nghiêng (1.18g...).
  if (!actInit) { magBaseline = a; actEMA = 0.0f; actInit = true; }
  else {
    magBaseline = magBaseline * 0.97f + a * 0.03f;   // đường nền trượt chậm
    float dev = fabs(a - magBaseline);
    actEMA = actEMA * 0.85f + dev * 0.15f;           // mức vận động
  }

  if (state == ST_NORMAL) {
    bool prevMoving = isMoving;
    if (!isMoving && actEMA > ACTIVITY_ON_G)  isMoving = true;   // bắt đầu vận động
    else if (isMoving && actEMA < ACTIVITY_OFF_G) isMoving = false; // dừng lại
    // Đổi trạng thái vận động -> cập nhật màn hình + đẩy server NGAY
    if (isMoving != prevMoving) {
      if (isMoving) {
        showStatus("VAN DONG", "Dang di chuyen", TFT_BLUE, TFT_WHITE);
        ledGreen();                       // vẫn bình thường -> xanh lá
        publishState("MOVING", false, a);
      } else {
        showStatus("BINH THUONG", "Dang dung yen", TFT_DARKGREEN, TFT_WHITE);
        ledGreen();                       // bình thường -> xanh lá
        publishState("STILL", false, a);
      }
    }
  }

  // ---- GỬI TRẠNG THÁI định kỳ mỗi 5s (kèm phân loại vận động) ----
  static uint32_t lastHeartbeat = 0;
  if (state == ST_NORMAL && millis() - lastHeartbeat >= 5000) {
    lastHeartbeat = millis();
    if (ensureMqtt(2000)) {
      char hb[110];
      snprintf(hb, sizeof(hb),
               "{\"fall\":false,\"status\":\"%s\",\"moving\":%s,\"activity\":%.2f,\"acc\":%.2f}",
               isMoving ? "MOVING" : "STILL", isMoving ? "true" : "false", actEMA, a);
      bool ok = mqtt.publish(TB_TELEMETRY_TOPIC, hb);
      Serial.printf("MQTT heartbeat -> %s | %s\n", ok ? "OK" : "FAIL", hb);
      mqtt.loop();
    }
  }

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
        ledBlue();                          // va chạm -> xanh dương
        showStatus("VA CHAM!", "Theo doi bat dong...", TFT_ORANGE, TFT_BLACK);
        publishState("IMPACT", false, a);   // đổi trạng thái -> đẩy lên server ngay
      }
      // reset cờ free-fall nếu quá hạn
      if (freefallSeen && millis() - freefallTime > FREEFALL_TO_IMPACT_MS)
        freefallSeen = false;
      break;
    }

    // -------- THEO DÕI BẤT ĐỘNG sau va chạm (có chờ ổn định) --------
    case ST_MONITOR_IMMOBILE: {
      uint32_t elapsed = millis() - monitorStart;
      // Bỏ qua giai đoạn rung/trượt NGAY sau va chạm (không đưa vào buffer)
      if (elapsed < SETTLE_AFTER_IMPACT_MS) break;

      // Sau khi đã ổn định: thu thập độ lớn gia tốc vào cửa sổ đo
      if (bufIdx < MAXBUF) magBuf[bufIdx++] = a;

      if (elapsed >= SETTLE_AFTER_IMPACT_MS + IMMOBILE_WINDOW_MS) {
        float mean, stdev;
        bufStats(mean, stdev);
        lastMean = mean; lastStd = stdev;   // lưu để gửi server
        // NGÃ = sau va chạm thì NẰM YÊN (dao động nhỏ).
        // THỂ THAO/VA CHẠM = cử động LIÊN TỤC (std lớn) -> loại.
        bool immobile = (stdev < IMMOBILE_STD_G) &&
                        (mean > IMMOBILE_MEAN_LOW) &&
                        (mean < IMMOBILE_MEAN_HIGH);
        Serial.printf("Cửa sổ: mean=%.2f std=%.2f immobile=%d\n",
                      mean, stdev, immobile);

        if (immobile) {
          Serial.println("=> XÁC NHẬN NGÃ. Bắt đầu cảnh báo!");
          state = ST_ALERT;
          alertStart = millis();
          showStatus("PHAT HIEN NGA!", "Nhan nut de huy", TFT_RED, TFT_WHITE);
          publishState("FALL", true, mean);   // đổi trạng thái -> đẩy lên server ngay
        } else {
          Serial.println("=> Không phải ngã (cử động/thể thao). Bỏ qua.");
          state = ST_NORMAL;
          ledGreen();                       // trở lại bình thường -> xanh lá
          showStatus("BINH THUONG", "He thong san sang", TFT_DARKGREEN, TFT_WHITE);
          publishState("ACTIVITY", false, mean);   // thể thao/va chạm -> báo server ngay
        }
      }
      break;
    }

    // -------- CẢNH BÁO: nháy ĐỎy 60s, chờ nút BOOT --------
    case ST_ALERT: {
      // Nháy đỏ liên tục: 200ms on / 200ms off
      bool on = ((millis() / 200) % 2) == 0;
      if (on) ledRed(); else ledOff();

      if (buttonPressed()) {                // người dùng bấm BOOT để tắt -> ổn
        Serial.println("Người dùng tắt cảnh báo (an toàn).");
        ledGreen();                         // về bình thường -> xanh lá
        state = ST_NORMAL;
        showStatus("BINH THUONG", "Da huy canh bao", TFT_DARKGREEN, TFT_WHITE);
        publishState("CANCELLED", false, a);   // đổi trạng thái -> đẩy lên server ngay
        delay(300);                         // chống dội nút
        break;
      }

      if (millis() - alertStart >= ALERT_TIMEOUT_MS) {
        Serial.println("Hết 60s chưa tắt -> gửi cảnh báo lên IoT server!");
        ledOff();
        showStatus("DA GUI CANH BAO", "Len IoT server", TFT_RED, TFT_WHITE);
        sendFallToServer(lastMean, lastStd);
        // Sau khi gửi: tự tắt cảnh báo, về bình thường
        state = ST_NORMAL;
        ledGreen();
        showStatus("BINH THUONG", "He thong san sang", TFT_DARKGREEN, TFT_WHITE);
      }
      break;
    }
  }
}
