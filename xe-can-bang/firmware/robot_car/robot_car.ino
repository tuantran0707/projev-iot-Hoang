/**
 * Robot Car — SM-ESP32 + L298N + Gas Sensor (IO36)
 * IoT: ThingsBoard Cloud via MQTT
 *
 * Điều khiển qua Shared Attributes (ThingsBoard):
 *   x        : <-100..100>
 *   y        : <-100..100>
 *   duration : <ms, tuỳ chọn, 0 = chạy liên tục>
 *   stop     : <true/false>
 *
 * Trục X: tiến (+) / lùi (-)
 * Trục Y: rẽ phải (+) / rẽ trái (-)
 *
 * Wiring L298N → ESP32:
 *   ENA  → GPIO 32   (PWM bánh phải)
 *   IN1  → GPIO 33
 *   IN2  → GPIO 25
 *   ENB  → GPIO 14   (PWM bánh trái)
 *   IN3  → GPIO 26
 *   IN4  → GPIO 27
 *   GAS  → GPIO 36   (ADC, analog)
 *
 * Libraries (Arduino Library Manager):
 *   - PubSubClient by Nick O'Leary
 *   - ArduinoJson by Benoit Blanchon
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ── Cấu hình WiFi & ThingsBoard ────────────────────────────────────
const char* WIFI_SSID     = "TDTuan";
const char* WIFI_PASSWORD = "12345678";

const char* TB_SERVER = "thingsboard.cloud";
const int   TB_PORT   = 1883;
const char* TB_TOKEN  = "8kG78bmcD7x0h1hBPk1r";

// ── Chân GPIO ──────────────────────────────────────────────────────
// Bánh phải (Motor A)
#define PIN_ENA   32
#define PIN_IN1   33
#define PIN_IN2   25

// Bánh trái (Motor B)
#define PIN_ENB   14
#define PIN_IN3   26
#define PIN_IN4   27

// Cảm biến khí gas
#define PIN_GAS   36

// ── Cấu hình PWM (LEDC ESP32) ──────────────────────────────────────
#define PWM_FREQ      1000   // Hz
#define PWM_RES       8      // bit → 0-255
#define PWM_CH_RIGHT  0
#define PWM_CH_LEFT   1

// ── Cấu hình thời gian ─────────────────────────────────────────────
#define TELEMETRY_INTERVAL_MS  3000UL   // gửi telemetry mỗi 3 giây

// ── Biến toàn cục ──────────────────────────────────────────────────
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

int  gCurrentX    = 0;
int  gCurrentY    = 0;
bool gAutoStop    = false;
unsigned long gStopAt = 0;

unsigned long gLastTelemetry = 0;

// ESP32 core 2.x uses ledcSetup/ledcAttachPin/ledcWrite(channel, duty)
// ESP32 core 3.x uses ledcAttachChannel/ledcWrite(pin, duty)
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

// ═══════════════════════════════════════════════════════════════════
//  Điều khiển động cơ
// ═══════════════════════════════════════════════════════════════════

void setMotorRight(int speed) {
  // speed: -255 (lùi) ... +255 (tiến)
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    writePwmRight((uint8_t)speed);
  } else if (speed < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    writePwmRight((uint8_t)(-speed));
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    writePwmRight(0);
  }
}

void setMotorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    writePwmLeft((uint8_t)speed);
  } else if (speed < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    writePwmLeft((uint8_t)(-speed));
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    writePwmLeft(0);
  }
}

/**
 * Điều khiển xe theo trục X/Y (differential drive)
 * x: -100..100  → tiến/lùi
 * y: -100..100  → rẽ phải/trái
 */
void driveXY(int x, int y) {
  int left  = constrain((x + y) * 255 / 100, -255, 255);
  int right = constrain((x - y) * 255 / 100, -255, 255);
  setMotorLeft(left);
  setMotorRight(right);
  gCurrentX = x;
  gCurrentY = y;
}

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
  gCurrentX  = 0;
  gCurrentY  = 0;
  gAutoStop  = false;
}

// ═══════════════════════════════════════════════════════════════════
//  MQTT – xử lý Attributes từ ThingsBoard
// ═══════════════════════════════════════════════════════════════════

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.println("[MQTT] JSON parse error: " + String(err.c_str()));
    return;
  }

  String topicStr = String(topic);

  // Shared attributes push: v1/devices/me/attributes
  // Shared attributes response: v1/devices/me/attributes/response/+
  if (!topicStr.startsWith("v1/devices/me/attributes")) {
    return;
  }

  JsonVariant data = doc;
  if (doc.containsKey("shared")) {
    data = doc["shared"];
  }

  bool stop = data["stop"] | false;
  if (stop) {
    stopMotors();
    Serial.println("[ATTR] stop=true -> motor stopped");
    return;
  }

  bool hasX = data.containsKey("x");
  bool hasY = data.containsKey("y");
  bool hasDuration = data.containsKey("duration");

  if (hasX || hasY) {
    int x = data["x"] | gCurrentX;
    int y = data["y"] | gCurrentY;
    int duration = data["duration"] | 0;

    x = constrain(x, -100, 100);
    y = constrain(y, -100, 100);

    driveXY(x, y);

    if (hasDuration && duration > 0) {
      gAutoStop = true;
      gStopAt = millis() + (unsigned long)duration;
      Serial.printf("[ATTR] x=%d y=%d duration=%dms\n", x, y, duration);
    } else {
      gAutoStop = false;
      Serial.printf("[ATTR] x=%d y=%d continuous\n", x, y);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════
//  WiFi & MQTT kết nối
// ═══════════════════════════════════════════════════════════════════

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("\n[WiFi] Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000UL) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WiFi] Failed, will retry");
  }
}

void connectMQTT() {
  if (mqttClient.connected()) return;
  mqttClient.setServer(TB_SERVER, TB_PORT);
  mqttClient.setCallback(onMqttMessage);
  mqttClient.setKeepAlive(30);

  Serial.print("[MQTT] Connecting...");
  if (mqttClient.connect("ESP32_RobotCar", TB_TOKEN, NULL)) {
    Serial.println(" connected");
    mqttClient.subscribe("v1/devices/me/attributes");
    mqttClient.subscribe("v1/devices/me/attributes/response/+");

    // Yêu cầu server gửi shared attributes hiện tại sau khi reconnect.
    mqttClient.publish("v1/devices/me/attributes/request/1", "{\"sharedKeys\":\"x,y,duration,stop\"}");

    // Gửi telemetry ngay sau khi kết nối
    gLastTelemetry = 0;
  } else {
    Serial.printf(" failed rc=%d\n", mqttClient.state());
  }
}

// ═══════════════════════════════════════════════════════════════════
//  Telemetry định kỳ → ThingsBoard
// ═══════════════════════════════════════════════════════════════════

void publishTelemetry() {
  if (!mqttClient.connected()) return;

  int   gasRaw     = analogRead(PIN_GAS);
  float gasVoltage = gasRaw * 3.3f / 4095.0f;

  StaticJsonDocument<128> doc;
  doc["gas_raw"]     = gasRaw;
  doc["gas_voltage"] = roundf(gasVoltage * 100.0f) / 100.0f;
  doc["motor_x"]     = gCurrentX;
  doc["motor_y"]     = gCurrentY;
  doc["rssi"]        = WiFi.RSSI();

  char buf[128];
  serializeJson(doc, buf);
  mqttClient.publish("v1/devices/me/telemetry", buf);
  Serial.printf("[Telemetry] gas=%d  x=%d  y=%d\n", gasRaw, gCurrentX, gCurrentY);
}

// ═══════════════════════════════════════════════════════════════════
//  Setup & Loop
// ═══════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Robot Car Booting ===");

  // Chân điều khiển hướng
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // PWM cho ENA / ENB
  initPwmPin(PIN_ENA, PWM_CH_RIGHT);
  initPwmPin(PIN_ENB, PWM_CH_LEFT);

  stopMotors();

  connectWiFi();
  connectMQTT();

  Serial.println("=== Ready ===");
}

void loop() {
  // Đảm bảo kết nối luôn sống
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();

  // Dừng xe tự động sau duration (non-blocking)
  if (gAutoStop && millis() >= gStopAt) {
    stopMotors();
    Serial.println("[Motor] Auto-stopped after duration");
  }

  // Gửi telemetry định kỳ
  unsigned long now = millis();
  if (now - gLastTelemetry >= TELEMETRY_INTERVAL_MS) {
    gLastTelemetry = now;
    publishTelemetry();
  }
}
