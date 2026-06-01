/*
 * ============================================================================
 *  Cảnh báo điểm mù — ESP32-CAM + TensorFlow Lite Micro (KHÔNG dùng Edge Impulse)
 *  Model train trên LAPTOP (train_vehicle_classifier.py) -> object_model.h
 *
 *  Ý tưởng cho ĐƠN GIẢN:
 *    - Model chỉ phân loại "có vật thể / không" (2 lớp) trên ảnh 96x96 grayscale.
 *    - Để biết HƯỚNG: cắt khung hình làm 2 nửa TRÁI / PHẢI, chạy model cho từng nửa.
 *
 *  Pipeline:  CAPTURE -> CROP(L/R) -> CLASSIFY(TFLM) -> TRIGGER (buzzer + motor)
 *
 *  THƯ VIỆN cần cài trong Arduino IDE (chọn 1, đều cho ESP32):
 *    - "Chirale_TensorFlowLite"   (khuyến nghị, mới)   HOẶC
 *    - "TensorFlowLite_ESP32"     (by tanakamasayuki)
 *  -> Sửa #include cho khớp thư viện bạn cài (xem khối #include bên dưới).
 * ============================================================================
 */

#include "esp_camera.h"
#include "object_model.h"     // <-- mảng C sinh ra từ model int8 (xem .py)

#include <WiFi.h>
#include <WebServer.h>

// ---- TensorFlow Lite Micro (chọn đúng header theo thư viện đã cài) --------
#include <Chirale_TensorFlowLite.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
// Nếu dùng "TensorFlowLite_ESP32" thì thay 2 dòng include đầu bằng:
//   #include <TensorFlowLite_ESP32.h>

// ----------------------- WIFI / STREAM (ACCESS POINT) ---------------------
// ESP32 tự PHÁT WiFi riêng. Laptop/điện thoại bắt đúng WiFi này rồi mở IP.
// >>> Có thể đổi tên WiFi và mật khẩu tùy ý (mật khẩu tối thiểu 8 ký tự) <<<
const char* AP_SSID = "ESP32-BlindSpot";
const char* AP_PASS = "12345678";       // đặt "" nếu muốn WiFi mở (không mật khẩu)

WebServer server(80);                 // web server cổng 80

// Trạng thái mới nhất để hiển thị trên trình duyệt
volatile float gLeftProb = 0.0f, gRightProb = 0.0f;
volatile bool  gLeftActive = false, gRightActive = false;

// ----------------------- FreeRTOS (đa nhiệm 2 core) -----------------------
// Chỉ task inference đụng camera. Nó nén sẵn 1 ảnh JPEG vào buffer chung,
// task web chỉ gửi lại buffer đó -> không tranh camera, ảnh luôn mượt.
SemaphoreHandle_t jpgMutex = nullptr;   // bảo vệ buffer JPEG chung
uint8_t* gJpg    = nullptr;             // buffer JPEG mới nhất
size_t   gJpgLen = 0;
TaskHandle_t inferTaskHandle = nullptr;
TaskHandle_t webTaskHandle   = nullptr;

// ----------------------- CHÂN GPIO CẢNH BÁO --------------------------------
// ESP32-S3-N16-R8:
// - Motor vibration: IO21
// - Buzzer trái: IO47
// - Buzzer phải: IO38
#define PIN_MOTOR         21
#define PIN_BUZZER_LEFT   47
#define PIN_BUZZER_RIGHT  38

// ----------------------- THAM SỐ ------------------------------------------
#define IMG_SIZE         96      // khớp với model train
#define CONF_THRESHOLD   0.6f    // ngưỡng xác suất "object"
#define DEBOUNCE_FRAMES  3
#define CLEAR_FRAMES     3
#define OBJECT_CLASS     1       // 0=none, 1=object (khớp CLASS_NAMES trong .py)
#define INFER_EVERY_N    4       // chạy model mỗi N khung (stream mượt hơn). Tăng để mượt hơn.
#define JPEG_QUALITY     60      // chất lượng JPEG (thấp hơn = ảnh nhẹ = truyền nhanh = mượt)

// ----------------------- CHÂN CAMERA (ESP32-S3-CAM N16R8) ------------------
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 17
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 12
#define Y5_GPIO_NUM 10
#define Y4_GPIO_NUM 8
#define Y3_GPIO_NUM 9
#define Y2_GPIO_NUM 11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

// ----------------------- TFLM toàn cục -------------------------------------
namespace {
  const tflite::Model*       model = nullptr;
  tflite::MicroInterpreter*  interpreter = nullptr;
  TfLiteTensor*              input  = nullptr;
  TfLiteTensor*              output = nullptr;
  constexpr int kArenaSize = 120 * 1024;          // ~120KB (đặt trong PSRAM)
  uint8_t* tensor_arena = nullptr;
}

// Trạng thái debounce
int leftHits=0, rightHits=0, leftMiss=0, rightMiss=0;
bool leftActive=false, rightActive=false;

// =========================================================================
//                            KHỞI TẠO CAMERA
// =========================================================================
bool camera_init() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0=Y2_GPIO_NUM; config.pin_d1=Y3_GPIO_NUM; config.pin_d2=Y4_GPIO_NUM;
  config.pin_d3=Y5_GPIO_NUM; config.pin_d4=Y6_GPIO_NUM; config.pin_d5=Y7_GPIO_NUM;
  config.pin_d6=Y8_GPIO_NUM; config.pin_d7=Y9_GPIO_NUM;
  config.pin_xclk=XCLK_GPIO_NUM; config.pin_pclk=PCLK_GPIO_NUM;
  config.pin_vsync=VSYNC_GPIO_NUM; config.pin_href=HREF_GPIO_NUM;
  config.pin_sccb_sda=SIOD_GPIO_NUM; config.pin_sccb_scl=SIOC_GPIO_NUM;
  config.pin_pwdn=PWDN_GPIO_NUM; config.pin_reset=RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_QVGA;     // 320x240
  config.pixel_format = PIXFORMAT_GRAYSCALE; // lấy thẳng grayscale -> khỏi decode JPEG
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_count     = 3;          // 3 buffer: chia cho 2 task (infer + web)

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { Serial.printf("Camera lỗi 0x%x\n", err); return false; }
  return true;
}

// =========================================================================
//   Lấy 1 nửa khung (trái/phải) từ QVGA grayscale, resize -> input int8 model
//   half = 0: nửa trái ; half = 1: nửa phải
// =========================================================================
void fillInputFromHalf(camera_fb_t* fb, int half) {
  // QVGA: 320x240. Mỗi nửa rộng 160, cao 240.
  const int srcW = 320, srcH = 240;
  const int halfW = srcW / 2;             // 160
  const int x0 = half * halfW;            // 0 hoặc 160

  // Lượng tử hoá: int8 = (val/255 - mean)/scale ... dùng tham số của tensor input
  float scale = input->params.scale;
  int   zero  = input->params.zero_point;

  for (int y = 0; y < IMG_SIZE; y++) {
    for (int x = 0; x < IMG_SIZE; x++) {
      // map (x,y) của 96x96 về toạ độ trong nửa khung (nearest neighbor)
      int sx = x0 + (x * halfW) / IMG_SIZE;
      int sy = (y * srcH) / IMG_SIZE;
      uint8_t pix = fb->buf[sy * srcW + sx];     // 0..255 grayscale

      float norm = pix / 255.0f;                 // khớp Rescaling(1/255) khi train
      int8_t q = (int8_t)round(norm / scale + zero);
      input->data.int8[y * IMG_SIZE + x] = q;
    }
  }
}

// Chạy model, trả xác suất lớp "object"
float classifyObjectProb() {
  if (interpreter->Invoke() != kTfLiteOk) return 0.0f;
  // output int8 -> dequant về xác suất
  float scale = output->params.scale;
  int   zero  = output->params.zero_point;
  int8_t raw  = output->data.int8[OBJECT_CLASS];
  return (raw - zero) * scale;
}

// =========================================================================
//                       LOGIC CẢNH BÁO THEO HƯỚNG
// =========================================================================
void setMotor(bool on) { digitalWrite(PIN_MOTOR, on?HIGH:LOW); }

void updateAlert(bool seenLeft, bool seenRight) {
  if (seenLeft) { leftHits++; leftMiss=0; } else { leftMiss++; leftHits=0; }
  if (leftHits>=DEBOUNCE_FRAMES) leftActive=true;
  if (leftMiss>=CLEAR_FRAMES)    leftActive=false;

  if (seenRight){ rightHits++; rightMiss=0;} else { rightMiss++; rightHits=0; }
  if (rightHits>=DEBOUNCE_FRAMES) rightActive=true;
  if (rightMiss>=CLEAR_FRAMES)    rightActive=false;

  bool danger = leftActive || rightActive;
  setMotor(danger);

  int period = (leftActive && rightActive) ? 120 : 250;
  bool pulse = ((millis()/period)%2) != 0;
  digitalWrite(PIN_BUZZER_LEFT, (leftActive && pulse) ? HIGH : LOW);
  digitalWrite(PIN_BUZZER_RIGHT, (rightActive && pulse) ? HIGH : LOW);
}

// =========================================================================
//                       WEB SERVER STREAM (IP LOCAL)
// =========================================================================
// Trang chính: hiển thị ảnh camera (tự reload) + trạng thái trái/phải.
void handleRoot() {
  String html =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Blind Spot Stream</title>"
    "<style>body{font-family:sans-serif;text-align:center;background:#111;color:#eee}"
    ".wrap{position:relative;display:inline-block;width:90%;max-width:480px}"
    "img{width:100%;display:block;border:2px solid #444;border-radius:8px}"
    // 2 ô khoanh vùng phủ lên nửa trái/phải, mặc định ẩn (trong suốt)
    ".zone{position:absolute;top:2px;bottom:2px;width:48%;border:3px solid transparent;"
    "border-radius:6px;box-sizing:border-box;pointer-events:none}"
    "#zL{left:2px}#zR{right:2px}.zone.hit{border-color:#ff2d2d;box-shadow:0 0 12px #ff2d2d}"
    ".box{display:inline-block;padding:8px 16px;margin:6px;border-radius:8px;font-weight:bold}"
    ".on{background:#c0392b;color:#fff}.off{background:#27ae60;color:#fff}"
    "#log{margin:10px auto;padding:10px;max-width:480px;border-radius:8px;font-weight:bold;"
    "background:#222;color:#27ae60}#log.alert{background:#c0392b;color:#fff}</style></head>"
    "<body><h2>ESP32-S3 Blind Spot</h2>"
    "<div class='wrap'><img id='cam' src='/jpg'>"
    "<div id='zL' class='zone'></div><div id='zR' class='zone'></div></div>"
    "<div><span id='L' class='box off'>TRÁI: -</span>"
    "<span id='R' class='box off'>PHẢI: -</span></div>"
    "<div id='log'>An toàn — chưa phát hiện vật thể</div>"
    "<script>"
    // Tự tải lại ảnh liên tục (không dùng MJPEG blocking) -> không treo inference
    "function refreshImg(){let img=document.getElementById('cam');"
    "img.onload=()=>setTimeout(refreshImg,10);img.onerror=()=>setTimeout(refreshImg,150);"
    "img.src='/jpg?t='+Date.now();}refreshImg();"
    "setInterval(async()=>{try{let r=await fetch('/status');let j=await r.json();"
    "let L=document.getElementById('L'),R=document.getElementById('R');"
    "L.textContent='TRÁI: '+(j.l*100).toFixed(0)+'%';R.textContent='PHẢI: '+(j.r*100).toFixed(0)+'%';"
    "L.className='box '+(j.la?'on':'off');R.className='box '+(j.ra?'on':'off');"
    // khoanh vùng đỏ theo nửa phát hiện
    "document.getElementById('zL').className='zone'+(j.la?' hit':'');"
    "document.getElementById('zR').className='zone'+(j.ra?' hit':'');"
    // dòng log cảnh báo
    "let lg=document.getElementById('log');let m;"
    "if(j.la&&j.ra){m='⚠️ PHÁT HIỆN VẬT THỂ CẢ HAI BÊN!';}"
    "else if(j.la){m='⬅️ Phát hiện vật thể bên TRÁI';}"
    "else if(j.ra){m='Phát hiện vật thể bên PHẢI ➡️';}"
    "else{m='An toàn — chưa phát hiện vật thể';}"
    "lg.textContent=m;lg.className=(j.la||j.ra)?'alert':'';"
    "}catch(e){}},300);</script></body></html>";
  server.send(200, "text/html", html);
}

// Trả trạng thái dưới dạng JSON
void handleStatus() {
  char buf[96];
  snprintf(buf, sizeof(buf),
           "{\"l\":%.2f,\"r\":%.2f,\"la\":%s,\"ra\":%s}",
           gLeftProb, gRightProb,
           gLeftActive ? "true" : "false",
           gRightActive ? "true" : "false");
  server.send(200, "application/json", buf);
}

// Trả ảnh JPEG đã được task inference nén sẵn (KHÔNG đụng camera ở đây)
void handleJpg() {
  if (xSemaphoreTake(jpgMutex, pdMS_TO_TICKS(300)) != pdTRUE) {
    server.send(503, "text/plain", "busy"); return;
  }
  if (!gJpg || gJpgLen == 0) {
    xSemaphoreGive(jpgMutex);
    server.send(503, "text/plain", "no frame"); return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(gJpgLen);
  server.send(200, "image/jpeg", "");
  server.client().write(gJpg, gJpgLen);   // gửi buffer đã nén sẵn
  xSemaphoreGive(jpgMutex);
}

void startWebServer() {
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/jpg", handleJpg);
  server.begin();
  Serial.println("Web server đã chạy.");
}

// =========================================================================
//                 TASK FreeRTOS: INFERENCE (chạy trên core 1)
// =========================================================================
void inferenceTask(void* pv) {
  uint8_t frameCnt = 0;
  for (;;) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

    // 1) Nén JPEG cho web MỖI khung -> stream mượt (việc này nhẹ)
    uint8_t* jb = nullptr; size_t jl = 0;
    bool jok = frame2jpg(fb, JPEG_QUALITY, &jb, &jl);   // grayscale -> JPEG
    if (jok) {
      // đổi buffer JPEG chung (giải phóng cái cũ)
      if (xSemaphoreTake(jpgMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (gJpg) free(gJpg);
        gJpg = jb; gJpgLen = jl;
        xSemaphoreGive(jpgMutex);
      } else {
        free(jb);   // không lấy được khoá -> bỏ khung này
      }
    }

    // 2) Chạy model MỖI 3 KHUNG (nặng) -> stream không bị ghì lại
    if (++frameCnt >= INFER_EVERY_N) {
      frameCnt = 0;
      fillInputFromHalf(fb, 0);  float pLeft  = classifyObjectProb();   // nửa TRÁI
      fillInputFromHalf(fb, 1);  float pRight = classifyObjectProb();   // nửa PHẢI

      bool seenLeft  = pLeft  >= CONF_THRESHOLD;
      bool seenRight = pRight >= CONF_THRESHOLD;

      Serial.printf("L=%.2f %s | R=%.2f %s\n",
          pLeft,  seenLeft?"VAT":"-",
          pRight, seenRight?"VAT":"-");

      updateAlert(seenLeft, seenRight);

      // Cập nhật trạng thái cho web (sau khi debounce)
      gLeftProb = pLeft;  gRightProb = pRight;
      gLeftActive = leftActive;  gRightActive = rightActive;
    }

    esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(2));   // nhường CPU (ít thôi để ra nhiều khung)
  }
}

// =========================================================================
//                 TASK FreeRTOS: WEB SERVER (chạy trên core 0)
// =========================================================================
void webTask(void* pv) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =========================================================================
//                                SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_MOTOR, OUTPUT);         setMotor(false);
  pinMode(PIN_BUZZER_LEFT, OUTPUT);   digitalWrite(PIN_BUZZER_LEFT, LOW);
  pinMode(PIN_BUZZER_RIGHT, OUTPUT);  digitalWrite(PIN_BUZZER_RIGHT, LOW);

  if (!camera_init()) { while(true) delay(1000); }

  // Cấp tensor arena trong PSRAM
  tensor_arena = (uint8_t*) ps_malloc(kArenaSize);
  if (!tensor_arena) { Serial.println("Thiếu PSRAM cho arena!"); while(true) delay(1000); }

  // Nạp model
  model = tflite::GetModel(object_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema không khớp!"); while(true) delay(1000);
  }
  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors lỗi!"); while(true) delay(1000);
  }
  input  = interpreter->input(0);
  output = interpreter->output(0);

  // Tạo mutex bảo vệ buffer JPEG chung trước khi bật các task
  jpgMutex = xSemaphoreCreateMutex();

  // PHÁT WiFi riêng (Access Point) để laptop/điện thoại bắt vào xem stream
  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(AP_SSID, (strlen(AP_PASS) >= 8) ? AP_PASS : nullptr);
  if (apOk) {
    IPAddress ip = WiFi.softAPIP();
    Serial.println("\n==== WiFi ACCESS POINT đã bật ====");
    Serial.printf("  Tên WiFi (SSID): %s\n", AP_SSID);
    Serial.printf("  Mật khẩu      : %s\n", (strlen(AP_PASS) >= 8) ? AP_PASS : "(không)");
    Serial.print ("  Mở trình duyệt tới: http://");
    Serial.println(ip);
    Serial.println("=================================");
    startWebServer();
  } else {
    Serial.println("\nKhông bật được Access Point (vẫn chạy cảnh báo bình thường).");
  }

  Serial.println("TFLM Blind Spot sẵn sàng.");

  // ---- Tạo 2 task chạy song song trên 2 core ----
  // Inference: core 1 (APP_CPU), stack lớn vì dùng TFLM
  xTaskCreatePinnedToCore(inferenceTask, "infer", 8192, nullptr, 2, &inferTaskHandle, 1);
  // Web server: core 0 (PRO_CPU), cùng core với WiFi stack
  xTaskCreatePinnedToCore(webTask, "web", 8192, nullptr, 1, &webTaskHandle, 0);
}

// =========================================================================
//                                 LOOP
// =========================================================================
// Mọi việc đã chuyển sang 2 task FreeRTOS ở trên -> loop() để rỗng.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
