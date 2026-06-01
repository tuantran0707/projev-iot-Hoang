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

// ---- TensorFlow Lite Micro (chọn đúng header theo thư viện đã cài) --------
#include <Chirale_TensorFlowLite.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
// Nếu dùng "TensorFlowLite_ESP32" thì thay 2 dòng include đầu bằng:
//   #include <TensorFlowLite_ESP32.h>

// ----------------------- CHÂN GPIO CẢNH BÁO --------------------------------
#define PIN_BUZZER       12
#define PIN_MOTOR_LEFT   13
#define PIN_MOTOR_RIGHT  15

// ----------------------- THAM SỐ ------------------------------------------
#define IMG_SIZE         96      // khớp với model train
#define CONF_THRESHOLD   0.6f    // ngưỡng xác suất "object"
#define DEBOUNCE_FRAMES  3
#define CLEAR_FRAMES     3
#define OBJECT_CLASS     1       // 0=none, 1=object (khớp CLASS_NAMES trong .py)

// ----------------------- CHÂN CAMERA (AI-THINKER) --------------------------
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

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
  config.fb_count     = 1;

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
void setLeft(bool on)  { digitalWrite(PIN_MOTOR_LEFT,  on?HIGH:LOW); }
void setRight(bool on) { digitalWrite(PIN_MOTOR_RIGHT, on?HIGH:LOW); }

void updateAlert(bool seenLeft, bool seenRight) {
  if (seenLeft) { leftHits++; leftMiss=0; } else { leftMiss++; leftHits=0; }
  if (leftHits>=DEBOUNCE_FRAMES) leftActive=true;
  if (leftMiss>=CLEAR_FRAMES)    leftActive=false;

  if (seenRight){ rightHits++; rightMiss=0;} else { rightMiss++; rightHits=0; }
  if (rightHits>=DEBOUNCE_FRAMES) rightActive=true;
  if (rightMiss>=CLEAR_FRAMES)    rightActive=false;

  setLeft(leftActive); setRight(rightActive);

  bool danger = leftActive || rightActive;
  if (danger) {
    int period = (leftActive && rightActive) ? 120 : 250;
    digitalWrite(PIN_BUZZER, ((millis()/period)%2)?HIGH:LOW);
  } else digitalWrite(PIN_BUZZER, LOW);
}

// =========================================================================
//                                SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_BUZZER, OUTPUT);      digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_MOTOR_LEFT, OUTPUT);  setLeft(false);
  pinMode(PIN_MOTOR_RIGHT, OUTPUT); setRight(false);

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

  Serial.println("TFLM Blind Spot sẵn sàng.");
}

// =========================================================================
//                                 LOOP
// =========================================================================
void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { delay(50); return; }

  // Nửa TRÁI
  fillInputFromHalf(fb, 0);
  float pLeft = classifyObjectProb();

  // Nửa PHẢI
  fillInputFromHalf(fb, 1);
  float pRight = classifyObjectProb();

  esp_camera_fb_return(fb);

  bool seenLeft  = pLeft  >= CONF_THRESHOLD;
  bool seenRight = pRight >= CONF_THRESHOLD;

  Serial.printf("L=%.2f %s | R=%.2f %s\n",
      pLeft,  seenLeft?"VẬT":"-",
      pRight, seenRight?"VẬT":"-");

  updateAlert(seenLeft, seenRight);
  delay(20);
}
