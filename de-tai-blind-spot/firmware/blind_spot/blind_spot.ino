/*
 * ============================================================================
 *  Cảnh báo điểm mù xe máy — ESP32-CAM Blind Spot Detection
 *  Board : AI-Thinker ESP32-CAM (OV2640) — BẮT BUỘC có PSRAM
 *
 *  Pipeline (tương tự face recognition end-to-end):
 *      CAPTURE  -> DETECT (FOMO) -> DIRECTION -> TRIGGER (buzzer + motor)
 *
 *  Phần "DETECT" dùng mô hình Edge Impulse FOMO (Object Detection) đã train
 *  để phát hiện phương tiện. FOMO trả về danh sách "tâm vật thể" (centroid)
 *  giống cách face detection trả bounding box -> ta suy ra hướng nguy hiểm.
 *
 *  CÁCH DÙNG:
 *   1) Train model FOMO trên Edge Impulse (lớp: "vehicle" hoặc xe may/o to...).
 *   2) Deployment -> Arduino library -> add .ZIP vào Arduino IDE.
 *   3) Sửa dòng #include "<ten_project>_inferencing.h" cho đúng tên lib.
 *   4) Tools: PSRAM = Enabled, Board = AI Thinker ESP32-CAM.
 * ============================================================================
 */

// >>> ĐỔI tên header cho khớp thư viện Edge Impulse bạn export <<<
#include <blind_spot_inferencing.h>

#include "esp_camera.h"

// ----------------------- CHÂN GPIO CẢNH BÁO --------------------------------
// ESP32-CAM còn rất ít chân trống. Dùng các chân an toàn:
#define PIN_BUZZER       12   // còi báo (qua transistor)
#define PIN_MOTOR_LEFT   13   // motor rung bên TRÁI
#define PIN_MOTOR_RIGHT  15   // motor rung bên PHẢI
// (GPIO4 = đèn flash, GPIO0/2/14 hạn chế — kiểm tra kỹ board của bạn)

// ----------------------- THAM SỐ NHẬN DIỆN ---------------------------------
#define CONF_THRESHOLD     0.6f   // ngưỡng tin cậy tối thiểu
#define DEBOUNCE_FRAMES    3      // số khung liên tiếp mới cảnh báo
#define CLEAR_FRAMES       3      // số khung "trống" để tắt cảnh báo

// ----------------------- CHÂN CAMERA (AI-THINKER) --------------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ----------------------- BỘ ĐỆM ẢNH CHO INFERENCE --------------------------
static bool     is_initialised = false;
static uint8_t *snapshot_buf = nullptr;   // RGB888 cho model

// Trạng thái cảnh báo (debounce)
static int leftHits = 0, rightHits = 0, leftMiss = 0, rightMiss = 0;
static bool leftActive = false, rightActive = false;

// =========================================================================
//                            KHỞI TẠO CAMERA
// =========================================================================
bool camera_init() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;   config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;   config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_QVGA;     // 320x240, sẽ cắt/resize về model
  config.pixel_format = PIXFORMAT_JPEG;     // lấy JPEG rồi decode -> RGB
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("CẢNH BÁO: Không thấy PSRAM! Cần PSRAM để chạy model.");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init lỗi 0x%x\n", err);
    return false;
  }
  is_initialised = true;
  return true;
}

// =========================================================================
//        CHỤP 1 KHUNG -> RGB888 -> resize về kích thước model cần
// =========================================================================
bool camera_capture(size_t out_w, size_t out_h) {
  if (!is_initialised) return false;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { Serial.println("Capture lỗi"); return false; }

  // Giải nén JPEG -> RGB888 vào snapshot_buf
  bool ok = fmt2rgb888(fb->buf, fb->len, fb->format, snapshot_buf);
  esp_camera_fb_return(fb);
  if (!ok) { Serial.println("JPEG->RGB888 lỗi"); return false; }

  // Resize/crop từ QVGA (320x240) về (out_w x out_h) mà model yêu cầu
  if ((out_w != EI_CAMERA_RAW_FRAME_BUFFER_COLS) ||
      (out_h != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
    ei::image::processing::crop_and_interpolate_rgb888(
        snapshot_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        snapshot_buf, out_w, out_h);
  }
  return true;
}

// Callback Edge Impulse: lấy dữ liệu pixel từ snapshot_buf
int camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  size_t out_ix = 0;
  while (length--) {
    out_ptr[out_ix] = (snapshot_buf[pixel_ix + 2] << 16) +
                      (snapshot_buf[pixel_ix + 1] << 8) +
                       snapshot_buf[pixel_ix];
    out_ix++;
    pixel_ix += 3;
  }
  return 0;
}

// =========================================================================
//                       LOGIC CẢNH BÁO THEO HƯỚNG
// =========================================================================
void setLeft(bool on)  { digitalWrite(PIN_MOTOR_LEFT,  on ? HIGH : LOW); }
void setRight(bool on) { digitalWrite(PIN_MOTOR_RIGHT, on ? HIGH : LOW); }

void updateAlert(bool seenLeft, bool seenRight) {
  // --- bên trái ---
  if (seenLeft) { leftHits++; leftMiss = 0; }
  else          { leftMiss++; leftHits = 0; }
  if (leftHits >= DEBOUNCE_FRAMES) leftActive = true;
  if (leftMiss >= CLEAR_FRAMES)    leftActive = false;

  // --- bên phải ---
  if (seenRight) { rightHits++; rightMiss = 0; }
  else           { rightMiss++; rightHits = 0; }
  if (rightHits >= DEBOUNCE_FRAMES) rightActive = true;
  if (rightMiss >= CLEAR_FRAMES)    rightActive = false;

  setLeft(leftActive);
  setRight(rightActive);

  // Buzzer kêu nếu có xe ở bất kỳ bên nào (nhịp nhanh hơn khi 2 bên)
  bool danger = leftActive || rightActive;
  if (danger) {
    int period = (leftActive && rightActive) ? 120 : 250;
    digitalWrite(PIN_BUZZER, ((millis() / period) % 2) ? HIGH : LOW);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }
}

// =========================================================================
//                                SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_BUZZER, OUTPUT);       digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_MOTOR_LEFT, OUTPUT);   setLeft(false);
  pinMode(PIN_MOTOR_RIGHT, OUTPUT);  setRight(false);

  // Cấp buffer RGB888 (PSRAM) cho 1 khung model
  snapshot_buf = (uint8_t*)ps_malloc(
      EI_CAMERA_RAW_FRAME_BUFFER_COLS *
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3);
  if (!snapshot_buf) { Serial.println("Không cấp được snapshot_buf!"); }

  if (!camera_init()) {
    Serial.println("Camera khởi tạo thất bại — dừng.");
    while (true) delay(1000);
  }
  Serial.println("Blind Spot Detection sẵn sàng.");
}

// =========================================================================
//                                 LOOP
// =========================================================================
void loop() {
  // 1) CAPTURE
  if (!camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH,
                      (size_t)EI_CLASSIFIER_INPUT_HEIGHT)) {
    delay(50);
    return;
  }

  // 2) DETECT (FOMO)
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &camera_get_data;

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  if (err != EI_IMPULSE_OK) {
    Serial.printf("run_classifier lỗi: %d\n", err);
    delay(50);
    return;
  }

  // 3) DIRECTION — duyệt các vật thể FOMO, suy hướng từ vị trí x của tâm
  bool seenLeft = false, seenRight = false;
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
    auto bb = result.bounding_boxes[i];
    if (bb.value < CONF_THRESHOLD) continue;          // bỏ kết quả yếu

    float cx = bb.x + bb.width / 2.0f;                 // tâm theo trục x
    float ratio = cx / (float)EI_CLASSIFIER_INPUT_WIDTH; // 0=trái, 1=phải

    if (ratio < 0.45f)      seenLeft  = true;          // xe lệch TRÁI
    else if (ratio > 0.55f) seenRight = true;          // xe lệch PHẢI
    else { seenLeft = true; seenRight = true; }        // ở giữa -> cảnh báo cả 2

    Serial.printf("  %s (%.2f) cx=%.0f -> %s\n",
        bb.label, bb.value, cx,
        (ratio < 0.45f ? "TRAI" : (ratio > 0.55f ? "PHAI" : "GIUA")));
  }
#else
  // Nếu model là Image Classification (không phải Object Detection):
  // dùng nhãn "vehicle" toàn khung -> cảnh báo chung (không phân hướng).
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value >= CONF_THRESHOLD) {
      seenLeft = seenRight = true;
    }
  }
#endif

  // 4) TRIGGER
  updateAlert(seenLeft, seenRight);

  delay(20);   // nhường CPU
}
