/*
 * ============================================================================
 *  FallDetection_inferencing.h
 * ----------------------------------------------------------------------------
 *  Mô hình phát hiện ngã — triển khai on-device (Edge AI / TinyML).
 *  Header cung cấp API tương thích Edge Impulse (run_classifier, signal_t,
 *  ei_impulse_result_t...) để firmware fall_detection_ei.ino gọi suy luận.
 *
 *  Bộ phân loại trích đặc trưng thống kê trên cửa sổ gia tốc
 *  (min, max, mean, std của |a|) và phân vào 3 nhãn:
 *      "binh_thuong" | "the_thao" | "NGA"
 *  bằng các ranh giới quyết định đã hiệu chỉnh từ dữ liệu huấn luyện
 *  (xem README §4 — tham số mô hình).
 *
 *  Muốn dùng mô hình NN của Edge Impulse Studio: export "Arduino library" và
 *  thay header này bằng thư viện <ProjectName>_inferencing.h tương ứng — firmware
 *  KHÔNG cần đổi vì cùng API run_classifier().
 * ============================================================================
 */
#ifndef FALL_DETECTION_INFERENCING_H
#define FALL_DETECTION_INFERENCING_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

// ----- Cấu hình mô hình -----
// Window 2000 ms @ 100 Hz = 200 mẫu, mỗi mẫu 3 trục (ax, ay, az) -> frame = 600.
#define EI_CLASSIFIER_FREQUENCY              100
#define EI_CLASSIFIER_RAW_SAMPLE_COUNT       200
#define EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME  3
#define EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE \
        (EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME)
#define EI_CLASSIFIER_LABEL_COUNT            3

// ----- Kiểu dữ liệu (API tương thích Edge Impulse) -----
typedef enum {
    EI_IMPULSE_OK = 0,
    EI_IMPULSE_ERROR_SHAPES_DONT_MATCH = -1
} EI_IMPULSE_ERROR;

typedef struct {
    size_t total_length;
    int (*get_data)(size_t offset, size_t length, float *out_ptr);
} signal_t;

typedef struct {
    const char *label;
    float value;
} ei_impulse_result_classification_t;

typedef struct {
    ei_impulse_result_classification_t classification[EI_CLASSIFIER_LABEL_COUNT];
    float anomaly;
} ei_impulse_result_t;

// ----- Tham số mô hình (ranh giới quyết định, hiệu chỉnh từ dữ liệu) -----
#define EI_FD_FREEFALL_G     0.50f   // min(|a|) < 0.5g  -> có pha rơi tự do
#define EI_FD_IMPACT_G       2.60f   // max(|a|) > 2.6g  -> có cú va chạm
#define EI_FD_IMMOBILE_STD   0.20f   // std(|a|) nhỏ     -> nằm yên sau va chạm
#define EI_FD_IMMOBILE_MEAN_LOW   0.70f
#define EI_FD_IMMOBILE_MEAN_HIGH  1.30f
#define EI_FD_MOVING_STD     0.15f   // std(|a|) lớn khi không ngã -> đang vận động

// =========================================================================
//  run_classifier(): trích đặc trưng cửa sổ -> phân loại 3 nhãn.
//  Đọc dữ liệu qua signal->get_data (firmware nạp ax,ay,az xen kẽ vào buffer).
// =========================================================================
static inline EI_IMPULSE_ERROR run_classifier(signal_t *signal,
                                              ei_impulse_result_t *result,
                                              bool debug) {
    (void)debug;
    const char *labels[EI_CLASSIFIER_LABEL_COUNT] = { "binh_thuong", "the_thao", "NGA" };

    // --- Trích đặc trưng: min/max/mean/std của độ lớn gia tốc |a| ---
    float mn = 1e9f, mx = -1e9f, sum = 0.0f, sumSq = 0.0f;
    const uint16_t N = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    for (uint16_t i = 0; i < N; i++) {
        float axyz[3];
        if (signal->get_data((size_t)i * 3, 3, axyz) != 0) {
            return EI_IMPULSE_ERROR_SHAPES_DONT_MATCH;
        }
        float a = sqrtf(axyz[0]*axyz[0] + axyz[1]*axyz[1] + axyz[2]*axyz[2]);
        if (a < mn) mn = a;
        if (a > mx) mx = a;
        sum   += a;
        sumSq += a * a;
    }
    float mean = sum / N;
    float var  = sumSq / N - mean * mean;
    if (var < 0) var = 0;
    float stdev = sqrtf(var);

    // --- Suy luận: tính điểm cho từng nhãn rồi chuẩn hoá thành xác suất ---
    float sFall = 0.0f, sSport = 0.0f, sNormal = 0.0f;

    bool hasFreefall = (mn < EI_FD_FREEFALL_G);
    bool hasImpact   = (mx > EI_FD_IMPACT_G);
    bool immobile    = (stdev < EI_FD_IMMOBILE_STD) &&
                       (mean > EI_FD_IMMOBILE_MEAN_LOW) &&
                       (mean < EI_FD_IMMOBILE_MEAN_HIGH);

    if (hasFreefall && hasImpact && immobile) {
        sFall = 0.95f;                 // free-fall -> impact -> nằm yên = NGÃ
    } else if (hasImpact || stdev > EI_FD_MOVING_STD) {
        sSport = 0.85f;                // va chạm/dao động lớn nhưng không nằm yên
    } else {
        sNormal = 0.90f;               // ổn định quanh 1g = bình thường
    }

    float total = sNormal + sSport + sFall + 1e-6f;
    float probs[3] = { sNormal / total, sSport / total, sFall / total };

    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        result->classification[i].label = labels[i];
        result->classification[i].value = probs[i];
    }
    result->anomaly = 0.0f;
    return EI_IMPULSE_OK;
}

#endif // FALL_DETECTION_INFERENCING_H
