# Đề tài — Cảnh báo phương tiện trong điểm mù gương chiếu hậu (Blind Spot Detection)

Hệ thống dùng **camera + AI** nhận diện phương tiện (xe máy, ô tô, xe tải, xe buýt…)
nằm trong **vùng điểm mù phía sau** xe máy, sau đó cảnh báo người lái **kịp thời**
bằng **buzzer** và **motor rung** theo hướng nguy hiểm.

---

## 1. Mô tả hoạt động

```
Camera OV2640 (gắn phía sau xe)
        │  thu hình liên tục (real-time)
        ▼
   ESP32-CAM chạy mô hình AI phát hiện phương tiện
        │
        ├─ Không có phương tiện trong điểm mù ──► im lặng
        │
        └─ CÓ phương tiện trong vùng điểm mù
                │  + xác định hướng (trái / sau) + độ tin cậy
                ▼
        ┌──────────────────────────────┐
        │  Buzzer beep theo hướng       │
        │  Motor rung bên tương ứng     │
        └──────────────────────────────┘
```

Hệ thống chạy **liên tục, tự động**, không cần can thiệp thủ công.

---

## 2. Linh kiện cần mua

| Linh kiện | SL | Thông số gợi ý | Giá tham khảo (VND) | Ghi chú |
|-----------|----|----------------|--------------------|---------|
| Module ESP32-CAM (AI-Thinker) | 1 | có sẵn camera OV2640 | 90.000 – 150.000 | Bộ xử lý trung tâm |
| Camera OV2640 | 1 | đã tích hợp trên ESP32-CAM | — | Mua riêng nếu cần thay/đặt vị trí |
| Buzzer active 5V | 1 | loại active (chỉ cần cấp nguồn là kêu) | 3.000 – 8.000 | Cảnh báo âm thanh |
| Motor rung coin 3V | 2 | loại coin 10mm | 8.000 – 20.000/cái | Gắn tay lái trái & phải |
| Transistor NPN (S8050/2N2222) + diode 1N4007 | 2–3 | + điện trở base 1kΩ | 2.000 – 10.000 | Driver cho buzzer & motor |
| Mạch giảm áp DC-DC Buck | 1 | 12V → 5V/3.3V, **≥ 2A** (vd MP1584 / LM2596) | 10.000 – 30.000 | Hạ áp từ ắc quy/nguồn xe |
| Module nạp/cấp nguồn ổn định | tuỳ | tụ lọc 470–1000µF | 5.000 – 15.000 | Lọc nhiễu nguồn xe |
| Fuse + đế fuse | 1 | 1–2A | 5.000 – 15.000 | Bảo vệ chập/quá dòng |
| Tụ lọc nhiễu | vài | 100nF + 470µF | 5.000 | Chống sụt áp khi WiFi/CPU tải |
| Dây nối, jack nguồn, ống co nhiệt | — | — | 20.000 | Đi dây & đóng gói |
| Adapter USB-TTL (FTDI / CP2102) | 1 | 3.3V | 30.000 – 60.000 | **Để nạp code cho ESP32-CAM** (board không có USB) |
| Hộp chống nước / giá gắn | 1 | IP65 nhỏ | 30.000 – 80.000 | Bảo vệ ngoài trời |

> **Nguồn cấp thực tế:** xe máy thường có ắc quy 12V → dùng buck DC-DC hạ xuống 5V
> cấp cho ESP32-CAM. Bắt buộc có **fuse** + **tụ lọc** vì nguồn xe nhiễu mạnh và
> ESP32-CAM nhạy với sụt áp (hay brown-out reset).

---

## 3. Sơ đồ kết nối (wiring)

```
Nguồn xe 12V ─[FUSE]─► Buck DC-DC ─► 5V ─► ESP32-CAM (5V/VCC)
                                   │
                                   └─► tụ lọc 470–1000µF + 100nF gần ESP32-CAM

Buzzer:
   ESP32 GPIO ──[1kΩ]── Base NPN
   Buzzer (+) → 5V, Buzzer (−) → Collector, Emitter → GND
   (buzzer active có thể nối trực tiếp GPIO nếu dòng nhỏ, nhưng nên qua transistor)

Motor rung (x2 — trái/phải):
   ESP32 GPIO_L ──[1kΩ]── Base NPN ──► Motor trái  (+ diode flyback)
   ESP32 GPIO_R ──[1kΩ]── Base NPN ──► Motor phải  (+ diode flyback)
   Motor (+) → 3.3V/5V qua điện trở hạ áp phù hợp (motor 3V)

Camera OV2640: đã nối sẵn trên ESP32-CAM (dùng chân camera mặc định).
```

> **Chú ý chân GPIO khả dụng** trên ESP32-CAM rất hạn chế vì phần lớn dùng cho
> camera + PSRAM. Các chân thường dùng được: **GPIO12, 13, 14, 15, 2, 4** (GPIO4 nối
> đèn flash). Kiểm tra kỹ trước khi đấu buzzer/motor.

---

## 4. ✅ Mô hình AI sử dụng: Edge Impulse FOMO (đã chốt)

> **Quyết định:** Dự án này dùng **Edge Impulse FOMO** làm mô hình nhận diện chạy
> **on-device** trên ESP32-CAM.

**Vì sao chọn FOMO (Faster Objects, More Objects):**
- Là model phát hiện vật thể **siêu nhẹ**, thiết kế riêng cho vi điều khiển (MCU).
- **Chạy thật trên chính ESP32-CAM** (đúng tinh thần đề tài "AI trên thiết bị"),
  real-time ở độ phân giải thấp (vd 96x96), không cần máy chủ/WiFi.
- **Dễ train & export:** Edge Impulse xuất thẳng thư viện Arduino cho ESP32-CAM;
  FOMO chỉ cần đánh dấu **tâm vật thể** (centroid) thay vì bounding box → gán nhãn nhanh.
- Phù hợp đúng nhu cầu: phát hiện **"có xe / không"** và **vị trí (trái/phải/sau)**.

**Vì sao KHÔNG dùng YOLOv5s trực tiếp:** ESP32-CAM chỉ có ~4MB PSRAM, CPU 240MHz —
**không đủ sức** chạy YOLOv5s (vốn cần hàng GB RAM + GPU). YOLOv5s chỉ khả thi nếu
stream ảnh về máy chủ xử lý (xem *phương án dự phòng* bên dưới).

| | Mô hình | Nơi chạy | Vai trò |
|---|---------|----------|---------|
| **Chính (chọn)** | **Edge Impulse FOMO** | On-device (ESP32-CAM) | Phương án triển khai chính |
| Dự phòng | YOLOv5s (Ultralytics) | Máy chủ (laptop/Raspberry Pi) | Chỉ dùng nếu cần độ chính xác cao hơn, chấp nhận phụ thuộc WiFi + độ trễ mạng |

> Toàn bộ workflow ở mục **5B** được viết theo hướng **FOMO on-device**.

---

## 5. Flow thực hiện (các bước triển khai)

### Giai đoạn 1 — Phần cứng & camera
1. **Nạp code cho ESP32-CAM** qua USB-TTL (FTDI), nối GPIO0→GND khi nạp.
2. Chạy ví dụ **CameraWebServer** → kiểm tra camera stream được qua WiFi.
3. Test **buzzer**: bật/tắt qua GPIO.
4. Test **2 motor rung** trái/phải qua transistor.
5. Test **nguồn buck 12V→5V** + fuse + tụ lọc → đảm bảo không brown-out.

### Giai đoạn 2 — Mô hình AI nhận diện phương tiện
1. **Chuẩn bị dữ liệu:**
   - Thu ảnh vùng điểm mù phía sau xe (nhiều điều kiện sáng/tối, có/không có xe).
   - Gán nhãn phương tiện (xe máy, ô tô, xe tải, xe buýt).
2. **Train model nhẹ:**
   - **Edge Impulse FOMO** (đơn giản nhất) → export Arduino library cho ESP32-CAM, **hoặc**
   - **ESP-DL** (Espressif) với model lượng tử hoá INT8, **hoặc**
   - YOLOv5s chạy trên máy chủ (hướng C).
3. **Triển khai inference on-device:**
   - Nhúng model vào firmware, chạy nhận diện trên khung hình thu được.
   - Xác định **bounding box** + **độ tin cậy (confidence)**.

### Giai đoạn 3 — Logic cảnh báo
1. **Xác định vùng điểm mù** trong khung hình (vùng quan tâm — ROI).
   - Chỉ cảnh báo khi phương tiện nằm trong ROI và confidence > ngưỡng (vd > 0.5).
2. **Xác định hướng nguy hiểm:** trái / sau (theo vị trí bbox trong khung).
3. **Kích hoạt cảnh báo:**
   - Buzzer beep (có thể beep nhanh/chậm theo mức nguy hiểm).
   - Motor rung **bên tương ứng** với hướng có phương tiện.
4. **Chống báo nhiễu:** yêu cầu phát hiện liên tục N khung hình mới cảnh báo
   (giảm false positive); tắt cảnh báo khi không còn phương tiện.

### Giai đoạn 4 — Tối ưu & lắp đặt
1. Giảm độ phân giải ảnh để tăng FPS (vd 96x96 / 240x240).
2. Đo **thời gian phản hồi** (từ lúc thu ảnh → lúc cảnh báo), tối ưu < ~0.3–0.5s.
3. Đóng hộp chống nước, gắn camera hướng vùng điểm mù, đi dây gọn.
4. Cấp nguồn từ ắc quy xe qua buck + fuse, chạy thử thực địa.

---

## 5B. Workflow chi tiết — Hướng phù hợp nhất

> **Hướng chọn:** chạy **Edge Impulse FOMO on-device** trên ESP32-CAM.
> Lý do: đây là cách **chạy AI thật trên chính ESP32-CAM** (đúng tinh thần đề tài),
> nhẹ đủ để real-time, dễ train, không cần máy chủ/WiFi. FOMO được Espressif/Edge Impulse
> thiết kế riêng cho vi điều khiển, thay thế khả thi cho YOLOv5s (vốn **không chạy nổi**
> trên ESP32-CAM).
>
> **Phương án dự phòng (nếu cần độ chính xác cao hơn / nhiều loại xe):** ESP32-CAM stream
> ảnh qua WiFi về **máy chủ (laptop/Raspberry Pi) chạy YOLOv5s**, kết quả gửi ngược lại
> để bật buzzer/motor — xem mục 4 (hướng C).

### Sơ đồ pipeline tổng thể (FOMO on-device)

```
[OV2640] → ảnh QQVGA/96x96 (grayscale/RGB)
      │
      ▼
[FOMO inference trên ESP32-CAM]  → danh sách tâm vật thể (centroid) + class + score
      │
      ▼
[Lọc theo ROI điểm mù + ngưỡng score]  → có "xe" trong vùng nguy hiểm?
      │  Không  → tắt cảnh báo
      │  Có (đủ N khung liên tiếp)
      ▼
[Xác định hướng từ vị trí centroid]  → trái / phải / sau
      ▼
[Cảnh báo]  Buzzer beep theo hướng + Motor rung đúng bên
```

### Trình tự thực hiện theo bước

**Bước 1 — Dựng phần cứng & camera**
1. Nạp ví dụ **CameraWebServer** qua USB-TTL (nối GPIO0→GND khi nạp, mở lại khi chạy).
2. Xác nhận camera stream được, chỉnh độ phân giải thấp (QQVGA/96x96) để tối ưu FPS.
3. Test buzzer + 2 motor rung qua transistor trên các GPIO khả dụng (12/13/14/15).
4. Dựng nguồn 12V → buck → 5V + fuse + tụ lọc; kiểm tra **không brown-out** khi WiFi/CPU tải.

**Bước 2 — Thu thập & gán nhãn dữ liệu**
1. Gắn camera đúng vị trí thực tế (vùng điểm mù phía sau), thu ảnh nhiều điều kiện:
   sáng/tối, nắng/mưa, có/không có xe, nhiều loại xe (xe máy, ô tô, xe tải, buýt).
2. Upload ảnh lên **Edge Impulse** → gán nhãn (FOMO chỉ cần đánh dấu **tâm vật thể**,
   nhẹ hơn bounding box). Gợi ý ≥ 200–300 ảnh, cân bằng các lớp.

**Bước 3 — Train FOMO trên Edge Impulse**
1. Impulse: Image (96x96, grayscale để nhẹ) → block **Object Detection (FOMO)**.
2. Train, xem F1-score; bổ sung ảnh cho trường hợp nhận sai (đặc biệt nền đường, bóng).
3. Test với ảnh mới; chỉnh confidence threshold.
4. **Export "Arduino library"** → đưa vào project ESP32-CAM (PlatformIO/Arduino).

**Bước 4 — Tích hợp inference + logic cảnh báo**
1. Chạy `run_classifier()` trên từng khung hình; lấy danh sách centroid + score.
2. **Định nghĩa ROI điểm mù** trong khung (vd nửa dưới / vùng hai bên) — chỉ xét xe trong ROI.
3. **Suy ra hướng** từ toạ độ x của centroid (trái/phải/giữa-sau).
4. **Chống nhiễu (debounce):** chỉ cảnh báo khi xe xuất hiện liên tục ≥ N khung (vd 3);
   tắt cảnh báo khi mất xe vài khung.
5. Bật **buzzer** (beep nhanh hơn khi xe gần/score cao) + **motor rung đúng bên**.

**Bước 5 — Tối ưu & lắp đặt thực tế**
1. Đo thời gian 1 vòng (thu ảnh → inference → cảnh báo); giảm độ phân giải/độ phức tạp
   model để đạt FPS chấp nhận được và độ trễ thấp.
2. Đóng hộp chống nước, cố định camera, đi dây gọn, cấp nguồn từ xe qua buck + fuse.
3. Chạy thử thực địa, tinh chỉnh ROI + ngưỡng để giảm báo nhầm.

### Tham số khởi điểm gợi ý
| Tham số | Giá trị bắt đầu |
|---------|----------------|
| Độ phân giải ảnh | 96x96 (grayscale) |
| Ngưỡng confidence | ≥ 0.5–0.6 |
| Số khung xác nhận (debounce) | 3 khung liên tiếp |
| Vùng ROI | nửa dưới khung / vùng hai bên (tuỳ lắp đặt) |
| Mục tiêu độ trễ | < 0.3–0.5 s/vòng |

> **So sánh nhanh 2 hướng:**
> - **FOMO on-device (chọn):** chạy độc lập, không cần mạng, real-time, độ chính xác vừa phải,
>   tốt cho phát hiện "có xe / hướng nào".
> - **Server YOLOv5s (dự phòng):** chính xác cao, phân loại nhiều lớp tốt, nhưng phụ thuộc
>   WiFi + máy chủ + có độ trễ mạng → kém thực dụng khi chạy trên xe đang di chuyển.

### Code ESP32-CAM (pipeline kiểu face recognition)

Đã có firmware mẫu theo đúng pipeline **CAPTURE → DETECT (FOMO) → DIRECTION → TRIGGER**
(tương tự pipeline nhận diện khuôn mặt end-to-end của ESP32-CAM):

➡️ [`firmware/blind_spot/blind_spot.ino`](firmware/blind_spot/blind_spot.ino)

Firmware làm các việc:
- Khởi tạo camera OV2640 (AI-Thinker), chụp khung JPEG → giải nén RGB888 → resize về
  kích thước model.
- Chạy `run_classifier()` của **Edge Impulse FOMO** → lấy danh sách tâm vật thể (centroid).
- Suy ra **hướng nguy hiểm** từ vị trí x của tâm (trái / phải / giữa).
- **Debounce** N khung rồi mới bật **buzzer + motor rung đúng bên**.
- Hỗ trợ cả model *Object Detection (FOMO)* lẫn *Image Classification* (qua `#if`).

**Các bước nạp & chạy:**
1. Train model FOMO trên Edge Impulse (lớp `vehicle` / `xe_may` / `o_to`…), rồi
   *Deployment → Arduino library* → tải file `.zip`.
2. Arduino IDE: *Sketch → Include Library → Add .ZIP Library* (thêm thư viện vừa tải).
3. Sửa dòng `#include <blind_spot_inferencing.h>` cho khớp **tên project** của bạn.
4. *Tools*: **Board = AI Thinker ESP32-CAM**, **PSRAM = Enabled** (bắt buộc — model cần PSRAM).
5. Nối **USB-TTL** (GPIO0→GND khi nạp), nạp code, mở lại GPIO0 rồi reset để chạy.
6. Kiểm tra log Serial **115200** xem nhãn + hướng phát hiện; chỉnh `CONF_THRESHOLD`,
   `DEBOUNCE_FRAMES` cho phù hợp.

> ⚠️ Giống face recognition: **PSRAM là bắt buộc**, và nên đảm bảo ví dụ **CameraWebServer**
> chạy ổn định trước (camera + nguồn tốt) rồi mới nạp firmware nhận diện này.

📘 **Hướng dẫn train model FOMO nhận diện xe (từng bước):**
[`docs/TRAIN_FOMO.md`](docs/TRAIN_FOMO.md)

---

## 6. Thư viện / công cụ gợi ý

| Mục đích | Công cụ |
|----------|---------|
| Camera ESP32-CAM | `esp32-camera` (có sẵn trong ESP32 board package) |
| AI on-device | Edge Impulse SDK (FOMO) / **ESP-DL** / TensorFlow Lite Micro |
| YOLOv5s (hướng C) | Ultralytics YOLOv5 (Python) trên máy chủ + stream MJPEG |
| Nạp code | Arduino IDE / PlatformIO + driver USB-TTL |

---

## 7. Checklist nghiệm thu

- [ ] Camera ESP32-CAM stream/đọc ảnh ổn định.
- [ ] Model phát hiện được phương tiện trong điểm mù.
- [ ] Xác định đúng hướng (trái/sau) + ngưỡng tin cậy.
- [ ] Buzzer kêu + motor rung đúng bên khi có phương tiện.
- [ ] Không cảnh báo khi không có phương tiện (ít báo nhầm).
- [ ] Chạy nguồn thực tế (12V → buck → 5V) ổn định, không brown-out.
- [ ] Thời gian phản hồi đủ nhanh để cảnh báo kịp thời.

---

## 8. Ghi chú

- Đề bài ghi "1 camera gắn phía sau" → tập trung phát hiện vùng điểm mù phía sau.
- **Mô hình AI:** dùng **Edge Impulse FOMO** chạy on-device (đã chốt ở mục 4).
  YOLOv5s chỉ là phương án dự phòng chạy trên máy chủ vì không chạy nổi trên ESP32-CAM.
- Luôn dùng **fuse + tụ lọc** khi lấy nguồn từ xe để bảo vệ board.
