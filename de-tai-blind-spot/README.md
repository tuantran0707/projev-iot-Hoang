# Đề tài — Cảnh báo phương tiện trong điểm mù gương chiếu hậu (Blind Spot Detection)

Hệ thống dùng **camera + AI** nhận diện phương tiện (xe máy, ô tô, xe tải, xe buýt…)
nằm trong **vùng điểm mù phía sau** xe máy, sau đó cảnh báo người lái **kịp thời**
bằng **buzzer** và **motor rung** theo hướng nguy hiểm.

---

## 1. Mô tả hoạt động

```
Camera (gắn phía sau xe)
        │  thu hình liên tục (real-time)
        ▼
   ESP32-S3 chạy mô hình AI (CNN + TFLM) phát hiện phương tiện
        │
        ├─ Không có phương tiện trong điểm mù ──► im lặng
        │
        └─ CÓ phương tiện trong vùng điểm mù (trái hoặc phải)
                │  + độ tin cậy vượt ngưỡng
                ▼
        ┌──────────────────────────────┐
        │  Buzzer kêu (IO47)            │
        │  Motor rung (IO21)            │
        └──────────────────────────────┘
```

Hệ thống chạy **liên tục, tự động**, không cần can thiệp thủ công.

---

## 1B. Kiến trúc & lý thuyết vận hành hệ thống (cho báo cáo)

### Phần cứng thực tế đã dùng
- **Vi điều khiển:** **ESP32‑S3** (lõi kép Xtensa LX7 @240MHz, **8MB PSRAM**, 16MB Flash) —
  mạnh hơn ESP32‑CAM cũ, đủ chạy TFLM + WiFi đồng thời.
- **Camera:** OV‑series, chế độ **QVGA 320×240 GRAYSCALE** (ảnh xám → nhẹ, khỏi giải nén JPEG khi suy luận).
- **Cảnh báo:** **1 motor rung (IO21)** + **1 buzzer (IO47)** — kích hoạt khi phát hiện vật ở **bất kỳ bên nào**.
- **Giao tiếp:** ESP32 tự phát **WiFi Access Point**; người dùng kết nối để xem **luồng hình + trạng thái** trên trình duyệt.

### Pipeline xử lý end-to-end
```
CAPTURE ──► PREPROCESS ──► INFERENCE (TFLM) ──► TEMPORAL FILTER ──► ALERT
(camera)   (cắt 2 nửa,      (CNN INT8 cho        (EMA + debounce)    (motor + buzzer)
            area-average     từng nửa L/R)
            96×96)                │
                                  └────────────► STREAM (web UI hiển thị % + ảnh)
```

### Mô hình đa nhiệm hai nhân (FreeRTOS) — lý thuyết then chốt
Hệ thống vừa phải **suy luận AI** (nặng) vừa **phục vụ web/stream** (cần đáp ứng nhanh).
Giải pháp là **phân tách tác vụ trên 2 nhân** bằng RTOS:

| Tác vụ | Nhân | Vai trò |
|--------|------|---------|
| `inferenceTask` | Core 1 (APP) | Chụp ảnh, nén JPEG, chạy CNN, cập nhật trạng thái |
| `webTask` (cổng 80) | Core 0 (PRO) | Phục vụ trang HTML + JSON trạng thái |
| `streamTask` (cổng 81) | Core 0 (PRO) | Đẩy luồng **MJPEG** liên tục cho trình duyệt |

**Cơ sở lý thuyết:**
- **Phân tách mối quan tâm (separation of concerns):** việc nặng (AI) tách khỏi việc thời gian thực (mạng)
  → stream không bị "khựng" khi model chạy.
- **Đồng bộ bằng mutex:** buffer ảnh JPEG dùng chung được bảo vệ bằng **semaphore/mutex**; dữ liệu được
  **sao chép trong vùng khoá rồi mới gửi ra mạng (ngoài khoá)** → giữ khoá tối thiểu, tránh **đảo ưu tiên/nghẽn**.
- **Biến chia sẻ `volatile`:** cờ trạng thái đọc/ghi giữa các nhân khai báo `volatile` để tránh tối ưu sai bộ nhớ đệm.

### Tối ưu luồng video (streaming)
- Dùng **MJPEG** (`multipart/x-mixed-replace`) trên **một kết nối giữ liên tục** thay vì tải lại từng ảnh
  → **giảm độ trễ**, hình mượt.
- **Suy luận thưa** (`INFER_EVERY_N`): không chạy model mỗi khung mà mỗi N khung → dành CPU cho việc đẩy hình,
  cân bằng giữa **độ mượt** và **tần suất nhận diện**.

### Các tham số điều chỉnh chính (giải thích trong báo cáo)
| Tham số | Ý nghĩa | Đánh đổi |
|---------|---------|----------|
| `CONF_THRESHOLD` (~0.6) | Ngưỡng xác suất báo "có xe" | Cao → ít báo nhầm, dễ bỏ sót; thấp → nhạy hơn, dễ báo nhầm |
| `DEBOUNCE_FRAMES` / `CLEAR_FRAMES` | Số khung xác nhận bật/tắt | Lớn → ổn định, phản ứng chậm; nhỏ → nhanh, dễ rung |
| `INFER_EVERY_N` | Tần suất chạy model | Lớn → stream mượt, nhận diện thưa |
| `JPEG_QUALITY` | Chất lượng ảnh truyền | Cao → nét, nặng; thấp → nhẹ, mượt |
| `α` của EMA (lên/xuống) | Tốc độ bám tín hiệu | Lên nhanh → phát hiện tức thì; xuống chậm → ít nhấp nháy |

---

## 2. Linh kiện cần mua

| Linh kiện | SL | Thông số gợi ý | Giá tham khảo (VND) | Ghi chú |
|-----------|----|----------------|--------------------|---------|
| Module **ESP32-S3** có camera (vd ESP32-S3-CAM / N16R8) | 1 | **8MB PSRAM**, 16MB Flash, camera tích hợp | 150.000 – 250.000 | Bộ xử lý trung tâm (USB tích hợp) |
| Buzzer 5V (loại thường, kêu khi cấp 3.3V) | 1 | active-HIGH | 3.000 – 8.000 | Cảnh báo âm thanh (IO47) |
| Motor rung coin 3V | 1 | loại coin 10mm | 8.000 – 20.000 | Cảnh báo rung (IO21) |
| Transistor NPN (S8050/2N2222) + diode 1N4007 | 1–2 | + điện trở base 1kΩ | 2.000 – 10.000 | Driver cho buzzer & motor |
| Mạch giảm áp DC-DC Buck | 1 | 12V → 5V, **≥ 2A** (vd MP1584 / LM2596) | 10.000 – 30.000 | Hạ áp từ ắc quy/nguồn xe |
| Module nạp/cấp nguồn ổn định | tuỳ | tụ lọc 470–1000µF | 5.000 – 15.000 | Lọc nhiễu nguồn xe |
| Fuse + đế fuse | 1 | 1–2A | 5.000 – 15.000 | Bảo vệ chập/quá dòng |
| Tụ lọc nhiễu | vài | 100nF + 470µF | 5.000 | Chống sụt áp khi WiFi/CPU tải |
| Dây nối, jack nguồn, ống co nhiệt | — | — | 20.000 | Đi dây & đóng gói |
| Cáp USB | 1 | USB-C/Micro tuỳ board | 10.000 – 30.000 | **Nạp code trực tiếp** (ESP32-S3 có USB) |
| Hộp chống nước / giá gắn | 1 | IP65 nhỏ | 30.000 – 80.000 | Bảo vệ ngoài trời |

> **Nguồn cấp thực tế:** xe máy thường có ắc quy 12V → dùng buck DC-DC hạ xuống 5V
> cấp cho ESP32-S3. Bắt buộc có **fuse** + **tụ lọc** vì nguồn xe nhiễu mạnh và
> board nhạy với sụt áp (hay brown-out reset).

---

## 3. Sơ đồ kết nối (wiring)

```
Nguồn xe 12V ─[FUSE]─► Buck DC-DC ─► 5V ─► ESP32-S3 (5V/VCC)
                                   │
                                   └─► tụ lọc 470–1000µF + 100nF gần ESP32-S3

Buzzer (IO47):
   ESP32 IO47 ──[1kΩ]── Base NPN
   Buzzer (+) → 5V, Buzzer (−) → Collector, Emitter → GND
   (buzzer dòng nhỏ có thể nối trực tiếp IO47, nhưng nên qua transistor)

Motor rung (IO21):
   ESP32 IO21 ──[1kΩ]── Base NPN ──► Motor  (+ diode flyback 1N4007)
   Motor (+) → 3.3V/5V qua điện trở hạ áp phù hợp (motor 3V)

Camera: đã nối sẵn trên board ESP32-S3-CAM (dùng chân camera mặc định).
```

> **Chú ý chân GPIO:** tránh các chân dùng cho camera + OPI PSRAM. Dự án dùng
> **IO21 (motor)** và **IO47 (buzzer)** — đều trống trên ESP32-S3-CAM N16R8.
> Kiểm tra sơ đồ chân board cụ thể trước khi đấu nối.

---

## 4. Mô hình AI sử dụng: CNN tự train + TensorFlow Lite Micro (đã triển khai)

> **Quyết định triển khai thực tế:** Dự án dùng một **mạng nơ-ron tích chập (CNN)
> cỡ nhỏ tự huấn luyện trên laptop**, sau đó **lượng tử hoá INT8** và nhúng vào
> ESP32‑S3 chạy bằng **TensorFlow Lite Micro (TFLM)** — KHÔNG dùng Edge Impulse.
> Cách này giúp chủ động hoàn toàn về dữ liệu, kiến trúc model và mã nguồn,
> phù hợp để trình bày **lý thuyết** trong báo cáo.

### 4.1. Vì sao chọn CNN nhỏ + TFLM (cơ sở lý thuyết)

- **CNN (Convolutional Neural Network)** là kiến trúc tiêu chuẩn cho thị giác máy tính:
  các lớp **tích chập (convolution)** học **đặc trưng cục bộ** (cạnh, góc, kết cấu)
  với số tham số ít hơn nhiều so với mạng kết nối đầy đủ (fully‑connected), nhờ
  **chia sẻ trọng số (weight sharing)** và **bất biến tịnh tiến (translation invariance)**.
  → Rất hợp với vi điều khiển vì **ít tham số = ít RAM/Flash**.
- **Bài toán được đơn giản hoá** thành **phân loại nhị phân** "có vật thể / không"
  (2 lớp) trên ảnh **96×96 grayscale**, thay vì phát hiện đối tượng (object detection)
  vốn nặng. Hướng **trái/phải** được suy ra ở firmware bằng cách **cắt khung làm 2 nửa**
  rồi chạy model cho từng nửa (xem 4.4).
- **TFLM (TensorFlow Lite Micro)** là runtime suy luận (inference) **không cần hệ điều hành**,
  chạy model đã lượng tử hoá ngay trên MCU, dùng **bộ nhớ tĩnh (tensor arena)** cấp phát
  trước → tất định, không phân mảnh heap.

### 4.2. Kiến trúc mạng (chi tiết để viết báo cáo)

Model là CNN tuần tự, đầu vào **96×96×1** (grayscale), đầu ra **2 lớp softmax**:

| # | Lớp | Tham số | Kích thước ra | Ý nghĩa |
|---|-----|---------|---------------|---------|
| 0 | Input | — | 96×96×1 | Ảnh xám chuẩn hoá về [0,1] |
| 1 | Conv2D(8, 3×3) + ReLU | 8 bộ lọc 3×3 | 94×94×8 | Học đặc trưng cấp thấp (cạnh, gờ) |
| 2 | MaxPooling2D(2×2) | — | 47×47×8 | Giảm chiều, tăng bất biến vị trí |
| 3 | Conv2D(16, 3×3) + ReLU | 16 bộ lọc | 45×45×16 | Đặc trưng cấp trung (góc, mảng) |
| 4 | MaxPooling2D(2×2) | — | 22×22×16 | Giảm chiều |
| 5 | Conv2D(32, 3×3) + ReLU | 32 bộ lọc | 20×20×32 | Đặc trưng cấp cao (hình dạng) |
| 6 | MaxPooling2D(2×2) | — | 10×10×32 | Giảm chiều |
| 7 | Flatten | — | 3200 | Trải phẳng thành vector |
| 8 | Dense(32) + ReLU | 3200×32 | 32 | Kết hợp đặc trưng |
| 9 | Dropout(0.3) | — | 32 | Chống quá khớp (overfitting) |
| 10 | Dense(2) + Softmax | 32×2 | 2 | Xác suất `empty` / `object` |

**Khái niệm cốt lõi để giải thích trong báo cáo:**
- **Tích chập (convolution):** trượt một **bộ lọc (kernel) 3×3** trên ảnh, tính tổng có
  trọng số → tạo **bản đồ đặc trưng (feature map)**. Mỗi bộ lọc học một loại đặc trưng.
- **Hàm kích hoạt ReLU** $f(x)=\max(0,x)$: tạo **phi tuyến**, giúp mạng học quan hệ phức tạp,
  tính nhanh, giảm hiện tượng **triệt tiêu gradient (vanishing gradient)**.
- **Max pooling:** lấy giá trị lớn nhất trong vùng 2×2 → **giảm kích thước**, **giảm tham số**,
  tăng **bất biến với dịch chuyển nhỏ**.
- **Softmax:** chuẩn hoá đầu ra thành **phân phối xác suất** tổng bằng 1:
  $\text{softmax}(z_i)=\dfrac{e^{z_i}}{\sum_j e^{z_j}}$. Lớp `object` (chỉ số 1) cho xác suất "có xe".
- **Dropout:** ngẫu nhiên "tắt" 30% nơ-ron khi train → **chính quy hoá (regularization)**,
  buộc mạng học đặc trưng bền vững, giảm overfitting.

### 4.3. Huấn luyện & lượng tử hoá (training pipeline)

```
Ảnh thô (empty/, object/)
   │  resize 96×96, grayscale
   ▼
Tăng cường dữ liệu (augmentation): lật ngang, đổi sáng/tương phản, zoom
   │  Rescaling 1/255  (chuẩn hoá [0,1])
   ▼
Huấn luyện CNN  (Adam + sparse categorical cross-entropy, 30 epochs)
   │
   ▼
Lượng tử hoá INT8 (Post-Training Quantization) với representative dataset
   │
   ▼
object_model_int8.tflite  →  object_model.h  (mảng C nhúng firmware)
```

**Giải thích lý thuyết các thành phần:**
- **Tăng cường dữ liệu (data augmentation):** sinh biến thể (lật, đổi sáng, tương phản, zoom)
  → tăng đa dạng, giúp model **tổng quát hoá** tốt hơn trong điều kiện sáng/tối/góc khác nhau,
  giảm overfitting khi dữ liệu ít.
- **Hàm mất mát (loss):** *sparse categorical cross-entropy* đo độ lệch giữa phân phối dự đoán
  và nhãn thật; tối thiểu hoá nó tương đương **cực đại hoá xác suất đúng**.
- **Tối ưu Adam:** thuật toán hạ gradient thích nghi tốc độ học theo từng tham số → hội tụ nhanh, ổn định.
- **Lượng tử hoá INT8 (Post-Training Quantization):** chuyển trọng số/giá trị từ
  **float32 → int8**. Mỗi giá trị thực $r$ được biểu diễn qua số nguyên $q$:
  $r = S\,(q - Z)$ với **S = scale**, **Z = zero‑point**. Lợi ích:
  giảm **~4× dung lượng**, tính số nguyên **nhanh hơn** trên MCU không có FPU mạnh.
  **Representative dataset** cung cấp vài chục mẫu thật để **hiệu chỉnh dải giá trị** (calibration),
  giữ độ chính xác sau khi nén.

### 4.4. Mẹo suy ra HƯỚNG bằng cách cắt 2 nửa (đóng góp của đề tài)

Model chỉ trả lời "có vật thể / không", **không cho biết vị trí**. Để biết
**bên trái hay bên phải** mà vẫn giữ model siêu nhẹ, firmware:

1. Lấy khung **QVGA 320×240 grayscale** từ camera.
2. **Cắt thành 2 nửa**: nửa **TRÁI** (cột 0–159) và nửa **PHẢI** (cột 160–319).
3. **Thu nhỏ mỗi nửa về 96×96** bằng **lấy trung bình vùng (area‑averaging)**: mỗi điểm
   ảnh đầu ra là trung bình một khối điểm nguồn → **khử răng cưa (anti‑aliasing)**, ít nhiễu,
   model ổn định hơn so với lấy mẫu lân cận (nearest‑neighbor).
4. **Chạy model 2 lần** (một cho mỗi nửa) → 2 xác suất `object`.

→ Đây là cách **đổi 1 bài toán phát hiện vị trí (nặng) thành 2 lần phân loại (nhẹ)**,
rất phù hợp ràng buộc tài nguyên của MCU.

### 4.5. Lọc nhiễu thời gian: EMA + debounce (ổn định cảnh báo)

Xác suất từng khung dao động, nếu cảnh báo trực tiếp sẽ **nhấp nháy**. Hệ thống dùng 2 tầng lọc:

- **Trung bình trượt mũ (EMA — Exponential Moving Average)** trên xác suất:
  $\text{ema}_t = \alpha\,p_t + (1-\alpha)\,\text{ema}_{t-1}$.
  Dùng **EMA bất đối xứng**: khi xác suất **tăng** dùng $\alpha$ lớn (bám nhanh → phát hiện tức thì),
  khi **giảm** dùng $\alpha$ nhỏ (hạ chậm → ít rung). So với ngưỡng `CONF_THRESHOLD` (~0.6).
- **Debounce (chống dội):** chỉ **bật** cảnh báo khi phát hiện liên tục ≥ `DEBOUNCE_FRAMES` khung,
  và chỉ **tắt** khi mất vật ≥ `CLEAR_FRAMES` khung → loại bỏ báo nhầm tức thời.

### 4.6. Lý do KHÔNG dùng YOLO/SSD trực tiếp trên thiết bị

ESP32‑S3 có CPU 240MHz + 8MB PSRAM — **không đủ** để chạy các mạng phát hiện đối tượng
hiện đại (YOLOv5s/SSD cần hàng GB RAM + GPU). CNN phân loại nhị phân + cắt 2 nửa là
**giải pháp thực dụng** để đạt **suy luận thời gian thực ngay trên thiết bị**, không phụ
thuộc máy chủ hay mạng WiFi.

| | Mô hình | Nơi chạy | Vai trò |
|---|---------|----------|---------|
| **Chính (đã triển khai)** | **CNN nhỏ + TFLM (INT8)** | On‑device (ESP32‑S3) | Phương án triển khai chính |
| Dự phòng | YOLOv5s (Ultralytics) | Máy chủ (laptop/Raspberry Pi) | Chỉ khi cần độ chính xác cao, chấp nhận phụ thuộc WiFi + độ trễ mạng |

---

## 5. Flow thực hiện (các bước triển khai)

### Giai đoạn 1 — Phần cứng & camera
1. **Nạp code cho ESP32‑S3** qua cổng USB (USB‑CDC tích hợp).
2. Trong Arduino IDE: **Board = ESP32S3 Dev Module**, **PSRAM = OPI PSRAM**, **Flash Size = 16MB**.
3. Test **buzzer (IO47)**: bật/tắt qua GPIO (self‑test lúc khởi động).
4. Test **motor rung (IO21)** qua transistor (self‑test lúc khởi động).
5. Cấp nguồn ổn định 5V; với nguồn từ xe dùng **buck 12V→5V + fuse + tụ lọc** để tránh brown‑out.

### Giai đoạn 2 — Mô hình AI (train trực tiếp trên laptop)
1. **Chuẩn bị dữ liệu:** thu ảnh đúng góc camera thật, 2 thư mục:
   - `laptop-training/object/` — ảnh **CÓ** vật thể trong điểm mù.
   - `laptop-training/empty/` — ảnh nền **KHÔNG** có vật thể.
   - Mỗi lớp ≥ 150–300 ảnh, đa dạng sáng/tối/góc.
2. **Train CNN trên laptop** bằng `train_vehicle_classifier.py` (TensorFlow):
   - Ảnh 96×96 grayscale, augmentation, 30 epochs.
   - Xuất **`object_model_int8.tflite`** (đã lượng tử hoá INT8).
3. **Chuyển sang mảng C** `object_model.h` (xxd hoặc đoạn Python trong file `.py`), copy vào thư mục sketch.
4. **Suy luận on‑device** bằng **TensorFlow Lite Micro** ngay trong firmware ESP32‑S3.

### Giai đoạn 3 — Logic cảnh báo
1. **Cắt khung làm 2 nửa (trái/phải)**, chạy model cho từng nửa → 2 xác suất `object`.
2. **Lọc thời gian:** EMA (bám nhanh khi tăng) + so ngưỡng `CONF_THRESHOLD`.
3. **Chống nhiễu (debounce):** phát hiện liên tục ≥ `DEBOUNCE_FRAMES` khung mới bật;
   mất vật ≥ `CLEAR_FRAMES` khung mới tắt.
4. **Kích hoạt cảnh báo:** phát hiện **bất kỳ bên nào** → bật **motor rung (IO21)** + **buzzer (IO47)**.

### Giai đoạn 4 — Tối ưu & lắp đặt
1. Giữ độ phân giải thấp (QVGA→96×96) và **suy luận thưa** (`INFER_EVERY_N`) để stream mượt.
2. Đo **thời gian phản hồi** (thu ảnh → cảnh báo), tinh chỉnh ngưỡng/EMA.
3. Đóng hộp chống nước, gắn camera hướng vùng điểm mù, đi dây gọn.
4. Cấp nguồn từ ắc quy xe qua buck + fuse, chạy thử thực địa.

---

## 5B. Workflow chi tiết — Train trên laptop + TFLM on-device

> **Hướng triển khai:** **huấn luyện CNN trực tiếp trên laptop** bằng TensorFlow, lượng tử
> hoá INT8, rồi nhúng vào ESP32‑S3 chạy bằng **TensorFlow Lite Micro** — **KHÔNG dùng Edge Impulse**.
> Lý do: chủ động hoàn toàn dữ liệu, kiến trúc model và mã nguồn; chạy AI thật ngay trên
> thiết bị, không cần máy chủ/WiFi.
>
> **Phương án dự phòng (nếu cần độ chính xác cao hơn / nhiều loại xe):** stream ảnh qua WiFi
> về **máy chủ (laptop/Raspberry Pi) chạy YOLOv5s**, kết quả gửi ngược lại để bật buzzer/motor.

### Sơ đồ pipeline tổng thể (TFLM on-device)

```
[Camera] → ảnh QVGA 320×240 grayscale
      │  cắt 2 nửa (trái/phải) + thu nhỏ 96×96 (area-average)
      ▼
[CNN INT8 (TFLM) trên ESP32-S3]  → xác suất "object" cho từng nửa
      │
      ▼
[EMA + ngưỡng + debounce]  → có "xe" ổn định không?
      │  Không  → tắt cảnh báo
      │  Có (đủ N khung liên tiếp)
      ▼
[Cảnh báo]  Buzzer kêu (IO47) + Motor rung (IO21)
```

### Trình tự thực hiện theo bước

**Bước 1 — Dựng phần cứng & camera**
1. Nạp firmware qua **USB** (ESP32‑S3 có USB‑CDC tích hợp, không cần USB‑TTL).
2. Xác nhận camera hoạt động; dùng **QVGA 320×240 grayscale** để nhẹ.
3. Test buzzer (IO47) + motor rung (IO21) bằng **self‑test lúc khởi động**.
4. Cấp nguồn 5V ổn định; nếu lấy từ xe: 12V → buck → 5V + fuse + tụ lọc, kiểm tra **không brown‑out**.

**Bước 2 — Thu thập dữ liệu**
1. Gắn camera đúng vị trí thực tế (vùng điểm mù phía sau), thu ảnh nhiều điều kiện:
   sáng/tối, có/không có xe, nhiều loại xe.
2. **Tự phân thư mục** (không cần công cụ gán nhãn online):
   - `laptop-training/object/` — ảnh **CÓ** vật thể.
   - `laptop-training/empty/`  — ảnh **KHÔNG** có vật thể.
   - Mỗi lớp ≥ 150–300 ảnh, cân bằng số lượng.

**Bước 3 — Train CNN trực tiếp trên laptop (TensorFlow)**
1. Cài đặt: `pip install tensorflow pillow numpy` (Python 3.11).
2. Chạy: `python train_vehicle_classifier.py` — ảnh 96×96 grayscale, augmentation, 30 epochs.
3. Kết quả: **`object_model_int8.tflite`** (đã lượng tử hoá INT8). Xem `accuracy`/`val_accuracy`,
   bổ sung ảnh cho trường hợp nhận sai (nền đường, bóng).
4. **Chuyển sang mảng C**: `xxd -i object_model_int8.tflite > object_model.h` (hoặc đoạn
   Python in sẵn cuối file `.py`); đảm bảo tên mảng là `object_model[]` và `object_model_len`.

**Bước 4 — Tích hợp inference + logic cảnh báo (TFLM)**
1. Copy `object_model.h` vào cùng thư mục sketch `blind_spot_tflm.ino`.
2. Firmware **cắt khung làm 2 nửa**, thu nhỏ 96×96 (area‑average), chạy CNN cho từng nửa.
3. **EMA + ngưỡng + debounce** để ổn định; phát hiện **bất kỳ bên nào** → bật motor + buzzer.
4. Xem log Serial **115200** để theo dõi xác suất L/R và trạng thái cảnh báo.

**Bước 5 — Tối ưu & lắp đặt thực tế**
1. Giữ độ phân giải thấp + **suy luận thưa** (`INFER_EVERY_N`) để stream mượt.
2. Đóng hộp chống nước, cố định camera, đi dây gọn, cấp nguồn từ xe qua buck + fuse.
3. Chạy thử thực địa, tinh chỉnh `CONF_THRESHOLD` + EMA để giảm báo nhầm.

### Tham số khởi điểm gợi ý
| Tham số | Giá trị bắt đầu |
|---------|----------------|
| Độ phân giải model | 96×96 (grayscale) |
| Ngưỡng `CONF_THRESHOLD` | ≥ 0.5–0.6 |
| `DEBOUNCE_FRAMES` / `CLEAR_FRAMES` | 3 khung liên tiếp |
| `INFER_EVERY_N` | 4 (chạy model mỗi 4 khung) |
| Mục tiêu độ trễ | < 0.3–0.5 s/vòng |

> **So sánh nhanh 2 hướng:**
> - **CNN + TFLM on-device (đã triển khai):** chạy độc lập, không cần mạng, real-time, chủ động
>   toàn bộ dữ liệu/mã nguồn; tốt cho phát hiện "có xe / hướng nào".
> - **Server YOLOv5s (dự phòng):** chính xác cao, nhiều lớp, nhưng phụ thuộc WiFi + máy chủ +
>   độ trễ mạng → kém thực dụng khi chạy trên xe đang di chuyển.

### Firmware chính (đã triển khai)

Pipeline **CAPTURE → PREPROCESS → INFERENCE (TFLM) → FILTER → TRIGGER**:

➡️ [`firmware/blind_spot_tflm/blind_spot_tflm.ino`](firmware/blind_spot_tflm/blind_spot_tflm.ino)

Firmware làm các việc:
- Khởi tạo camera (QVGA grayscale), chụp khung → cắt 2 nửa → thu nhỏ 96×96 (area-average).
- Nạp model `object_model.h` và chạy **TensorFlow Lite Micro** (AllOpsResolver, tensor arena trong PSRAM).
- Lọc xác suất bằng **EMA + debounce**, phát hiện bất kỳ bên nào → bật **motor (IO21) + buzzer (IO47)**.
- Phát **WiFi Access Point** + **MJPEG stream** (cổng 81) + trang điều khiển (cổng 80) qua đa nhiệm FreeRTOS.

**Các bước nạp & chạy:**
1. Train trên laptop → sinh `object_model.h`, copy vào thư mục sketch.
2. Arduino IDE cài thư viện **Chirale_TensorFlowLite**.
3. *Tools*: **Board = ESP32S3 Dev Module**, **PSRAM = OPI PSRAM**, **Flash Size = 16MB**.
4. Nạp qua USB, mở Serial Monitor **115200**.
5. Nghe **self-test** (motor + buzzer) lúc khởi động; kết nối WiFi `ESP32-BlindSpot` rồi mở `http://192.168.4.1`.
6. Chỉnh `CONF_THRESHOLD`, `DEBOUNCE_FRAMES`, EMA cho phù hợp.

> ⚠️ **PSRAM là bắt buộc** (Tools > PSRAM = OPI PSRAM) — thiếu sẽ lỗi
> `frame buffer malloc failed`. Tensor arena và buffer camera đều nằm trong PSRAM.

📘 **Hướng dẫn train model trên laptop (từng bước):**
[`laptop-training/train_vehicle_classifier.py`](laptop-training/train_vehicle_classifier.py)

---

## 6. Thư viện / công cụ gợi ý

| Mục đích | Công cụ |
|----------|---------|
| Camera ESP32 | `esp32-camera` (có sẵn trong ESP32 board package) |
| Train model (laptop) | **TensorFlow / Keras** (Python 3.11) |
| AI on-device | **TensorFlow Lite Micro** (thư viện `Chirale_TensorFlowLite`) |
| YOLOv5s (dự phòng) | Ultralytics YOLOv5 (Python) trên máy chủ + stream MJPEG |
| Nạp code | Arduino IDE / PlatformIO (USB trực tiếp, ESP32-S3) |

---

## 7. Checklist nghiệm thu

- [ ] Camera ESP32-S3 đọc ảnh ổn định, stream MJPEG mượt trên trình duyệt.
- [ ] Model (CNN + TFLM) phân biệt đúng có/không có vật thể.
- [ ] Suy ra đúng nửa trái/phải có vật thể (% hiển thị trên web).
- [ ] Buzzer (IO47) kêu + motor (IO21) rung khi phát hiện.
- [ ] Không cảnh báo khi không có vật thể (ít báo nhầm nhờ EMA + debounce).
- [ ] Chạy nguồn thực tế (12V → buck → 5V) ổn định, không brown-out.
- [ ] Thời gian phản hồi đủ nhanh để cảnh báo kịp thời.

---

## 8. Ghi chú

- Đề bài ghi "1 camera gắn phía sau" → tập trung phát hiện vùng điểm mù phía sau.
- **Mô hình AI:** **CNN nhỏ tự train trên laptop** (TensorFlow) + lượng tử hoá INT8, chạy
  on-device bằng **TensorFlow Lite Micro** — **KHÔNG dùng Edge Impulse**. YOLOv5s chỉ là
  phương án dự phòng chạy trên máy chủ.
- **PSRAM (OPI) bắt buộc** trên ESP32-S3 để chứa tensor arena + buffer camera.
- Luôn dùng **fuse + tụ lọc** khi lấy nguồn từ xe để bảo vệ board.
