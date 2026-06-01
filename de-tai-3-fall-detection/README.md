# Đề tài 3 — Thiết bị đeo phát hiện ngã thông minh (Smart Fall Detection)

Thiết bị đeo trên người (cổ tay / thắt lưng) dùng cảm biến gia tốc – con quay
**MPU6050** để phát hiện cú ngã đột ngột. Hệ thống dùng **học máy** để phân biệt
giữa *vận động mạnh khi thể thao* và *té ngã thật*, sau đó cảnh báo bằng **motor rung**
và (tuỳ chọn) gửi thông báo về **Telegram**.

---

## 1. Mô tả hoạt động

```
MPU6050 (gia tốc + con quay)
        │  đọc liên tục ~50–100 Hz
        ▼
   Phát hiện sự kiện ngã (ngưỡng + ML)
        │
        ├─ Không phải ngã (đi/chạy/thể thao) ──► bỏ qua
        │
        └─ Nghi ngờ NGÃ
                │
                ├─ Người đứng dậy / cử động bình thường ──► huỷ cảnh báo
                │
                └─ Nằm yên (bất động)
                        │
                        ▼
                Rung cảnh báo + đếm 10 giây
                        │
                        ├─ Bấm nút tắt trong 10s ──► huỷ (báo nhầm / đã ổn)
                        │
                        └─ Hết 10s chưa tắt ──► GỬI THÔNG BÁO TELEGRAM
```

**Yêu cầu cốt lõi:**
- Phát hiện ngã từ tín hiệu MPU6050.
- **ML** phân biệt vận động mạnh (thể thao) vs té ngã thật.
- Cảnh báo rung 10s, nút bấm để tắt.
- (Tuỳ chọn) gửi cảnh báo Telegram nếu không tắt.

---

## 2. Linh kiện cần mua

### Đã có sẵn (em gửi cho anh)
| Linh kiện | SL | Ghi chú |
|-----------|----|---------|
| ESP32-S3 mini (vd: ESP32-S3 Super Mini / DevKit mini) | 1 | Bộ xử lý chính, có WiFi (cần cho Telegram) |
| MPU6050 (GY-521) | 1 | Cảm biến gia tốc 3 trục + con quay 3 trục |
| LCD mini (0.96" OLED I²C / TFT nhỏ) | 1 | Hiển thị trạng thái |

### Cần bổ sung (anh thêm vào)
| Linh kiện | SL | Thông số gợi ý | Giá tham khảo (VND) | Ghi chú |
|-----------|----|----------------|--------------------|---------|
| Motor rung (coin / ERM) | 1–2 | 3V, loại coin 10mm hoặc loại tròn | 8.000 – 20.000 | Cảnh báo rung trên cổ tay |
| Transistor NPN (2N2222 / S8050) **hoặc** module driver | 1 | + điện trở base 1kΩ + diode 1N4007 flyback | 2.000 – 10.000 | Để ESP32 điều khiển motor (không nối trực tiếp) |
| Pin Li-Po 1 cell | 1 | 3.7V, 400–1000 mAh (tuỳ kích thước đeo) | 40.000 – 90.000 | Nguồn cho thiết bị đeo |
| Mạch sạc Li-Po | 1 | TP4056 (có bảo vệ pin DW01) USB-C | 8.000 – 20.000 | Sạc pin qua USB |
| Nút bấm (tactile / nút nhấn nhả) | 1–2 | 6x6mm hoặc nút nhấn tròn | 1.000 – 5.000 | Tắt cảnh báo / nguồn |
| Công tắc nguồn | 1 | Gạt slide switch | 2.000 – 5.000 | Bật/tắt thiết bị |
| Điện trở | vài | 1kΩ, 10kΩ | 1.000 | Base transistor, pull-up nút |
| Dây nối / dây silicon mảnh | — | 26–30 AWG | 10.000 | Đi dây gọn cho thiết bị đeo |
| Vỏ hộp / dây đeo | 1 | In 3D hoặc hộp nhựa nhỏ + dây velcro | 20.000 – 80.000 | Đóng gói đeo tay/thắt lưng |

> **Lưu ý nguồn:** ESP32-S3 mini ăn dòng đỉnh khi bật WiFi (~200–400mA).
> Nên dùng pin ≥ 500mAh và TP4056 để sạc. Motor rung nên có **diode flyback**
> để bảo vệ transistor.

---

## 3. Sơ đồ kết nối (wiring)

```
MPU6050  ── I²C ──► ESP32-S3
   VCC → 3V3
   GND → GND
   SDA → GPIO (SDA, vd GPIO8)
   SCL → GPIO (SCL, vd GPIO9)
   INT → GPIO (tuỳ chọn, để ngắt khi có chuyển động)

LCD OLED ── I²C ──► chung bus với MPU6050 (khác địa chỉ)
   VCC → 3V3, GND → GND, SDA/SCL → chung

Motor rung:
   ESP32 GPIO ──[1kΩ]── Base (NPN)
   Motor (+) → 3V3 (hoặc VBAT)
   Motor (−) → Collector
   Emitter   → GND
   Diode 1N4007 song song motor (catot về +V)

Nút bấm:
   GPIO ── nút ── GND  (dùng INPUT_PULLUP)

Pin & sạc:
   Pin Li-Po → TP4056 (B+ / B−)
   TP4056 OUT → công tắc → VIN/5V ESP32 (hoặc 3V3 nếu board nhận)
```

> Kiểm tra chân GPIO thực tế trên board ESP32-S3 mini của anh (mỗi board khác nhau).
> Tránh dùng các chân strapping (GPIO0, 45, 46…) cho nút bấm.

---

## 4. Flow thực hiện (các bước triển khai)

### Giai đoạn 1 — Phần cứng chạy được (tối thiểu)
1. **Lắp ráp & test từng module:**
   - Nạp code blink/test cho ESP32-S3.
   - Test I²C: quét địa chỉ MPU6050 (0x68) và OLED (0x3C).
   - Đọc và in giá trị gia tốc/gyro ra Serial.
2. **Test motor rung:** điều khiển GPIO bật/tắt qua transistor.
3. **Test nút bấm:** đọc trạng thái nhấn (INPUT_PULLUP).
4. **Test nguồn pin:** chạy bằng pin + TP4056, kiểm tra sạc/xả.
5. **Hiển thị LCD:** in trạng thái ("OK", "FALL?", "ALERT").

### Giai đoạn 2 — Thuật toán phát hiện ngã (rule-based)
1. Tính **độ lớn gia tốc tổng** $a = \sqrt{a_x^2 + a_y^2 + a_z^2}$.
2. Đặc trưng một cú ngã điển hình:
   - **Free-fall:** $a$ tụt gần 0g (rơi tự do) trong thời gian ngắn.
   - **Impact:** ngay sau đó $a$ vọt lên cao (vd > 2.5–3g).
   - **Bất động:** sau va chạm, $a$ ổn định quanh 1g và ít thay đổi (nằm yên).
3. Logic:
   - Phát hiện impact > ngưỡng → vào trạng thái "nghi ngờ ngã".
   - Theo dõi 2–3s sau: nếu **bất động** → cảnh báo; nếu **có cử động đứng dậy** → huỷ.
4. Khi cảnh báo: rung + đếm 10s + chờ nút bấm.

### Giai đoạn 3 — Học máy (ML) phân biệt thể thao vs té ngã
1. **Thu thập dữ liệu (data logging):**
   - Ghi log dữ liệu MPU6050 (ax, ay, az, gx, gy, gz) ra Serial/thẻ nhớ/laptop.
   - Gán nhãn các hành động: đi bộ, chạy, nhảy, ngồi xuống nhanh, vận động thể thao,
     và **các kiểu ngã** (ngã trước/sau/nghiêng).
2. **Trích đặc trưng (feature) theo cửa sổ thời gian (vd 2s):**
   - max/min/mean/std của $a$, jerk (đạo hàm gia tốc), năng lượng tín hiệu,
     biên độ gyro, số lần vượt ngưỡng…
3. **Huấn luyện mô hình nhẹ:**
   - Dùng **Edge Impulse** (khuyến nghị — xuất thẳng thư viện Arduino cho ESP32-S3)
     hoặc scikit-learn (Decision Tree / SVM / Random Forest nhỏ) rồi chuyển sang C.
   - Mô hình phân loại: `bình thường` / `thể thao mạnh` / `ngã`.
4. **Triển khai on-device:**
   - Nhúng mô hình vào firmware (TensorFlow Lite Micro qua Edge Impulse).
   - Chỉ kích hoạt cảnh báo khi ML kết luận **"ngã"** + điều kiện bất động.
5. **Tinh chỉnh:** test thực tế, giảm báo nhầm (false positive) khi tập thể dục.

> **Gợi ý nhanh nhất:** dùng **Edge Impulse Studio** — thu data qua điện thoại/board,
> gán nhãn, train, rồi export "Arduino library" để include vào code ESP32-S3.

### Giai đoạn 4 — Cảnh báo & Telegram (tuỳ chọn)
1. Khi xác nhận ngã + nằm yên:
   - Bật motor rung, hiện "ALERT" trên LCD, đếm ngược 10s.
2. Trong 10s:
   - Nhấn nút → tắt rung, về trạng thái bình thường (coi như báo nhầm/đã ổn).
3. Hết 10s chưa tắt:
   - ESP32-S3 kết nối WiFi → gọi **Telegram Bot API**:
     `https://api.telegram.org/bot<TOKEN>/sendMessage?chat_id=<ID>&text=...`
   - Nội dung: "⚠️ Phát hiện NGÃ! Người dùng không phản hồi sau 10s."
4. **Cách lấy Telegram bot:** chat với **@BotFather** → tạo bot → lấy `TOKEN`;
   lấy `chat_id` qua `@userinfobot` hoặc API `getUpdates`.

---

## 4B. Workflow chi tiết — Hướng phù hợp nhất

> **Hướng chọn:** *Rule-based phát hiện sự kiện ngã* (lọc nhanh) **kết hợp**
> *ML phân loại bằng Edge Impulse* (xác nhận ngã thật vs thể thao), chạy **on-device**
> trên ESP32-S3. Đây là cách cân bằng giữa **độ chính xác** và **độ phức tạp** — phù hợp
> nhất cho phần cứng có sẵn.

### Sơ đồ pipeline tổng thể

```
[MPU6050 @100Hz]
      │  buffer cửa sổ trượt 2s (200 mẫu)
      ▼
[Tầng 1: Rule-based gác cổng]   ── a < 0.4g (free-fall)?  rồi  a > 2.5g (impact)?
      │  KHÔNG  → bỏ qua (tiết kiệm CPU, không chạy ML)
      │  CÓ (nghi ngờ)
      ▼
[Tầng 2: ML phân loại cửa sổ]   ── Edge Impulse: {bình thường | thể thao | NGÃ}
      │  ≠ NGÃ  → bỏ qua
      │  = NGÃ
      ▼
[Tầng 3: Kiểm tra bất động 2–3s]  ── std(a) thấp & |θ| nằm ngang?
      │  Có cử động đứng dậy → huỷ
      │  Nằm yên
      ▼
[Tầng 4: Cảnh báo]  rung + LCD "ALERT" + đếm 10s + chờ nút
      │  Bấm nút → huỷ
      │  Hết 10s
      ▼
[Tầng 5: Telegram]  WiFi → sendMessage (tuỳ chọn)
```

### Trình tự thực hiện theo tuần (gợi ý)

**Tuần 1 — Nền tảng phần cứng + đọc cảm biến**
1. Lắp ESP32-S3 + MPU6050 + OLED trên breadboard, quét I²C xác nhận địa chỉ.
2. Viết hàm đọc 6 trục @ tần số cố định (dùng timer, vd 100Hz) → in Serial.
3. Hiển thị `a_total` và góc nghiêng lên OLED để quan sát trực quan.
4. Test motor rung (qua transistor) + nút bấm (INPUT_PULLUP) + chạy bằng pin/TP4056.

**Tuần 2 — Tầng 1 (rule-based) + thu thập dữ liệu**
1. Cài đặt bộ phát hiện free-fall → impact (ngưỡng cấu hình được).
2. Khi tầng 1 kích hoạt → bật ghi log toàn bộ cửa sổ 2s ra Serial (CSV).
3. **Thu dataset có nhãn:** lặp lại nhiều lần các nhóm hành động:
   - `bình thường`: đi bộ, lên/xuống cầu thang, ngồi xuống ghế nhanh, nằm xuống chủ động.
   - `thể thao`: chạy, nhảy dây, hít đất, vỗ tay mạnh, lắc tay.
   - `NGÃ`: ngã trước, ngã sau, ngã nghiêng trái/phải (ngã có kiểm soát lên đệm).
   - Mục tiêu ≥ 50–100 mẫu/lớp, nhiều người đeo càng tốt.

**Tuần 3 — Huấn luyện ML trên Edge Impulse**
1. Tạo project Edge Impulse → import dữ liệu (hoặc thu trực tiếp qua data forwarder).
2. **Impulse design:** Time series (window 2000ms, stride 200ms) → block
   **Spectral Analysis** (hoặc Flatten + thống kê) → block **Classification**.
3. Train (NN nhỏ hoặc cây quyết định), xem ma trận nhầm lẫn; tinh chỉnh đặc trưng.
4. Test với dữ liệu mới; mục tiêu phân biệt tốt **thể thao vs NGÃ** (giảm false alarm).
5. **Export "Arduino library"** (đã lượng tử hoá INT8) → đưa vào project ESP32-S3.

**Tuần 4 — Tích hợp + cảnh báo + Telegram**
1. Gọi `run_classifier()` của Edge Impulse trên cửa sổ khi tầng 1 kích hoạt.
2. Ghép tầng 3 (kiểm tra bất động) → tầng 4 (rung + 10s + nút bấm).
3. Thêm WiFi + `UniversalTelegramBot`: gửi cảnh báo khi hết 10s chưa tắt.
4. **Tinh chỉnh thực địa:** đeo thử khi tập thể dục để chắc chắn không báo nhầm;
   chỉnh ngưỡng tầng 1 và ngưỡng confidence của ML.

### Tham số khởi điểm gợi ý (tinh chỉnh sau)
| Tham số | Giá trị bắt đầu |
|---------|----------------|
| Tần số lấy mẫu | 100 Hz |
| Cửa sổ ML | 2000 ms, stride 200 ms |
| Ngưỡng free-fall | a < 0.4 g |
| Ngưỡng impact | a > 2.5 g |
| Thời gian xác nhận bất động | 2–3 s, std(a) < 0.1 g |
| Ngưỡng confidence "NGÃ" | ≥ 0.7 |
| Thời gian cảnh báo | 10 s |

> **Vì sao 2 tầng?** Tầng rule-based lọc 99% trường hợp bình thường → ESP32-S3 chỉ chạy
> ML khi thực sự cần → tiết kiệm pin & CPU. ML chỉ làm nhiệm vụ khó: phân biệt cú sốc
> mạnh do thể thao với cú ngã thật.

---

## 4C. Train trước bằng dataset có sẵn (KHÔNG cần đo lại từ đầu)

> Mục tiêu: dùng **ngưỡng kinh điển** + **dataset công khai** để có model/thuật toán
> chạy được ngay, sau đó mới tinh chỉnh bằng vài mẫu của chính mình. Tiết kiệm rất nhiều
> công đo đạc.

### A. Ngưỡng rule-based "ăn liền" (đã nhúng sẵn trong code)
Các ngưỡng dưới đây lấy từ tài liệu phát hiện ngã phổ biến — **dùng được luôn cho
MPU6500/MPU6050** (cả hai chung thanh ghi, chỉ khác mã WHO_AM_I):

| Tham số | Giá trị | Ý nghĩa |
|---------|---------|---------|
| Free-fall | a < **0.5 g** | rơi tự do trước khi chạm đất |
| Impact | a > **2.6 g** (±16g range) | cú va chạm khi tiếp đất |
| Cửa sổ FF→Impact | < **600 ms** | free-fall liền ngay impact |
| Theo dõi bất động | **2.5 s** | nằm yên sau va chạm |
| std(a) bất động | < **0.12 g** | gần như không cử động |
| mean(a) bất động | **0.8–1.2 g** | nằm yên quanh 1g |

> ⚠️ Đặt **dải đo accel ±16g** (code đã set `0x18`) để impact **không bị clip** — đây là
> lỗi hay gặp khiến không bắt được cú ngã.

### B. Dataset công khai để train ML (dùng dữ liệu người khác đã đo)
Có thể tải về, upload thẳng lên **Edge Impulse** để train mà không cần tự đo:

| Dataset | Nội dung | Ghi chú |
|---------|----------|---------|
| **SisFall** | 19 loại sinh hoạt (ADL) + 15 loại ngã, nhiều người, có gia tốc kế | Phổ biến nhất cho fall detection |
| **MobiFall / MobiAct** | ADL + ngã đo bằng cảm biến điện thoại (accel + gyro) | Có cả gyro như MPU6500 |
| **UMAFall** | ngã + hoạt động, nhiều vị trí đeo (cổ tay, hông…) | Hợp với thiết bị đeo |
| **Up-Fall / FARSEEING** | ngã thực tế / mô phỏng | Bổ sung đa dạng |

**Cách dùng nhanh với Edge Impulse:**
1. Tải dataset (CSV: ax, ay, az[, gx, gy, gz]).
2. **Chuẩn hoá đơn vị về `g`** giống thiết bị của bạn (chia cho 9.81 nếu data ở m/s²).
3. Edge Impulse → *Data acquisition* → **Upload CSV**, gán nhãn theo file
   (`fall` / `adl` / hoặc `binh_thuong` / `the_thao` / `nga`).
4. Impulse: window **2000ms**, stride **200ms** → **Spectral Analysis** → **Classification**.
5. Train → kiểm tra confusion matrix → **Export Arduino library**.
6. Cắm vào code: thay phần trong hàm `classifyWindow_ML()` (xem file firmware) bằng
   `run_classifier()` và đọc xác suất nhãn `fall`.

> **Lưu ý quan trọng về "domain shift":** dataset người khác đo ở **vị trí đeo / cảm biến
> khác** → model có thể lệch. Cách xử lý: chuẩn hoá về `g`, dùng đặc trưng **bất biến hướng**
> (độ lớn `a`, std, jerk…), và **fine-tune** bằng 10–20 mẫu của chính bạn sau khi chạy thật.

### C. Code Arduino chạy ngay
Đã có sẵn firmware rule-based hoàn chỉnh (lấy mẫu 100Hz, phát hiện ngã, rung cảnh báo
10s, nút bấm huỷ, chỗ cắm ML + Telegram):

➡️ [`firmware/fall_detection/fall_detection.ino`](firmware/fall_detection/fall_detection.ino)

**Nạp & chạy:**
1. Arduino IDE → Board **ESP32S3 Dev Module** (hoặc board tương ứng).
2. Sửa lại các `#define PIN_*` cho đúng GPIO board của bạn.
3. Nạp → mở Serial Monitor **115200** để xem WHO_AM_I và log phát hiện ngã.
4. (Tuỳ chọn) Đặt `ENABLE_TELEGRAM 1` và điền WiFi/token để bật gửi Telegram.
5. Khi ổn định → thêm thư viện Edge Impulse và hoàn thiện `classifyWindow_ML()`.

---

## 5. Thư viện phần mềm gợi ý

| Mục đích | Thư viện |
|----------|----------|
| Đọc MPU6050 | `MPU6050` (Electronic Cats) / `Adafruit_MPU6050` |
| OLED | `Adafruit_SSD1306` + `Adafruit_GFX` |
| ML on-device | Edge Impulse SDK / TensorFlow Lite Micro |
| WiFi + HTTPS | `WiFi.h` + `WiFiClientSecure` / `HTTPClient` |
| Telegram | `UniversalTelegramBot` (Brian Lough) |

---

## 6. Checklist nghiệm thu

- [ ] Đọc được dữ liệu MPU6050 ổn định.
- [ ] Phát hiện cú ngã giả lập (thả thiết bị / ngã có kiểm soát).
- [ ] Không báo nhầm khi đi/chạy/nhảy (nhờ ML).
- [ ] Rung cảnh báo 10s, nút bấm tắt được.
- [ ] (Tuỳ chọn) Gửi được tin nhắn Telegram khi hết 10s.
- [ ] Chạy bằng pin, sạc được qua TP4056.

---

## 7. Ghi chú

- Đề tài này em mua luôn phần cứng → chi phí linh kiện bổ sung sẽ tính thêm.
- Ưu tiên **phần cứng chạy được + ML hoạt động**; phần Telegram là điểm cộng.
- Đặt cảm biến ở **thắt lưng/hông** cho độ chính xác phát hiện ngã tốt hơn cổ tay
  (nếu cho phép), nhưng cổ tay tiện đeo hơn — tuỳ chọn theo nhu cầu.
