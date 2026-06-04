# Xe Robot 2 Bánh — SM-ESP32 + L298N + ThingsBoard

Điều khiển xe 2 bánh (differential drive) qua IoT ThingsBoard Cloud.
Có 2 chế độ: Manual (điều khiển tay) và Auto (tự đi đến mục tiêu, quét gas, quay về).

---

## 1. Tổng quan hệ thống

| Thành phần | Mô tả |
|-----------|-------|
| Board | SM-ESP32 |
| Driver | L298N |
| Động cơ | 2 motor DC giảm tốc (trái + phải) |
| Cảm biến | Gas analog ở GPIO36 |
| IoT | ThingsBoard Cloud qua MQTT (Shared Attributes) |
| Lưu trữ | LittleFS lưu lịch hẹn giờ auto |
| Thời gian | NTP để lấy epoch hiện tại (không cache) |

### Nguyên lý:
- Không có encoder → open-loop theo thời gian.
- Tốc độ giới hạn 40% PWM để dễ calibrate và giảm trượt.
- Auto mode đi thẳng + rẽ 90 độ (không đi chéo).
- Map kích thước 300cm × 300cm, gốc tọa độ luôn là (0,0).

---

## 2. Sơ đồ chân

```
ESP32          L298N
─────          ─────
GPIO32 (PWM) → ENA   ─┐
GPIO33       → IN1    ├─ Motor phải
GPIO25       → IN2   ─┘

GPIO14 (PWM) → ENB   ─┐
GPIO26       → IN3    ├─ Motor trái
GPIO27       → IN4   ─┘

GPIO36 (ADC) ← Gas sensor AO
```

> GPIO36 chỉ là input analog. Tất cả GND phải nối chung.

---

## 3. Chế độ hoạt động

### 3.1 Manual (mặc định khi khởi động)

Điều khiển trực tiếp bằng x, y. Duration chỉ là timeout tự dừng.

Công thức vi sai:
- Bánh trái = (x + y) × 40% × 255 / 100
- Bánh phải = (x - y) × 40% × 255 / 100

| x | y | Hành vi |
|---|---|---------|
| 40 | 0 | Tiến thẳng |
| -40 | 0 | Lùi thẳng |
| 0 | 40 | Xoay phải tại chỗ |
| 0 | -40 | Xoay trái tại chỗ |
| 40 | 20 | Tiến + rẽ phải |

### 3.2 Auto

Luồng hoạt động:
1. Nhận lệnh auto → lưu lịch vào LittleFS.
2. Chờ đến `run_at_epoch` (hoặc chạy ngay nếu = 0).
3. Xuất phát từ (0,0), hướng North.
4. Đi thẳng theo trục Y đến target_y.
5. Rẽ 90° → đi thẳng theo trục X đến target_x.
6. Đến đích → quay 360° tại chỗ, đo gas liên tục.
7. Tìm góc có gas cao nhất.
8. Quay về (0,0) theo đường ngược lại.
9. Xóa lịch khỏi LittleFS, kết thúc.

Các event gửi lên ThingsBoard:
- `auto_started` — bắt đầu chạy
- `arrived_target` — đến đích
- `scan_sample` — mỗi mẫu gas khi quét
- `scan_done` — quét xong, kèm max gas + góc
- `returned_home` — về tới (0,0)

---

## 4. LittleFS — Lưu lịch hẹn giờ

File: `/auto_schedule.json`

```json
{
  "enabled": true,
  "target_x": 180,
  "target_y": 260,
  "run_at_epoch": 1749052800
}
```

**Khi nào lưu:**
- Mỗi khi nhận attributes mới có auto config → ghi đè file.

**Khi nào đọc:**
- Lúc khởi động → nếu có lịch enabled thì tự chuyển sang auto mode chờ giờ.

**Khi nào xóa:**
- Mission hoàn thành (returned_home).
- Nhận stop=true.
- Nhận auto_enabled=false.

**Tại sao lưu lịch mà không lưu NTP:**
- NTP chỉ cần sync 1 lần khi có WiFi là đủ.
- Lịch hẹn giờ mới quan trọng vì nếu mất điện/reboot, xe phải nhớ còn nhiệm vụ.

---

## 5. JSON Shared Attributes gửi xuống

### 5.1 Key chung
| Key | Kiểu | Mô tả |
|-----|------|-------|
| mode | "manual" / "auto" | Chuyển chế độ |
| stop | true | Dừng khẩn cấp, hủy auto, xóa lịch |

### 5.2 Key manual
| Key | Kiểu | Mô tả |
|-----|------|-------|
| x | -100..100 | Tiến/lùi |
| y | -100..100 | Rẽ phải/trái |
| duration | ms | Timeout tự dừng |

### 5.3 Key auto
| Key | Kiểu | Mô tả |
|-----|------|-------|
| auto_enabled | bool | Bật/tắt auto |
| target_x | 0..300 | Tọa độ X đích (cm) |
| target_y | 0..300 | Tọa độ Y đích (cm) |
| run_at_epoch | uint32 | Thời gian chạy (Unix epoch UTC), 0 = ngay |
| auto_start_now | true | Chạy auto ngay lập tức |

### 5.4 Key calibration
| Key | Kiểu | Mô tả |
|-----|------|-------|
| move_ms_cm_x | float | ms để đi 1 cm theo trục X |
| move_ms_cm_y | float | ms để đi 1 cm theo trục Y |
| turn_90_ms | uint32 | ms để quay đúng 90° |
| scan_360_ms | uint32 | ms để quay đúng 360° |

---

## 6. Telemetry gửi lên (mỗi 3 giây)

| Key | Mô tả |
|-----|-------|
| gas_raw | ADC thô 0..4095 |
| gas_voltage | Điện áp quy đổi (V) |
| motor_x | x đang áp dụng |
| motor_y | y đang áp dụng |
| mode | manual / auto |
| auto_state | disabled / waiting_time / moving_to_target / scanning_gas / returning_home / finished |
| auto_enabled | Trạng thái auto |
| target_x, target_y | Tọa độ đích |
| est_x, est_y | Vị trí ước lượng |
| heading | 0=North, 1=East, 2=South, 3=West |
| run_at_epoch | Lịch hẹn |
| epoch | Thời gian hiện tại |
| rssi | WiFi signal |

---

## 7. Hướng dẫn cài đặt

### Arduino IDE
1. Board: ESP32 Dev Module (hoặc SM-ESP32 tương ứng).
2. Partition Scheme: Default 4MB with spiffs (để có LittleFS).
3. Thư viện cần cài:
   - PubSubClient (Nick O'Leary) ≥ 2.8
   - ArduinoJson (Benoit Blanchon) ≥ 6.x

### Cấu hình trong code
```cpp
const char* WIFI_SSID     = "TDTuan";
const char* WIFI_PASSWORD = "12345678";
const char* TB_SERVER     = "thingsboard.cloud";
const char* TB_TOKEN      = "8kG78bmcD7x0h1hBPk1r";
```

---

## 8. HƯỚNG DẪN TEST TỪNG BƯỚC

### Bước 1 — Nạp firmware và kiểm tra kết nối

1. Compile & upload `firmware/robot_car/robot_car.ino`.
2. Mở Serial Monitor (115200 baud).
3. Kiểm tra log:
   ```
   [WiFi] Connected: 192.168.x.x
   [MQTT] ok
   [NTP] Synced, epoch=...
   [FS] LittleFS mounted
   === Ready ===
   ```
4. Nếu không thấy WiFi connected → kiểm tra SSID/password.
5. Nếu MQTT fail → kiểm tra token trên ThingsBoard.

### Bước 2 — Test manual cơ bản

Trên ThingsBoard → Device → Shared attributes, set:

**Test tiến thẳng 1 giây:**
```json
{"mode": "manual", "x": 40, "y": 0, "duration": 1000}
```

**Test lùi 1 giây:**
```json
{"mode": "manual", "x": -40, "y": 0, "duration": 1000}
```

**Test xoay phải tại chỗ:**
```json
{"mode": "manual", "x": 0, "y": 40, "duration": 500}
```

**Test xoay trái tại chỗ:**
```json
{"mode": "manual", "x": 0, "y": -40, "duration": 500}
```

**Dừng khẩn cấp:**
```json
{"stop": true}
```

Kiểm tra:
- [ ] Xe tiến đúng hướng mong muốn.
- [ ] Xe lùi đúng hướng.
- [ ] Xe xoay phải/trái đúng chiều.
- [ ] Xe dừng sau duration.
- [ ] Stop dừng ngay lập tức.

> Nếu hướng bị ngược: đảo IN1/IN2 hoặc IN3/IN4 trong code.

### Bước 3 — Calibration tiến thẳng

Mục tiêu: tìm `move_ms_cm_y` (ms để đi 1 cm khi tiến thẳng).

1. Đặt xe ở vạch 0 cm trên sàn phẳng.
2. Gửi:
   ```json
   {"mode": "manual", "x": 40, "y": 0, "duration": 2000}
   ```
3. Xe dừng → dùng thước đo quãng đường (cm).
4. Tính: `move_ms_cm_y = 2000 / quãng_đường_cm`
5. **Lặp lại 5 lần**, ghi bảng:

| Lần | Duration (ms) | Quãng đường (cm) | ms/cm |
|-----|--------------|------------------|-------|
| 1 | 2000 | ? | ? |
| 2 | 2000 | ? | ? |
| 3 | 2000 | ? | ? |
| 4 | 2000 | ? | ? |
| 5 | 2000 | ? | ? |
| **TB** | | | **?** |

6. Lấy trung bình → đó là `move_ms_cm_y`.

### Bước 4 — Calibration trục X

Tương tự bước 3 nhưng xe hướng sang ngang (đã quay 90°):

1. Quay xe 90° sang phải bằng tay.
2. Gửi duration cố định, đo quãng đường.
3. Tính `move_ms_cm_x`.

> Thông thường move_ms_cm_x ≈ move_ms_cm_y nếu sàn đồng nhất.

### Bước 5 — Calibration quay 90 độ

Mục tiêu: tìm `turn_90_ms`.

1. Đánh dấu hướng ban đầu của xe trên sàn.
2. Gửi:
   ```json
   {"mode": "manual", "x": 0, "y": 40, "duration": 400}
   ```
3. Đo góc xe đã quay (dùng thước đo góc hoặc vạch trên sàn).
4. Nếu < 90° → tăng duration. Nếu > 90° → giảm duration.
5. Lặp lại cho đến khi quay đúng 90°.
6. Thử cả quay trái (y=-40) → lấy trung bình.

| Lần | y | Duration (ms) | Góc thực (°) | Ghi chú |
|-----|---|--------------|--------------|---------|
| 1 | 40 | 400 | ? | Quay phải |
| 2 | 40 | 420 | ? | |
| 3 | -40 | 420 | ? | Quay trái |
| 4 | -40 | 430 | ? | |

→ Chọn giá trị duration mà góc gần 90° nhất = `turn_90_ms`.

### Bước 6 — Calibration quay 360 độ

1. Đánh dấu hướng ban đầu.
2. Gửi:
   ```json
   {"mode": "manual", "x": 0, "y": 40, "duration": 1700}
   ```
3. Kiểm tra xe quay đủ 1 vòng về đúng hướng ban đầu.
4. Điều chỉnh duration cho đến khi đúng 360°.
5. → `scan_360_ms` = duration đó.

> Mẹo: `scan_360_ms` ≈ `turn_90_ms` × 4 nhưng thực tế có thể lệch do quán tính.

### Bước 7 — Đẩy calibration lên ThingsBoard

Sau khi có 4 giá trị, gửi shared attributes:
```json
{
  "move_ms_cm_x": 24.0,
  "move_ms_cm_y": 24.0,
  "turn_90_ms": 420,
  "scan_360_ms": 1700
}
```

Firmware nhận và áp dụng ngay (không cần reboot).

### Bước 8 — Test auto ngắn

Test tuyến đơn giản trước:
```json
{
  "mode": "auto",
  "target_x": 0,
  "target_y": 50,
  "auto_start_now": true
}
```

Kỳ vọng:
1. Xe đi thẳng 50 cm theo trục Y.
2. Dừng, quay 1 vòng scan gas.
3. Quay về (0,0).
4. Serial hiện: auto_started → arrived_target → scan_done → returned_home.
5. ThingsBoard nhận event telemetry tương ứng.

Kiểm tra:
- [ ] Xe đi đúng ~50 cm.
- [ ] Xe quay 1 vòng đầy đủ.
- [ ] Xe quay về đúng vị trí xuất phát (sai số ≤ 10 cm là OK).
- [ ] File `/auto_schedule.json` bị xóa sau khi xong.

### Bước 9 — Test auto với rẽ

```json
{
  "mode": "auto",
  "target_x": 80,
  "target_y": 100,
  "auto_start_now": true
}
```

Kỳ vọng:
1. Xe đi Y=100 cm (tiến thẳng).
2. Rẽ phải 90°.
3. Đi X=80 cm.
4. Scan gas 360°.
5. Quay trái 90°, đi ngược Y=100 cm.
6. Rẽ phải 90°, đi ngược X=80 cm (hoặc ngược lại tùy planner).
7. Về (0,0).

### Bước 10 — Test hẹn giờ + reboot

1. Lấy epoch 5 phút sau thời điểm hiện tại:
   - Vào https://www.epochconverter.com/
   - Lấy epoch hiện tại + 300.
2. Gửi:
   ```json
   {
     "mode": "auto",
     "auto_enabled": true,
     "target_x": 100,
     "target_y": 100,
     "run_at_epoch": <epoch_5_phut_sau>
   }
   ```
3. Kiểm tra Serial: `[FS] Schedule saved`.
4. **Rút điện ESP32, chờ 30 giây, cắm lại.**
5. Kiểm tra Serial sau reboot:
   ```
   [FS] Schedule loaded: en=1 tx=100 ty=100 epoch=...
   [FS] Auto mode resumed from saved schedule
   ```
6. Chờ đến giờ → xe tự chạy.
7. Sau khi mission xong → Serial hiện `returned_home`.

### Bước 11 — Test dừng khẩn cấp giữa chừng

1. Bắt đầu auto mission bất kỳ.
2. Giữa lúc xe đang chạy, gửi:
   ```json
   {"stop": true}
   ```
3. Kỳ vọng: xe dừng ngay, auto bị hủy, file schedule bị xóa.

### Bước 12 — Test tuyến dài (sau khi calibration ổn)

```json
{"mode": "auto", "target_x": 0, "target_y": 300, "auto_start_now": true}
```
```json
{"mode": "auto", "target_x": 300, "target_y": 0, "auto_start_now": true}
```
```json
{"mode": "auto", "target_x": 300, "target_y": 300, "auto_start_now": true}
```

Ghi lại sai số vị trí cuối cùng để đánh giá chất lượng calibration.

---

## 9. Xử lý sự cố

| Triệu chứng | Nguyên nhân | Cách xử lý |
|-------------|-------------|-------------|
| Xe không chạy khi gửi manual | WiFi/MQTT chưa kết nối | Xem Serial log |
| Xe chạy sai hướng | IN1/IN2 hoặc IN3/IN4 đấu ngược | Đảo 2 dây hoặc sửa code |
| Xe tiến nhưng bị lệch | Motor 2 bên tốc độ khác nhau | Bình thường ở open-loop, cần encoder để fix |
| Auto chạy quá xa / quá ngắn | Calibration sai | Chạy lại bước 3-4 |
| Xe quay không đủ / quá 90° | turn_90_ms sai | Chạy lại bước 5 |
| NTP sync failed | Không có internet | Kiểm tra router có kết nối WAN |
| Xe không tự chạy sau reboot | LittleFS mount failed | Chọn Partition Scheme có spiffs |
| Gas đọc toàn 0 | Sensor chưa nóng đủ | Chờ 1-2 phút warm-up |

---

## 10. Mẫu payload tham khảo nhanh

### Chuyển manual
```json
{"mode": "manual"}
```

### Manual chạy tiến 1.5 giây
```json
{"mode": "manual", "x": 40, "y": 0, "duration": 1500}
```

### Dừng khẩn
```json
{"stop": true}
```

### Chạy auto ngay
```json
{"mode": "auto", "target_x": 120, "target_y": 200, "auto_start_now": true}
```

### Hẹn giờ auto
```json
{
  "mode": "auto",
  "auto_enabled": true,
  "target_x": 180,
  "target_y": 260,
  "run_at_epoch": 1749052800
}
```

### Đẩy calibration
```json
{
  "move_ms_cm_x": 24.0,
  "move_ms_cm_y": 24.0,
  "turn_90_ms": 420,
  "scan_360_ms": 1700
}
```

---

## 11. Cấu trúc thư mục

```
xe-can-bang/
├── README.md
└── firmware/
    └── robot_car/
        └── robot_car.ino
```

---

## 12. Kế hoạch phát triển

| Giai đoạn | Nội dung | Trạng thái |
|-----------|---------|-----------|
| A | Giới hạn 40% PWM, đi thẳng + rẽ 90°, calibration qua attributes, lưu lịch LittleFS | ✅ Done |
| B | Thêm IMU giảm sai số hướng, profile tăng/hạ tốc, lọc gas | Chưa |
| C | Thêm encoder, odometry, PID closed-loop | Chưa |
