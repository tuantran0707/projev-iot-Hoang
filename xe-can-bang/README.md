# Đề tài — Xe / Robot tự cân bằng 2 bánh có chân co duỗi (Self-Balancing Wheeled-Leg Robot)

Robot tự cân bằng trên **2 bánh** (con lắc ngược – inverted pendulum), gắn trên **2 chân
có thể co duỗi** để thay đổi chiều cao / hấp thụ địa hình. Bộ điều khiển trung tâm là
**ESP32**, dùng cảm biến IMU + thuật toán **PID** để giữ thăng bằng.

---

## 1. Mô tả hoạt động

```
        IMU (MPU6050) đo góc nghiêng θ
                    │
                    ▼
        ESP32 chạy vòng điều khiển PID
                    │
        ┌───────────┴───────────┐
        ▼                       ▼
  Động cơ bánh trái       Động cơ bánh phải
  (giữ thăng bằng + di chuyển)

  Song song:
  ESP32 ──► Servo khớp chân (co/duỗi)  ──► thay đổi chiều cao / tư thế
```

**Hai cơ cấu chính:**
1. **Cân bằng & di chuyển:** 2 động cơ bánh điều khiển bằng PID dựa trên góc nghiêng.
2. **Chân co duỗi:** servo/khớp thay đổi độ dài chân (nâng/hạ thân, thích nghi địa hình).

---

## 2. Hướng phù hợp nhất (đã chọn)

> Đây là phương án **dễ làm – ổn định – chi phí hợp lý** cho ESP32.

| Hạng mục | Lựa chọn khuyến nghị | Lý do |
|----------|---------------------|-------|
| Não điều khiển | **ESP32** (có sẵn) | Đủ mạnh, có WiFi/BT để chỉnh PID & điều khiển |
| Cảm biến cân bằng | **MPU6050** | Rẻ, phổ biến, dư dùng cho balance |
| Động cơ bánh | **DC motor giảm tốc CÓ encoder** (JGB37-520) | Điều khiển mượt, có phản hồi tốc độ, dễ lên PID |
| Driver bánh | **TB6612FNG** | Hiệu suất tốt hơn L298N, nhỏ gọn, đủ dòng |
| Chân co duỗi | **Servo mô-men cao MG996R/DS3218 + PCA9685** | Đơn giản nhất để co duỗi khớp, điều khiển nhiều servo dễ |
| Nguồn | **LiPo 3S 11.1V** + **Buck DC-DC** ra 5V | Đủ áp cho motor, hạ 5V cho ESP32/servo |

> Lựa chọn thay thế: nếu muốn **cân bằng cực ổn định và đơn giản thuật toán** →
> dùng **stepper NEMA17 + A4988** cho bánh (nhưng nặng & tốn pin hơn).

---

## 3. Linh kiện cần mua

### Đã có sẵn
| Linh kiện | SL | Ghi chú |
|-----------|----|---------|
| ESP32 (DevKit / mini) | 1 | Bộ xử lý trung tâm |

### Cần mua thêm
| Nhóm | Linh kiện | SL | Thông số gợi ý | Giá tham khảo (VND) |
|------|-----------|----|----------------|--------------------|
| Cảm biến | MPU6050 (GY-521) | 1 | IMU 6 trục | 15.000 – 30.000 |
| Bánh | DC motor giảm tốc **có encoder** (JGB37-520) | 2 | 12V, ~178–333 RPM | 90.000 – 150.000/cái |
| Bánh | Driver **TB6612FNG** | 1 | 2 kênh, ~1.2A/kênh | 25.000 – 45.000 |
| Bánh | Bánh xe + khớp nối trục (coupling) | 2 | Ø65–90mm | 20.000 – 60.000 |
| Chân | Servo **MG996R** (hoặc DS3218 khoẻ hơn) | 2–4 | Mô-men cao, kim loại | 60.000 – 120.000/cái |
| Chân | Module **PCA9685** (16 kênh PWM) | 1 | Điều khiển nhiều servo | 25.000 – 45.000 |
| Nguồn | Pin **LiPo 3S** 1500–2200mAh + jack XT60 | 1 | 11.1V | 200.000 – 350.000 |
| Nguồn | Sạc cân bằng LiPo (B6 / iMAX) | 1 | Sạc 3S an toàn | 200.000 – 400.000 |
| Nguồn | Buck DC-DC (→5V, ≥3A, vd LM2596/XL4015) | 1–2 | Cấp 5V cho ESP32 + servo | 15.000 – 40.000 |
| Bảo vệ | Công tắc nguồn + fuse 5–10A + tụ 1000µF | 1 bộ | Bảo vệ & lọc nguồn | 20.000 – 40.000 |
| Khung | Khung in 3D / mica / nhôm (thân + 2 chân + giá motor) | 1 | Theo thiết kế | 100.000 – 300.000 |
| Khác | Dây nối, jumper, ốc vít, gối đỡ trục, đầu nối | — | — | 50.000 |

> **Lưu ý nguồn servo:** servo MG996R có thể kéo dòng đỉnh ~1–2A/cái. Cấp 5V cho servo
> qua **buck riêng (≥3A)**, KHÔNG lấy từ chân 5V của ESP32. Nối chung GND.

---

## 4. Sơ đồ kết nối (wiring)

```
Pin LiPo 3S ─[FUSE]─[Công tắc]─┬─► Driver TB6612FNG (VM, motor 12V)
                               │
                               └─► Buck DC-DC ──► 5V ─┬─► ESP32 (5V/VIN)
                                                      └─► Servo / PCA9685 (V+)

MPU6050 ── I²C ──► ESP32 (SDA=GPIO21, SCL=GPIO22) ; VCC 3V3, GND
PCA9685 ── I²C ──► chung bus với MPU6050 (khác địa chỉ 0x40) ; nối servo vào PCA9685

TB6612FNG ──► ESP32:
   PWMA, AIN1, AIN2  → motor trái
   PWMB, BIN1, BIN2  → motor phải
   STBY → 3V3 (hoặc 1 GPIO)
Encoder motor ──► ESP32 (mỗi motor 2 chân A/B vào GPIO có ngắt)

Servo (chân trái/phải) ──► kênh PCA9685 (PWM), V+ từ buck 5V, GND chung
```

> Tránh dùng các chân strapping (GPIO0, 2, 12, 15) cho encoder/PWM quan trọng.
> Tất cả GND (pin, buck, ESP32, driver, servo) phải **nối chung**.

---

## 5. Workflow thực hiện (chi tiết)

### Bước 1 — Lắp khung & kiểm tra cơ khí
1. In/gia công khung: thân chính, 2 chân, giá bắt motor, trục bánh.
2. Lắp motor bánh ở **đáy chân**, servo ở **khớp chân**.
3. Đảm bảo trọng tâm (pin nặng) đặt **cao** để con lắc ngược dễ điều khiển hơn.
4. Cân chỉnh sao cho robot đối xứng trái/phải.

### Bước 2 — Nguồn & nạp code cơ bản
1. Đấu nguồn: LiPo → fuse → công tắc → driver + buck → ESP32.
2. Test buck ra đúng 5V. Test ESP32 nạp được code (blink).
3. **Test riêng từng phần:**
   - Đọc MPU6050 → in góc nghiêng ra Serial.
   - Quay từng motor (tiến/lùi) qua TB6612FNG.
   - Đọc encoder (đếm xung khi quay tay).
   - Quét servo qua PCA9685 (co/duỗi chân thử).

### Bước 3 — Lọc góc nghiêng (sensor fusion)
1. MPU6050 cho gia tốc + gyro. Gia tốc nhiễu, gyro bị trôi (drift).
2. Kết hợp bằng **Complementary filter** (đơn giản) hoặc **Kalman filter**:
   $$\theta = 0.98\,(\theta + \omega_{gyro}\,dt) + 0.02\,\theta_{accel}$$
3. Mục tiêu: ra góc nghiêng θ ổn định, ít trễ, ít nhiễu.

### Bước 4 — Vòng cân bằng PID (cốt lõi)
1. **Vòng góc (angle PID):** đầu vào = θ, mục tiêu = góc thẳng đứng (~0°),
   đầu ra = tốc độ/PWM cho 2 motor để kéo robot về thẳng đứng.
2. Tinh chỉnh PID theo thứ tự:
   - Tăng **Kp** đến khi robot bắt đầu giữ thẳng (hơi dao động).
   - Thêm **Kd** để giảm rung/dao động.
   - Thêm **Ki** nhỏ để khử sai số tĩnh (lệch nhẹ).
3. Thêm **giới hạn an toàn:** nếu |θ| > ~45° → tắt motor (đã ngã, tránh hỏng).
4. (Nâng cao) Thêm **vòng tốc độ/vị trí** (cascade PID) dùng encoder để robot
   đứng yên một chỗ, không trôi.

### Bước 5 — Điều khiển chân co duỗi
1. Định nghĩa các "tư thế" servo: **đứng cao / hạ thấp / cân bằng địa hình**.
2. Khi đổi chiều cao chân → **trọng tâm thay đổi** → cần PID đủ khoẻ để vẫn cân bằng.
3. Co duỗi **từ từ** (nội suy góc servo theo thời gian) để không gây sốc mất thăng bằng.
4. (Tuỳ chọn) Dùng θ để tự điều chỉnh chân giữ thân ngang trên dốc.

### Bước 6 — Di chuyển & điều khiển từ xa
1. Cộng thêm offset vào setpoint để robot **tiến/lùi**; chênh lệch PWM 2 bánh để **rẽ**.
2. Điều khiển qua **Bluetooth/WiFi** (app điện thoại hoặc web ESP32) — gửi lệnh
   tiến/lùi/rẽ và lệnh co/duỗi chân.

### Bước 7 — Tinh chỉnh & hoàn thiện
1. Chỉnh PID qua WiFi/Serial real-time (gửi Kp, Ki, Kd) cho nhanh.
2. Lọc nhiễu nguồn (tụ), bó dây gọn để không lệch trọng tâm.
3. Test cân bằng tĩnh → di chuyển → co duỗi chân → trên địa hình.

---

## 6. Thư viện gợi ý

| Mục đích | Thư viện |
|----------|----------|
| Đọc MPU6050 | `MPU6050` (Jeff Rowberg, có DMP) / `Adafruit_MPU6050` |
| Lọc Kalman | `Kalman` (TKJ Electronics) |
| PID | `PID_v1` (Brett Beauregard) hoặc tự viết |
| Servo PCA9685 | `Adafruit_PWMServoDriver` |
| Điều khiển bánh | tự viết với `ledcWrite` (PWM của ESP32) |
| BT/WiFi điều khiển | `BluetoothSerial` / `WebServer` / `WebSocket` |

---

## 7. Checklist nghiệm thu

- [ ] Đọc góc nghiêng ổn định từ MPU6050 (đã lọc).
- [ ] 2 motor + encoder hoạt động, điều khiển được 2 chiều.
- [ ] Robot **tự đứng cân bằng** tại chỗ (PID).
- [ ] Servo co/duỗi chân được, không làm ngã robot.
- [ ] Di chuyển tiến/lùi/rẽ ổn định.
- [ ] Điều khiển từ xa (BT/WiFi) hoạt động.
- [ ] Có ngắt an toàn khi nghiêng quá ngưỡng.

---

## 8. Ghi chú & mẹo

- **Khó nhất là tinh chỉnh PID** → dành nhiều thời gian; chỉnh từng tham số một.
- Đặt pin (nặng) lên cao giúp cân bằng dễ hơn (mô-men quán tính lớn, phản ứng chậm hơn → dễ kiểm soát).
- Bánh có **độ bám tốt** (bọc cao su) giúp cân bằng ổn định hơn.
- Khi đổi chiều dài chân, nên có **bảng PID/tham số riêng** cho từng độ cao nếu cần.
- Bắt đầu với **chân cố định** (cân bằng trước) → khi ổn mới thêm cơ cấu co duỗi.
