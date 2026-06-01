# Đề tài 3 — Thiết bị đeo phát hiện ngã thông minh (Smart Fall Detection)

Thiết bị đeo trên người (cổ tay / thắt lưng) dùng cảm biến gia tốc – con quay
**MPU6050/MPU6500** để phát hiện cú ngã đột ngột. Hệ thống dùng **thuật toán
rule-based** (free-fall → impact → bất động) kết hợp **phân tích thống kê** để phân
biệt giữa *vận động mạnh khi thể thao* và *té ngã thật*, cảnh báo tại chỗ bằng
**LED RGB + màn hình TFT** và đẩy dữ liệu lên **IoT server (ThingsBoard Cloud) qua MQTT**.

---

## 1. Mô tả hoạt động

```
MPU6050/MPU6500 (gia tốc + con quay)
        │  đọc liên tục 100 Hz (I²C)
        ▼
   Phân loại VẬN ĐỘNG realtime  ──► Đứng yên (STILL) / Di chuyển (MOVING)
        │                              (đẩy lên IoT ngay khi đổi + định kỳ 5s)
        ▼
   Phát hiện sự kiện ngã (rule-based)
        │
        ├─ Không phải ngã (đi/chạy/thể thao) ──► bỏ qua
        │
        └─ Nghi ngờ NGÃ (free-fall → impact)
                │
                ├─ Chờ ổn định 1s rồi đo bất động 2s
                │
                ├─ Còn cử động liên tục (thể thao) ──► huỷ
                │
                └─ Nằm yên (bất động)
                        │
                        ▼
                LED nháy đỏ + màn báo "PHAT HIEN NGA" + đếm 60 giây
                        │
                        ├─ Bấm nút BOOT trong 60s ──► huỷ (báo nhầm / đã ổn)
                        │
                        └─ Hết 60s chưa tắt ──► GỬI CẢNH BÁO LÊN ThingsBoard (MQTT)
```

**Yêu cầu cốt lõi (đã triển khai):**
- Phát hiện ngã từ tín hiệu MPU6050/MPU6500 (rule-based + thống kê).
- Phân biệt vận động mạnh (thể thao) vs té ngã thật (chờ ổn định + kiểm tra bất động).
- Phân loại trạng thái vận động realtime (đứng yên / di chuyển).
- Cảnh báo bằng LED RGB + màn hình TFT, có nút bấm để tắt.
- Đẩy trạng thái + cảnh báo lên **IoT server ThingsBoard qua MQTT**.
- Hiển thị **đồng hồ thời gian thực** lấy qua **NTP**.

---

## 2. Phần cứng sử dụng

### Board chính: ESP32-2432S024 (CYD — "Cheap Yellow Display" 2.4")
Đây là board tích hợp sẵn, **không cần đi dây màn hình**:

| Thành phần tích hợp sẵn | Thông số | Ghi chú |
|-------------------------|----------|---------|
| MCU | ESP32-WROOM-32 (ESP32 classic, dual-core, WiFi+BT) | Đủ cho WiFi/MQTT |
| Màn hình | TFT 2.4" **ILI9341** 240×320, SPI | Hàn cứng sẵn trên PCB |
| LED RGB onboard | R=GPIO4, G=GPIO16, B=GPIO17 (chung anode, bật = LOW) | Báo trạng thái |
| Nút BOOT (SW1) | GPIO0 | Dùng để tắt cảnh báo |
| Cảm ứng | XPT2046 (bus SPI riêng) | Hiện chưa dùng |
| Khác | USB-C, khe thẻ TF, loa, kết nối pin | |

### Linh kiện gắn thêm

| Linh kiện | SL | Kết nối | Ghi chú |
|-----------|----|---------|---------|
| MPU6050 / MPU6500 (GY-521) | 1 | I²C: SDA=GPIO21, SCL=GPIO22 (connector mở rộng) | Cảm biến gia tốc 3 trục + con quay 3 trục |
| (Tuỳ chọn) Motor rung | 1 | Qua transistor (chân GPIO4 — chung với kênh đỏ LED) | Cảnh báo rung |
| (Tuỳ chọn) Pin Li-Po + TP4056 | 1 | Cấp nguồn qua chân pin của board | Cho thiết bị đeo |

> **Lưu ý nguồn:** ESP32 ăn dòng đỉnh khi bật WiFi (~200–400mA). Nên dùng pin ≥ 500mAh.

---

## 3. Sơ đồ kết nối (wiring)

```
MPU6050/MPU6500 ── I²C ──► ESP32-2432S024 (CYD)
   VCC → 3V3
   GND → GND
   SDA → GPIO21   (connector mở rộng)
   SCL → GPIO22   (connector mở rộng)

Màn hình TFT ILI9341  ── đã hàn cứng trên PCB (KHÔNG đi dây):
   MISO 12 / MOSI 13 / SCLK 14 / CS 15 / DC 2 / RST -1 / BL 27

LED RGB onboard (chung anode, BẬT = mức LOW):
   R → GPIO4 , G → GPIO16 , B → GPIO17

Nút BOOT (SW1):
   GPIO0 ── nút ── GND  (INPUT_PULLUP)

(Tuỳ chọn) Motor rung:
   GPIO4 ──[1kΩ]── Base (NPN) ; Motor (+)→VBAT ; Motor(−)→Collector ; Emitter→GND
   Diode 1N4007 flyback song song motor
```

> ⚠️ **Không dùng các chân của màn hình** (2, 12, 13, 14, 15, 27) cho mục đích khác.
> ⚠️ GPIO0 là chân strapping — **không giữ nút BOOT khi cấp nguồn / nạp code**.

---

## 4. Thuật toán phát hiện ngã (rule-based + thống kê)

### 4.1. Đo độ lớn gia tốc
$$a = \sqrt{a_x^2 + a_y^2 + a_z^2} \quad (\text{đơn vị } g)$$

Cảm biến đặt dải **±16g** (`ACCEL_CFG = 0x18`) để cú va chạm **không bị clip**.

### 4.2. Máy trạng thái (state machine)

```
ST_NORMAL ──(free-fall a<0.5g  →  impact a>2.6g)──► ST_MONITOR_IMMOBILE
                                                          │
                              chờ ổn định 1s, rồi đo 2s   │
                                                          ▼
              ┌────────── std nhỏ & mean≈1g (nằm yên) ──► ST_ALERT (ngã!)
              │
              └────────── còn cử động liên tục (std lớn) ──► ST_NORMAL (thể thao)
```

| Tham số | Giá trị | Ý nghĩa |
|---------|---------|---------|
| Free-fall | a < **0.5 g** | rơi tự do trước khi chạm đất |
| Impact | a > **2.6 g** | cú va chạm khi tiếp đất |
| Cửa sổ FF→Impact | < **600 ms** | free-fall liền ngay impact |
| **Chờ ổn định sau va chạm** | **1000 ms** | bỏ qua giai đoạn rung/trượt ngay sau va chạm |
| Cửa sổ đo bất động | **2000 ms** | đo sau khi đã ổn định |
| std(a) bất động | < **0.20 g** | cho phép cử động NHẸ sau khi ngã |
| mean(a) bất động | **0.70–1.30 g** | nằm yên quanh 1g |
| Timeout cảnh báo | **60 s** | tự gửi server nếu không tắt |
| Tần số lấy mẫu | **100 Hz** | |

> 💡 **Vì sao có bước "chờ ổn định 1s"?** Ngay sau va chạm, cơ thể còn rung/trượt làm
> std bị đội lên → dễ bỏ sót cú ngã thật. Chờ 1s cho ổn định rồi mới đo giúp phân biệt
> chính xác: **ngã = nằm yên sau va chạm**, **thể thao = cử động liên tục**.

### 4.3. Phân loại vận động realtime (đứng yên / di chuyển)
Chạy song song ở trạng thái bình thường, dùng **độ lệch so với đường nền trượt (baseline)**
thay vì so với hằng số 1.0g — nhờ đó **tự khử bias khi thiết bị đeo nghiêng**:

- `baseline` = EMA chậm của |a| (bám theo trọng lực dù nghiêng).
- mức vận động = EMA nhanh của |a − baseline|.
- Có **hysteresis** (bật 0.12g / tắt 0.06g) chống nhấp nháy.

→ Kết quả: **STILL** (đứng yên) hoặc **MOVING** (di chuyển / thể thao).

---

## 5. Báo hiệu & IoT

### 5.1. LED RGB theo trạng thái
| Trạng thái | Màu LED |
|------------|---------|
| Bình thường (đứng yên / vận động) | 🟢 Xanh lá |
| Va chạm (đang theo dõi bất động) | 🔵 Xanh dương |
| Xác nhận NGÃ | 🔴 Nháy đỏ liên tục |

### 5.2. Màn hình TFT (dashboard)
Bố cục chia 4 vùng:
- **Tiêu đề** (nền xanh navy): tên đề tài.
- **Đồng hồ**: giờ thực `HH:MM:SS` lấy qua **NTP** (GMT+7).
- **Panel trạng thái** (đổi màu): BINH THUONG / VAN DONG / VA CHAM / PHAT HIEN NGA.
- **Chân trang**: gia tốc hiện tại + đỉnh.

### 5.3. Gửi dữ liệu lên ThingsBoard (MQTT)
- Giao thức: **MQTT** (PubSubClient), host `thingsboard.cloud`, port `1883`.
- Xác thực: **username = device access token**, topic `v1/devices/me/telemetry`.
- **Đẩy ngay khi đổi trạng thái** + **heartbeat định kỳ 5s**.

Ví dụ telemetry:
```json
{"fall":false,"status":"MOVING","moving":true,"activity":0.28,"acc":1.24}
{"fall":false,"status":"IMPACT","acc":5.66}
{"fall":true,"status":"FALL","acc_mean":0.97,"acc_std":0.00}
```

---

## 6. Code & nạp firmware

➡️ [`firmware/fall_detection/fall_detection.ino`](firmware/fall_detection/fall_detection.ino)

**Thư viện cần cài (Library Manager):**
| Mục đích | Thư viện |
|----------|----------|
| Màn hình TFT | `TFT_eSPI` (Bodmer) — cấu hình trong `User_Setup.h` |
| MQTT | `PubSubClient` (Nick O'Leary) |
| WiFi / NTP / I²C | `WiFi.h`, `Wire.h` (có sẵn trong ESP32 core) |

**Cấu hình TFT_eSPI cho board CYD** (`User_Setup.h` trong thư mục thư viện):
- Driver: `ILI9341_2_DRIVER` (init thay thế, hợp panel CYD clone).
- Chân: MISO 12 / MOSI 13 / SCLK 14 / CS 15 / DC 2 / RST -1 / BL 27.
- `TFT_RGB_ORDER TFT_BGR`, `SPI_FREQUENCY 20000000` (panel này tối đa ~24MHz).

**Các bước:**
1. Arduino IDE → Board **ESP32 Dev Module** (board CYD dùng ESP32-WROOM-32 classic).
2. Cài 2 thư viện ở trên; cấu hình `User_Setup.h` như trên.
3. Trong sketch: điền `WIFI_SSID`, `WIFI_PASS`, `TB_TOKEN` (device token ThingsBoard).
4. Nạp → mở Serial Monitor **115200** để xem WHO_AM_I và log phát hiện ngã.
5. Đeo thử / ngã có kiểm soát lên đệm để kiểm tra.

> ⚠️ Sau khi sửa `User_Setup.h`, đóng/mở lại Arduino IDE để **build lại sạch** (TFT_eSPI cache cấu hình).

---

## 7. Vận hành & kiểm thử

| Thao tác | Kết quả mong đợi |
|----------|------------------|
| Bật nguồn | Màn hiện tiêu đề + đồng hồ; LED xanh lá; nối WiFi/MQTT |
| Đứng yên | Màn "BINH THUONG / Dang dung yen"; gửi `STILL` |
| Đi lại / lắc tay | Màn "VAN DONG"; gửi `MOVING` |
| Đập mạnh thiết bị | LED xanh dương; màn "VA CHAM"; gửi `IMPACT` |
| Ngã + nằm yên | LED nháy đỏ; màn "PHAT HIEN NGA"; gửi `FALL` |
| Bấm nút BOOT khi cảnh báo | Tắt ngay, LED về xanh lá; gửi `CANCELLED` |
| Không tắt sau 60s | Gửi cảnh báo lên ThingsBoard, tự về bình thường |

---

## 8. Checklist nghiệm thu

- [ ] Đọc được dữ liệu MPU6050/MPU6500 ổn định (WHO_AM_I = 0x68/0x70).
- [ ] Màn hình TFT hiển thị đầy đủ + đồng hồ NTP chạy.
- [ ] Phân loại đúng đứng yên (STILL) vs di chuyển (MOVING).
- [ ] Phát hiện cú ngã giả lập (ngã có kiểm soát lên đệm).
- [ ] Không báo nhầm khi đi/chạy/nhảy (nhờ bước chờ ổn định + kiểm tra bất động).
- [ ] LED RGB đúng màu theo trạng thái.
- [ ] Nút BOOT tắt được cảnh báo; timeout 60s tự gửi server.
- [ ] Dữ liệu telemetry hiện trên ThingsBoard Cloud.

---

## 9. Ghi chú

- Hệ thống dùng **rule-based + thống kê** (không dùng ML/Edge Impulse) — đủ chính xác,
  nhẹ, chạy realtime trên ESP32 classic.
- Cảnh báo đẩy lên **ThingsBoard qua MQTT** (thay cho Telegram ở bản trước) — ổn định,
  không vướng lỗi TLS của HTTPS.
- Đặt cảm biến ở **thắt lưng/hông** cho độ chính xác phát hiện ngã tốt hơn cổ tay
  (nếu cho phép), nhưng cổ tay tiện đeo hơn — tuỳ chọn theo nhu cầu.

