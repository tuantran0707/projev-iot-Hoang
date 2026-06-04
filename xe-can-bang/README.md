# Xe robot 2 banh - SM-ESP32 + L298N + ThingsBoard

Du an nay dieu khien xe 2 banh theo kieu differential drive.
Xe nhan lenh qua ThingsBoard bang Shared Attributes (khong dung RPC).
Do chua co encoder, toc do duoc dieu khien bang PWM va can theo thoi gian duration.

## 1. Tong quan nguyen ly chay

Xe co 2 dong co:
- Dong co phai
- Dong co trai

Moi dong co gom:
- 2 chan huong (IN1/IN2 hoac IN3/IN4): quyet dinh tien/lui
- 1 chan PWM (ENA/ENB): quyet dinh muc cong suat

Firmware nhan 2 truc dieu khien:
- x: tien/lui
- y: re trai/phai

Cong thuc tron 2 truc thanh toc do tung banh:
- banh trai = x + y
- banh phai = x - y

Sau do gia tri duoc gioi han trong [-100..100], quy doi ra PWM 8-bit [0..255].

## 2. So do chan dung theo firmware hien tai

Ket noi L298N voi ESP32:
- ENA -> GPIO32 (PWM banh phai)
- IN1 -> GPIO33
- IN2 -> GPIO25
- ENB -> GPIO14 (PWM banh trai)
- IN3 -> GPIO26
- IN4 -> GPIO27

Cam bien gas:
- AO -> GPIO36 (ADC)

Luu y:
- GPIO36 la input-only, chi dung de doc analog.
- Tat ca GND cua nguon, ESP32, L298N, cam bien phai noi chung.

## 3. Giao tiep ThingsBoard dang dung

Firmware ket noi MQTT den:
- host: thingsboard.cloud
- port: 1883
- auth: device token

Firmware subscribe 2 topic attributes:
- v1/devices/me/attributes
- v1/devices/me/attributes/response/+

Sau khi connect, firmware chu dong request shared attributes:
- publish den: v1/devices/me/attributes/request/1
- payload: {"sharedKeys":"x,y,duration,stop"}

Muc tieu la de xe lay lai trang thai dieu khien gan nhat sau khi mat mang/reboot.

## 4. JSON gui xuong la gi

Ban cap nhat Shared Attributes tren ThingsBoard voi cac key:
- x: so nguyen trong [-100..100]
- y: so nguyen trong [-100..100]
- duration: thoi gian ms, >0 thi tu dung sau thoi gian nay
- stop: true/false

Firmware chap nhan ca 2 dang payload:

Dang A, object truc tiep:
```json
{
  "x": 70,
  "y": 20,
  "duration": 1500
}
```

Dang B, co boc shared:
```json
{
  "shared": {
    "x": 70,
    "y": 20,
    "duration": 1500
  }
}
```

## 5. Xe chay nhu the nao khi nhan JSON

### Truong hop 1: co stop=true
Vi du:
```json
{
  "stop": true
}
```
Hanh vi:
1. Firmware uu tien xu ly stop truoc.
2. Goi stopMotors().
3. Ca 2 banh ve PWM = 0, IN ve muc dung.
4. Huy co che auto-stop dang cho (neu co).

### Truong hop 2: co x/y va co duration > 0
Vi du:
```json
{
  "x": 60,
  "y": -20,
  "duration": 2000
}
```
Hanh vi:
1. Tinh toc do banh trai = 60 + (-20) = 40.
2. Tinh toc do banh phai = 60 - (-20) = 80.
3. Quy doi PWM:
   - trai: 40% -> gan 102/255
   - phai: 80% -> gan 204/255
4. Dat huong IN theo dau cua moi toc do.
5. Dat hen gio auto-stop sau 2000 ms (khong block loop).
6. Het 2000 ms, xe tu dung.

### Truong hop 3: co x/y, khong co duration hoac duration <= 0
Vi du:
```json
{
  "x": 50,
  "y": 0
}
```
Hanh vi:
1. Xe chay lien tuc theo x/y hien tai.
2. Khong tu dung theo thoi gian.
3. Chi dung khi:
   - nhan stop=true, hoac
   - nhan lenh x/y moi, hoac
   - mat nguon/reset.

### Truong hop 4: chi gui mot trong hai key x hoac y
Vi du:
```json
{
  "x": 30
}
```
Hanh vi:
- Key con lai duoc giu gia tri truoc do.
- Vi du y truoc do la -10 thi lenh moi se thanh x=30, y=-10.

## 6. Bang hanh vi theo truc x/y

Quy uoc trong firmware:
- x duong: tien
- x am: lui
- y duong: re phai
- y am: re trai

Mot so vi du:
- {"x": 70, "y": 0} -> tien thang
- {"x": -70, "y": 0} -> lui thang
- {"x": 0, "y": 60} -> xoay/phai tai cho
- {"x": 0, "y": -60} -> xoay/trai tai cho
- {"x": 60, "y": 30} -> tien va re phai

## 7. Telemetry gui len ThingsBoard

Chu ky gui: moi 3 giay.

Cac key telemetry:
- gas_raw: gia tri ADC thô 0..4095 tu GPIO36
- gas_voltage: dien ap quy doi xap xi theo 3.3V
- motor_x: gia tri x dang ap dung
- motor_y: gia tri y dang ap dung
- rssi: cuong do song WiFi

## 8. PWM va tinh theo thoi gian (vi chua co encoder)

Do chua co encoder, he thong hien tai la open-loop:
- PWM cao hon thi banh quay nhanh hon (xu huong chung)
- quang duong thuc te con phu thuoc mat san, pin, tai trong, ma sat

Vi vay duration duoc dung de can thoi gian chay:
- Muon di ngan: giam duration
- Muon di xa: tang duration

Khuyen nghi calib nhanh:
1. Chon x co dinh, vi du x=50, y=0.
2. Thu duration 500/1000/1500 ms.
3. Do quang duong thuc te, lap bang map rieng cho xe.

## 9. Luu y tuong thich board ESP32

Firmware da dung ham boc LEDC de chay duoc ca:
- ESP32 core 2.x
- ESP32 core 3.x

Neu ban gap loi compile lien quan PWM, can gui:
- Ten board dang chon trong Arduino IDE
- Version ESP32 Boards package
- Log loi day du

## 10. Quy trinh test nhanh

1. Nap firmware len board.
2. Mo Serial Monitor 115200.
3. Cho board vao WiFi va MQTT connected.
4. Tren ThingsBoard, set shared attributes:
   - x=60, y=0, duration=1500
5. Xac nhan xe chay roi tu dung.
6. Set stop=true de thu dung khan.
7. Theo doi telemetry gas_raw va motor_x/motor_y.

## 11. Cau hinh can sua truoc khi nap

Trong firmware robot_car.ino, kiem tra:
- WIFI_SSID
- WIFI_PASSWORD
- TB_SERVER
- TB_TOKEN

Neu doi board hoac doi day, sua lai cac define PIN cho dung thuc te.
