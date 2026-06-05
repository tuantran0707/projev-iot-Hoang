# Xe Di Theo Map (toa do X,Y) - Cau hinh THUC TE khong encoder

> Ban nay da duoc chinh theo dung linh kien ban dang co:
> - 2 motor giam toc 12V khong encoder
> - 3 cell 18650 Panasonic 3400mAh
> - co san 2 mach L293 mini (B17)
>
> Muc tieu: van chay Auto/Manual qua WiFi + ThingsBoard, nhung Auto la dead-reckoning
> (gyro + PWM/time), sai so se tich luy theo quang duong.

---

## 1. Ket luan nhanh cho linh kien hien tai

1. 2 motor 12V khong encoder: dung tot de chay xe (tien/lui/re), nhung khong dem xung duoc.
2. 3 pin 18650 Panasonic 3400mAh: dung duoc de lap bo 3S (11.1V danh dinh, 12.6V day).
3. 2 mach L293 mini (B17): chi nen dung test nhe, khong khuyen nghi cho tai that cua xe tank.

Khuyen nghi thuc dung:
- Driver: L298N (de lam, re) hoac BTS7960 (khoe hon).
- Cam bien goc: MPU6050 de giu huong.
- Firmware su dung: [firmware/map_navigation_noenc/map_navigation_noenc.ino](firmware/map_navigation_noenc/map_navigation_noenc.ino)

---

## 2. Kien truc he thong (khong encoder)

```
Raspberry Pi (GUI map + ThingsBoard)
             |
             | WiFi / MQTT
             v
ESP32 --------> Driver (L298N) --------> 2 motor 12V
  |                  ^
  | I2C              |
  +--> MPU6050 ------+

Nguon:
Pin 3S (3x18650) ---> Driver 12V
                  ---> Buck 5V ---> ESP32 + MPU6050
```

- Manual: dieu khien truc tiep PWM trai/phai.
- Auto: ESP32 quay dung huong theo gyro roi tien theo quang duong uoc luong.

---

## 3. BOM cap nhat theo huong khong encoder

| # | Linh kien | Trang thai | Ghi chu |
|---|-----------|------------|--------|
| 1 | Khung tank TP101 + 2 motor 12V | Da co | Khong encoder |
| 2 | ESP32 DevKit (WROOM-32) | Can co | Bo dieu khien |
| 3 | MPU6050 (GY-521) | Can mua/da co | Bat buoc de giu huong |
| 4 | Driver dong co L298N | Nen mua | Khuyen nghi cho ban hien tai |
| 5 | 3x18650 Panasonic 3400mAh | Da co | Lap 3S |
| 6 | De pin 3x18650 noi tiep (3S) | Can mua | Chon dung loai cell |
| 7 | BMS 3S (10A tro len) | Rat nen mua | Bao ve qua xa/sac/cham |
| 8 | Sac 3S (12.6V) | Can mua | Sac dung chuan 3S |
| 9 | Buck 12V->5V (LM2596, 3A) | Can mua | Cap ESP32 |
| 10 | Cong tac + cau chi 5A | Can mua | An toan nguon |
| 11 | Tu 470-1000uF/25V | Nen co | Giam sut ap luc de motor |
| 12 | Day 18AWG/22AWG, ong co nhiet | Can co | Han tay |

### L293 mini (B17) co dung duoc khong?

- Co the dung de test logic/chay khong tai.
- Khong khuyen nghi de chay that voi xe tank 12V vi:
  - dong moi kenh thap,
  - sut ap lon,
  - de nong/chay khi de tai.

Ket luan: Neu lam do an nghiem tuc, nen dung L298N toi thieu.

---

## 4. So do chan ESP32 (L298N + MPU6050)

| Chuc nang | GPIO ESP32 | Ghi chu |
|----------|------------|--------|
| Motor trai PWM (ENA) | GPIO25 | PWM |
| Motor trai IN1 | GPIO26 | Chieu |
| Motor trai IN2 | GPIO27 | Chieu |
| Motor phai PWM (ENB) | GPIO14 | PWM |
| Motor phai IN3 | GPIO18 | Chieu |
| Motor phai IN4 | GPIO19 | Chieu |
| MPU6050 SDA | GPIO21 | I2C |
| MPU6050 SCL | GPIO22 | I2C |
| ESP32 VIN | 5V buck out | Khong cap 12V truc tiep |
| ESP32 GND | GND chung | Bat buoc |

---

## 5. Dau noi phan cung chi tiet

### 5.1 L298N voi ESP32

```
ESP32 GPIO25 -> ENA
ESP32 GPIO26 -> IN1
ESP32 GPIO27 -> IN2
ESP32 GPIO14 -> ENB
ESP32 GPIO18 -> IN3
ESP32 GPIO19 -> IN4

L298N OUT1/OUT2 -> Motor trai
L298N OUT3/OUT4 -> Motor phai
```

### 5.2 Nguon

```
Pack 3S 18650 (+) -> Cong tac -> Cau chi 5A -> L298N +12V
Pack 3S 18650 (-) ----------------------------> L298N GND

Pack 3S (+/-) -> Buck IN
Buck OUT 5V -> ESP32 VIN
Buck OUT GND -> ESP32 GND

MPU6050 VCC -> 3.3V (hoac 5V tuy module)
MPU6050 GND -> GND
MPU6050 SDA -> GPIO21
MPU6050 SCL -> GPIO22
```

Quan trong:
- Tat ca GND phai noi chung.
- Chinh buck dung 5.0V truoc khi cam ESP32.
- Khong lay 5V cap ESP32 tu L298N.

---

## 6. Firmware ban nen dung

- Ban co encoder (giu lai de nang cap sau): [firmware/map_navigation/map_navigation.ino](firmware/map_navigation/map_navigation.ino)
- Ban KHONG encoder (dung hien tai): [firmware/map_navigation_noenc/map_navigation_noenc.ino](firmware/map_navigation_noenc/map_navigation_noenc.ino)

Firmware khong encoder da ho tro:
- Manual: forward/back/left/right/stop
- Auto: setTarget (x,y)
- Telemetry ThingsBoard: x, y, theta, mode
- RPC bo sung:
  - setCalib: chinh he so toc do theo PWM
  - setPose: reset lai vi tri x,y,theta

---

## 7. Quy trinh hieu chuan bat buoc (khong encoder)

### Buoc A - hieu chuan toc do thang

1. Dat xe tren mat phang, cho chay manual forward voi speed co dinh (vi du 160).
2. Do quang duong di duoc trong 5 giay (vi du 1.6m).
3. Tinh van toc: v = 1.6 / 5 = 0.32 m/s.
4. Tinh he so trong code:
   K_SPEED_MPS = v / speed = 0.32 / 160 = 0.0020
5. Cap nhat K_SPEED_MPS trong firmware no-encoder.

### Buoc B - hieu chuan quay

1. Gui lenh left (hoac right) trong 1 khoang thoi gian co dinh.
2. Quan sat theta telemetry, chinh Kp_h/Kd_h de quay gan dung goc muc tieu.

### Buoc C - test map ngan

- Thu target nho: (0.5, 0), (0.5, 0.5), (0, 0.5).
- Neu lech nhieu, uu tien chinh K_SPEED_MPS va giam toc do chay.

---

## 8. Gioi han va cach giam sai so

Khong encoder thi sai so la binh thuong, do:
- truot xich,
- pin giam ap theo thoi gian,
- ma sat san thay doi.

Cach giam sai so:
1. Giu map ngan (moi chang 0.5-1.0m).
2. Chen diem reset pose tren map (moc chuan).
3. Chay toc do vua phai, tranh de/pheu gap.
4. Dung pin day va dong deu nhung lan test.

---

## 9. Checklist truoc khi cap nguon

- [ ] Buck da chinh dung 5.0V.
- [ ] GND chung giua pin, L298N, buck, ESP32, MPU6050.
- [ ] Day motor dung cuc, khong cham chap.
- [ ] Co cong tac va cau chi tren duong duong pin.
- [ ] Da nap dung file firmware no-encoder.
- [ ] Da doi TB_TOKEN, WIFI_SSID, WIFI_PASS trong code.

---

## 10. Huong nang cap sau (neu can chinh xac cao)

Neu can do chinh xac x,y cao hon, nang cap len motor co encoder:
- giu nguyen bo nguon va L298N,
- doi firmware sang ban encoder tai [firmware/map_navigation/map_navigation.ino](firmware/map_navigation/map_navigation.ino).
