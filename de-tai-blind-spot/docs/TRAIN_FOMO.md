# Hướng dẫn train model FOMO nhận diện xe (Edge Impulse) cho ESP32-CAM

Tài liệu này hướng dẫn **từng bước** tạo mô hình **FOMO (Object Detection)** nhận diện
phương tiện, rồi xuất ra thư viện Arduino để nhúng vào
[`../firmware/blind_spot/blind_spot.ino`](../firmware/blind_spot/blind_spot.ino).

> FOMO = *Faster Objects, More Objects* — mô hình phát hiện vật thể **siêu nhẹ** cho vi
> điều khiển. Nó trả về **tâm vật thể (centroid)**, đủ để biết "có xe / ở hướng nào".

---

## 0. Chuẩn bị

- Tài khoản miễn phí tại **https://studio.edgeimpulse.com**
- ESP32-CAM + USB-TTL đã nạp được code (ví dụ CameraWebServer chạy ổn).
- Khoảng **200–400 ảnh** vùng điểm mù (càng đa dạng càng tốt).

---

## 1. Thu thập ảnh (dataset)

Mục tiêu: ảnh giống nhất với điều kiện thật khi gắn lên xe.

**Nên thu:**
- Đúng **góc đặt camera thật** (gắn sau xe, hướng vùng điểm mù).
- Nhiều điều kiện: **sáng/tối, nắng/mưa, sáng sớm/chiều tối**.
- Nhiều loại xe: **xe máy, ô tô, xe tải, xe buýt**.
- Nhiều khoảng cách & vị trí (xe lệch trái, lệch phải, ở giữa, xa/gần).
- **Ảnh nền (negative):** đường trống, không có xe — rất quan trọng để giảm báo nhầm.

**Cách thu ảnh — chọn 1:**
| Cách | Mô tả |
|------|-------|
| **A. Thu thẳng bằng ESP32-CAM** | Edge Impulse → *Devices* → kết nối ESP32-CAM (firmware Edge Impulse) → *Data acquisition* chụp trực tiếp. Sát thực tế nhất. |
| **B. Quay video rồi tách frame** | Gắn camera/điện thoại ở vị trí xe, quay video, tách thành ảnh (vd dùng `ffmpeg -i video.mp4 -vf fps=2 img_%04d.jpg`). |
| **C. Dùng ảnh có sẵn / dataset xe** | Tải ảnh xe công khai để bổ sung, nhưng **ưu tiên ảnh tự chụp** đúng góc camera. |

> Gợi ý số lượng: ≥ **150–200 ảnh có xe** + **50–100 ảnh nền trống**, cân bằng các hướng.

---

## 2. Tạo project & upload dữ liệu

1. Đăng nhập Edge Impulse → **Create new project** → đặt tên (vd `blind-spot`).
2. Vào **Data acquisition** → **Upload data** → chọn ảnh đã thu.
3. Để Edge Impulse tự chia **Train / Test** (vd 80/20).

---

## 3. Gán nhãn (labeling)

FOMO chỉ cần đánh dấu **vùng/tâm vật thể**, nhẹ hơn bounding box truyền thống.

1. Vào **Data acquisition** → tab **Labeling queue**.
2. Với mỗi ảnh: kéo 1 ô bao quanh **mỗi chiếc xe**, đặt nhãn:
   - Đơn giản nhất: dùng **1 nhãn chung** `vehicle` cho mọi loại xe.
   - Hoặc tách: `xe_may`, `o_to`, `xe_tai`, `xe_buyt` (nhiều nhãn → cần nhiều dữ liệu hơn).
3. Ảnh nền trống thì **không vẽ ô nào** (để trống — là mẫu "không có xe").
4. Lưu lần lượt đến hết.

> **Khuyến nghị:** bắt đầu với **1 nhãn `vehicle`** cho dễ train & đủ cho mục tiêu cảnh báo.

---

## 4. Thiết kế Impulse

1. Vào **Impulse design** → **Create impulse**.
2. **Image data:** đặt kích thước ảnh **96 x 96** (nhẹ, hợp ESP32-CAM).
   - Có thể thử **160x160** nếu cần chính xác hơn (nhưng nặng/chậm hơn).
3. Thêm **Processing block: Image** → màu **Grayscale** (nhẹ hơn RGB).
4. Thêm **Learning block: Object Detection (FOMO)**.
5. **Save Impulse**.

---

## 5. Trích đặc trưng & Train

1. Vào **Image** (processing block) → **Generate features** → kiểm tra biểu đồ phân bố.
2. Vào **Object detection** → chọn model **FOMO (MobileNetV2 0.35)** (mặc định).
3. Đặt **Training cycles** (vd 60–100 epoch), **Learning rate** mặc định.
4. Nhấn **Start training**.
5. Xem kết quả **F1 score**:
   - F1 thấp → thêm ảnh, gán nhãn kỹ hơn, bổ sung ảnh nền.
   - Chú ý các lỗi do **bóng đổ, vạch kẻ đường** → thêm ảnh các tình huống đó.

---

## 6. Kiểm thử (Model testing)

1. Vào **Model testing** → **Classify all** (dùng tập Test chưa train).
2. Xem độ chính xác & ảnh bị nhận sai.
3. Lặp lại bước 1–5 (thêm dữ liệu / train lại) đến khi đạt yêu cầu.

---

## 7. Xuất thư viện Arduino & nhúng vào firmware

1. Vào **Deployment** → ô tìm kiếm chọn **Arduino library**.
2. (Tuỳ chọn) Bật **EON Compiler** + **Quantized (int8)** để model nhẹ & nhanh hơn.
3. Nhấn **Build** → tải về file `.zip` (tên kiểu `ten-project_inferencing.zip`).
4. Arduino IDE: **Sketch → Include Library → Add .ZIP Library…** → chọn file vừa tải.
5. Mở [`../firmware/blind_spot/blind_spot.ino`](../firmware/blind_spot/blind_spot.ino),
   sửa dòng include cho khớp tên thư viện, ví dụ:
   ```cpp
   #include <blind_spot_inferencing.h>   // đổi thành tên project của bạn
   ```
6. **Tools:** Board = **AI Thinker ESP32-CAM**, **PSRAM = Enabled**, Partition Scheme
   = **Huge APP** (nếu thiếu bộ nhớ chương trình).
7. Nạp code (GPIO0→GND khi nạp), reset, mở Serial 115200 để xem nhãn + hướng.

---

## 8. Tinh chỉnh khi chạy thật

| Vấn đề | Cách xử lý |
|--------|-----------|
| Hay báo nhầm (không có xe vẫn báo) | Tăng `CONF_THRESHOLD`, tăng `DEBOUNCE_FRAMES`, thêm ảnh nền khi train |
| Bỏ sót xe (có xe không báo) | Giảm `CONF_THRESHOLD`, thêm ảnh xe ở khoảng cách/điều kiện đó |
| Sai hướng trái/phải | Chỉnh ngưỡng `ratio` (0.45/0.55) trong code cho khớp góc camera |
| Chậm/giật | Dùng model 96x96 grayscale, bật quantized int8, giảm `fb_count` |
| Reset ngẫu nhiên | Kiểm tra **PSRAM enabled** + nguồn 5V đủ khoẻ (buck ≥2A + tụ lọc) |

> **Mẹo:** sau khi gắn lên xe, thu thêm vài chục ảnh **tại chính vị trí lắp** rồi train lại
> (fine-tune) — đây là cách tăng độ chính xác nhanh nhất vì khớp đúng góc nhìn thật.

---

## 9. Tóm tắt quy trình

```
Thu ảnh (đúng góc camera)
      ▼
Upload + Gán nhãn "vehicle" (FOMO)
      ▼
Impulse: Image 96x96 grayscale -> Object Detection (FOMO)
      ▼
Generate features -> Train -> xem F1
      ▼
Model testing -> bổ sung dữ liệu nếu cần
      ▼
Deployment -> Arduino library (.zip)
      ▼
Add .ZIP vào IDE -> sửa #include -> bật PSRAM -> nạp ESP32-CAM
      ▼
Chạy thật -> tinh chỉnh ngưỡng -> (fine-tune tại vị trí lắp)
```
