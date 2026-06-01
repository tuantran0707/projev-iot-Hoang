# Tổng quan 2 dự án IoT

Repo này chứa tài liệu (linh kiện cần mua + flow thực hiện) cho **2 đề tài**:

| # | Đề tài | Thư mục | README |
|---|--------|---------|--------|
| 1 | Thiết bị đeo phát hiện ngã thông minh (Fall Detection) | [`de-tai-3-fall-detection/`](de-tai-3-fall-detection/) | [README](de-tai-3-fall-detection/README.md) |
| 2 | Cảnh báo điểm mù gương chiếu hậu xe máy (Blind Spot Detection) | [`de-tai-blind-spot/`](de-tai-blind-spot/) | [README](de-tai-blind-spot/README.md) |
| 3 | Xe / Robot tự cân bằng 2 bánh có chân co duỗi (Self-Balancing) | [`xe-can-bang/`](xe-can-bang/) | [README](xe-can-bang/README.md) |

---

## 1. Thiết bị đeo phát hiện ngã thông minh

- **Mục tiêu:** Đeo trên người, dùng MPU6050 phát hiện cú ngã đột ngột.
  Nếu người dùng nằm yên không đứng dậy → rung cảnh báo 10s → nếu không tắt thì gửi
  thông báo về Telegram.
- **Điểm nhấn:** Có **học máy (ML)** để phân biệt *vận động mạnh khi thể thao* và
  *té ngã thật*.
- **Phần cứng có sẵn:** ESP32-S3 mini, LCD mini, MPU6050.
- **Cần bổ sung:** motor rung, pin Li-Po, mạch sạc, nút bấm…

➡️ Chi tiết: [de-tai-3-fall-detection/README.md](de-tai-3-fall-detection/README.md)

---

## 2. Cảnh báo điểm mù gương chiếu hậu (xe máy)

- **Mục tiêu:** Camera + AI phát hiện phương tiện trong vùng điểm mù phía sau,
  cảnh báo người lái bằng **buzzer** và **motor rung** theo hướng nguy hiểm.
- **Phần cứng chính:** ESP32-CAM (OV2640), buzzer, 2 motor rung coin, mạch hạ áp DC-DC.
- **AI:** Mô hình nhận diện phương tiện (lưu ý về giới hạn của ESP32-CAM ở README chi tiết).

➡️ Chi tiết: [de-tai-blind-spot/README.md](de-tai-blind-spot/README.md)

---

## 3. Xe / Robot tự cân bằng 2 bánh có chân co duỗi

- **Mục tiêu:** Robot tự cân bằng trên 2 bánh (con lắc ngược) bằng IMU + PID,
  gắn trên **2 chân co duỗi được** để đổi chiều cao / thích nghi địa hình.
- **Phần cứng chính:** ESP32, MPU6050, DC motor có encoder + TB6612FNG, servo MG996R + PCA9685,
  pin LiPo 3S + buck DC-DC.
- **Điểm nhấn:** Thuật toán **PID cân bằng** + điều khiển khớp chân co duỗi.

➡️ Chi tiết: [xe-can-bang/README.md](xe-can-bang/README.md)

---

## Lưu ý chung

- Tài liệu này tập trung vào **danh sách linh kiện** và **flow triển khai**.
- Bảng giá chỉ mang tính tham khảo thị trường Việt Nam (có thể đổi theo thời điểm mua).
- Code/firmware sẽ được bổ sung trong từng thư mục con khi triển khai.
