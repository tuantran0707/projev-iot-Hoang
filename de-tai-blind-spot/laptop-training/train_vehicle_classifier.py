"""
============================================================================
 Train trên LAPTOP -> xuất model cho ESP32-CAM (TensorFlow Lite Micro)
 Bài toán: phân loại ảnh "có vật thể" / "không có vật thể" (nhẹ, chạy trên ESP32)
 Hướng (trái/phải) xử lý ở firmware bằng cách cắt khung làm 2 nửa.
============================================================================

CÀI ĐẶT (1 lần):
    pip install tensorflow pillow numpy

CHUẨN BỊ DỮ LIỆU:
    laptop-training/
        object/      <- ảnh CÓ vật thể (mô hình/đồ vật phía sau), đa dạng góc/sáng
        empty/       <- ảnh nền trống, KHÔNG có vật thể
    (mỗi lớp nên >= 150-300 ảnh, chụp đúng góc camera thật)

CHẠY:
    python train_vehicle_classifier.py

KẾT QUẢ:
    object_model_int8.tflite   <- model đã lượng tử hoá int8
    -> Bước cuối: convert sang mảng C để nhúng Arduino (xem hướng dẫn cuối file)
============================================================================
"""

import os
import sys
import numpy as np
import tensorflow as tf

# Ép stdout sang UTF-8 để in được tiếng Việt trên console Windows (cp1252).
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

# ----------------------------- CẤU HÌNH -----------------------------------
IMG_SIZE   = 96          # 96x96 — khớp với firmware ESP32-CAM
CHANNELS   = 1           # grayscale cho nhẹ
BATCH      = 16
EPOCHS     = 30
DATA_DIR   = "dataset"   # sẽ tự động resolve ở bên dưới
CLASS_NAMES = ["empty", "object"]   # 0 = empty, 1 = object

# Ưu tiên dùng dữ liệu ngay trong thư mục hiện tại (empty/object).
# Nếu có thư mục dataset/ thì vẫn hỗ trợ để tương thích cách tổ chức cũ.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if os.path.isdir(os.path.join(SCRIPT_DIR, "dataset")):
    DATA_DIR = os.path.join(SCRIPT_DIR, "dataset")
else:
    DATA_DIR = SCRIPT_DIR

for cls in CLASS_NAMES:
    cls_dir = os.path.join(DATA_DIR, cls)
    if not os.path.isdir(cls_dir):
        raise FileNotFoundError(f"Khong tim thay thu muc label: {cls_dir}")

# ----------------------- TẢI & TIỀN XỬ LÝ DỮ LIỆU --------------------------
train_ds = tf.keras.utils.image_dataset_from_directory(
    DATA_DIR, validation_split=0.2, subset="training", seed=123,
    image_size=(IMG_SIZE, IMG_SIZE), color_mode="grayscale",
    batch_size=BATCH, class_names=CLASS_NAMES)

val_ds = tf.keras.utils.image_dataset_from_directory(
    DATA_DIR, validation_split=0.2, subset="validation", seed=123,
    image_size=(IMG_SIZE, IMG_SIZE), color_mode="grayscale",
    batch_size=BATCH, class_names=CLASS_NAMES)

# Tăng cường dữ liệu (augmentation) để model tổng quát hơn
augment = tf.keras.Sequential([
    tf.keras.layers.RandomFlip("horizontal"),
    tf.keras.layers.RandomBrightness(0.2),
    tf.keras.layers.RandomContrast(0.2),
    tf.keras.layers.RandomZoom(0.1),
])

normalize = tf.keras.layers.Rescaling(1.0 / 255)

train_ds = train_ds.map(lambda x, y: (normalize(augment(x)), y))
val_ds   = val_ds.map(lambda x, y: (normalize(x), y))
train_ds = train_ds.prefetch(tf.data.AUTOTUNE)
val_ds   = val_ds.prefetch(tf.data.AUTOTUNE)

# ----------------------------- MODEL NHỎ -----------------------------------
# CNN gọn để vừa bộ nhớ ESP32 (~vài trăm KB sau khi int8)
model = tf.keras.Sequential([
    tf.keras.layers.Input((IMG_SIZE, IMG_SIZE, CHANNELS)),
    tf.keras.layers.Conv2D(8, 3, activation="relu"),
    tf.keras.layers.MaxPooling2D(),
    tf.keras.layers.Conv2D(16, 3, activation="relu"),
    tf.keras.layers.MaxPooling2D(),
    tf.keras.layers.Conv2D(32, 3, activation="relu"),
    tf.keras.layers.MaxPooling2D(),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(32, activation="relu"),
    tf.keras.layers.Dropout(0.3),
    tf.keras.layers.Dense(2, activation="softmax"),   # 2 lớp
])

model.compile(optimizer="adam",
              loss="sparse_categorical_crossentropy",
              metrics=["accuracy"])
model.summary()

model.fit(train_ds, validation_data=val_ds, epochs=EPOCHS)

# ----------------- CHUYỂN SANG TFLITE INT8 (lượng tử hoá) -------------------
def representative_data_gen():
    # Lấy vài batch làm mẫu để hiệu chỉnh lượng tử hoá
    for images, _ in train_ds.take(20):
        for i in range(images.shape[0]):
            yield [np.expand_dims(images[i].numpy(), axis=0).astype(np.float32)]

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_data_gen
# Ép input/output về int8 để chạy trên TFLM
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8
converter.inference_output_type = tf.int8

tflite_model = converter.convert()

out_path = "object_model_int8.tflite"
with open(out_path, "wb") as f:
    f.write(tflite_model)

print(f"\nĐÃ LƯU: {out_path}  ({len(tflite_model)/1024:.1f} KB)")
print("""
============================================================================
 BƯỚC CUỐI — convert .tflite thành mảng C để nhúng Arduino:

   Linux/Mac:
     xxd -i object_model_int8.tflite > object_model.h

   Windows (PowerShell, nếu không có xxd) — dùng Python:
     python -c "data=open('object_model_int8.tflite','rb').read(); \\
       open('object_model.h','w').write('const unsigned char object_model[]={'+\\
       ','.join(str(b) for b in data)+'};\\nconst unsigned int object_model_len='+\\
       str(len(data))+';\\n')"

 Sau đó:
   - Đổi tên mảng trong object_model.h thành: object_model[]  và  object_model_len
   - Copy object_model.h vào cùng thư mục sketch Arduino (blind_spot_tflm.ino)
============================================================================
""")
