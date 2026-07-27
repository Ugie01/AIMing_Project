import os
import glob
import random
import numpy as np
import cv2
import tensorflow as tf
from tensorflow.keras import layers, models
from sklearn.model_selection import train_test_split
from PIL import Image, ImageDraw

# --- 1단계: 시뮬레이션 맞춤형 데이터셋 생성 및 폴더 분할 저장 (지평선 배경 + 부정 샘플 30%) ---
def create_or_verify_dataset(dataset_dir="./synthetic_green_dataset", num_samples=2500, img_size=95):
    test_img_dir = os.path.join(dataset_dir, "test", "images")
    if os.path.exists(test_img_dir) and len(os.listdir(test_img_dir)) > 0:
        print("📦 [1단계] 기존에 분할된 데이터셋이 존재합니다. 로드를 진행합니다.")
        return

    print(f"📦 [1단계] 총 {num_samples}장의 시뮬레이션 맞춤형 데이터(지평선 배경/타겟 유무)를 생성하고 분할 저장합니다...")
    images_data = []
    
    for i in range(num_samples):
        # PyBullet 시뮬레이션 배경 모사 (상단 하늘, 하단 바닥 구조 + 지평선)
        bg_array = np.zeros((img_size, img_size, 3), dtype=np.uint8)
        
        sky_color = [random.randint(220, 255), random.randint(220, 255), random.randint(230, 255)]
        ground_color = [random.randint(160, 200), random.randint(140, 180), random.randint(120, 160)]
        
        horizon_y = random.randint(int(img_size * 0.3), int(img_size * 0.7))
        bg_array[:horizon_y, :] = sky_color
        bg_array[horizon_y:, :] = ground_color
        
        # 가우스 노이즈 추가로 질감 다양성 부여
        noise = np.random.normal(loc=0, scale=15, size=(img_size, img_size, 3))
        bg_array = np.clip(bg_array.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        
        img = Image.fromarray(bg_array, 'RGB')
        draw = ImageDraw.Draw(img)
        
        # 30% 확률로 초록색 타겟이 없는 순수 배경(Negative) 샘플 생성
        has_target = 0.0 if random.random() < 0.3 else 1.0
        
        if has_target > 0.0:
            obj_w = random.randint(10, 40)
            obj_h = random.randint(10, 40)
            
            x1 = random.randint(0, max(0, img_size - obj_w))
            y1 = random.randint(0, max(0, img_size - obj_h))
            x2 = x1 + obj_w
            y2 = y1 + obj_h
            
            green_color = (
                random.randint(0, 50),     # R
                random.randint(150, 255),  # G (초록색 강조)
                random.randint(0, 50)      # B
            )
            
            if random.random() > 0.5:
                draw.rectangle([x1, y1, x2, y2], fill=green_color)
            else:
                draw.ellipse([x1, y1, x2, y2], fill=green_color)
                
            x_center = ((x1 + x2) / 2.0) / img_size
            y_center = ((y1 + y2) / 2.0) / img_size
        else:
            # 타겟이 없을 때는 좌표를 임의의 값(0.5)으로 지정하고 confidence는 0
            x_center, y_center = 0.5, 0.5
        
        # 라벨 포맷: [confidence, x_center, y_center]
        label_str = f"{has_target} {x_center:.6f} {y_center:.6f}\n"
        images_data.append((np.array(img), label_str))

    train_data, temp_data = train_test_split(images_data, test_size=0.2, random_state=42)
    val_data, test_data = train_test_split(temp_data, test_size=0.5, random_state=42)
    
    splits = {"train": train_data, "val": val_data, "test": test_data}
    
    for split_name, data in splits.items():
        img_dir = os.path.join(dataset_dir, split_name, "images")
        lbl_dir = os.path.join(dataset_dir, split_name, "labels")
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(lbl_dir, exist_ok=True)
        
        for idx, (img_np, label_str) in enumerate(data):
            file_name = f"sample_{idx:05d}"
            cv2.imwrite(os.path.join(img_dir, f"{file_name}.png"), cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR))
            with open(os.path.join(lbl_dir, f"{file_name}.txt"), 'w') as f:
                f.write(label_str)
                
    print(f"✅ 데이터 분할 저장 완료! (Train: {len(train_data)}, Val: {len(val_data)}, Test: {len(test_data)})\n")


# 데이터 로드 함수 ([confidence, x, y] 반환)
def load_data_from_folder(img_dir, lbl_dir, img_size=95):
    image_paths = sorted(glob.glob(os.path.join(img_dir, "*.png")))
    label_paths = sorted(glob.glob(os.path.join(lbl_dir, "*.txt")))
    
    X, y = [], []
    for img_p, lbl_p in zip(image_paths, label_paths):
        img = cv2.imread(img_p)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        X.append(img)
        
        with open(lbl_p, 'r') as f:
            parts = f.readline().strip().split()
            conf = float(parts[0])
            x_center = float(parts[1])
            y_center = float(parts[2])
        y.append([conf, x_center, y_center])
        
    return np.array(X, dtype=np.float32) / 255.0, np.array(y, dtype=np.float32)


# --- 2단계: 커스텀 손실 함수 정의 (Confidence + 좌표 회귀 복합 손실) ---
def custom_loss(y_true, y_pred):
    t_conf = y_true[:, 0:1]
    t_coords = y_true[:, 1:3]
    
    p_conf = y_pred[:, 0:1]
    p_coords = y_pred[:, 1:3]
    
    # 1. Confidence 손실 (Binary Crossentropy)
    loss_conf = tf.keras.losses.binary_crossentropy(t_conf, p_conf)
    
    # 2. 좌표 손실 (MSE) - 타겟이 실제로 존재하는 샘플(t_conf == 1)에 대해서만 마스킹 처리
    mse_coords = tf.reduce_mean(tf.square(t_coords - p_coords), axis=1, keepdims=True)
    loss_coords = t_conf * mse_coords 
    
    total_loss = loss_conf + (10.0 * loss_coords)
    return tf.reduce_mean(total_loss)


def main():
    dataset_dir = "./synthetic_green_dataset"
    img_size = 95
    
    create_or_verify_dataset(dataset_dir, num_samples=2500, img_size=img_size)
    
    print("🔄 [2단계] Train 및 Val 데이터를 로드 중입니다...")
    X_train, y_train = load_data_from_folder(os.path.join(dataset_dir, "train", "images"), os.path.join(dataset_dir, "train", "labels"), img_size)
    X_val, y_val = load_data_from_folder(os.path.join(dataset_dir, "val", "images"), os.path.join(dataset_dir, "val", "labels"), img_size)

    # --- 3단계: CNN 모델 구조 정의 (출력 3개: [conf, x, y]) ---
    print("🧠 [3단계] 신뢰도 출력이 포함된 CNN 모델 학습 시작 (Epochs: 15)...")
    inputs = layers.Input(shape=(img_size, img_size, 3))
    x = layers.Conv2D(16, (3, 3), activation="relu", padding="same")(inputs)
    x = layers.MaxPooling2D((2, 2))(x)
    
    x = layers.Conv2D(32, (3, 3), activation="relu", padding="same")(x)
    x = layers.MaxPooling2D((2, 2))(x)
    
    x = layers.Conv2D(64, (3, 3), activation="relu", padding="same")(x)
    x = layers.MaxPooling2D((2, 2))(x)
    
    x = layers.Flatten()(x)
    x = layers.Dense(128, activation="relu")(x)
    
    outputs = layers.Dense(3, activation="sigmoid")(x)
    
    model = models.Model(inputs=inputs, outputs=outputs)
    model.compile(optimizer="adam", loss=custom_loss)

    model.fit(
        X_train, y_train, 
        validation_data=(X_val, y_val),
        epochs=15, 
        batch_size=32
    )

    # --- 4단계: TFLite 변환 및 저장 ---
    print("\n⚙️ [4단계] TFLite 모델로 변환 및 저장 중...")
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    tflite_model = converter.convert()

    tflite_path = "green_detector.tflite"
    with open(tflite_path, "wb") as f:
        f.write(tflite_model)

    print(f"🎉 성공! 시뮬레이션 맞춤형 TFLite 파일이 저장되었습니다: '{os.path.abspath(tflite_path)}'")

if __name__ == "__main__":
    main()