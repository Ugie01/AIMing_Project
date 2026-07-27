import os
import glob
import random
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import tensorflow as tf

def main():
    # 💡 red_detector.tflite 모델을 사용하도록 경로 수정
    tflite_path = "simulation_code/red_detector.tflite"
    # 💡 synthetic_red_dataset 경로로 수정
    test_img_dir = "./simulation_code/synthetic_red_dataset/test/images"
    test_lbl_dir = "./simulation_code/synthetic_red_dataset/test/labels"

    if not os.path.exists(tflite_path) or not os.path.exists(test_img_dir):
        print("[-] TFLite 파일 또는 test 폴더를 찾을 수 없습니다!")
        return

    interpreter = tf.lite.Interpreter(model_path=tflite_path)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    test_imgs = sorted(glob.glob(os.path.join(test_img_dir, "*.png")))
    test_lbls = sorted(glob.glob(os.path.join(test_lbl_dir, "*.txt")))
    total_samples = len(test_imgs)

    total_distances = []
    accurate_count = 0
    img_size = 95
    threshold_pixels = 5.0

    for img_path, label_path in zip(test_imgs, test_lbls):
        img = Image.open(img_path).convert('RGB')
        img_np = np.array(img, dtype=np.float32) / 255.0
        input_data = np.expand_dims(img_np, axis=0)

        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        
        # 모델 출력: [confidence, x_center, y_center]
        pred = interpreter.get_tensor(output_details[0]['index'])[0]
        conf = pred[0]
        pred_coords = pred[1:3]

        with open(label_path, 'r') as f:
            parts = f.readline().strip().split()
            true_conf = float(parts[0])
            target = np.array([float(parts[1]), float(parts[2])], dtype=np.float32)

        # 타겟이 실제로 존재하는 샘플에 대해서만 오차 계산
        if true_conf > 0.5 and conf > 0.5:
            pred_pixels = pred_coords * img_size
            target_pixels = target * img_size
            distance = np.sqrt(np.sum((pred_pixels - target_pixels) ** 2))
            
            total_distances.append(distance)
            if distance <= threshold_pixels:
                accurate_count += 1

    mean_pixel_error = np.mean(total_distances) if total_distances else 0.0
    print("================================")
    print(f" [✔] 신뢰도 포함 TFLite 테스트 성능 (RED)")
    print(f" - 평균 픽셀 오차: {mean_pixel_error:.2f} px")
    print("================================")

    # 시각화 (30장) - 신뢰도 값도 함께 표시
    sample_indices = random.sample(range(total_samples), min(30, total_samples))
    plt.figure(figsize=(15, 12))
    for i, idx in enumerate(sample_indices):
        img_path = test_imgs[idx]
        label_path = test_lbls[idx]
        
        img = Image.open(img_path).convert('RGB')
        img_np = np.array(img, dtype=np.float32) / 255.0
        
        input_data = np.expand_dims(img_np, axis=0)
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        pred = interpreter.get_tensor(output_details[0]['index'])[0]
        
        conf = pred[0]
        pred_x, pred_y = pred[1] * img_size, pred[2] * img_size
        
        with open(label_path, 'r') as f:
            parts = f.readline().strip().split()
            true_conf = float(parts[0])
            target = [float(parts[1]), float(parts[2])]
            
        true_x, true_y = target[0] * img_size, target[1] * img_size
        
        plt.subplot(5, 6, i + 1)
        plt.imshow(img_np)
        
        # 💡 빨간색 타겟 탐지에 맞게 정답 표시 마커 색상을 'magenta' 또는 눈에 띄는 색으로 변경 가능 (여기서는 시인성을 위해 유지 또는 조정)
        if true_conf > 0.5:
            plt.scatter([true_x], [true_y], color='cyan', marker='o', s=30, label='True')
        if conf > 0.5:
            plt.scatter([pred_x], [pred_y], color='yellow', marker='x', s=30, label='Pred')
            
        plt.title(f"#{i+1} Conf:{conf:.2f}", fontsize=8)
        plt.axis('off')
        
    plt.suptitle("TFLite Test Results with Confidence (Red Target)", fontsize=14)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()