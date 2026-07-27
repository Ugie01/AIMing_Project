import os
import random
import numpy as np
from PIL import Image, ImageDraw
from tqdm import tqdm

def generate_synthetic_dataset(
    output_dir="./synthetic_green_dataset", 
    num_samples=1000, 
    img_size=95, 
    min_obj_size=10, 
    max_obj_size=40
):
    """
    초록 탐지 야매 모델을 위한 합성 데이터셋 생성 함수
    
    Args:
        output_dir (str): 저장할 폴더 경로
        num_samples (int): 생성할 이미지 총 개수
        img_size (int): 이미지 가로/세로 크기 (기본 95)
        min_obj_size (int): 초록색 객체의 최소 크기 (가로/세로 중 최솟값 기준)
        max_obj_size (int): 초록색 객체의 최대 크기 (가로/세로 중 최댓값 기준)
    """
    # 1. 저장 폴더 생성 (이미 존재하면 건너뜀)
    images_dir = os.path.join(output_dir, "images")
    labels_dir = os.path.join(output_dir, "labels")
    
    os.makedirs(images_dir, exist_ok=True)
    os.makedirs(labels_dir, exist_ok=True)
    
    print(f"[*] 데이터셋 생성을 시작합니다. 총 샘플 수: {num_samples}장")
    print(f"[*] 저장 경로: {os.path.abspath(output_dir)}")
    
    for i in tqdm(range(num_samples), desc="Generating Dataset"):
        # 2. 적절히 노이즈가 섞인 배경 생성 (기본 회색/어두운 톤에 랜덤 가우스 노이즈 추가)
        base_color = random.randint(30, 200)
        bg_array = np.random.normal(loc=base_color, scale=25, size=(img_size, img_size, 3))
        bg_array = np.clip(bg_array, 0, 255).astype(np.uint8)
        img = Image.fromarray(bg_array, 'RGB')
        draw = ImageDraw.Draw(img)
        
        # 3. 객체의 크기 랜덤 설정 (너무 작거나 크지 않게 범위 지정)
        obj_w = random.randint(min_obj_size, max_obj_size)
        obj_h = random.randint(min_obj_size, max_obj_size)
        
        # 4. 중앙 좌표(또는 박스 위치) 완전 랜덤 설정
        # 이미지를 벗어나지 않도록 범위 조절
        x1 = random.randint(0, max(0, img_size - obj_w))
        y1 = random.randint(0, max(0, img_size - obj_h))
        x2 = x1 + obj_w
        y2 = y1 + obj_h
        
        # 5. 초록색 객체 그리기 (RGB 중 G를 높게 설정)
        green_color = (
            random.randint(0, 50),     # R
            random.randint(150, 255),  # G (초록색 강조)
            random.randint(0, 50)      # B
        )
        
        # 단순 사각형 또는 타원 형태로 초록색 객체 생성 (필요에 따라 변경 가능)
        shape_type = random.choice(['rectangle', 'ellipse'])
        if shape_type == 'rectangle':
            draw.rectangle([x1, y1, x2, y2], fill=green_color)
        else:
            draw.ellipse([x1, y1, x2, y2], fill=green_color)
            
        # 6. 파일 저장
        file_name = f"sample_{i:05d}"
        img_path = os.path.join(images_dir, f"{file_name}.png")
        label_path = os.path.join(labels_dir, f"{file_name}.txt")
        
        img.save(img_path)
        
        # 정규화된 YOLO 형식 등으로 라벨 저장 (class_id, x_center, y_center, width, height)
        # 여기서는 간단히 [x_center, y_center, w, h] 정규화 좌표를 텍스트로 저장합니다.
        x_center = ((x1 + x2) / 2.0) / img_size
        y_center = ((y1 + y2) / 2.0) / img_size
        norm_w = obj_w / img_size
        norm_h = obj_h / img_size
        
        with open(label_path, 'w') as f:
            # 클래스 0을 초록색 객체로 가정
            f.write(f"0 {x_center:.6f} {y_center:.6f} {norm_w:.6f} {norm_h:.6f}\n")
            
    # 7. 생성된 데이터셋 크기(용량 및 파일 개수) 확인 기능
    total_size_bytes = 0
    total_files = 0
    for root, dirs, files in os.walk(output_dir):
        for file in files:
            filepath = os.path.join(root, file)
            total_size_bytes += os.path.getsize(filepath)
            total_files += 1
            
    total_size_mb = total_size_bytes / (1024 * 1024)
    
    print("\n[✔] 데이터셋 생성 완료!")
    print(f" - 총 파일 개수: {total_files}개 (이미지 + 라벨)")
    print(f" - 총 데이터셋 용량: {total_size_mb:.2f} MB")
    print(f" - 저장된 디렉토리: {os.path.abspath(output_dir)}")

if __name__ == "__main__":
    # 실행 예시 (필요에 따라 샘플 수나 크기 범위 조절 가능)
    generate_synthetic_dataset(
        output_dir="./synthetic_green_dataset",
        num_samples=2000,       # 생성할 이미지 수
        img_size=95,            # 95*95 스케일
        min_obj_size=12,        # 최소 크기
        max_obj_size=35         # 최대 크기
    )