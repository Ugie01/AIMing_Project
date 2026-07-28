import os
import random
import csv
import numpy as np
from PIL import Image, ImageDraw

def generate_bounding_box_csv(output_csv="dataset_boxes.csv", img_dir="dataset_images", num_samples=2000, img_size=95):
    # 이미지 저장을 위한 폴더 생성
    os.makedirs(img_dir, exist_ok=True)
    
    print(f"📦 총 {num_samples}장의 이미지와 바운딩 박스 CSV를 생성합니다...")
    
    with open(output_csv, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(["file_name", "classes", "xmin", "xmax", "ymin", "ymax"])
        
        for i in range(num_samples):
            file_name = f"cubes_training_{i}.jpg"
            
            # 지평선 배경 모사 (상단 하늘, 하단 바닥 구조)
            bg_array = np.zeros((img_size, img_size, 3), dtype=np.uint8)
            
            sky_color = [random.randint(220, 255), random.randint(220, 255), random.randint(230, 255)]
            ground_color = [random.randint(160, 200), random.randint(140, 180), random.randint(120, 160)]
            
            horizon_y = random.randint(int(img_size * 0.3), int(img_size * 0.7))
            bg_array[:horizon_y, :] = sky_color
            bg_array[horizon_y:, :] = ground_color
            
            # 가우스 노이즈 추가
            noise = np.random.normal(loc=0, scale=15, size=(img_size, img_size, 3))
            bg_array = np.clip(bg_array.astype(np.float32) + noise, 0, 255).astype(np.uint8)
            
            img = Image.fromarray(bg_array, 'RGB')
            draw = ImageDraw.Draw(img)
            
            # 빨간색 타겟 생성
            obj_w = random.randint(10, 40)
            obj_h = random.randint(10, 40)
            
            xmin = random.randint(0, max(0, img_size - obj_w))
            ymin = random.randint(0, max(0, img_size - obj_h))
            xmax = xmin + obj_w
            ymax = ymin + obj_h
            
            red_color = (
                random.randint(150, 255),  # R
                random.randint(0, 50),     # G
                random.randint(0, 50)      # B
            )
            
            if random.random() > 0.5:
                draw.rectangle([xmin, ymin, xmax, ymax], fill=red_color)
            else:
                draw.ellipse([xmin, ymin, xmax, ymax], fill=red_color)
            
            # 생성된 이미지를 파일로 저장
            img.save(os.path.join(img_dir, file_name))
            
            # CSV 행 기록
            writer.writerow([file_name, "red", xmin, xmax, ymin, ymax])

    print(f"✅ 이미지 저장 폴더: '{img_dir}/'")
    print(f"✅ CSV 파일 생성 완료: '{output_csv}'")

if __name__ == "__main__":
    generate_bounding_box_csv()