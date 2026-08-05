# FOMO_Local

STM32 임베디드 환경 적용을 위한 경량 객체 탐지(FOMO) 모델 개발 코드입니다.
데이터 전처리, 모델 학습, 평가, 양자화 및 TensorFlow Lite 배포 모델 생성을 로컬 환경에서 수행하기 위한 AI Pipeline입니다.
---

# 1. 개발 배경
초기 모델 개발은 Edge Impulse Studio를 활용하여 진행했습니다.

- 데이터 업로드
- 모델 학습
- 경량화
- 배포 파일 생성

Edge Impulse Studio는 빠른 개발에는 적합하지만 다음과 같은 제한이 있습니다.
- 모델 구조 및 학습 방식 튜닝 제한
- 데이터셋 규모 증가 시 학습 제한
- 양자화 및 최적화 과정 제어 제한

따라서 모델 최적화와 성능 개선을 위해 TensorFlow 기반 로컬 학습 및 경량화 Pipeline을 개발하고 있습니다.


개발 Flow:

Dataset
↓
YOLO Label Parsing
↓
Heatmap 변환
↓
FOMO Training
↓
Evaluation
↓
PTQ / QAT INT8 Quantization
↓
TensorFlow Lite Export
↓
STM32 적용

---


# 2. Folder Structure

FOMO_Local

├── configs

│ └── config.py # 모델 및 학습 설정
│

├── utils

│ ├── dataset.py # YOLO Dataset Loader

│ └── heatmap.py # FOMO Heatmap 생성

│

├── train.py # 모델 학습

│

├── evaluate.py # Test 평가

├── validation_evaluate.py # Validation 평가

│

├── quantize_model_PTQ.py # PTQ INT8 변환

├── quantize_model_QAT.py # QAT 학습

├── export_qat_int8.py # QAT → TFLite 변환

│

├── evaluate_quantized.py

└── validation_evaluate_quantized.py

# INT8 모델 평가

---


# 3. 실행 Flow

## 1) Dataset 준비
YOLO 형식 Dataset 사용

dataset

├── train

│ ├── images

│ └── labels
│

├── validation

└── test

---


## 2) Model Training

### train.py
Float32 FOMO 모델 학습


결과:

checkpoints/

└── best_fomo.keras

---


## 3) Model Evaluation

### validation_evaluate.py
Validation Dataset 평가


### evaluate.py
Test Dataset 평가

평가 항목:
- Precision
- Recall
- F1 Score
- Inference Time
- FPS

---


## 4) Quantization

### PTQ

Float Model

↓

INT8 TFLite Model


파일:
quantize_model_PTQ.py


### QAT

Float Model

↓

QAT Fine Tuning

↓

INT8 TFLite Model


파일:

quantize_model_QAT.py

export_qat_int8.py

---


## 5) STM32 적용 Flow

AI Part:

Dataset 생성

↓

Model Training

↓

Quantization

↓

.tflite 생성



Embedded Part:

TensorFlow Lite Model 전달

↓

CubeIDE 적용

↓

STM32 Build 및 실행

---


# 4. Current Status

구현 완료:
- YOLO Dataset Loader
- FOMO Heatmap Generator
- TensorFlow Training Pipeline
- Validation/Test Evaluation
- PTQ INT8 Quantization
- QAT Pipeline
- TensorFlow Lite Export


추가 개선 예정:
- Model 구조 최적화
- Dataset Augmentation 개선
- STM32 환경 최적화

