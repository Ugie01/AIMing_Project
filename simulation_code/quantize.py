import tensorflow as tf

# TFLite 모델 로드
interpreter = tf.lite.Interpreter(model_path="red_detector_uint8.tflite")
interpreter.allocate_tensors()

# 모델 내 모든 텐서(가중치 및 활성화 값 포함)의 상세 정보 확인
tensor_details = interpreter.get_tensor_details()

count=1
for tensor in tensor_details:
     
    print(f"Name: {tensor['name']}, Shape: {tensor['shape']}, Type: {tensor['dtype']}, Quantization: {tensor['quantization']}")

    count+=1

    if (count >50):
        break