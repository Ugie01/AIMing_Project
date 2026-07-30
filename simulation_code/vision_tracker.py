import cv2
import numpy as np
import tensorflow as tf
import time


class PIDController:

  def __init__(self, kp, ki, kd, max_output=0.1):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.previous_error = 0.0
    self.integral = 0.0
    self.max_output = max_output
    self.last_time = None  # ⏱️ 실제 시간 측정을 위한 변수 추가

  def compute(self, error):
    current_time = time.time()
    if self.last_time is None:
      dt = 0.17  # 첫 호출 시 기본 AI 주기 가정
    else:
      dt = current_time - self.last_time
      if dt <= 0:
        dt = 1e-4

    self.last_time = current_time

    # 1. 적분 계산 (실제 시간 dt 반영)
    self.integral += error * dt

    # 2. 미분 계산 (실제 경과 시간 dt 반영하여 정확한 속도/브레이크 감지)
    derivative = (error - self.previous_error) / dt

    output = self.kp * error + self.ki * self.integral + self.kd * derivative

    # 출력 클램핑 적용 (필요시 주석 해제)
    # output = max(-self.max_output, min(self.max_output, output))

    self.previous_error = error
    return output

  def reset(self):
    self.previous_error = 0.0
    self.integral = 0.0
    self.last_time = None  # 리셋 시 시간 기록도 초기화

class VisionTracker:

  def __init__(
      self,
      model_path="red_detector.tflite",
      kp=0.001,
      ki=0.0,
      kd=0.0002,
  ):
    # 짐벌 모터 제어를 위한 PID 제어기 초기화
    self.pan_pid = PIDController(kp, ki, kd, max_output=0.03)
    self.tilt_pid = PIDController(kp, ki, kd, max_output=0.03)

    # TFLite 모델 로드 (95x95 회귀 모델)
    try:
      self.interpreter = tf.lite.Interpreter(model_path=model_path)
      self.interpreter.allocate_tensors()
      print(f"✅ [Vision] TFLite 모델 로드 성공: {model_path}")
    except Exception as e:
      print(f"❌ [Vision] TFLite 모델 로드 실패: {e}")
      exit()

    self.input_details = self.interpreter.get_input_details()
    self.output_details = self.interpreter.get_output_details()
    
    # 모델 입력 크기 확인 (예: [1, 95, 95, 3])
    self.input_shape = self.input_details[0]["shape"]
    self.model_img_size = self.input_shape[1]  # 95

    # --- [속도 계산을 위한 변수 초기화] ---
    self.prev_cx = None
    self.prev_cy = None
    self.prev_time = None
    self.current_speed = 0.0  # 픽셀/초 단위 속도

  def process_frame(self, frame, dead_zone = 10):
    """프레임을 모델 입력 크기(95x95)로 전처리 후 TFLite 추론 수행 (신뢰도 항상 표시, 0.8 이상만 제어)"""
    orig_height, orig_width, _ = frame.shape

    # 1. 모델 입력 크기(95x95)에 맞게 리사이즈 및 정규화
    input_size = (self.model_img_size, self.model_img_size)
    resized_frame = cv2.resize(frame, input_size)

    input_data = (
        cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB).astype(np.float32)
        / 255.0
    )
    input_data = np.expand_dims(input_data, axis=0)

    # 2. TFLite 추론 실행
    self.interpreter.set_tensor(self.input_details[0]["index"], input_data)
    self.interpreter.invoke()
    output_data = self.interpreter.get_tensor(self.output_details[0]["index"])

    pan_adj, tilt_adj = 0.0, 0.0
    has_target = False

    try:
      # 모델 출력값: [confidence, x_center, y_center]
      pred = np.squeeze(output_data)  
      confidence = float(pred[0])
      norm_x = float(pred[1])
      norm_y = float(pred[2])

      # 원본 화면 픽셀 좌표로 역변환
      best_cx = int(norm_x * orig_width)
      best_cy = int(norm_y * orig_height)

    except Exception as e:
      print(f"❌ 좌표 파싱 에러: {e}")
      return False, 0.0, 0.0, frame

    # 3. 신뢰도가 0.8 이상일 때만 타겟 포착 및 PID 제어 수행
    if confidence >= 0.8:
      has_target = True

      # 속도 계산 로직 ... (기존과 동일)

      img_center_x = orig_width // 2
      img_center_y = orig_height // 2

      error_x = best_cx - img_center_x
      error_y = best_cy - img_center_y

      # 🔥 동적으로 전달받은 데드존 적용
      # if abs(error_x) < dead_zone:
      #   error_x = 0.0
      # if abs(error_y) < dead_zone:
      #   error_y = 0.0

      pan_adj = self.pan_pid.compute(error_x)
      tilt_adj = self.tilt_pid.compute(error_y)

      # 신뢰도가 0.8 이상일 때만 타겟 위치에 초록색 원과 좌표 표시
      cv2.circle(frame, (best_cx, best_cy), 10, (0, 255, 0), 2)
      cv2.putText(
          frame,
          f"Target: ({best_cx}, {best_cy})",
          (max(0, best_cx - 40), max(20, best_cy - 15)),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5,
          (0, 255, 0),
          2,
      )

    # # 4. 신뢰도 값은 타겟 유무와 상관없이 '항상' 검은색((0, 0, 0))으로 화면에 출력
    # cv2.putText(
    #     frame,
    #     f"Conf: {confidence:.2f}",
    #     (10, 30),
    #     cv2.FONT_HERSHEY_SIMPLEX,
    #     0.6,
    #     (0, 0, 0),  # 검은색
    #     2,
    # )
    # # 화면에 신뢰도 및 계산된 속도(px/s) 출력
    # cv2.putText(
    #     frame,
    #     f"Conf: {confidence:.2f} | Speed: {self.current_speed:.1f} px/s",
    #     (10, 30),
    #     cv2.FONT_HERSHEY_SIMPLEX,
    #     0.5,
    #     (0, 0, 0),  
    #     2,
    # )

    # 화면 중앙 십자가 기준선 표시
    cv2.line(frame, (orig_width // 2, 0), (orig_width // 2, orig_height), (255, 255, 255), 1)
    cv2.line(frame, (0, orig_height // 2), (orig_width, orig_height // 2), (255, 255, 255), 1)

    return has_target, pan_adj, tilt_adj, frame