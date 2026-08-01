import serial
import numpy as np
import cv2

# 시리얼 포트 설정 (사용 환경에 맞게 COM 포트 변경)
PORT = 'COM13'
BAUD = 115200

FRAME_W = 160
FRAME_H = 120
FRAME_BYTES = FRAME_W * FRAME_H * 2  # 38,400 bytes

# 확대 배율 설정 (3배 확대 -> 480x360)
SCALE_FACTOR = 3  

ser = serial.Serial(PORT, BAUD, timeout=5)
print(f"OV2640 Live View Started on {PORT}... (Press 'q' to quit)")

while True:
    try:
        line = ser.readline()
        print(line)
        # 1. RGB565 모드 프레임 수신
        if b'---START_RGB---' in line:
            raw_data = ser.read(FRAME_BYTES)
            if len(raw_data) == FRAME_BYTES:
                # 160x120 크기의 2채널(YUV422) 배열로 복원
                raw_16 = np.frombuffer(raw_data, dtype=np.uint8).reshape((FRAME_H, FRAME_W, 2))
                raw_16 = raw_16.byteswap()

                # YUV422 -> BGR 컬러 변환
                bgr_img = cv2.cvtColor(raw_16, cv2.COLOR_BGR5652BGR)               
                # 3배 확대
                resized_img = cv2.resize(bgr_img, (FRAME_W * SCALE_FACTOR, FRAME_H * SCALE_FACTOR), interpolation=cv2.INTER_NEAREST)
                
                cv2.imshow("OV2640 View", resized_img)

        # 2. Grayscale 흑백 모드 프레임 수신
        elif b'---START_GRAY---' in line or b'---START---' in line:
            raw_data = ser.read(FRAME_BYTES)
            if len(raw_data) == FRAME_BYTES:
                raw_arr = np.frombuffer(raw_data, dtype=np.uint8)
                
                # YUV422 포맷 중 Y(밝기) 바이트만 추출
                gray_img = raw_arr[1::2].reshape((FRAME_H, FRAME_W))
                
                # 3배 확대
                resized_img = cv2.resize(gray_img, (FRAME_W * SCALE_FACTOR, FRAME_H * SCALE_FACTOR), interpolation=cv2.INTER_NEAREST)
                
                cv2.imshow("OV2640 View", resized_img)

        # 'q' 키 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except Exception as e:
        print("에러 발생:", e)
        break

ser.close()
cv2.destroyAllWindows()