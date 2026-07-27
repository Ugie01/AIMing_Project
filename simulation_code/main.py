import time
import cv2
import numpy as np
import pybullet as p
import pybullet_data
from gimbal import GimbalSystem
from vision_tracker import VisionTracker

# 트랙바 이벤트 처리를 위한 빈 콜백 함수
def nothing(x):
    pass

def main():
    # 1. 시뮬레이터 연결 및 환경 설정
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # 바닥 생성
    p.loadURDF("plane.urdf")
  #============================================================
    # 초록색 타겟을 초기 랜덤 위치에 생성하고 ID를 반환받음
    # 초록색 네모
    # initial_target_pos = [0.2, 0.6, 0.1]
    # target_visual = p.createVisualShape(
    #     p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03], rgbaColor=[0, 1, 0, 1]
    # )
    # targetId = p.createMultiBody(
    #     baseMass=0.0,
    #     baseVisualShapeIndex=target_visual,
    #     basePosition=initial_target_pos,
    #     baseOrientation=[0, 0, 0, 1],
    # )

    # 드론 형태
    # 초록색 타겟 크기를 실제 드론이나 사람 크기로 수정
    # 예시: 가로 20cm, 세로 20cm, 높이 10cm 크기의 드론 형태일 경우
    # initial_target_pos = [0.2, 0.6, 0.1]
    # target_visual = p.createVisualShape(
    #     p.GEOM_BOX, 
    #     halfExtents=[0.1, 0.1, 0.05],  # 👈 이 부분을 원하는 크기의 절반 값으로 수정
    #     rgbaColor=[0, 1, 0, 1]
    # )
    # targetId = p.createMultiBody(
    #     baseMass=0.0,
    #     baseVisualShapeIndex=target_visual,
    #     basePosition=initial_target_pos,
    #     baseOrientation=[0, 0, 0, 1],
    # )

    #사람 형태
    # 예시: 가로 50cm, 세로 25cm, 높이 170cm 크기의 사람 형태일 경우
    initial_target_pos = [0.2, 0.6, 0.1]
    target_visual = p.createVisualShape(
        p.GEOM_BOX, 
        halfExtents=[0.25, 0.125, 0.85],  # 👈 이 부분을 원하는 크기의 절반 값으로 수정
        rgbaColor=[0, 1, 0, 1]
    )
    targetId = p.createMultiBody(
        baseMass=0.0,
        baseVisualShapeIndex=target_visual,
        basePosition=initial_target_pos,
        baseOrientation=[0, 0, 0, 1],
    )
  #=====================================================

    print("🟢 초록색 타겟 큐브가 생성되었습니다.")
    print("=== 조작 안내 ===")
    print(" [방향키 (←/→/↑/↓)] : 짐벌 수동 조작 (Pan/Tilt)")
    print(" [R / F]           : 타겟 Y축 이동 (앞 / 뒤)")
    print(" [D / G]           : 타겟 X축 이동 (좌 / 우)")
    print(" [S / E]           : 타겟 Z축 이동 (상승 / 하강)")
    print(" [Space Bar]       : PID 자동 추적 모드 토글 (ON / OFF)")

    # 2. 클래스 인스턴스화
    gimbal = GimbalSystem()
    tracker = VisionTracker(kp=0.0003, ki=0.0, kd=0.0000)

    # 창 이름 정의
    cam_window_name = "Smart Turret Simulation"
    debug_window_name = "PID Control & Debug"

    # 창 생성 및 PID 슬라이더를 '디버깅 전용 창'에 배치
    cv2.namedWindow(cam_window_name)
    cv2.namedWindow(debug_window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(debug_window_name, 450, 250)

    cv2.createTrackbar("KP x10000", debug_window_name, 3, 100, nothing)
    cv2.createTrackbar("KI x10000", debug_window_name, 0, 50, nothing)
    cv2.createTrackbar("KD x100000", debug_window_name, 0, 50, nothing)

    # 제어 상태 변수
    auto_tracking = False
    step_size_gimbal = 0.05
    #step_size_target = 0.02 # 시속 17km
    step_size_target = 0.12 # 시속 100km

    current_target_pos = list(initial_target_pos)

    last_keys = {}

    # 디버깅 출력을 위한 초기 변수값 선언
    current_kp, current_ki, current_kd = 0.0003, 0.0, 0.0
    error_x, error_y = 0.0, 0.0
    pan_adj, tilt_adj = 0.0, 0.0

    try:
        while True:
            p.stepSimulation()

            # --- [디버그 창 트랙바 값 읽어와서 PID 게인에 반영] ---
            raw_kp = cv2.getTrackbarPos("KP x10000", debug_window_name)
            raw_ki = cv2.getTrackbarPos("KI x10000", debug_window_name)
            raw_kd = cv2.getTrackbarPos("KD x100000", debug_window_name)

            current_kp = raw_kp / 10000.0
            current_ki = raw_ki / 10000.0
            current_kd = raw_kd / 100000.0

            tracker.pan_pid.kp = current_kp
            tracker.pan_pid.ki = current_ki
            tracker.pan_pid.kd = current_kd

            tracker.tilt_pid.kp = current_kp
            tracker.tilt_pid.ki = current_ki
            tracker.tilt_pid.kd = current_kd
            # ----------------------------------------------------

            # 1. 키보드 입력 처리
            keys = p.getKeyboardEvents()
            for key, value in keys.items():
                if value & p.KEY_IS_DOWN or value & p.KEY_WAS_TRIGGERED:

                    if not auto_tracking:
                        if key == p.B3G_LEFT_ARROW:
                            gimbal.pan_angle += step_size_gimbal
                        elif key == p.B3G_RIGHT_ARROW:
                            gimbal.pan_angle -= step_size_gimbal
                        elif key == p.B3G_UP_ARROW:
                            gimbal.tilt_angle += step_size_gimbal
                        elif key == p.B3G_DOWN_ARROW:
                            gimbal.tilt_angle -= step_size_gimbal

                    if key == ord("r"):
                        current_target_pos[1] += step_size_target
                    elif key == ord("f"):
                        current_target_pos[1] -= step_size_target
                    elif key == ord("d"):
                        current_target_pos[0] -= step_size_target
                    elif key == ord("g"):
                        current_target_pos[0] += step_size_target
                    elif key == ord("s"):
                        current_target_pos[2] += step_size_target
                    elif key == ord("e"):
                        current_target_pos[2] -= step_size_target

                    if (
                        key == ord(" ")
                        and key not in last_keys
                        and (value & p.KEY_WAS_TRIGGERED)
                    ):
                        auto_tracking = not auto_tracking
                        mode_str = "🟢 [AUTO TRACKING ON]" if auto_tracking else "🔴 [MANUAL MODE]"
                        print(mode_str)

            p.resetBasePositionAndOrientation(
                targetId, current_target_pos, [0, 0, 0, 1]
            )

            last_keys = keys

            # 2. 카메라 위치 동기화 및 이미지 캡처
            cam_pos, cam_target, cam_up = gimbal.update_camera_pose()

            view_matrix = p.computeViewMatrix(
                cameraEyePosition=cam_pos,
                cameraTargetPosition=cam_target,
                cameraUpVector=cam_up,
            )
            proj_matrix = p.computeProjectionMatrixFOV(
                fov=68, aspect=1.0, nearVal=0.1, farVal=100.0
            )

            width, height, rgbImg, _, _ = p.getCameraImage(
                width=320, height=320, viewMatrix=view_matrix, projectionMatrix=proj_matrix
            )

            frame = np.reshape(rgbImg, (height, width, 4)).astype(np.uint8)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

            # 3. 비전 트래커를 통한 타겟 검출 및 오차 계산
            has_target, pan_adj, tilt_adj, processed_frame = tracker.process_frame(frame)

            # VisionTracker 내부에서 계산된 직전 오차값 가져오기 (없으면 0 처리)
            error_x = getattr(tracker, 'last_error_x', 0.0)
            error_y = getattr(tracker, 'last_error_y', 0.0)

            # 4. 모터 제어 모드 분기
            if auto_tracking and has_target:
                gimbal.pan_angle -= pan_adj
                gimbal.tilt_angle -= tilt_adj
                gimbal.set_target_angles(gimbal.pan_angle, gimbal.tilt_angle)

                cv2.putText(
                    processed_frame,
                    "MODE: AUTO TRACKING",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
            else:
                gimbal.set_target_angles(gimbal.pan_angle, gimbal.tilt_angle)
                pan_adj, tilt_adj = 0.0, 0.0  # 비동작 시 출력을 0으로 초기화

                mode_text = (
                    "MODE: MANUAL" if not auto_tracking else "MODE: AUTO (TARGET LOST)"
                )
                cv2.putText(
                    processed_frame,
                    mode_text,
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                )

            # 5. [신규] 디버그 전용 창(`PID Control & Debug`)에 텍스트 대시보드 구성
            debug_board = np.zeros((220, 450, 3), dtype=np.uint8)
            
            texts = [
                f"[PID Gains]",
                f"  KP: {current_kp:.5f} | KI: {current_ki:.5f} | KD: {current_kd:.5f}",
                f"[Error Values]",
                f"  Error X: {error_x:.2f} px | Error Y: {error_y:.2f} px",
                f"[Motor Outputs]",
                f"  Pan Adj: {pan_adj:.5f}   | Tilt Adj: {tilt_adj:.5f}",
                f"[Status]",
                f"  Mode: {'AUTO TRACKING' if auto_tracking else 'MANUAL'} (Target: {has_target})"
            ]

            y_offset = 25
            for t in texts:
                color = (0, 255, 0) if "AUTO" in t or t.startswith("[") else (255, 255, 255)
                cv2.putText(debug_board, t, (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                y_offset += 22

            # 6. 두 개의 창 출력 (카메라 화면 + 디버그 화면)
            cv2.imshow(cam_window_name, processed_frame)
            cv2.imshow(debug_window_name, debug_board)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            time.sleep(1.0 / 240.0)

    except p.error:
        pass

    cv2.destroyAllWindows()
    p.disconnect()

if __name__ == "__main__":
    main()