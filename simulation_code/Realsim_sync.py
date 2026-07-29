import time
import cv2
import numpy as np
import pybullet as p
import pybullet_data
import random
from gimbal import GimbalSystem
from vision_tracker import VisionTracker

def nothing(x):
    pass

def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
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

   # 큐브 형태
    #예시: 가로 50cm, 세로 25cm, 높이 170cm 크기의 사람 형태일 경우
    initial_target_pos = [0.2, 0.6, 0.1]
    half_extents = [0.25, 0.25, 0.25]

    # 1. 충돌체와 시각체를 모두 생성
    target_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    target_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[1, 0, 0, 1])

    # 2. 두 개를 모두 포함하여 멀티바디 생성
    targetId = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=target_collision,  # 👈 충돌체 추가!
        baseVisualShapeIndex=target_visual,
        basePosition=initial_target_pos,
        baseOrientation=[0, 0, 0, 1],
    )
    # 타겟 물체 크기를 빨간 네모 크기([0.005, 0.01, 0.01])와 똑같이 설정
    # initial_target_pos = [0.2, 0.6, 0.1]
    # target_visual = p.createVisualShape(
    #     p.GEOM_BOX, 
    #     halfExtents=[0.005, 0.01, 0.01],  # 👈 빨간 네모와 정확히 동일한 크기
    #     rgbaColor=[1, 0, 0, 1]  # 타겟
    # )
    # targetId = p.createMultiBody(
    #     baseMass=0.0,
    #     baseVisualShapeIndex=target_visual,
    #     basePosition=initial_target_pos,
    #     baseOrientation=[0, 0, 0, 1],
    # )
    # #=====================================================
    print("🟢 초록색 타겟 큐브가 생성되었습니다.")
    print("=== 조작 안내 ===")
    print(" [방향키 (←/→/↑/↓)] : 짐벌 수동 조작 (Pan/Tilt)")
    print(" [R / F]           : 타겟 Y축 이동 (앞 / 뒤)")
    print(" [D / G]           : 타겟 X축 이동 (좌 / 우)")
    print(" [S / E]           : 타겟 Z축 이동 (상승 / 하강)")
    print(" [Space Bar]       : PID 자동 추적 모드 토글 (ON / OFF)")

    gimbal = GimbalSystem()
    tracker = VisionTracker(kp=0.0003, ki=0.0, kd=0.0000)

    cam_window_name = "Smart Turret Simulation"
    debug_window_name = "PID Control & Debug"
    cv2.namedWindow(cam_window_name)
    cv2.namedWindow(debug_window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(debug_window_name, 450, 250)

    cv2.createTrackbar("KP x10000", debug_window_name, 20, 100, nothing)
    cv2.createTrackbar("KI x10000", debug_window_name, 0, 50, nothing)
    cv2.createTrackbar("KD x100000", debug_window_name, 1, 50, nothing)

    auto_tracking = False
    step_size_gimbal = 0.05
    step_size_target = 0.06  # 0.12 =  시속 21km 설정
    current_target_pos = list(initial_target_pos)
    last_keys = {}

    current_kp, current_ki, current_kd = 0.0020, 0.0, 0.00001
    error_x, error_y = 0.0, 0.0
    pan_adj, tilt_adj = 0.0, 0.0

    # --- [STM32 20ms 제어 주기 및 SG90 물리 속도 한계 설정] ---
    CONTROL_PERIOD = 0.02  # 20ms (50Hz) 인터럽트 주기
    AI_INFERENCE_PERIOD = 0.14

    last_control_time = time.time()
    last_ai_time = time.time()

    SG90_MAX_SPEED_DEG_PER_SEC = 600.0  # 초당 최대 600도 (0.1s / 60 deg)
    MAX_STEP_DEG = SG90_MAX_SPEED_DEG_PER_SEC * CONTROL_PERIOD  # 20ms당 최대 이동 한계 (~12도)
    MAX_STEP_RAD = np.radians(MAX_STEP_DEG)

    # 짐벌 모터의 직전 명령 각도 기억 변수 (속도 제한 계산용)
    prev_pan_cmd = gimbal.pan_angle
    prev_tilt_cmd = gimbal.tilt_angle

    has_target = False
    current_speed = 0.0
    current_conf = 0.0
    processed_frame = None
    
    # [추가] 레이저 라인 ID 추적 변수 초기화
    laser_line_id = -1

    try:
        while True:
            p.stepSimulation()
            current_time = time.time()

            # ==============================================================
            # 1. STM32 AI 연산 주기 (140ms) : 이미지 입력 및 모델 추론, 타겟 위치 갱신
            # ==============================================================
            if current_time - last_ai_time >= AI_INFERENCE_PERIOD:
                last_ai_time = current_time

                # 카메라 위치 동기화 및 이미지 캡처
                cam_pos, cam_target, cam_up = gimbal.update_camera_pose()
                view_matrix = p.computeViewMatrix(cameraEyePosition=cam_pos, cameraTargetPosition=cam_target, cameraUpVector=cam_up)
                proj_matrix = p.computeProjectionMatrixFOV(fov=68, aspect=1.0, nearVal=0.1, farVal=100.0)

                width, height, rgbImg, _, _ = p.getCameraImage(width=160, height=120, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
                frame = np.reshape(rgbImg, (height, width, 4)).astype(np.uint8)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

                # 비전 추적 및 오차 연산 (TFLite 추론 포함)
                has_target, pan_adj, tilt_adj, processed_frame = tracker.process_frame(frame)
                error_x = getattr(tracker, 'last_error_x', 0.0)
                error_y = getattr(tracker, 'last_error_y', 0.0)

                current_speed = getattr(tracker, 'current_speed', 0.0)
                current_conf = getattr(tracker, 'confidence', 0.0)

                # 비전 도메인 노이즈 주입
                if has_target:
                    noise_std = 0.5  
                    pan_adj += np.random.normal(0, noise_std * 0.0001)
                    tilt_adj += np.random.normal(0, noise_std * 0.0001)

            # ==============================================================
            # 2. STM32 모터 제어 주기 (20ms) : 기존 제어 및 속도 제한 로직 유지[cite: 6]
            # ==============================================================
            if current_time - last_control_time >= CONTROL_PERIOD:
                last_control_time = current_time

                # 트랙바 값 읽기
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

                # 키보드 입력 처리
                keys = p.getKeyboardEvents()
                for key, value in keys.items():
                    if value & p.KEY_IS_DOWN or value & p.KEY_WAS_TRIGGERED:
                        if not auto_tracking:
                            if key == p.B3G_LEFT_ARROW: gimbal.pan_angle += step_size_gimbal
                            elif key == p.B3G_RIGHT_ARROW: gimbal.pan_angle -= step_size_gimbal
                            elif key == p.B3G_UP_ARROW: gimbal.tilt_angle += step_size_gimbal
                            elif key == p.B3G_DOWN_ARROW: gimbal.tilt_angle -= step_size_gimbal

                        if key == ord("r"): current_target_pos[1] += step_size_target
                        elif key == ord("f"): current_target_pos[1] -= step_size_target
                        elif key == ord("d"): current_target_pos[0] -= step_size_target
                        elif key == ord("g"): current_target_pos[0] += step_size_target
                        elif key == ord("s"): current_target_pos[2] += step_size_target
                        elif key == ord("e"): current_target_pos[2] -= step_size_target

                        if key == ord(" ") and key not in last_keys and (value & p.KEY_WAS_TRIGGERED):
                            auto_tracking = not auto_tracking
                            mode_str = "🟢 [AUTO TRACKING ON]" if auto_tracking else "🔴 [MANUAL MODE]"
                            print(mode_str)

                p.resetBasePositionAndOrientation(targetId, current_target_pos, [0, 0, 0, 1])
                last_keys = keys

                # 모드별 목표 각도 산출 (140ms마다 갱신된 pan_adj, tilt_adj 활용)
                if auto_tracking and has_target:
                    desired_pan = gimbal.pan_angle - pan_adj
                    desired_tilt = gimbal.tilt_angle - tilt_adj
                else:
                    desired_pan = gimbal.pan_angle
                    desired_tilt = gimbal.tilt_angle
                    pan_adj, tilt_adj = 0.0, 0.0

                # 모터 물리 속도 한계(Rate Limiting) 적용
                pan_diff = desired_pan - prev_pan_cmd
                tilt_diff = desired_tilt - prev_tilt_cmd

                pan_diff = np.clip(pan_diff, -MAX_STEP_RAD, MAX_STEP_RAD)
                tilt_diff = np.clip(tilt_diff, -MAX_STEP_RAD, MAX_STEP_RAD)

                gimbal.pan_angle = prev_pan_cmd + pan_diff
                gimbal.tilt_angle = prev_tilt_cmd + tilt_diff

                prev_pan_cmd = gimbal.pan_angle
                prev_tilt_cmd = gimbal.tilt_angle

                gimbal.set_target_angles(gimbal.pan_angle, gimbal.tilt_angle)

                # ======= [수정] 잔상 없는 10m 레이저 갱신 =======
                hit_target, laser_line_id = gimbal.update_laser_beam(targetId, laser_line_id)
                
                # ==============================================================
                # UI 디스플레이 갱신
                if processed_frame is not None:
                    display_frame = processed_frame.copy()
                    if auto_tracking and has_target:
                        cv2.putText(display_frame, "MODE: AUTO TRACKING (140ms)", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    else:
                        mode_text = "MODE: MANUAL" if not auto_tracking else "MODE: AUTO (TARGET LOST)"
                        cv2.putText(display_frame, mode_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    cv2.imshow(cam_window_name, display_frame)

                debug_board = np.zeros((280, 450, 3), dtype=np.uint8)
                texts = [
                    f"[STM32 + SG90 + 140ms AI Cycle]",
                    f"  KP: {current_kp:.5f} | KI: {current_ki:.5f} | KD: {current_kd:.5f}",
                    f"[System Status]",
                    f"  Mode   : {'AUTO TRACKING' if auto_tracking else 'MANUAL'} (Target: {has_target})",
                    f"  Laser  : {'🔥 TARGET HIT!' if hit_target else '--- (Scanning)'}",
                    f"  Conf   : {current_conf:.2f}",
                    f"[Target Motion & Error]",
                    f"  Speed  : {current_speed:.1f} px/s",
                    f"  Error X: {error_x:.2f} px | Error Y: {error_y:.2f} px",
                    f"[Motor Outputs]",
                    f"  Pan Adj: {pan_adj:.5f}   | Tilt Adj: {tilt_adj:.5f}"
                ]

                y_offset = 25
                for t in texts:
                    color = (0, 255, 0) if "AUTO" in t or t.startswith("[") else (255, 255, 255)
                    cv2.putText(debug_board, t, (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
                    y_offset += 22

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