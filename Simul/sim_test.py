import time
import os
import csv
import cv2
import numpy as np
import pybullet as p
import pybullet_data
import random
from datetime import datetime
from gimbal import GimbalSystem
from vision_tracker import VisionTracker

def nothing(x):
    pass

def main():
    # ==========================================
    # 🧪 [실험 설정 모드 스위치]
    # ==========================================
    EXPERIMENT_MODE = "FULL_SWEEP"  # 또는 "FULL_SWEEP"
    REPEAT = 5

    # 🖥️ 실행 모드 설정 (True: GUI 화면 켜기 / False: 헤드리스 고속 실행)
    USE_GUI = False

    if USE_GUI:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    # ==========================================
    # 🎯 [실험 파라미터 정의 및 세팅]
    # ==========================================
    if EXPERIMENT_MODE == "DEAD_ZONE":
        deadzone_list = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20]
        target_size_options = [0.5]
        target_speed_options = [7]
        distance_options = [7]

        total_combinations = len(deadzone_list)
    else:
        deadzone_list = [4]
        target_size_options = [0.2, 0.4, 0.6, 0.8, 1.0]
        target_speed_options = [3, 5, 10, 20, 40, 60, 80]
        distance_options = [4,6,8,10, 12]
        
        total_combinations = len(deadzone_list) * len(target_size_options) * len(target_speed_options) * len(distance_options)

    # 초기 파라미터 값 할당
    target_size_scale = target_size_options[0]
    target_speed_val = target_speed_options[0]
    target_distance = distance_options[0]
    current_deadzone = deadzone_list[0]

    # 원 운동을 위한 초기 각도를 완전한 랜덤(0 ~ 2π)으로 지정
    theta = random.uniform(-np.pi/12, np.pi/12)

    # 타겟 생성 (구 형태)
    target_radius = target_size_scale
    target_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=target_radius)
    target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=target_radius, rgbaColor=[1, 0, 0, 1])

    # 짐벌(0,0) 중심의 정확한 원주상 초기 위치 계산
    initial_target_x = target_distance * np.cos(theta)
    initial_target_y = target_distance * np.sin(theta)
    initial_target_pos = [initial_target_x, initial_target_y, 0.5]

    targetId = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=target_collision,
        baseVisualShapeIndex=target_visual,
        basePosition=initial_target_pos,
        baseOrientation=[0, 0, 0, 1],
    )

    max_total_trials = total_combinations * REPEAT
    print(f"🟢 [셋업 완료] 모드: {EXPERIMENT_MODE} | 총 조합 수: {total_combinations} | 반복(REPEAT): {REPEAT} | 총 트라이얼: {max_total_trials}")
    print(f"예상 최대 시간 : {REPEAT*total_combinations*10}초 ( {REPEAT*total_combinations*10/3600:.2f} 시간 )")
    gimbal = GimbalSystem()
    tracker = VisionTracker(kp=0.0020, ki=0.0, kd=0.0001)

    lock_stable_count = 0
    episode_step = 0
    MAX_EPISODE_STEPS = 500  

    if USE_GUI:
        cam_window_name = "Smart Turret Simulation"
        debug_window_name = "PID Control & Debug"
        cv2.namedWindow(cam_window_name)
        cv2.namedWindow(debug_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(debug_window_name, 450, 320)
        cv2.createTrackbar("KP x10000", debug_window_name, 41, 100, nothing)
        cv2.createTrackbar("KI x10000", debug_window_name, 2, 50, nothing)
        cv2.createTrackbar("KD x100000", debug_window_name, 3, 50, nothing)

    auto_tracking = True 
    current_kp, current_ki, current_kd = 0.0020, 0.0, 0.00001
    error_x, error_y = 0.0, 0.0
    pan_adj, tilt_adj = 0.0, 0.0

    CONTROL_PERIOD = 0.02  
    AI_INFERENCE_PERIOD = 0.08

    last_control_time = time.time()
    last_ai_time = time.time()

    SG90_MAX_SPEED_DEG_PER_SEC = 600.0
    MAX_STEP_DEG = SG90_MAX_SPEED_DEG_PER_SEC * CONTROL_PERIOD
    MAX_STEP_RAD = np.radians(MAX_STEP_DEG)

    prev_pan_cmd = gimbal.pan_angle
    prev_tilt_cmd = gimbal.tilt_angle

    has_target = False
    current_speed = 0.0
    current_conf = 0.0
    processed_frame = None
    laser_line_id = -1

    # CSV 로거 세팅
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")


    if EXPERIMENT_MODE == "DEAD_ZONE":
        csv_filename = f"{EXPERIMENT_MODE}_experiment_results_{target_speed_val}km_{target_distance}m.csv"

    else : 
        csv_filename = f"{EXPERIMENT_MODE}_experiment_results_{timestamp}.csv"
        
    file_exists = os.path.isfile(csv_filename)
    csv_file = open(csv_filename, mode='a', newline='', encoding='utf-8')
    csv_writer = csv.writer(csv_file)
    if not file_exists:
        csv_writer.writerow(['trial_id', 'deadzone_px', 'target_size', 'target_speed', 'target_distance', 'success', 'time_to_lock', 'hit_ratio'])

    try:
        trial_id = 1
        running = True
        while running:
            p.stepSimulation()
            current_time = time.time()
            

            # ==============================================================
            # 🔄 [타겟 모션 제어] : 짐벌(0,0) 기준 완벽한 XY 평면 원 운동
            # ==============================================================
            # 선속도를 반지름(target_distance)으로 나누어 순간 각속도 계산 (v = r * omega -> omega = v / r)
            target_linear_speed = target_speed_val/3.6

            angular_velocity = target_linear_speed / target_distance
            
            # 각도 업데이트 (CONTROL_PERIOD 또는 시뮬레이션 스텝 시간인 0.02초 반영)
            theta += angular_velocity * 0.02
            target_x = target_distance * np.cos(theta)
            target_y = target_distance * np.sin(theta)
            target_z = 0.5 + np.random.normal(0, 0.01)

            current_target_pos = [target_x, target_y, target_z]
            p.resetBasePositionAndOrientation(targetId, current_target_pos, [0, 0, 0, 1])

            # 1. AI 연산 주기
            if current_time - last_ai_time >= AI_INFERENCE_PERIOD:
                last_ai_time = current_time

                cam_pos, cam_target, cam_up = gimbal.update_camera_pose()
                view_matrix = p.computeViewMatrix(cameraEyePosition=cam_pos, cameraTargetPosition=cam_target, cameraUpVector=cam_up)
                proj_matrix = p.computeProjectionMatrixFOV(fov=68, aspect=1.0, nearVal=0.1, farVal=100.0)

                width, height, rgbImg, _, _ = p.getCameraImage(width=160, height=120, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
                frame = np.reshape(rgbImg, (height, width, 4)).astype(np.uint8)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

                has_target, pan_adj, tilt_adj, processed_frame = tracker.process_frame(frame, dead_zone=current_deadzone)
                error_x = getattr(tracker, 'last_error_x', 0.0)
                error_y = getattr(tracker, 'last_error_y', 0.0)
                current_speed = getattr(tracker, 'current_speed', 0.0)
                current_conf = getattr(tracker, 'confidence', 0.0)

            # 2. 모터 제어 주기
            if current_time - last_control_time >= CONTROL_PERIOD:
                last_control_time = current_time
                episode_step += 1

                # 💡 [추가] 제어 주기마다 트래커의 최신 오차 값을 실시간 동기화
                error_x = getattr(tracker, 'last_error_x', 0.0)
                error_y = getattr(tracker, 'last_error_y', 0.0)

                if USE_GUI:
                    raw_kp = cv2.getTrackbarPos("KP x10000", debug_window_name)
                    raw_ki = cv2.getTrackbarPos("KI x10000", debug_window_name)
                    raw_kd = cv2.getTrackbarPos("KD x100000", debug_window_name)
                    current_kp, current_ki, current_kd = raw_kp / 10000.0, raw_ki / 10000.0, raw_kd / 100000.0

                tracker.pan_pid.kp, tracker.pan_pid.ki, tracker.pan_pid.kd = current_kp, current_ki, current_kd
                tracker.tilt_pid.kp, tracker.tilt_pid.ki, tracker.tilt_pid.kd = current_kp, current_ki, current_kd

                if auto_tracking and has_target:
                    desired_pan = gimbal.pan_angle - pan_adj
                    desired_tilt = gimbal.tilt_angle - tilt_adj
                else:
                    desired_pan = gimbal.pan_angle
                    desired_tilt = gimbal.tilt_angle
                    pan_adj, tilt_adj = 0.0, 0.0

                pan_diff = np.clip(desired_pan - prev_pan_cmd, -MAX_STEP_RAD, MAX_STEP_RAD)
                tilt_diff = np.clip(desired_tilt - prev_tilt_cmd, -MAX_STEP_RAD, MAX_STEP_RAD)

                gimbal.pan_angle = prev_pan_cmd + pan_diff
                gimbal.tilt_angle = prev_tilt_cmd + tilt_diff
                prev_pan_cmd, prev_tilt_cmd = gimbal.pan_angle, gimbal.tilt_angle

                gimbal.set_target_angles(gimbal.pan_angle, gimbal.tilt_angle)
                hit_target, laser_line_id = gimbal.update_laser_beam(targetId, laser_line_id)

                # if has_target and hit_target:
                #     lock_stable_count += 1
                if (
                        has_target
                        and abs(error_x) <= current_deadzone
                        and abs(error_y) <= current_deadzone
                        and hit_target
                    ):
                    lock_stable_count += 1
                else:
                    lock_stable_count = 0

                is_locked_on = (lock_stable_count >= 25)  
                
                #print(f"episode_step : {episode_step}")
                # 에피소드 종료 조건 (성공 또는 타임아웃)
                if is_locked_on or episode_step >= MAX_EPISODE_STEPS:
                    
                    success = 1 if is_locked_on else 0
                    time_to_lock = episode_step * 0.02
                    hit_ratio = 100.0 if success else 0.0  

                    csv_writer.writerow([trial_id, current_deadzone, target_size_scale, target_speed_val, target_distance, success, time_to_lock, hit_ratio])
                    csv_file.flush()
                    print(f"📊 [Trial {trial_id}/{max_total_trials}] Deadzone: {current_deadzone}px | Success: {success} | Time: {time_to_lock:.2f}s")

                    # 전체 실험 종료 체크
                    if trial_id >= max_total_trials:
                        print("🎉 모든 반복 실험이 완료되었습니다!")
                        running = False
                        break

                    # 다음 트라이얼 준비
                    trial_id += 1
                    episode_step = 0
                    lock_stable_count = 0

                    # 🔄 [추가] 짐벌 각도 초기화 (정면으로 리셋)
                    gimbal.pan_angle = 0.0
                    gimbal.tilt_angle = 0.0
                    prev_pan_cmd = 0.0
                    prev_tilt_cmd = 0.0
                    gimbal.set_target_angles(0.0, 0.0)

                    # 🔄 [추가] 비전 트래커(PID 제어기)의 적분/이전 오차 초기화
                    # (PID 내부의 누적 오차나 미분 값이 남아있는 것을 방지)
                    if hasattr(tracker, 'pan_pid'):
                        tracker.pan_pid.reset()
                    if hasattr(tracker, 'tilt_pid'):
                        tracker.tilt_pid.reset()
                    
                    # 매 트라이얼마다 새로운 랜덤 시작 각도 부여
                    theta = random.uniform(-np.pi/12, np.pi/12)

                    # 파라미터 순환 로직
                    if EXPERIMENT_MODE == "DEAD_ZONE":
                        current_deadzone = deadzone_list[(trial_id - 1) % len(deadzone_list)]

                    elif EXPERIMENT_MODE == "FULL_SWEEP":
                        idx = trial_id - 1
                        current_deadzone = deadzone_list[idx % len(deadzone_list)]
                        target_size_scale = target_size_options[(idx // len(deadzone_list)) % len(target_size_options)]
                        target_speed_val = target_speed_options[(idx // (len(deadzone_list) * len(target_size_options))) % len(target_speed_options)]
                        target_distance = distance_options[(idx // (len(deadzone_list) * len(target_size_options) * len(target_speed_options))) % len(distance_options)]

                    # 타겟 재생성 및 새로운 원주상 위치 반영
                    p.removeBody(targetId)
                    target_radius = target_size_scale
                    target_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=target_radius)
                    target_visual = p.createVisualShape(p.GEOM_SPHERE, radius=target_radius, rgbaColor=[1, 0, 0, 1])
                    
                    initial_target_x = target_distance * np.cos(theta)
                    initial_target_y = target_distance * np.sin(theta)
                    initial_target_pos = [initial_target_x, initial_target_y, 0.5]

                    targetId = p.createMultiBody(
                        baseMass=0.0,
                        baseCollisionShapeIndex=target_collision,
                        baseVisualShapeIndex=target_visual,
                        basePosition=initial_target_pos,
                        baseOrientation=[0, 0, 0, 1],
                    )

                if USE_GUI and processed_frame is not None:
                    display_frame = processed_frame.copy()
                    cv2.imshow(cam_window_name, display_frame)

                    debug_board = np.zeros((320, 480, 3), dtype=np.uint8)
                    lock_status_str = "🔥 [LOCKED ON & STABLE!]" if is_locked_on else ("⚡ [HIT]" if hit_target else "❌ [SEARCHING...]")
                    lock_color = (0, 255, 0) if is_locked_on else ((0, 255, 255) if hit_target else (0, 0, 255))

                    texts = [
                        f"[Experiment Mode: {EXPERIMENT_MODE}]",
                        f"  Trial  : {trial_id} / {max_total_trials} | Step: {episode_step}",
                        f"  Deadzone: {current_deadzone} px | Dist: {target_distance}m",
                        f"  Target Size: {target_size_scale} | Speed: {target_speed_val}",
                        f"  LOCK   : {lock_status_str}",
                        f"  Error X: {error_x:.2f} px | Error Y: {error_y:.2f} px"
                    ]

                    y_offset = 25
                    for t in texts:
                        color = lock_color if "LOCK   :" in t else (255, 255, 255)
                        cv2.putText(debug_board, t, (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
                        y_offset += 24

                    cv2.imshow(debug_window_name, debug_board)

            if USE_GUI and (cv2.waitKey(1) & 0xFF == ord("q")):
                break

            # if not USE_GUI:
            #     time.sleep(0.001)

    except p.error:
        pass
    finally:
        csv_file.close()
        if USE_GUI:
            cv2.destroyAllWindows()
        p.disconnect()

if __name__ == "__main__":
    main()