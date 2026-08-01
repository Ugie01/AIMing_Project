import pybullet as p
import numpy as np

class GimbalSystem:
    def __init__(self):
        link_Masses = [0.0, 0.03]  
        link_CollisionShapeIndices = [
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.0165, 0.0165, 0.015]),
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.0165, 0.0165, 0.0175]),
        ]
        link_VisualShapeIndices = [
            p.createVisualShape(p.GEOM_BOX, halfExtents=[0.0165, 0.0165, 0.015], rgbaColor=[0.2, 0.2, 0.2, 1]),
            p.createVisualShape(p.GEOM_BOX, halfExtents=[0.0165, 0.0165, 0.0175], rgbaColor=[0.3, 0.3, 0.3, 1]),
        ]
        link_Positions = [[0, 0, 0.015], [0, 0, 0.03]]
        link_Orientations = [[0, 0, 0, 1], [0, 0, 0, 1]]
        link_JointTypes = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
        link_JointAxis = [[0, 0, 1], [1, 0, 0]]  
        linkParentIndices = [0, 1]

        self.gimbalId = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.0165, 0.0165, 0.015]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.0165, 0.0165, 0.015], rgbaColor=[0.1, 0.1, 0.1, 1]),
            basePosition=[0, 0, 0.015],
            baseOrientation=[0, 0, 0, 1],
            linkMasses=link_Masses,
            linkCollisionShapeIndices=link_CollisionShapeIndices,
            linkVisualShapeIndices=link_VisualShapeIndices,
            linkPositions=link_Positions,
            linkOrientations=link_Orientations,
            linkInertialFramePositions=[[0, 0, 0], [0, 0, 0]],
            linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
            linkParentIndices=linkParentIndices,
            linkJointTypes=link_JointTypes,
            linkJointAxis=link_JointAxis,
        )

        p.changeDynamics(self.gimbalId, 0, jointDamping=0.02)
        p.changeDynamics(self.gimbalId, 1, jointDamping=0.02)

        # 빨간색 네모 크기 정의 (halfExtents=[0.005, 0.01, 0.01] -> 전체 크기 폭 1cm, 길이 2cm, 높이 2cm)
        self.cam_box_half_extents = [0.005, 0.01, 0.01]
        
        cam_cube_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=self.cam_box_half_extents, rgbaColor=[1, 0, 0, 1])
        self.camCubeId = p.createMultiBody(
            baseMass=0.0,
            baseVisualShapeIndex=cam_cube_visual,
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],
        )

        self.pan_angle = 0.0
        self.tilt_angle = 0.0

    def update_camera_pose(self):
        link_state = p.getLinkState(self.gimbalId, 1)
        link_pos = link_state[4]
        link_orn = link_state[5]

        rot_matrix = p.getMatrixFromQuaternion(link_orn)
        rot_matrix_np = np.array(list(rot_matrix)).reshape(3, 3)

        # 짐벌 틸트 링크 기준 '진짜 정면' 방향 벡터
        forward_dir = np.dot(rot_matrix_np, np.array([0.0, 1.0, 0.0]))

        # 빨간 네모의 중심 위치 계산
        cube_pos = np.array(link_pos) + np.dot(rot_matrix_np, np.array([0.0, 0.03, 0.015]))
        
        # [핵심 수정] 빨간 네모가 짐벌의 회전을 올바르게 따라가도록 설정하되 오인식 방지
        p.resetBasePositionAndOrientation(self.camCubeId, cube_pos.tolist(), link_orn)

        # 카메라의 눈 위치를 빨간 네모의 바깥 정면 표면으로 정확히 배치
        cam_pos = cube_pos + forward_dir * self.cam_box_half_extents[1]
        
        # 카메라가 바라보는 타겟 지점과 상향 벡터 설정
        cam_target = cam_pos + forward_dir
        cam_up = np.dot(rot_matrix_np, np.array([0.0, 0.0, 1.0]))

        return cam_pos, cam_target, cam_up

    def set_target_angles(self, pan, tilt):
        self.tilt_angle = max(-0.78, min(0.78, tilt))
        p.setJointMotorControl2(self.gimbalId, 0, p.POSITION_CONTROL, targetPosition=self.pan_angle, force=2.0)
        p.setJointMotorControl2(self.gimbalId, 1, p.POSITION_CONTROL, targetPosition=self.tilt_angle, force=2.0)

    def update_laser_beam(self, target_id, current_line_id):
        # 1. 빨간 네모의 현재 위치와 회전값(오리엔테이션)을 직접 가져옴
        cube_pos, cube_orn = p.getBasePositionAndOrientation(self.camCubeId)
        
        # 2. 회전값을 회전 행렬로 변환
        rot_matrix = p.getMatrixFromQuaternion(cube_orn)
        rot_matrix_np = np.array(list(rot_matrix)).reshape(3, 3)
        
        # 3. 빨간 네모의 로컬 기준 정면 방향 (Y축) 벡터 추출
        forward_dir = np.dot(rot_matrix_np, np.array([0.0, 1.0, 0.0]))
        
        # 4. 레이저 시작점: 빨간 네모의 표면 바깥쪽으로 수직 밀착 배치
        ray_start = np.array(cube_pos) + forward_dir * self.cam_box_half_extents[1]
        
        # 5. 전방 방향 그대로 50미터 뻗어나가기
        max_laser_length = 50.0
        ray_end = ray_start + forward_dir * max_laser_length
        
        # 6. 충돌 감지 수행
        ray_results = p.rayTest(ray_start.tolist(), ray_end.tolist())
        hit_target = False
        actual_end = ray_end  # 기본값: 허공일 때 50m 끝점

        if ray_results and len(ray_results) > 0:
            for hit_result in ray_results:
                hit_object_id = hit_result[0]
                
                # 허공(-1)은 무시
                if hit_object_id == -1:
                    continue
                
                # 짐벌 자신이나 카메라 자체와의 충돌이 아닐 때
                if hit_object_id != self.gimbalId and hit_object_id != self.camCubeId:
                    actual_end = np.array(hit_result[3])
                    # 맞은 물체가 우리가 지정한 초록색 타겟(targetId)인지 확인
                    if hit_object_id == target_id:
                        hit_target = True
                    break

        # 타겟 명중 시 빨간색([1, 0, 0]), 평소엔 초록색([0, 1, 0])
        laser_color = [1, 0, 0] if hit_target else [0, 1, 0]

        new_line_id = p.addUserDebugLine(
            ray_start.tolist(), 
            actual_end.tolist(), 
            laser_color, 
            lineWidth=3, 
            lifeTime=0, 
            replaceItemUniqueId=current_line_id
        )

        return hit_target, new_line_id