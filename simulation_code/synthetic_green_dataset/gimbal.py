import pybullet as p
import numpy as np

class GimbalSystem:
    def __init__(self):
        # 1. 짐벌 링크 구조 정의 (실제 규격 33x33x65mm 반영)
        # Link 0: Pan 바디/모터 부근 (하단 ~30mm)
        # Link 1: Tilt 바디 및 상단 브라켓 (상단 ~35mm)
        link_Masses = [0.0, 0.03]  # 상단 틸트 구조물에 요청하신 30g(0.03kg) 부여
        
        link_CollisionShapeIndices = [
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.0165, 0.0165, 0.015]),
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.0165, 0.0165, 0.0175]),
        ]
        
        link_VisualShapeIndices = [
            p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.0165, 0.0165, 0.015],
                rgbaColor=[0.2, 0.2, 0.2, 1], # 베이스/팬 부위
            ),
            p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.0165, 0.0165, 0.0175],
                rgbaColor=[0.3, 0.3, 0.3, 1], # 틸트 부위
            ),
        ]

        # 위치 및 조인트 축 설정 (높이 총합 ~65mm 구현)
        link_Positions = [[0, 0, 0.015], [0, 0, 0.03]]
        link_Orientations = [[0, 0, 0, 1], [0, 0, 0, 1]]
        link_JointTypes = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
        link_JointAxis = [[0, 0, 1], [1, 0, 0]]  # Z축(Pan), X축(Tilt)
        linkParentIndices = [0, 1]

        # 짐벌 본체 생성
        self.gimbalId = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[0.0165, 0.0165, 0.015]
            ),
            baseVisualShapeIndex=p.createVisualShape(
                p.GEOM_BOX, halfExtents=[0.0165, 0.0165, 0.015], rgbaColor=[0.1, 0.1, 0.1, 1]
            ),
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

        # --- [Sim-to-Real 강화] 조인트 마찰력(Joint Damping) 부여 ---
        # 플라스틱 기어 서보모터의 뻑뻑함과 감쇠를 모사하기 위해 마찰력 설정
        p.changeDynamics(self.gimbalId, 0, jointDamping=0.02)
        p.changeDynamics(self.gimbalId, 1, jointDamping=0.02)

        # 2. 카메라 위치 시각화용 큐브 생성 (약 45mm 높이에서 전방으로 30mm 튀어나온 구조)
        cam_cube_visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.005, 0.01, 0.01], rgbaColor=[1, 0, 0, 1]
        )
        self.camCubeId = p.createMultiBody(
            baseMass=0.0,
            baseVisualShapeIndex=cam_cube_visual,
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],
        )

        self.pan_angle = 0.0
        self.tilt_angle = 0.0

    def update_camera_pose(self):
        """틸트 링크의 움직임에 맞춰 전방 30mm 돌출된 카메라 시각화 큐브와 뷰 행렬을 동기화"""
        link_state = p.getLinkState(self.gimbalId, 1)
        link_pos = link_state[4]
        link_orn = link_state[5]

        rot_matrix = p.getMatrixFromQuaternion(link_orn)
        rot_matrix_np = np.array(list(rot_matrix)).reshape(3, 3)

        # 틸트 중심(약 45mm 높이) 기준에서 전방(Y축 방향)으로 30mm(0.03m) 튀어나온 위치 계산
        cam_pos = np.array(link_pos) + np.dot(rot_matrix_np, np.array([0.0, 0.03, 0.015]))
        p.resetBasePositionAndOrientation(self.camCubeId, cam_pos, link_orn)

        cam_target = cam_pos + np.dot(rot_matrix_np, np.array([0.0, 1.0, 0.0]))
        cam_up = np.dot(rot_matrix_np, np.array([0.0, 0.0, 1.0]))

        return cam_pos, cam_target, cam_up

    def set_target_angles(self, pan, tilt):
        """모터에 목표 각도 명령 전달"""
        self.tilt_angle = max(-0.78, min(0.78, tilt))

        p.setJointMotorControl2(
            self.gimbalId,
            0,
            p.POSITION_CONTROL,
            targetPosition=self.pan_angle,
            force=2.0,  # SG90 소형 모터의 약한 토크 반영
        )
        p.setJointMotorControl2(
            self.gimbalId,
            1,
            p.POSITION_CONTROL,
            targetPosition=self.tilt_angle,
            force=2.0,
        )