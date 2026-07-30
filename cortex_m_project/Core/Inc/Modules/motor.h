#ifndef MOTOR_H_
#define MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "joystick.h"

// ==============================================================================
// 매크로 및 상수 정의
// ==============================================================================

// 서보모터 PWM CCR 범위
#define ANGLE_MIN   700     // 최소 각도 (0.700ms)
#define ANGLE_MID   1500    // 중앙 각도 (1.500ms)
#define ANGLE_MAX   2400    // 최대 각도 (2.400ms)

// 카메라 이미지 중심 좌표 (96x96 해상도 기준)
#define IMG_CENTER_X 48.0f
#define IMG_CENTER_Y 48.0f

// MANUAL 모드 조이스틱 최대 이동 속도 (CCR/초)
#define MANUAL_SPEED_PER_SEC   400.0f

// 모터마다 물리적 마찰력 떄문에 최소 PWM 데드존
#define PAN_MIN_POWER 50.0f

// 비전 데이터 크롭 영역의 중심점 (96x96 기준)
#define VISION_CENTER_X 48.0f
#define VISION_CENTER_Y 48.0f

// 360도 서보모터 정지 PWM 값
#define PAN_STOP_PWM 1500.0f

// 기존 카메라 중심점 (48, 48)
#define CAMERA_CENTER_X 48.0f
#define CAMERA_CENTER_Y 48.0f

// 레이저와 카메라 사이의 물리적 위치 차이로 인한 픽셀 오프셋
// 예: 레이저가 카메라보다 오른쪽에 있고 위쪽에 있다면 오프셋 설정
#define LASER_OFFSET_X  0.0f  // (픽셀 단위 오프셋)
#define LASER_OFFSET_Y  +5.0f

// 최종 PID 목표점
#define LASER_TARGET_X (CAMERA_CENTER_X + LASER_OFFSET_X)
#define LASER_TARGET_Y (CAMERA_CENTER_Y + LASER_OFFSET_Y)

// ==============================================================================
// 🎯 [하이퍼파라미터] 레이저 추적 및 순찰(Patrol) 설정 영역
// ==============================================================================
// 1. 대기/순찰 (IDLE) 설정
#define PATROL_STEP_ANGLE       1.0f   // [속도] 순찰 시 1프레임당 Pan 이동량 (서보 CCR 기준, 높을수록 빠름)
#define PATROL_PAN_MIN          700.0f // [범위] 좌측 최대 순찰 각도
#define PATROL_PAN_MAX          2400.0f // [범위] 우측 최대 순찰 각도

// 2. 발사 (LOCKON) 설정
#define LOCKON_ERROR_MARGIN     10.0f    // [정밀도] 타겟 중심과 레이저 타겟 간의 최대 허용 오차 (픽셀 단위)
#define LOCKON_MAINTAIN_COUNT   3       // [시간] 오차 범위 내에 몇 프레임 연속 머물러야 레이저를 발사할지 (3프레임 = 약 0.4초)

// ==============================================================================
// 구조체 정의
// ==============================================================================

// PID 제어기 상태를 저장하는 구조체
typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    float current;
    float integral;
    float prev_error;
} PID_Controller;

// ==============================================================================
// 외부 공개 API 함수
// ==============================================================================

// [수동 제어 (Manual)] ---------------------------------------------------------
// 수동 제어 모드에서 조이스틱 입력값에 따라 모터 각도 갱신
void Motor_ManualProcess(const JoystickInput_t *joy, TIM_HandleTypeDef *htim, uint32_t dt_ms);
// 수동 제어의 현재 Pan/Tilt CCR 값 강제 설정 (AUTO -> MANUAL 전환 시 충격 방지)
void Motor_ManualSetPosition(float pan_ccr, float tilt_ccr);
// 수동 제어의 현재 Pan/Tilt CCR 값 읽기 (MANUAL -> AUTO 전환 시 초기값 제공)
void Motor_ManualGetPosition(float *pan_ccr, float *tilt_ccr);

// [자동 제어 (PID)] ------------------------------------------------------------
// PID 제어기 초기화 (계수 및 초기값 설정)
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float initial_val);
// 현재 값과 목표 값을 바탕으로 PID 제어량 연산
float PID_Compute(PID_Controller *pid, float current_val);
// 계산된 PID 값을 바탕으로 Pan/Tilt 모터 PWM 즉시 갱신
void Motor_PID_Process(PID_Controller *pan_pid, PID_Controller *tilt_pid, TIM_HandleTypeDef *htim);

// [선형 매핑 제어] -------------------------------------------------------------
// 타겟 좌표(0~95)를 서보 범위(ANGLE_MIN~MAX)로 직접 선형 매핑하여 구동
void Motor_PID_Process_With_Error(PID_Controller *pan, PID_Controller *tilt, float target_x, float target_y, TIM_HandleTypeDef *htim);

// [더미 테스트용] --------------------------------------------------------------
// 테스트를 위한 원형 이동 가상 타겟 좌표 생성기
void Get_Dummy_Target_Coords(float *target_x, float *target_y);
// 간단한 난수 생성기
uint32_t simple_random(uint32_t min, uint32_t max);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H_ */
