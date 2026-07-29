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

// 비전 데이터 크롭 영역의 중심점 (96x96 기준)
#define VISION_CENTER_X 48.0f
#define VISION_CENTER_Y 48.0f

// 360도 서보모터 정지 PWM 값
#define PAN_STOP_PWM 1500.0f

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
