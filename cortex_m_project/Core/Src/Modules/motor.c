#include "motor.h"
#include <math.h>

// ==============================================================================
// 모듈 정적 상태 변수 모음
// ==============================================================================

// 수동 모드(Manual)에서 현재 유지하고 있는 Pan/Tilt 모터의 각도 (중앙에서 시작)
static float manual_pan = (float) ANGLE_MID;
static float manual_tilt = (float) ANGLE_MID;


// ==============================================================================
// 내부 데이터 처리 함수
// ==============================================================================

// 입력값을 모터의 물리적 한계 각도(MIN ~ MAX) 내로 클램핑(제한)
static float Motor_ClampAngle(float value) {
    if (value > (float) ANGLE_MAX)
        return (float) ANGLE_MAX;
    if (value < (float) ANGLE_MIN)
        return (float) ANGLE_MIN;
    return value;
}

// ==============================================================================
// 수동(Manual) 모터 제어 API
// ==============================================================================

// 외부에서 수동 모터 위치를 강제로 설정
void Motor_ManualSetPosition(float pan_ccr, float tilt_ccr) {
    manual_pan = Motor_ClampAngle(pan_ccr);
    manual_tilt = Motor_ClampAngle(tilt_ccr);
}

// 현재 수동 모터 위치를 외부로 반환
void Motor_ManualGetPosition(float *pan_ccr, float *tilt_ccr) {
    if (pan_ccr != NULL)
        *pan_ccr = manual_pan;
    if (tilt_ccr != NULL)
        *tilt_ccr = manual_tilt;
}

// 조이스틱 입력값을 기반으로 수동 모터 위치 계산 및 타이머 PWM 반영
void Motor_ManualProcess(const JoystickInput_t *joy, TIM_HandleTypeDef *htim, uint32_t dt_ms) {
    if ((joy == NULL) || (htim == NULL) || (dt_ms == 0U))
        return;

    // 호출 주기(dt_ms)에 비례한 최대 이동 가능량(Step) 계산
    float step = MANUAL_SPEED_PER_SEC * ((float) dt_ms / 1000.0f);

    // 조이스틱 축 값을 -1.0 ~ +1.0 비율로 변환 (데드존은 Joystick_Read에서 기처리됨)
    float ratio_x = (float) joy->x / (float) JOYSTICK_AXIS_MAX;
    float ratio_y = (float) joy->y / (float) JOYSTICK_AXIS_MAX;

    // 현재 각도에 증분을 누적하고 클램핑 적용
    manual_pan = Motor_ClampAngle(manual_pan + (ratio_x * step));
    manual_tilt = Motor_ClampAngle(manual_tilt + (ratio_y * step));

    manual_pan = PAN_STOP_PWM + (ratio_x * 400.0f);
    manual_pan = Motor_ClampAngle(manual_pan);

    // 타이머 CCR(PWM) 레지스터 갱신
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t ) manual_pan);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t ) manual_tilt);
}

// ==============================================================================
// 자동(PID) 모터 제어 API
// ==============================================================================

// PID 제어기 구조체 초기화
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float initial_val) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = initial_val;
    pid->current = initial_val;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// 현재값 대비 오차를 구하여 비례(P), 적분(I), 미분(D) 제어량 산출
float PID_Compute(PID_Controller *pid, float current_val) {
    pid->current = current_val;
    float error = pid->target - pid->current;

    float p_out = pid->kp * error;            // P: 오차 비례

    pid->integral += error;
    float i_out = pid->ki * pid->integral;    // I: 오차 누적 (정상 상태 오차 제거)

    float d_out = pid->kd * (error - pid->prev_error); // D: 오차 변화율 (급격한 변화 방지)
    pid->prev_error = error;

    return p_out + i_out + d_out;
}

// Pan/Tilt 두 축에 대한 PID 연산 수행 및 타이머 PWM 반영
void Motor_PID_Process(PID_Controller *pan_pid, PID_Controller *tilt_pid, TIM_HandleTypeDef *htim) {
    // Pan 축 PID 연산 및 범위 제한 적용
    float pan_output = PID_Compute(pan_pid, pan_pid->current);
    pan_pid->current += pan_output;
    if (pan_pid->current > ANGLE_MAX) pan_pid->current = ANGLE_MAX;
    if (pan_pid->current < ANGLE_MIN) pan_pid->current = ANGLE_MIN;

    // Tilt 축 PID 연산 및 범위 제한 적용
    float tilt_output = PID_Compute(tilt_pid, tilt_pid->current);
    tilt_pid->current += tilt_output;
    if (tilt_pid->current > ANGLE_MAX) tilt_pid->current = ANGLE_MAX;
    if (tilt_pid->current < ANGLE_MIN) tilt_pid->current = ANGLE_MIN;

    // 서보모터 PWM 갱신
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t )pan_pid->current); // 주석 해제하여 정상 작동하도록 복원
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)tilt_pid->current);
}

// 화면 내 대상 위치를 기반으로 PID 연산을 수행하는 함수
void Motor_PID_Process_With_Error(PID_Controller *pan, PID_Controller *tilt, float target_x, float target_y, TIM_HandleTypeDef *htim) {

    // ==========================================
    // 1. Pan 축 (360도 연속회전 - 속도 제어)
    // ==========================================
    pan->target = VISION_CENTER_X;

    float pan_speed_offset = PID_Compute(pan, target_x);

    // (값이 작아지면 오른쪽, 커지면 왼쪽) 스펙에 맞춤
    float pan_pwm = PAN_STOP_PWM - pan_speed_offset;
    pan_pwm = Motor_ClampAngle(pan_pwm);

    // ==========================================
    // 2. Tilt 축 (180도 위치제어)
    // ==========================================
    tilt->target = VISION_CENTER_Y;

    float tilt_angle_delta = PID_Compute(tilt, target_y);

    // 🚨 여기서 누적 변수는 무조건 'manual_tilt'를 사용해야 합니다.
    // (값이 2300으로 커지면 바닥, 700으로 작아지면 하늘) 스펙에 맞춤
    manual_tilt -= tilt_angle_delta;
    manual_tilt = Motor_ClampAngle(manual_tilt);

    // ==========================================
    // 3. 서보모터 PWM 갱신
    // ==========================================
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t )pan_pwm);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t )manual_tilt);
}

// ==============================================================================
// 더미 데이터 테스트용 API
// ==============================================================================

// 간단한 자체 난수 생성 함수 (LCG 방식)
uint32_t simple_random(uint32_t min, uint32_t max) {
    static uint32_t seed = 12345;
    seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
    return min + (seed % (max - min + 1));
}

// 모터 추적 로직 테스트를 위해 (48,48) 중심의 원 궤도를 그리는 가짜 타겟 좌표 생성
void Get_Dummy_Target_Coords(float *target_x, float *target_y) {
    uint32_t tick = HAL_GetTick();
    *target_x = IMG_CENTER_X + 25.0f * cosf((float) tick / 300.0f);
    *target_y = IMG_CENTER_Y + 25.0f * sinf((float) tick / 300.0f);
}
