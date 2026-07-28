#include "motor.h"

// 간단한 난수 생성 함수
uint32_t simple_random(uint32_t min, uint32_t max) {
    static uint32_t seed = 12345;
    seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
    return min + (seed % (max - min + 1));
}

// 1. 더미 좌표 수신 함수 (나중에 실제 카메라 데이터로 대체될 부분)
void Get_Dummy_Target_Coords(float *target_x, float *target_y) {
    // 테스트를 위해 95x95 안에서 움직이는 가상의 타겟 좌표 생성 (예: 원 형태로 움직임)
    uint32_t tick = HAL_GetTick();
    *target_x = IMG_CENTER_X + 40.0f * cosf((float)tick / 300.0f);
    *target_y = IMG_CENTER_Y + 40.0f * sinf((float)tick / 300.0f);
}

// 오차 기반 PID 연산 및 CCR 값 갱신 함수
// 오차 기반 PID 연산 및 CCR 값 갱신 함수
// void Motor_PID_Process_With_Error(PID_Controller *pan, PID_Controller *tilt, float target_x, float target_y, TIM_HandleTypeDef *htim) {
//     // 1. 중앙과의 픽셀 오차 계산 (-48 ~ +48 범위)
//     float error_x = target_x - IMG_CENTER_X; 
//     float error_y = target_y - IMG_CENTER_Y; 
    
//     // 2. Pan 축 PID 직접 연산 (current를 덮어쓰지 않고 오차를 바로 활용)
//     pan->target = error_x; // 참고용 저장
//     float p_pan = pan->kp * error_x;
//     pan->integral += error_x;
//     float i_pan = pan->ki * pan->integral;
//     float d_pan = pan->kd * (error_x - pan->prev_error);
//     pan->prev_error = error_x;
    
//     float pan_output = p_pan + i_pan + d_pan;
//     pan->current += pan_output; // 현재 모터 CCR에 오차에 따른 제어량 가감

//     // 3. Tilt 축 PID 직접 연산
//     tilt->target = error_y;
//     float p_tilt = tilt->kp * error_y;
//     tilt->integral += error_y;
//     float i_tilt = tilt->ki * tilt->integral;
//     float d_tilt = tilt->kd * (error_y - tilt->prev_error);
//     tilt->prev_error = error_y;
    
//     float tilt_output = p_tilt + i_tilt + d_tilt;
//     tilt->current += tilt_output;

//     // 4. 서보모터 PWM 범위 제한 (500 ~ 1250)
//     if (pan->current > ANGLE_MAX) pan->current = ANGLE_MAX;
//     if (pan->current < ANGLE_MIN) pan->current = ANGLE_MIN;

//     if (tilt->current > ANGLE_MAX) tilt->current = ANGLE_MAX;
//     if (tilt->current < ANGLE_MIN) tilt->current = ANGLE_MIN;

//     // 5. 최종 CCR 값 반영
//     __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)pan->current);
//     __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)tilt->current);

    
// }

void Motor_PID_Process_With_Error(PID_Controller *pan, PID_Controller *tilt, float target_x, float target_y, TIM_HandleTypeDef *htim) {
    // 1. 중앙 좌표와의 픽셀 오차 계산 (화면 중앙 맞추기)
    float error_x = target_x - IMG_CENTER_X; 
    float error_y = -(target_y - IMG_CENTER_Y); // Tilt 축이 반전되어 있어서!
    
    // 2. Pan 축 PID 연산 수행 (Delta CCR 도출)
    pan->target = target_x; // 디버깅/참고용 타겟 기록
    float pan_output = PID_Compute(pan, error_x);
    pan->current += pan_output; // 기존 CCR 위치에 Delta 값 누적

    // 3. Tilt 축 PID 연산 수행 (Delta CCR 도출)
    tilt->target = target_y;
    float tilt_output = PID_Compute(tilt, error_y);
    tilt->current += tilt_output; // 기존 CCR 위치에 Delta 값 누적

    // 4. 서보모터 PWM 안전 범위 제한 (Clamping: 700 ~ 2400)
    if (pan->current > ANGLE_MAX) pan->current = ANGLE_MAX;
    if (pan->current < ANGLE_MIN) pan->current = ANGLE_MIN;

    if (tilt->current > ANGLE_MAX) tilt->current = ANGLE_MAX;
    if (tilt->current < ANGLE_MIN) tilt->current = ANGLE_MIN;

    // 5. 최종 CCR 값을 타이머 채널에 반영
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)pan->current);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)tilt->current);
}

// PID 제어기 초기화 함수
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float initial_val) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = initial_val;
    pid->current = initial_val;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// PID 연산 함수

float PID_Compute(PID_Controller *pid, float error) {
    // 1. 비례(P) 항
    float p_out = pid->kp * error;
    
    // 2. 적분(I) 항 (Anti-Windup 적용 포함)
    pid->integral += error;
    float i_out = pid->ki * pid->integral;
    
    // 3. 미분(D) 항
    float d_out = pid->kd * (error - pid->prev_error);
    pid->prev_error = error;
    
    // 4. 최종 제어 출력 (Delta CCR)
    float output = p_out + i_out + d_out;
    
    return output;
}

// float PID_Compute(PID_Controller *pid, float current_val) {
//     pid->current = current_val;
//     float error = pid->target - pid->current;
    
//     // 비례(P) 항
//     float p_out = pid->kp * error;
    
//     // 적분(I) 항
//     pid->integral += error;
//     float i_out = pid->ki * pid->integral;
    
//     // 미분(D) 항
//     float d_out = pid->kd * (error - pid->prev_error);
//     pid->prev_error = error;
    
//     // 최종 제어 출력
//     float output = p_out + i_out + d_out;
    
//     return output;
// }

// Pan/Tilt 모터의 PID 연산 및 PWM 갱신 통합 함수
void Motor_PID_Process(PID_Controller *pan_pid, PID_Controller *tilt_pid, TIM_HandleTypeDef *htim) {
    // 1. Pan 축 PID 연산 및 범위 제한
    float pan_output = PID_Compute(pan_pid, pan_pid->current);
    pan_pid->current += pan_output;
    if (pan_pid->current > ANGLE_MAX) pan_pid->current = ANGLE_MAX;
    if (pan_pid->current < ANGLE_MIN) pan_pid->current = ANGLE_MIN;

    // 2. Tilt 축 PID 연산 및 범위 제한
    float tilt_output = PID_Compute(tilt_pid, tilt_pid->current);
    tilt_pid->current += tilt_output;
    if (tilt_pid->current > ANGLE_MAX) tilt_pid->current = ANGLE_MAX;
    if (tilt_pid->current < ANGLE_MIN) tilt_pid->current = ANGLE_MIN;

    // 3. 서보모터 PWM 갱신 (CCR 값 반영)
    //__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)pan_pid->current);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)tilt_pid->current);
}