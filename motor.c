#include "motor.h"

// 간단한 난수 생성 함수
uint32_t simple_random(uint32_t min, uint32_t max) {
    static uint32_t seed = 12345;
    seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
    return min + (seed % (max - min + 1));
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
float PID_Compute(PID_Controller *pid, float current_val) {
    pid->current = current_val;
    float error = pid->target - pid->current;
    
    // 비례(P) 항
    float p_out = pid->kp * error;
    
    // 적분(I) 항
    pid->integral += error;
    float i_out = pid->ki * pid->integral;
    
    // 미분(D) 항
    float d_out = pid->kd * (error - pid->prev_error);
    pid->prev_error = error;
    
    // 최종 제어 출력
    float output = p_out + i_out + d_out;
    
    return output;
}

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
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)pan_pid->current);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)tilt_pid->current);
}