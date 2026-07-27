#pragma once

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

// 서보모터 각도/CCR 범위 정의
#define ANGLE_MIN   500   // -90도에 해당하는 CCR 값 (0.5ms)
#define ANGLE_MID   1500  //  0도(중앙)에 해당하는 CCR 값 (1.5ms)
#define ANGLE_MAX   2500  // +90도에 해당하는 CCR 값 (2.5ms)

// PID 제어기 구조체 정의
typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    float current;
    float integral;
    float prev_error;
} PID_Controller;

// 함수 프로토타입 선언
uint32_t simple_random(uint32_t min, uint32_t max);
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float initial_val);
float PID_Compute(PID_Controller *pid, float current_val);
void Motor_PID_Process(PID_Controller *pan_pid, PID_Controller *tilt_pid, TIM_HandleTypeDef *htim);

#endif /* MOTOR_H */

