# STM32 Pan-Tilt Servo Motor PID Control System

> **Sim-to-Real 연구 및 개발을 위한 PyBullet 시뮬레이션 연동 및 STM32 기반 하드웨어 서보모터 PID 제어 프로젝트**입니다.

---

## 📌 1. 프로젝트 개요
이 프로젝트는 컴퓨터 비전(예: TFLite 객체 감지)을 활용하여 초록색 타겟을 실시간으로 추적하는 **2축(Pan/Tilt) 짐벌 시스템**을 제어합니다. 
* **시뮬레이션 환경 (`PyBullet`)**: 가상 3D 공간에서 짐벌의 물리적 거동, 비전 노이즈, 통신 지연(Jitter), 모터 속도 한계 등을 모사합니다.
* **실제 하드웨어 환경 (`STM32`)**: MCU의 하드웨어 타이머(PWM)와 인터럽트를 활용하여 실제 서보모터(예: SG90)를 정밀 제어합니다.

---

## 📁 2. 모터 제어 모듈 구조 (`motor.h` & `motor.c`)
메인 코드에 혼재되어 있던 모터 및 PID 제어 로직을 독립적인 모듈로 분리하여 재사용성과 가독성을 높였습니다.

### 🔹 `motor.h` (헤더 파일)
* **포함 내용:** 서보모터 PWM CCR 범위 정의, PID 제어기 구조체(`PID_Controller`) 선언, 함수 프로토타입
* **특징:** `#ifndef MOTOR_H` 가드(Include Guard)를 적용하여 헤더 파일이 중복 포함되는 것을 방지합니다.

```c
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

// 서보모터 각도/CCR 범위 정의 (0.5ms ~ 2.5ms)
#define ANGLE_MIN   500   // -90도
#define ANGLE_MID   1500  //  0도 (중앙)
#define ANGLE_MAX   2500  // +90도

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

// 함수 프로토타입
uint32_t simple_random(uint32_t min, uint32_t max);
uint32_t Angle_To_CCR(float angle_deg); // 각도를 PWM CCR 값으로 변환
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float initial_val);
float PID_Compute(PID_Controller *pid, float current_val);
void Motor_Angle_Process(PID_Controller *pan_pid, PID_Controller *tilt_pid, TIM_HandleTypeDef *htim, float target_pan_deg, float target_tilt_deg);

#endif /* MOTOR_H */
