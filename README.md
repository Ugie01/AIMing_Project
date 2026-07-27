
이 문서는 프로젝트의 개요부터 `motor.h`/`motor.c` 모듈 설명, 메인 코드 예시, 그리고 PyBullet 시뮬레이션 환경과의 연동 비교(Sim-to-Real 관점)까지 상세히 담고 있습니다.

---

```markdown
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

```

### 🔹 `motor.c` (소스 파일)

* **포함 내용:** 난수 생성기, PID 연산 로직, 각도-CCR 변환 매핑, 모터 통합 제어 프로세스 구현

```c
#include "motor.h"

// 각도(Degree)를 서보모터 PWM CCR 값으로 선형 매핑
uint32_t Angle_To_CCR(float angle_deg) {
    if (angle_deg > 90.0f)  angle_deg = 90.0f;
    if (angle_deg < -90.0f) angle_deg = -90.0f;

    float ccr = ((angle_deg + 90.0f) / 180.0f) * (float)(ANGLE_MAX - ANGLE_MIN) + (float)ANGLE_MIN;
    return (uint32_t)ccr;
}

// PID 연산 함수
float PID_Compute(PID_Controller *pid, float current_val) {
    pid->current = current_val;
    float error = pid->target - pid->current;
    
    float p_out = pid->kp * error;
    pid->integral += error;
    float i_out = pid->ki * pid->integral;
    float d_out = pid->kd * (error - pid->prev_error);
    pid->prev_error = error;
    
    return p_out + i_out + d_out;
}

// Pan/Tilt 각도 목표 기반 통합 PID 및 PWM 갱신 프로세스
void Motor_Angle_Process(PID_Controller *pan_pid, PID_Controller *tilt_pid, TIM_HandleTypeDef *htim, float target_pan_deg, float target_tilt_deg) {
    pan_pid->target = target_pan_deg;
    tilt_pid->target = target_tilt_deg;

    // PID 연산 후 현재 위치 갱신
    pan_pid->current += PID_Compute(pan_pid, pan_pid->current);
    tilt_pid->current += PID_Compute(tilt_pid, tilt_pid->current);

    // 각도를 CCR 값으로 변환하여 하드웨어 타이머 반영
    uint32_t pan_ccr = Angle_To_CCR(pan_pid->current);
    uint32_t tilt_ccr = Angle_To_CCR(tilt_pid->current);

    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pan_ccr);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, tilt_ccr);
}

```

---

## 💻 3. `main.c` 활용 예시 코드

분리된 모듈을 STM32 메인 루프에서 어떻게 호출하는지 보여주는 예시입니다.

```c
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "motor.h" // 모듈 포함

PID_Controller pan_pid;
PID_Controller tilt_pid;
uint8_t rx_data;

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  
  // 타이머 및 PWM 시작
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);

  // PID 초기화 (초기 위치 0도)
  PID_Init(&pan_pid, 0.3f, 0.01f, 0.05f, 0.0f);
  PID_Init(&tilt_pid, 0.3f, 0.01f, 0.05f, 0.0f);

  while (1)
  {
      // 매 20ms마다 각도 단위(예: 중앙 0도, 0도)를 타겟으로 주고 PID 제어 실행
      Motor_Angle_Process(&pan_pid, &tilt_pid, &htim2, 0.0f, 0.0f);

      // 서보모터 표준 제어 주기 (20ms)
      HAL_Delay(20);
  }
}

```

---

## 🔄 4. 시뮬레이션 환경 (PyBullet) vs STM32 하드웨어 비교

본 프로젝트는 가상 시뮬레이션과 실제 하드웨어 간의 괴리를 줄이기 위해(Sim-to-Real) 구동 방식을 상호 연동하도록 설계되었습니다.

| 비교 항목 | PyBullet 시뮬레이션 환경 (`Realsim_sync.py`) | STM32 실제 하드웨어 환경 (`main.c` & `motor.c`) |
| --- | --- | --- |
| **명령 전달 방식** | `p.setJointMotorControl2` API를 통해 **라디안/각도 단위**의 `targetPosition` 직접 명령 | 타이머 레지스터의 `CCR` 값(500 ~ 2500)을 조작하여 **PWM 펄스 폭** 생성 |
| **제어 주기 타이밍** | `time.time()` 기반 소프트웨어 루프 (Jitter/지연 무작위 주입 가능) | MCU 하드웨어 타이머 및 밀리초(`HAL_Delay`) 기반 **엄격한 실시간성** |
| **물리적 제약 모사** | 기어 마찰력(`jointDamping`), 초당 속도 한계(`MAX_STEP_RAD`), 비전 노이즈 코드로 구현 | 현실 세계의 전압 변동, 기구부 백레쉬, 실제 관성 및 마찰력 적용 |
| **목표 변경 대응** | 새 목표가 들어오면 관성과 속도 한계 내에서 방향 전환 | 새 목표가 들어오면 `target` 변수 갱신 후 즉시 다음 PID 출력 반영 |

---

## 🚀 Getting Started

1. STM32CubeMX에서 `TIM2` 채널 1, 2를 PWM Generation으로 설정합니다.
2. `motor.h`와 `motor.c` 파일을 프로젝트의 `Core/Inc`, `Core/Src` 경로에 추가합니다.
3. `main.c`에 `#include "motor.h"`를 선언하고 위 예시와 같이 코드를 연동하여 빌드 및 업로드합니다.

```

```
