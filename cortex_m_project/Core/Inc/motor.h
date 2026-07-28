#pragma once

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "joystick.h"

/*
 * 서보모터 CCR 범위.
 *
 * TIM2 는 120MHz / (120-1 +1) = 1MHz 로 돌고 Period 가 20000-1 이므로
 * 1 틱 = 1us, 주기 = 20ms(50Hz) 다. 즉 CCR 값이 곧 펄스폭(us) 이다.
 *
 *   ANGLE_MIN 500  -> 0.500ms
 *   ANGLE_MID 875  -> 0.875ms   (= (MIN+MAX)/2)
 *   ANGLE_MAX 1250 -> 1.250ms
 */
#define ANGLE_MIN   700   // 한쪽 끝 (0.500ms)
#define ANGLE_MID   1500   // 중앙    (0.875ms)
#define ANGLE_MAX   2400  // 반대쪽 끝 (1.250ms)
#define IMG_CENTER_X 48.0f
#define IMG_CENTER_Y 48.0f

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

/*
 * MANUAL 모드 최대 이동 속도 (CCR 단위 / 초).
 *
 * 스틱을 끝까지 밀었을 때의 속도다. 가동 범위가 750(1250-500) 이므로
 * 750 / 400 = 약 1.9초에 끝에서 끝까지 이동한다.
 */
#define MANUAL_SPEED_PER_SEC   400.0f

// 함수 프로토타입 선언
uint32_t simple_random(uint32_t min, uint32_t max);

/**
 * @brief MANUAL 모드 : 조이스틱 입력으로 Pan/Tilt 를 움직인다.
 *
 * 속도형 제어다. 기울인 양에 비례한 증분을 현재 각도에 누적하며,
 * 중립(데드존 안)이면 증분이 0 이라 그 자리에 멈춘다.
 * AUTO 에서 전환해 들어와도 모터가 튀지 않는다.
 *
 * 내부에서 ANGLE_MIN ~ ANGLE_MAX 로 클램프하므로 호출자가 따로
 * 범위를 확인할 필요가 없다.
 *
 * @param joy     Joystick_Read() 결과
 * @param htim    서보 PWM 타이머 (CH1 = Pan, CH2 = Tilt)
 * @param dt_ms   직전 호출로부터 경과한 시간(ms). 호출 주기를 넣는다.
 */
void Motor_ManualProcess(const JoystickInput_t *joy, TIM_HandleTypeDef *htim,
		uint32_t dt_ms);

/**
 * @brief MANUAL 제어의 현재 각도를 지정 CCR 값으로 맞춘다.
 *
 * AUTO 로 동작하다 MANUAL 로 전환할 때, 그 시점의 서보 위치를 알려주어
 * 첫 조작에서 튀지 않게 한다. 값은 ANGLE_MIN ~ ANGLE_MAX 로 클램프된다.
 */
void Motor_ManualSetPosition(float pan_ccr, float tilt_ccr);

/**
 * @brief MANUAL 제어의 현재 Pan/Tilt CCR 값을 읽는다.
 *
 * MANUAL 에서 AUTO 로 돌아갈 때 PID 초기값으로 넘겨주면 이어서 제어된다.
 * 포인터는 NULL 을 허용한다.
 */
void Motor_ManualGetPosition(float *pan_ccr, float *tilt_ccr);
void Get_Dummy_Target_Coords(float *target_x, float *target_y);
void Motor_PID_Process_With_Error(PID_Controller *pan, PID_Controller *tilt, float target_x, float target_y, TIM_HandleTypeDef *htim);
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float initial_val);
float PID_Compute(PID_Controller *pid, float current_val);
void Motor_PID_Process(PID_Controller *pan_pid, PID_Controller *tilt_pid, TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
