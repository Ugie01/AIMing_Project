#include "motor.h"

// --------------------------------------------------
// MANUAL 모드 (조이스틱 수동 제어)
// --------------------------------------------------

// 수동 제어가 들고 있는 현재 각도. 중앙에서 시작한다.
static float manual_pan = (float) ANGLE_MID;
static float manual_tilt = (float) ANGLE_MID;

// ANGLE_MIN ~ ANGLE_MAX 로 잘라낸다
static float Motor_ClampAngle(float value) {
	if (value > (float) ANGLE_MAX) {
		return (float) ANGLE_MAX;
	}
	if (value < (float) ANGLE_MIN) {
		return (float) ANGLE_MIN;
	}
	return value;
}

void Motor_ManualSetPosition(float pan_ccr, float tilt_ccr) {
	manual_pan = Motor_ClampAngle(pan_ccr);
	manual_tilt = Motor_ClampAngle(tilt_ccr);
}

void Motor_ManualGetPosition(float *pan_ccr, float *tilt_ccr) {
	if (pan_ccr != NULL) {
		*pan_ccr = manual_pan;
	}
	if (tilt_ccr != NULL) {
		*tilt_ccr = manual_tilt;
	}
}

void Motor_ManualProcess(const JoystickInput_t *joy, TIM_HandleTypeDef *htim,
		uint32_t dt_ms) {
	if ((joy == NULL) || (htim == NULL) || (dt_ms == 0U)) {
		return;
	}

	// 이번 주기에 움직일 수 있는 최대 이동량 (CCR 단위)
	float step = MANUAL_SPEED_PER_SEC * ((float) dt_ms / 1000.0f);

	// 기울기를 -1.0 ~ +1.0 으로 환산해 이동량에 곱한다.
	// 데드존은 Joystick_Read() 에서 이미 적용되어 중립이면 정확히 0 이다.
	float ratio_x = (float) joy->x / (float) JOYSTICK_AXIS_MAX;
	float ratio_y = (float) joy->y / (float) JOYSTICK_AXIS_MAX;

	manual_pan = Motor_ClampAngle(manual_pan + (ratio_x * step));
	manual_tilt = Motor_ClampAngle(manual_tilt + (ratio_y * step));

	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t ) manual_pan);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t ) manual_tilt);
}

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
	*target_x = IMG_CENTER_X + 25.0f * cosf((float) tick / 300.0f);
	*target_y = IMG_CENTER_Y + 25.0f * sinf((float) tick / 300.0f);
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

void Motor_PID_Process_With_Error(PID_Controller *pan, PID_Controller *tilt,
		float target_x, float target_y, TIM_HandleTypeDef *htim) {
	// 95x95 좌표(0 ~ 95)를 서보 범위(500 ~ 1250)로 직접 선형 매핑
	// Pan: 0(우측) -> 500, 95(좌측) -> 1250
	// Tilt: 0(위쪽) -> 500, 95(아래쪽) -> 1250

	float target_pan_ccr = ANGLE_MIN
			+ (target_x / 95.0f) * (ANGLE_MAX - ANGLE_MIN);
	float target_tilt_ccr = ANGLE_MIN
			+ (target_y / 95.0f) * (ANGLE_MAX - ANGLE_MIN);

	// 범위 제한 안전장치
	if (target_pan_ccr > ANGLE_MAX)
		target_pan_ccr = ANGLE_MAX;
	if (target_pan_ccr < ANGLE_MIN)
		target_pan_ccr = ANGLE_MIN;

	if (target_tilt_ccr > ANGLE_MAX)
		target_tilt_ccr = ANGLE_MAX;
	if (target_tilt_ccr < ANGLE_MIN)
		target_tilt_ccr = ANGLE_MIN;

	// 현재값 구조체에 반영
	pan->current = target_pan_ccr;
	tilt->current = target_tilt_ccr;

	// 최종 CCR 값 반영
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
    //__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)pan_pid->current);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)tilt_pid->current);
}
