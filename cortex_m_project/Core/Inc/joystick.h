#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * 축 출력 정규화 범위.
 *
 * 중립이 0 이고 스틱을 끝까지 밀면 ±JOYSTICK_AXIS_MAX 에 도달한다.
 * ADC 12bit(0~4095) 의 절반이라 원본 해상도를 그대로 살린 값이다.
 */
#define JOYSTICK_AXIS_MAX   2048

/**
 * @brief 조이스틱 1회 샘플.
 */
typedef struct {
	int16_t x; /*!< -2048 ~ +2047, 중립 0. + = 오른쪽 */
	int16_t y; /*!< -2048 ~ +2047, 중립 0. + = 위쪽   */
	bool button_down; /*!< 디바운스 후 현재 눌림 상태        */
	bool button_pressed; /*!< 이번 호출에서 새로 눌림(상승 엣지) */
} JoystickInput_t;

/**
 * @brief ADC 오프셋 캘리브레이션 + 중립값 자동 샘플링.
 *
 * 스틱에서 손을 뗀 상태로 부팅해야 중립이 제대로 잡힌다.
 * 스케줄러 시작 후, 조이스틱을 읽는 태스크에서 1회만 호출한다.
 *
 * STM32H7 은 캘리브레이션을 빠뜨리면 오프셋 오차가 커서 손을 뗐는데도
 * 축 값이 0 으로 떨어지지 않는다. CubeMX 는 이 호출을 생성해 주지 않는다.
 */
void Joystick_Init(void);

/**
 * @brief 2축 폴링 + 버튼 디바운스.
 *
 * 고정 주기로 호출해야 한다. 디바운스를 호출 횟수로 세기 때문에
 * 주기를 바꾸면 JOYSTICK_DEBOUNCE_COUNT 도 같이 맞춰야 한다.
 * 권장 주기는 20ms 이다.
 *
 * @param out 결과를 받을 구조체. NULL 이면 아무것도 하지 않는다.
 */
void Joystick_Read(JoystickInput_t *out);

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_H_ */
