#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

// 축 출력 정규화 최대 범위 (ADC 12bit 절반 크기로 원본 해상도 유지)
#define JOYSTICK_AXIS_MAX   2048

// 조이스틱 상태 입력 구조체
typedef struct {
    int16_t x;              // -2048 ~ +2047 (중립 0, 오른쪽 +)
    int16_t y;              // -2048 ~ +2047 (중립 0, 위쪽 +)
    bool button_down;       // 현재 디바운스 적용된 버튼 눌림 상태
    bool button_pressed;    // 이번 주기에서 새로 눌렸는지 여부 (상승 엣지)
} JoystickInput_t;

// 조이스틱 초기화 (ADC 오프셋 캘리브레이션 및 중립 오프셋 자동 설정)
void Joystick_Init(void);

// 조이스틱 2축 아날로그 값 폴링 및 버튼 디바운스 처리 (약 20ms 주기 호출 권장)
void Joystick_Read(JoystickInput_t *out);

void Joystick_Start_DMA(void);
void Joystick_Stop_DMA(void);

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_H_ */
