#ifndef APP_MAIN_H_
#define APP_MAIN_H_

// C++ 컴파일러가 이 헤더를 읽을 때만 extern "C"를 적용하여
// C 언어인 main.c에서도 이 함수를 찾을 수 있게 해줍니다.
#ifdef __cplusplus
extern "C" {
#endif
#pragma once
#include "main.h" // HAL 라이브러리 및 GPIO/UART 정의 사용

/**
 * @brief 시스템 동작 상태.
 *
 * 오버레이 조준점 색상과 서보 제어 방식을 함께 결정하므로
 * 선언의 주인(owner)은 app_main 이다. display 는 읽기만 한다.
 *
 * 주의: 여기서 display.h 를 include 하면 순환 참조가 된다.
 *       (display.h 가 app_main.h 를 include 한다)
 */
typedef enum {
	MACHINE_STATE_IDLE = 0, /*!< 목표 탐색 중          : 초록 */
	MACHINE_STATE_TRACKING = 1, /*!< 목표 포착/추적        : 노랑 */
	MACHINE_STATE_LOCKON = 2, /*!< 목표 사격 중          : 빨강 */
	MACHINE_STATE_MANUAL = 3 /*!< 조이스틱 수동 조작 중 : 시안 */
} MachineState_t;



/* 오버레이가 읽어가는 전역 상태. 실체는 app_main.cpp 에 정의되어 있다. */
extern volatile float g_current_fps; /*!< 측정된 FPS        */
extern volatile uint8_t g_track_state; /*!< MachineState_t 값 */

/**
 * @brief C++ 애플리케이션 진입점 함수
 */
void app_main(void);
void VisionTask(void);
void MotorTask(void);

extern const float test_features1[];
extern const float test_features2[];

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H_ */
