#ifndef DISPLAY_H_
#define DISPLAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// 공통 전역 상태 및 메시지 구조체 참조
#include "app_globals.h"

// 디스플레이 해상도 설정
#define DISPLAY_WIDTH   320U
#define DISPLAY_HEIGHT  240U

// RGB565 색상표
#define DISPLAY_COLOR_BLACK    0x0000U
#define DISPLAY_COLOR_WHITE    0xFFFFU
#define DISPLAY_COLOR_RED      0xF800U
#define DISPLAY_COLOR_GREEN    0x07E0U
#define DISPLAY_COLOR_BLUE     0x001FU
#define DISPLAY_COLOR_YELLOW   0xFFE0U
#define DISPLAY_COLOR_CYAN     0x07FFU
#define DISPLAY_COLOR_MAGENTA  0xF81FU

// 디스플레이 모듈 초기화
HAL_StatusTypeDef Display_Init(void);

// 카메라 이미지를 LCD 화면에 출력
HAL_StatusTypeDef Display_UpdateImage(const uint16_t *image, uint16_t width, uint16_t height);

// 오버레이 UI(조준선, FPS 등) 표시 여부 설정 및 확인
void ActivateOverlayWidget(bool enable);
bool IsOverlayWidgetActive(void);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H_ */
