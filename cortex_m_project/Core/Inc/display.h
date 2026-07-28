#ifndef DISPLAY_H_
#define DISPLAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Display resolution */
#define DISPLAY_WIDTH   320U
#define DISPLAY_HEIGHT  240U

/* RGB565 colors */
#define DISPLAY_COLOR_BLACK    0x0000U
#define DISPLAY_COLOR_WHITE    0xFFFFU
#define DISPLAY_COLOR_RED      0xF800U
#define DISPLAY_COLOR_GREEN    0x07E0U
#define DISPLAY_COLOR_BLUE     0x001FU
#define DISPLAY_COLOR_YELLOW   0xFFE0U
#define DISPLAY_COLOR_CYAN     0x07FFU
#define DISPLAY_COLOR_MAGENTA  0xF81FU

/**
 * @brief ILI9341 디스플레이를 초기화한다.
 *
 * SPI2와 GPIO 초기화가 끝난 후 호출해야 한다.
 */
HAL_StatusTypeDef Display_Init(void);

/**
 * @brief 카메라 프레임을 화면 전체에 스케일링하여 출력한다.
 *
 * 오버레이가 켜져 있으면 영상 위에 오버레이 UI가 함께 합성된다.
 */
HAL_StatusTypeDef Display_UpdateImage(
    const uint16_t *image,
    uint16_t width,
    uint16_t height
);


/* ------------------------------------------------------------------
 * 오버레이 UI
 * ------------------------------------------------------------------ */

/* ------------------------------------------------------------------
 * 시스템 상태 / FPS 연동부
 *
 * 이 상태는 오버레이 조준점 색상뿐 아니라 서보모터 제어에서도 쓰이므로
 * 선언의 주인(owner)은 app_main 이다. display 는 받아쓰기만 한다.
 *
 * app_main.h 가 제공하는 것:
 *   MachineState_t   (MACHINE_STATE_IDLE / TRACKING / LOCKON / MANUAL)
 *   g_current_fps    측정된 FPS
 *   g_track_state    MachineState_t 값
 *
 * 주의: app_main.h 에서 display.h 를 되받아 include 하면 순환 참조가 된다.
 *       display.h 는 app_main.cpp 에서만 include 한다.
 * ------------------------------------------------------------------ */
#include "app_main.h"

/**
 * @brief 오버레이 UI 표시 여부를 설정한다.
 *
 * true 로 호출하면 다음 Display_UpdateImage() 부터 카메라 영상 위에
 * 오버레이(중앙 조준점 + 우측 상단 FPS)가 합성되어 출력된다.
 * false 로 호출하면 영상만 출력된다.
 *
 * 별도의 화면 지우기는 필요 없다. 프레임마다 화면 전체를 다시 그리므로
 * false 로 바꾼 다음 프레임에서 오버레이는 자동으로 사라진다.
 */
void ActivateOverlayWidget(bool enable);

/**
 * @brief 현재 오버레이 UI가 켜져 있는지 반환한다.
 */
bool IsOverlayWidgetActive(void);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H_ */
