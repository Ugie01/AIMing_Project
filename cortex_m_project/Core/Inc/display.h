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
 * ── 1) app_main.h 의 extern "C" 블록 안에 아래를 추가한다 ──────────
 *
 *     #include <stdint.h>
 *
 *     typedef enum
 *     {
 *         MACHINE_STATE_IDLE     = 0,   // 목표 탐색 중
 *         MACHINE_STATE_TRACKING = 1,   // 목표 포착/추적
 *         MACHINE_STATE_LOCKON   = 2    // 목표 사격 중
 *     } MachineState_t;
 *
 *     extern volatile float   g_current_fps;   // 측정된 FPS
 *     extern volatile uint8_t g_track_state;   // MachineState_t 값
 *
 * ── 2) app_main.cpp 에 실체를 정의한다 ────────────────────────────
 *
 *     volatile float   g_current_fps = 0.0f;
 *     volatile uint8_t g_track_state = MACHINE_STATE_IDLE;
 *
 * ── 3) 아래 OVERLAY_USE_APP_GLOBALS 를 1 로 바꾼다 ────────────────
 *
 * 그러면 display 는 app_main.h 의 선언을 그대로 받아쓰고, 아래 임시
 * 정의 블록은 컴파일에서 빠진다. (임시 블록은 그때 지워도 된다)
 *
 * 주의: app_main.h 가 display.h 를 되받아 include 하면 순환 참조가 되므로
 *       app_main.h 에서는 display.h 를 include 하지 않는다.
 *       (app_main.cpp 에서만 include 한다 — 지금 구조 그대로면 문제없다)
 * ------------------------------------------------------------------ */
#define OVERLAY_USE_APP_GLOBALS   0

#if (OVERLAY_USE_APP_GLOBALS == 1)

/* MachineState_t / g_current_fps / g_track_state 를 app_main 에서 받아온다 */
#include "app_main.h"

#else

/*
 * app_main 에 아직 선언이 없어서 두는 임시 정의.
 * 위 1)~3) 을 마치면 이 블록 전체를 삭제한다.
 */
typedef enum
{
    MACHINE_STATE_IDLE     = 0,   /*!< 목표 탐색 중  : 초록 */
    MACHINE_STATE_TRACKING = 1,   /*!< 목표 포착/추적: 노랑 */
    MACHINE_STATE_LOCKON   = 2    /*!< 목표 사격 중  : 빨강 */
} MachineState_t;

#endif

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
