#ifndef DISPLAY_H_
#define DISPLAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
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

/**
 * @brief 동작 확인용 4분할 테스트 이미지를 출력한다.
 */
HAL_StatusTypeDef Display_UpdateImage(
    const uint16_t *image,
    uint16_t width,
    uint16_t height
);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_H_ */
