#include "camera.h"
#include "ov2640.h"
#include "dcmi.h"
#include "usart.h"

// 전역 변수 은닉화
ALIGN_32BYTES(static uint16_t frame_buffer[FRAME_PIXELS]);
static volatile uint8_t frame_ready = 0;

static uint8_t current_mode = 1; // 0: Grayscale, 1: RGB

// Getter / Setter
uint16_t* Camera_GetFrameBuffer(void) {
    return frame_buffer;
}

uint8_t Camera_IsFrameReady(void) {
    return frame_ready;
}

void Camera_ClearFrameReady(void) {
    frame_ready = 0;
}

void Camera_SetFrameReady(void) {
    frame_ready = 1;
}

uint8_t Camera_GetMode(void) {
    return current_mode;
}

// 카메라 DMA 캡처 시작 함수
void Camera_StartCapture(void) {
    HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t) frame_buffer, FRAME_WORDS);
}

// RGB565 모드
void Camera_SetMode(uint8_t is_rgb) {
    current_mode = is_rgb;

    CAMERA_IO_Write(0x60, 0xFF, 0x00);
    // DSP reset
    CAMERA_IO_Write(0x60, 0xE0, 0x04);
    // RGB enable
    CAMERA_IO_Write(0x60, 0xC2, 0x0C);
    // RGB565
    CAMERA_IO_Write(0x60, 0xDA, 0x09);
    // DSP enable
    CAMERA_IO_Write(0x60, 0xE0, 0x00);

    if (current_mode)
        ov2640_Config(0x60, CAMERA_BLACK_WHITE, CAMERA_BLACK_WHITE_NORMAL, 0);
    else
        ov2640_Config(0x60, CAMERA_BLACK_WHITE, CAMERA_BLACK_WHITE_BW, 0);
}

// 파이썬 뷰어로 1프레임 데이터 전송 함수 (UART)
void Camera_SendFrameToPC(void) {
    SCB_CleanDCache_by_Addr((uint32_t*) frame_buffer, FRAME_BYTES);

    if (current_mode == 1) {
        static const uint8_t start_rgb[] = "---START_RGB---\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*) start_rgb, sizeof(start_rgb) - 1, HAL_MAX_DELAY);
    } else {
        static const uint8_t start_gray[] = "---START_GRAY---\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*) start_gray, sizeof(start_gray) - 1, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart1, (uint8_t*) frame_buffer, FRAME_BYTES, HAL_MAX_DELAY);

    // 내부 Setter 사용
    Camera_ClearFrameReady();
    Camera_StartCapture();
}
