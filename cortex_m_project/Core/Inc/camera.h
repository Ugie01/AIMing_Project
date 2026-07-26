#ifndef __CAMERA_H
#define __CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 해상도 설정 (QQVGA 160x120)
#define FRAME_W       160
#define FRAME_H       120
#define FRAME_PIXELS   (FRAME_W * FRAME_H)
#define FRAME_BYTES    (FRAME_PIXELS * 2)     // 38400 bytes
#define FRAME_WORDS    (FRAME_BYTES / 4)      // DCMI DMA length in 32-bit words = 9600

// 전역 변수 외부 참조 선언
extern ALIGN_32BYTES(uint16_t frame_buffer[FRAME_PIXELS]);
extern volatile uint8_t frame_ready;
extern uint8_t current_mode;

// 카메라 제어 함수 프로토타입
void Camera_StartCapture(void);
void Camera_SetMode(uint8_t is_rgb);
void Camera_SendFrameToPC(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_H */
