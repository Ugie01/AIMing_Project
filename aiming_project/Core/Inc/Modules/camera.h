#ifndef __CAMERA_H
#define __CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define OV2640_I2C_ADDR (0x30 << 1)

#define FRAME_W       160
#define FRAME_H       120
#define FRAME_PIXELS   (FRAME_W * FRAME_H)
#define FRAME_BYTES    (FRAME_PIXELS * 2)
#define FRAME_WORDS    (FRAME_BYTES / 4)

#define CROP_W     96
#define CROP_H     96
#define BLUE_DOT_COLOR  0x001F


// 전역 변수(extern) 대신 접근 함수(Getter/Setter) 제공
uint16_t* Camera_GetFrameBuffer(void);
uint8_t Camera_GetMode(void);

uint8_t Camera_IsFrameReady(void);

void Camera_StartCapture(void);
void Camera_ClearFrameReady(void);

void Camera_SetFrameReady(void); // DCMI 콜백에서 사용
void Camera_SetMode(uint8_t is_rgb);

void Camera_SendFrameToPC(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_H */
