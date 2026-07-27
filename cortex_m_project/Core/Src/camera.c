/*
 * camera.c
 *
 *  Created on: 2026. 7. 26.
 *      Author: KCCISTC
 */




#include "camera.h"
#include "ov2640.h"
#include "dcmi.h"
#include "usart.h"

// H743 내부 SRAM용 32바이트 정렬 프레임 버퍼
ALIGN_32BYTES(uint16_t frame_buffer[FRAME_PIXELS]);

volatile uint8_t frame_ready = 0;
uint8_t current_mode = 0; // 0: Grayscale, 1: RGB

// --------------------------------------------------
// 카메라 DMA 캡처 시작 함수
// --------------------------------------------------
void Camera_StartCapture(void) {
	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t) frame_buffer,
	FRAME_WORDS);
}

// --------------------------------------------------
// 카메라 모드 설정 함수
// --------------------------------------------------
void Camera_SetMode(uint8_t is_rgb) {
	current_mode = is_rgb;

	if (current_mode == 1) {
		// RGB 컬러 모드 (Normal)
		ov2640_Config(0x60, CAMERA_BLACK_WHITE, CAMERA_BLACK_WHITE_NORMAL, 0);
	} else {
		// Grayscale 흑백 모드 (BW)
		ov2640_Config(0x60, CAMERA_BLACK_WHITE, CAMERA_BLACK_WHITE_BW, 0);
	}
}

// --------------------------------------------------
// 파이썬 뷰어로 1프레임 데이터 전송 함수
// --------------------------------------------------
void Camera_SendFrameToPC(void) {
	// D-Cache Clean (Flush)
	SCB_CleanDCache_by_Addr((uint32_t*) frame_buffer, FRAME_BYTES);

	// 파이썬 수신기가 모드를 자동 구분하도록 Header 마커 전송
	if (current_mode == 1) {
		static const uint8_t start_rgb[] = "---START_RGB---\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) start_rgb, sizeof(start_rgb) - 1,
		HAL_MAX_DELAY);
	} else {
		static const uint8_t start_gray[] = "---START_GRAY---\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) start_gray,
				sizeof(start_gray) - 1, HAL_MAX_DELAY);
	}

	// Raw 이미지 데이터 전송 (160 * 120 * 2 = 38,400 bytes)
	HAL_UART_Transmit(&huart1, (uint8_t*) frame_buffer, FRAME_BYTES,
	HAL_MAX_DELAY);

	// 플래그 초기화 후 다음 프레임 수신 재개
	frame_ready = 0;
	Camera_StartCapture();
}
