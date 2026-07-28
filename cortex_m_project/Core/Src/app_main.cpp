#include "app_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h" // osDelay 등을 쓰기 위해 포함
#include <stdarg.h> // va_list 사용을 위해 추가
#include <stdio.h>  // vsprintf 사용을 위해 추가
#include <string.h>
#include "ov2640.h"
#include "camera.h"
#include "display.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h" // Edge Impulse 핵심 헤더

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern DCMI_HandleTypeDef hdcmi;

#define OV2640_I2C_ADDR (0x30 << 1)  // 8비트 기준 Write 주소

#define CROP_W     96
#define CROP_H     96

ALIGN_32BYTES(static float ai_input_features[CROP_W * CROP_H]);

extern const float test_features1[];
extern const float test_features2[];

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
	size_t max_size = CROP_W * CROP_H; // 27648
	for (size_t i = 0; i < length; i++) {
		if ((offset + i) < max_size) {
			out_ptr[i] = ai_input_features[offset + i];
		} else {
			out_ptr[i] = 0.0f; // 범위 초과 시 0으로 방어
		}
	}
	return 0;
}


// 카메라 원본(frame_buffer)에서 정중앙 96x96을 크롭하여 AI 입력용 float 배열로 변환하는 함수
void Get_Cropped_AI_Features(const uint16_t *src_image, float *out_features) {
	uint16_t start_x = (FRAME_W - CROP_W) / 2; // 32
	uint16_t start_y = (FRAME_H - CROP_H) / 2; // 12[cite: 1]

	for (int y = 0; y < CROP_H; y++) {
		for (int x = 0; x < CROP_W; x++) {
			uint32_t src_x = start_x + x;
			uint32_t src_y = start_y + y;
			uint32_t src_index = (src_y * FRAME_W) + src_x;

			// LCD 디스플레이에 찍히는 원본 pixel 값
			uint16_t pixel = src_image[src_index];

			// 1. RGB565 각 채널 비트 추출
			uint32_t r_5 = (pixel >> 11) & 0x1F;
			uint32_t g_6 = (pixel >> 5) & 0x3F;
			uint32_t b_5 = pixel & 0x1F;

			// 2. 8비트(0~255) 스케일 확장 (255/31, 255/63 정밀 연산)
			uint32_t r_8 = (r_5 << 3) | (r_5 >> 2);
			uint32_t g_8 = (g_6 << 2) | (g_6 >> 4);
			uint32_t b_8 = (b_5 << 3) | (b_5 >> 2);

			// 3. Raw features 규격: 0x00RRGGBB (Red가 상위, Blue가 하위)
			uint32_t hex_val = (r_8 << 16) | (g_8 << 8) | b_8;

			int target_idx = (y * CROP_W + x);
			out_features[target_idx] = (float) hex_val;
		}
	}
}

void UART_Printf(const char *format, ...) {
	char loc_buf[256];
	va_list args;

	va_start(args, format);
	int len = vsnprintf(loc_buf, sizeof(loc_buf), format, args);
	va_end(args);

	if (len > 0)
		HAL_UART_Transmit(&huart1, (uint8_t*) loc_buf, (uint16_t) len,
		HAL_MAX_DELAY);
}

extern "C" {

void VisionTask(void) {
	// 하드웨어 리셋
	HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CAM_RET_GPIO_Port, CAM_RET_Pin, GPIO_PIN_RESET);
	osDelay(30);
	HAL_GPIO_WritePin(CAM_RET_GPIO_Port, CAM_RET_Pin, GPIO_PIN_SET);
	osDelay(20);

	// BSP 함수로 ID 읽기 테스트
	uint16_t pid = ov2640_ReadID(OV2640_I2C_ADDR);
	if (pid != OV2640_ID) { // 0x26
		UART_Printf("OV2640 Check Failed! Read ID = 0x%02X\r\n", pid);
		while (1)
			;
	}
	UART_Printf("OV2640 Connected! ID = 0x%02X\r\n", pid);

	// BSP 함수로 카메라 레지스터 및 해상도 초기화 (QQVGA 160x120)
	ov2640_Init(OV2640_I2C_ADDR, CAMERA_R160x120);

	// 초기 모드 적용 (1: RGB 모드, 0: Grayscale 모드)
	Camera_SetMode(current_mode);

	// 캡처 시작
	Camera_StartCapture();
	UART_Printf("Start Capture...\r\n");

	UART_Printf(" VisionTask Running inference...\r\n");
	UBaseType_t vision_stack = uxTaskGetStackHighWaterMark(NULL);

	if (Display_Init() != HAL_OK) {
		Error_Handler();
	}

	// FPS 측정을 위한 변수 추가
	uint32_t frame_count = 0;
	uint32_t last_tick = HAL_GetTick();
	uint32_t last_tick2 = HAL_GetTick();

	for (;;) {
		if (frame_ready) {
			UART_Printf("VisionTask Stack Free: %lu Words (%lu Bytes)\r\n",
					vision_stack, vision_stack * 4);
			frame_count++;      // 프레임 카운트 증가
			frame_ready = 0;    // 플래그 초기화

			// 카메라 DMA가 수신한 원본 프레임 버퍼 D-Cache 동기화
			SCB_InvalidateDCache_by_Addr((uint32_t*) frame_buffer, FRAME_BYTES);

			// 중앙 96x96 영역을 AI 입력 버퍼(ai_input_features)로 크롭 및 전처리
			Get_Cropped_AI_Features(frame_buffer, ai_input_features);

			// 원본(160x120)을 LCD에 바로 출력
			if (Display_UpdateImage(frame_buffer, FRAME_W, FRAME_H) != HAL_OK) {
				Error_Handler();
			}

			Camera_StartCapture();

			// CPU가 가공한 ai_input_features 배열을 RAM에 강제 반영
			SCB_CleanDCache_by_Addr((uint32_t*) ai_input_features,
					sizeof(ai_input_features));

			// AI 추론 신호 구조체 설정
			signal_t signal;
			signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
			signal.get_data = &raw_feature_get_data;

			ei_impulse_result_t result = { 0 };

			// 추론 실행
			EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
			if (res != 0) {
				UART_Printf("Failed to run classifier (Error code: %d)\r\n",
						res);
				osDelay(1000);
				continue;
			}

			// Performance Timing 출력
			UART_Printf("Timing: DSP %d ms, inference %d ms\r\n",
					result.timing.dsp, result.timing.classification);

			// FOMO 바운딩 박스 결과 출력 (Threshold 기준: 0.35 이상)
			bool object_detected = false;
			for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
				auto bb = result.bounding_boxes[ix];

				uint32_t current_tick = HAL_GetTick();
				if (current_tick - last_tick2 >= 1000) {
					UART_Printf(
							"Found '%s' (%.2f) at x: %ld, y: %ld, w: %ld, h: %ld\r\n",
							bb.label, bb.value, bb.x, bb.y, bb.width,
							bb.height);
					last_tick2 = current_tick; // 시간 갱신
				}
				object_detected = true;
			}

			if (!object_detected) {
				UART_Printf("No objects found in this frame.\r\n");
			}
		}

		// DCMI 하드웨어 에러 발생 시 복구
		if (hdcmi.State == HAL_DCMI_STATE_ERROR) {
			UART_Printf("DCMI ERROR 발생!\r\n", frame_count);

			HAL_DCMI_Stop(&hdcmi);
			hdcmi.State = HAL_DCMI_STATE_READY;
			frame_ready = 0;
			Camera_StartCapture();
		}

		// 1초(1000ms)마다 FPS 출력
		uint32_t current_tick = HAL_GetTick();
		if (current_tick - last_tick >= 1000) {
			UART_Printf("Current FPS: %lu\r\n", frame_count);
			frame_count = 0;          // 카운트 리셋
			last_tick = current_tick; // 시간 갱신
		}

		osDelay(1);
	}
}

void MotorTask(void) {
	UART_Printf(" MotorTask Running inference...\r\n");
	UBaseType_t motor_stack = uxTaskGetStackHighWaterMark(NULL);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	for (;;) {
//		UART_Printf("MotorTask Stack Free: %lu Words (%lu Bytes)\r\n",
//				motor_stack, motor_stack * 4);
// 큐에서 좌표 읽기 -> PID 연산 -> 모터 PWM 출력
		osDelay(10); // 10ms 주기
	}
}

// DCMI 프레임 수신 완료 콜백 함수
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi) {
	frame_ready = 1;
}

// 에러 발생 시 강제 복구 콜백 함수
void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi) {
	hdcmi->State = HAL_DCMI_STATE_READY;
	frame_ready = 0;
}


}
