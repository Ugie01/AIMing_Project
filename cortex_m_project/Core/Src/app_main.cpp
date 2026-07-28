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
#include "motor.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h" // Edge Impulse 핵심 헤더

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern DCMI_HandleTypeDef hdcmi;


#define OV2640_I2C_ADDR (0x30 << 1)  // 8비트 기준 Write 주소
#define CROP_W     96
#define CROP_H     96

#define CROP_RED_COLOR  0xF800


ALIGN_32BYTES(static float ai_input_features[CROP_W * CROP_H]);
ALIGN_32BYTES(static uint16_t crop_buffer[CROP_W * CROP_H]);

extern volatile float g_current_fps;
//extern uint16_t overlay_cross_color;
extern const float test_features1[];
extern const float test_features2[];
extern osMessageQueueId_t Queue1Handle;

#define STEP_SIZE 25
int tilt_toggle_state = 0;

uint8_t rx_data;             // 시리얼 수신 버퍼 1바이트
uint16_t pan_val = ANGLE_MID;  // 초기 Pan 각도 (중앙)
uint16_t tilt_val = ANGLE_MID; // 초기 Tilt 각도 (중앙)

// 큐에 사용할 구조체 타겟x, 타겟y, 타겟 여부
typedef struct {
	float x;
	float y;
	float detected; // 1.0f: 객체 있음, 0.0f: 객체 없음
} TargetCoord_t;


int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
	for (size_t i = 0; i < length; i++) {
		out_ptr[i] = ai_input_features[offset + i];
	}
	return 0;
}

// 카메라 원본(frame_buffer)에서 정중앙 96x96을 크롭하여 AI 입력용 float 배열로 변환하는 함수
void Get_Cropped_AI_Features(const uint16_t *src_image, float *out_features) {
	uint16_t start_x = (FRAME_W - CROP_W) / 2; // 32
	uint16_t start_y = (FRAME_H - CROP_H) / 2; // 12

	for (int y = 0; y < CROP_H; y++) {
		for (int x = 0; x < CROP_W; x++) {
			uint32_t src_x = start_x + x;
			uint32_t src_y = start_y + y;
			uint32_t src_index = (src_y * FRAME_W) + src_x;

			// LCD 디스플레이에 찍히는 원본 pixel 값
			uint16_t pixel = src_image[src_index];

			// RGB565 각 채널 비트 추출
			uint32_t r_5 = (pixel >> 11) & 0x1F;
			uint32_t g_6 = (pixel >> 5) & 0x3F;
			uint32_t b_5 = pixel & 0x1F;

			// 8비트(0~255) 스케일 확장 (255/31, 255/63 정밀 연산)
			uint32_t r_8 = (r_5 << 3) | (r_5 >> 2);
			uint32_t g_8 = (g_6 << 2) | (g_6 >> 4);
			uint32_t b_8 = (b_5 << 3) | (b_5 >> 2);

			// Raw features 규격: 0x00RRGGBB (Red가 상위, Blue가 하위)
			uint32_t hex_val = (r_8 << 16) | (g_8 << 8) | b_8;

			int target_idx = (y * CROP_W + x);
			out_features[target_idx] = (float) hex_val;
		}
	}
}

// 2. 크롭 픽셀 복사 함수
void Extract_Crop_Image(const uint16_t *src_image, uint16_t *dst_crop) {
	uint16_t start_x = (FRAME_W - CROP_W) / 2; // 32
	uint16_t start_y = (FRAME_H - CROP_H) / 2; // 12

	for (int y = 0; y < CROP_H; y++) {
		for (int x = 0; x < CROP_W; x++) {
			uint32_t src_index = ((start_y + y) * FRAME_W) + (start_x + x);
			dst_crop[y * CROP_W + x] = src_image[src_index];
		}
	}
}

// 5x5 점 그리기 함수
void Draw_5x5_RedDot(uint16_t *crop_buf, int center_x, int center_y) {
	for (int dy = -2; dy <= 2; dy++) {
		for (int dx = -2; dx <= 2; dx++) {
			int px = center_x + dx;
			int py = center_y + dy;

			// 96x96 경계를 벗어나지 않도록 안전장치 추가
			if (px >= 0 && px < CROP_W && py >= 0 && py < CROP_H) {
				crop_buf[py * CROP_W + px] = 0xF800; // RGB565 RED
			}
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
	uint32_t last_tick = HAL_GetTick();
	for (;;) {
//		if (frame_ready) {
////			UART_Printf("VisionTask Stack Free: %lu Words (%lu Bytes)\r\n",
////					vision_stack, vision_stack * 4);
//
//			uint32_t current_tick = HAL_GetTick();
//			g_current_fps = 1000.0f / (float) (current_tick - last_tick);
//			last_tick = current_tick;
//
//			frame_ready = 0;    // 플래그 초기화
//
//			// 카메라 DMA가 수신한 원본 프레임 버퍼 D-Cache 동기화
//			SCB_InvalidateDCache_by_Addr((uint32_t*) frame_buffer, FRAME_BYTES);
//
//			// 중앙 96x96 영역을 AI 입력 버퍼(ai_input_features)로 크롭 및 전처리
//			Get_Cropped_AI_Features(frame_buffer, ai_input_features);
//			Extract_Crop_Image(frame_buffer, crop_buffer);
//
////			// 원본(160x120)을 LCD에 바로 출력
////			if (Display_UpdateImage(frame_buffer, FRAME_W, FRAME_H) != HAL_OK) {
////				Error_Handler();
////			}
//
//			// CPU가 가공한 ai_input_features 배열을 RAM에 강제 반영
//			SCB_CleanDCache_by_Addr((uint32_t*) ai_input_features,
//					sizeof(ai_input_features));
//
//			// AI 추론 신호 구조체 설정
//			signal_t signal;
//			signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
//			signal.get_data = &raw_feature_get_data;
//
//			ei_impulse_result_t result = { 0 };
//
//			// 추론 실행
//			EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
//			if (res != 0) {
//				UART_Printf("Failed to run classifier (Error code: %d)\r\n",
//						res);
//				osDelay(1000);
//				continue;
//			}
//
//			// Performance Timing 출력
//			UART_Printf("Timing: DSP %d ms, inference %d ms\r\n",
//					result.timing.dsp, result.timing.classification);
//
//			// **큐로 넘겨줄 데이터 구조체 변수 생성**
//			TargetCoord_t target_msg = { 0.0f, 0.0f, 0.0f };
//			bool object_detected = false;
//
//			for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
//				auto bb = result.bounding_boxes[ix];
//				if (bb.value >= 0.35f) { // Threshold 조건
//					UART_Printf(
//							"Found '%s' (%.2f) at x: %ld, y: %ld, w: %ld, h: %ld\r\n",
//							bb.label, bb.value, bb.x, bb.y, bb.width,
//							bb.height);
//
//					int center_x = bb.x + (bb.width / 2);
//					int center_y = bb.y + (bb.height / 2);
//
//					Draw_5x5_RedDot(crop_buffer, center_x, center_y);
//
//					// **인식된 첫 번째 객체의 좌표를 구조체에 담기**
//					target_msg.x = (float) center_x;
//					target_msg.y = (float) center_y;
//					target_msg.detected = 1.0f; // 객체 있음 표시
//
//					object_detected = true;
//				}
//			}
//
//			if (!object_detected) {
//				UART_Printf("No objects found in this frame.\r\n");
//			}
//
//			osMessageQueuePut(Queue1Handle, &target_msg, 0, 0);
//
//			SCB_CleanDCache_by_Addr((uint32_t*) crop_buffer,
//					sizeof(crop_buffer));
//			if (Display_UpdateImage(crop_buffer, CROP_W, CROP_H) != HAL_OK) {
//				Error_Handler();
//			}
//
//		}
//
//		// DCMI 하드웨어 에러 발생 시 복구
//		if (hdcmi.State == HAL_DCMI_STATE_ERROR) {
//			UART_Printf("DCMI ERROR 발생!\r\n");
//
//			HAL_DCMI_Stop(&hdcmi);
//			hdcmi.State = HAL_DCMI_STATE_READY;
//			frame_ready = 0;
//			Camera_StartCapture();
//		}

		osDelay(1);
	}
}

void MotorTask(void) {
	UART_Printf(" MotorTask Running inference...\r\n");
	UBaseType_t motor_stack = uxTaskGetStackHighWaterMark(NULL);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	PID_Controller pan_pid;
	PID_Controller tilt_pid;

	static float current_x = 500.0f;
	static float current_y = 500.0f;

	TargetCoord_t rx_msg;

	HAL_UART_Receive_IT(&huart1, &rx_data, 1);

	for (;;) {
//		UART_Printf("MotorTask Stack Free: %lu Words (%lu Bytes)\r\n",
//				motor_stack, motor_stack * 4);
//
//		osStatus_t status = osMessageQueueGet(Queue1Handle, &rx_msg, NULL, 20);
//
//		if (status == osOK && rx_msg.detected > 0.5f) {
//
//			// 객체가 정상 감지되었을 때 PID 연산 수행
//			Motor_PID_Process_With_Error(&pan_pid, &tilt_pid, rx_msg.x,
//					rx_msg.y, &htim2);
//
//			UART_Printf("Target(%.1f, %.1f) | Pan_Cur: %d | Tilt_Cur: %d\r\n",
//					rx_msg.x, rx_msg.y, (int) pan_pid.current,
//					(int) tilt_pid.current);
//		} else {
//			// 객체가 없거나 큐 수신 실패 시 모터 제어를 멈추거나 대기 상태 유지
//			// (필요 시 여기서 멈춤 로직 추가)
//		}
		osDelay(1); // 10ms 주기
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		// 입력받은 문자(rx_data)에 따른 Pan/Tilt 값 조정
		if (rx_data == 'w' || rx_data == 'W') {
			tilt_val += STEP_SIZE; // 위쪽 (Tilt 증가)
		} else if (rx_data == 's' || rx_data == 'S') {
			tilt_val -= STEP_SIZE; // 아래쪽 (Tilt 감소)
		} else if (rx_data == 'a' || rx_data == 'A') {
			pan_val += STEP_SIZE;  // 오른쪽 (Pan 증가)
		} else if (rx_data == 'd' || rx_data == 'D') {

			pan_val -= STEP_SIZE;  // 왼쪽 (Pan 감소)
		} else if (rx_data == 't' || rx_data == 'T') {
			if (tilt_toggle_state == 0) {
				tilt_val = 500;
				tilt_toggle_state = 1;
			} else {
				tilt_val = 1250;
				tilt_toggle_state = 0;
			}
		}

		// 서보모터 안전 범위 제한 (500 ~ 2500) 강제 적용
//		if (pan_val > ANGLE_MAX)
//			pan_val = ANGLE_MAX;
//		if (pan_val < ANGLE_MIN)
//			pan_val = ANGLE_MIN;
//		if (tilt_val > ANGLE_MAX)
//			tilt_val = ANGLE_MAX;
//		if (tilt_val < ANGLE_MIN)
//			tilt_val = ANGLE_MIN;

		// 실제 타이머 CCR 값 갱신 (TIM2 채널 1: Pan, 채널 2: Tilt)
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pan_val);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, tilt_val);

		// 디버깅용 현재 값 출력
		UART_Printf("WASD Input [%c] -> Pan: %d, Tilt: %d\r\n", rx_data,
				pan_val, tilt_val);

		// 다음 1바이트 수신을 위해 인터럽트 재활성화
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}

}
