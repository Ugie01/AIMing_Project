#include "app_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h" // osDelay 등을 쓰기 위해 포함
#include "edge-impulse-sdk/classifier/ei_run_classifier.h" // Edge Impulse 핵심 헤더
#include <string.h>
#include <stdarg.h> // va_list 사용을 위해 추가
#include <stdio.h>  // vsprintf 사용을 위해 추가
#include "ov2640.h"
#include "camera.h"
#include "display.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern DCMI_HandleTypeDef hdcmi;

#define OV2640_I2C_ADDR (0x30 << 1)  // 8비트 기준 Write 주소

#define CROP_W     96
#define CROP_H     96
#define CROP_SIZE  (FEATURE_W * FEATURE_H)
static float ai_input_features[CROP_W * CROP_H];

extern const float test_features1[];
extern const float test_features2[];


int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
	for (size_t i = 0; i < length; i++) {
		out_ptr[i] = ai_input_features[offset + i];
	}
    return 0;
}

int raw_feature_get_face_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        // test_features 배열에서 데이터를 가져와서 모델에 공급
//        out_ptr[i] = test_features1[offset + i];
        out_ptr[i] = test_features2[offset + i];
    }
    return 0;
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

//static void Send_FeaturesToPC(void) {
//	uint8_t start_msg[] = "---START---\r\n";
//	HAL_UART_Transmit(&huart1, start_msg, sizeof(start_msg) - 1, HAL_MAX_DELAY);
//
//	HAL_UART_Transmit(&huart1, (uint8_t*) test_features1,
//	FEATURE_SIZE * sizeof(uint32_t), HAL_MAX_DELAY);
//}

// 카메라 원본(frame_buffer)에서 정중앙 96x96을 크롭하여 AI 입력용 float 배열로 변환하는 함수
void Get_Cropped_AI_Features(const uint16_t *src_image, float *out_features) {
	uint16_t start_x = (FRAME_W - CROP_W) / 2; // (160 - 96) / 2 = 32
	uint16_t start_y = (FRAME_H - CROP_H) / 2; // (120 - 96) / 2 = 12

	const uint8_t *p_bytes = (const uint8_t*) src_image;

	for (int y = 0; y < CROP_H; y++) {
		for (int x = 0; x < CROP_W; x++) {
			uint32_t src_x = start_x + x;
			uint32_t src_y = start_y + y;
			uint32_t src_index = (src_y * FRAME_W) + src_x;

			// 기존 그레이스케일 추출 방식 적용 (모드에 따라 맞게 조절 가능)
			uint8_t y_val = p_bytes[src_index * 2U + 1U];

			// AI 모델에 맞게 0.0f ~ 255.0f 또는 0.0f ~ 1.0f로 정규화하여 대입
			out_features[y * CROP_W + x] = (float) y_val;
		}
	}
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
	Camera_SetMode (current_mode);

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

	for (;;) {
//		// 기존 코드
//		if (frame_ready) {
//			UART_Printf("VisionTask Stack Free: %lu Words (%lu Bytes)\r\n",
//					vision_stack, vision_stack * 4);
//			frame_count++;      // 프레임 카운트 증가
//			frame_ready = 0;    // 플래그 초기화
//			Camera_SendFrameToPC();
//		}
//
//		if (hdcmi.State == HAL_DCMI_STATE_ERROR) {
//			HAL_DCMI_Stop(&hdcmi);
//			hdcmi.State = HAL_DCMI_STATE_READY;
//			frame_ready = 0;
//			Camera_StartCapture();
//		}
//
//		// 1초(1000ms)마다 FPS 출력
//		uint32_t current_tick = HAL_GetTick();
//		if (current_tick - last_tick >= 1000) {
//			UART_Printf("Current FPS: %lu\r\n", frame_count);
//			frame_count = 0;          // 카운트 리셋
//			last_tick = current_tick; // 시간 갱신
//		}
//
//		osDelay(1); // 반응성을 높이기 위해 딜레이를 1ms로 줄임 (기존 10ms)

		if (frame_ready) {
			UART_Printf("VisionTask Stack Free: %lu Words (%lu Bytes)\r\n",
					vision_stack, vision_stack * 4);
			frame_count++;      // 프레임 카운트 증가
			frame_ready = 0;    // 플래그 초기화

			// D-Cache Clean (Flush)
			SCB_InvalidateDCache_by_Addr((uint32_t*) frame_buffer, FRAME_BYTES);

			// Raw 이미지 데이터 전송 (160 * 120 * 2 = 38,400 bytes)
			if (Display_UpdateImage(frame_buffer, FRAME_W,
			FRAME_H) != HAL_OK) {
				Error_Handler();
			}

			// 3. [AI용 크롭] 카메라 원본에서 중앙 96x96을 잘라내어 AI 버퍼에 준비
			Get_Cropped_AI_Features(frame_buffer, ai_input_features);

			// 4. [AI 추론 실행] (Edge Impulse 등)
			signal_t signal;
			signal.total_length = CROP_W * CROP_H; // 9216
			signal.get_data = &raw_feature_get_data;

			ei_impulse_result_t result = { 0 };
			EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

			if (res == 0) {
				// 추론 결과 활용 (Bounding Box 등)
				UART_Printf("Timing: DSP %d ms, inference %d ms\r\n",
						result.timing.dsp, result.timing.classification);
				bool object_detected = false;
				for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
					auto bb = result.bounding_boxes[ix];
					if (bb.value == 0) {
						continue; // 확신도(Confidence)가 0인 빈 그리드는 건너뜀
					}
					UART_Printf(
							"Found '%s' (%.2f) at x: %ld, y: %ld, w: %ld, h: %ld\r\n",
							bb.label, bb.value, bb.x, bb.y, bb.width,
							bb.height);
					object_detected = true;
				}

				if (!object_detected) {
					UART_Printf("No objects found in this frame.\r\n");
				}
			}

			Camera_StartCapture();

		}

		if (hdcmi.State == HAL_DCMI_STATE_ERROR) {
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

////       모델에 데이터를 공급할 Signal 구조체 설정
//		signal_t signal;
//		signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
//		signal.get_data = &raw_feature_get_face_data;
//
////		 결과값을 저장할 구조체 초기화
//		ei_impulse_result_t result = { 0 };
//
////		 추론(Inference)
//		EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
//		if (res != 0) {
//			UART_Printf("Failed to run classifier (Error code: %d)\r\n", res);
//			osDelay(1000);
//			continue;
//		}
//
////		 실행 시간(타이밍) 출력 (STM32H7의 480MHz 파워를 확인할 수 있습니다)
//		UART_Printf("Timing: DSP %d ms, inference %d ms\r\n", result.timing.dsp,
//				result.timing.classification);
//
////		 FOMO 객체 인식 결과(Bounding Boxes) 출력
//		bool object_detected = false;
//		for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
//			auto bb = result.bounding_boxes[ix];
//			if (bb.value == 0) {
//				continue; // 확신도(Confidence)가 0인 빈 그리드는 건너뜀
//			}
//			UART_Printf(
//					"Found '%s' (%.2f) at x: %ld, y: %ld, w: %ld, h: %ld\r\n",
//					bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
//			object_detected = true;
//		}
//
//		if (!object_detected) {
//			UART_Printf("No objects found in this frame.\r\n");
//		}
//
//		UART_Printf("---------------------------------------\r\n");
////       카메라 영상 가져오기 -> AI 추론 -> 큐에 좌표 전송
//		osDelay(10); // 임시 딜레이
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
