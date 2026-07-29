#include "app_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdarg.h> // va_list 사용을 위해 추가 -> printf 함수 구현
#include <stdio.h>  // vsprintf 사용을 위해 추가 -> printf 함수 구현
#include <string.h>

// 프로젝트 중 개발 헤더파일
#include "ov2640.h"
#include "camera.h"
#include "display.h"
#include "motor.h"
#include "joystick.h"

// Edge Impulse 헤더파일
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

// 각 통신 포트 선언
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern DCMI_HandleTypeDef hdcmi;
// 태스크간 통신에 사용할 큐 선언
extern osMessageQueueId_t Queue1Handle;


PID_Controller pan_pid;
PID_Controller tilt_pid;

// AI 모델에 넣을 크롭된 이미지
ALIGN_32BYTES(static float ai_input_features[CROP_W * CROP_H]);
ALIGN_32BYTES(static uint16_t crop_buffer[CROP_W * CROP_H]);

// 오버레이로 표시할 FPS 및 상태 데이터
volatile float g_current_fps = 0.0f;
volatile uint8_t g_track_state = (uint8_t) MACHINE_STATE_IDLE;
// 오버레이 크로스헤어 상태 변수
extern uint16_t overlay_cross_color;

// 테스트 이미지 배열 (FOMO 모델 raw features)
extern const float test_features1[];
extern const float test_features2[];

// 서보 모터 관련 상수
#define STEP_SIZE 25
#define MOTOR_TASK_PERIOD_MS   20U
int tilt_toggle_state = 0;

// 서보 모터 pan, tilt 초기 각도를 중앙으로 초기화
uint16_t pan_val = ANGLE_MID;  // 초기 Pan 각도 (중앙)
uint16_t tilt_val = ANGLE_MID; // 초기 Tilt 각도 (중앙)

// 큐에 사용할 구조체 타겟x, 타겟y, 타겟 여부
typedef struct {
	float x;
	float y;
	float detected; // 1.0f: 객체 있음, 0.0f: 객체 없음
} TargetCoord_t;

// 객체 인식 모델에 들어갈 이미지 추출 함수
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
	for (size_t i = 0; i < length; i++) {
		out_ptr[i] = ai_input_features[offset + i];
	}
	return 0;
}

/**
 * @brief 카메라 원본 이미지에서 central 96x96 영역을 크롭하고,
 *        LCD 출력용(RGB565)과 AI 입력용(float 0x00RRGGBB) 데이터를 동시에 생성합니다.
 * @param src_image 원본 카메라 프레임 버퍼 (RGB565)
 * @param out_features AI 입력용 float 배열
 * @param dst_crop LCD 디스플레이용 크롭 버퍼 (RGB565)
 * @param enable_preprocessing true: Red 강조 전처리 적용 / false: 패스스루
 */
// 카메라 원본(frame_buffer)에서 정중앙 96x96을 크롭하여 AI 입력용 float 배열로 변환하는 함수
void Process_Crop_Image_And_Features(const uint16_t *src_image, float *out_features, uint16_t *dst_crop, uint8_t enable_preprocessing) {
    const uint16_t start_x = (FRAME_W - CROP_W) / 2; // 32
    const uint16_t start_y = (FRAME_H - CROP_H) / 2; // 12

    for (int y = 0; y < CROP_H; y++) {
        // 원본 프레임의 Y축 시작 주소 계산 (반복 곱셈 제거)
        const uint32_t src_row_offset = (start_y + y) * FRAME_W + start_x;
        const int target_row_offset = y * CROP_W;

        for (int x = 0; x < CROP_W; x++) {
            uint16_t pixel = src_image[src_row_offset + x];

            // RGB565 분리
            uint32_t r_5 = (pixel >> 11) & 0x1F;
            uint32_t g_6 = (pixel >> 5) & 0x3F;
            uint32_t b_5 = pixel & 0x1F;

            // 8비트(0~255) 스케일 변환
            uint32_t r_8 = (r_5 << 3) | (r_5 >> 2);
            uint32_t g_8 = (g_6 << 2) | (g_6 >> 4);
            uint32_t b_8 = (b_5 << 3) | (b_5 >> 2);

            uint16_t crop_pixel = pixel;

            // 전처리 (Red 강조 알고리즘) ON/OFF 제어
            if (enable_preprocessing) {
                if (r_8 > 150 && r_8 > (g_8 + 60) && r_8 > (b_8 + 60)) {
                    // Red 성분 극대화
                    r_8 = 255;
                    g_8 = 0;
                    b_8 = 0;
                    crop_pixel = 0xF800; // LCD용 RGB565 Pure Red
                }
            }

            int target_idx = target_row_offset + x;

            // LCD용 버퍼 출력
            dst_crop[target_idx] = crop_pixel;

            // AI 입력용 float 출력 (0x00RRGGBB 포맷)
            uint32_t hex_val = (r_8 << 16) | (g_8 << 8) | b_8;
            out_features[target_idx] = (float) hex_val;
        }
    }
}

// 5x5 점 그리기 함수
void Draw_5x5_BlueDot(uint16_t *crop_buf, int center_x, int center_y) {
	for (int dy = -2; dy <= 2; dy++) {
		for (int dx = -2; dx <= 2; dx++) {
			int px = center_x + dx;
			int py = center_y + dy;

			// 96x96 경계를 벗어나지 않도록 안전장치 추가
			if (px >= 0 && px < CROP_W && py >= 0 && py < CROP_H) {
				crop_buf[py * CROP_W + px] = BLUE_DOT_COLOR; // RGB565 BLUE
			}
		}
	}
}

// UART1 시리얼 프린트 (printf 함수와 동일하게 동작)
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

// Camera + AI + LCD
void VisionTask(void) {
    // 태스크 여유 스택 확인
    UART_Printf(" VisionTask Running inference...\r\n");
    UBaseType_t vision_stack = uxTaskGetStackHighWaterMark(NULL);

    // ov2640 활성화
	HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CAM_RET_GPIO_Port, CAM_RET_Pin, GPIO_PIN_RESET);
	osDelay(30);
	HAL_GPIO_WritePin(CAM_RET_GPIO_Port, CAM_RET_Pin, GPIO_PIN_SET);
	osDelay(20);

	// ov2640 모듈 ID 확인
	uint16_t pid = ov2640_ReadID(OV2640_I2C_ADDR);
	if (pid != OV2640_ID) { // 0x26
		UART_Printf("OV2640 Check Failed! Read ID = 0x%02X\r\n", pid);
		return Error_Handler();
	}
	UART_Printf("OV2640 Connected! ID = 0x%02X\r\n", pid);

	// 카메라 레지스터 및 해상도 초기화 (QQVGA 160x120)
	ov2640_Init(OV2640_I2C_ADDR, CAMERA_R160x120);

	// 초기 모드 적용 (1: RGB 모드, 0: Grayscale 모드)
	Camera_SetMode(current_mode);

	// 캡처 시작
	Camera_StartCapture();
	UART_Printf("Start Capture...\r\n");


	// ILI9341(LCD) 초기화
	if (Display_Init() != HAL_OK) {
		Error_Handler();
	}

	// FPS 측정을 위한 변수 추가
    uint32_t last_fps_tick = HAL_GetTick();
	for (;;) {
		if (frame_ready) {
//			UART_Printf("VisionTask Stack Free: %lu Words (%lu Bytes)\r\n",
//					vision_stack, vision_stack * 4);

            uint32_t current_fps_tick = HAL_GetTick();
            g_current_fps = 1000.0f / (float) (current_fps_tick - last_fps_tick);
            last_fps_tick = current_fps_tick;

			// 플래그 초기화
			frame_ready = 0;

            // 카메라 DMA 수신 버퍼 D-Cache Invalidate (RAM -> CPU 읽기)
			SCB_InvalidateDCache_by_Addr((uint32_t*) frame_buffer, FRAME_BYTES);

            // 96 * 96 크롭 + 전처리(ON/OFF 제어) + 두 버퍼 동시에 생성
            // 네 번째 인자에 true를 넣으면 전처리 실행, false를 넣으면 원본 크롭만 수행
            uint8_t use_preprocess = 1;
            Process_Crop_Image_And_Features(frame_buffer, ai_input_features, crop_buffer, use_preprocess);

//			// 원본(160x120)을 LCD에 바로 출력
//			if (Display_UpdateImage(frame_buffer, FRAME_W, FRAME_H) != HAL_OK) {
//				Error_Handler();
//			}

            // CPU가 새로 생성한 ai_input_features를 RAM으로 Flush (CPU -> RAM/NPU 전달)
            SCB_CleanDCache_by_Addr((uint32_t*) ai_input_features, sizeof(ai_input_features));

			// AI 추론 신호 구조체 설정
			signal_t signal;
            signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; // 입력 데이터 총 크기 (예: 96x96 = 9216)
            signal.get_data = &raw_feature_get_data;                  // 데이터를 읽어올 함수(콜백) 등록

			ei_impulse_result_t result = { 0 };

            // AI 모델 추론 실행
			EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
			if (res != 0) {
                UART_Printf("Failed to run classifier (Error code: %d)\r\n", res);
				continue;
			}

//            // Performance Timing 출력
//			UART_Printf("Timing: DSP %d ms, inference %d ms\r\n",
//					result.timing.dsp, result.timing.classification);

            // 큐로 넘겨줄 데이터 구조체 변수 생성
			TargetCoord_t target_msg = { 0.0f, 0.0f, 0.0f };

            // 검출된 객체가 최소 1개 이상 존재할 때
			if (result.bounding_boxes_count > 0) {
                // 배열의 첫 번째(0번) 요소가 가장 신뢰도가 높은 객체
				auto best_bb = result.bounding_boxes[0];

                // Threshold(0.35) 조건 만족 여부 확인
				if (best_bb.value >= 0.35f) {
                    UART_Printf("Best Target '%s' (%.2f) at x: %ld, y: %ld, w: %ld, h: %ld\r\n",
                            best_bb.label, best_bb.value, best_bb.x, best_bb.y,
                            best_bb.width, best_bb.height);

					int center_x = best_bb.x + (best_bb.width / 2);
					int center_y = best_bb.y + (best_bb.height / 2);

                    // 타겟 좌쵸 파란색 점 그리기
					Draw_5x5_BlueDot(crop_buffer, center_x, center_y);

                    // 가장 신뢰도 높은 객체 좌표 담기
					target_msg.x = (float) center_x;
					target_msg.y = (float) center_y;
					target_msg.detected = 1.0f; // 객체 있음

				}
			}

            if (!target_msg.detected) {
				UART_Printf("No objects found in this frame.\r\n");
			}

			osMessageQueuePut(Queue1Handle, &target_msg, 0, 0);

            SCB_CleanDCache_by_Addr((uint32_t*) crop_buffer, sizeof(crop_buffer));

			if (Display_UpdateImage(crop_buffer, CROP_W, CROP_H) != HAL_OK) {
				Error_Handler();
			}
		}

        // DCMI 하드웨어 에러 발생 시 복구
		if (hdcmi.State == HAL_DCMI_STATE_ERROR) {
            UART_Printf("** DCMI ERROR 발생 **\r\n");

			HAL_DCMI_Stop(&hdcmi);
			hdcmi.State = HAL_DCMI_STATE_READY;
			frame_ready = 0;
			Camera_StartCapture();
		}

		osDelay(1);
	}
}

// 서보모터 + 조이스틱
void MotorTask(void) {
	UART_Printf(" MotorTask Running inference...\r\n");
	UBaseType_t motor_stack = uxTaskGetStackHighWaterMark(NULL);

    // 서보모터 팬(ch1), 틸트(ch2) PWM 활성화
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    // 팬, 틸트 각각의 PID 값 초기화 (PID 값 수정 x)
	PID_Init(&pan_pid, 0.5f, 0.00f, 0.0f, ANGLE_MID);
	PID_Init(&tilt_pid, 0.5f, 0.00f, 0.0f, ANGLE_MID);

    // PWM 값 초기 값 (ANGLE MID)
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pan_val);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, tilt_val);

    // 조이스틱 초기화
	Joystick_Init();
	Motor_ManualSetPosition((float) ANGLE_MID, (float) ANGLE_MID);

    // 큐를 통해 받을 타겟 구조체
	TargetCoord_t rx_msg;

    // 조이스틱 구조체
	JoystickInput_t joy;


	for (;;) {
//		UART_Printf("MotorTask Stack Free: %lu Words (%lu Bytes)\r\n",
//				motor_stack, motor_stack * 4);
		Joystick_Read(&joy);

		if (joy.button_pressed) {
			if (g_track_state == MACHINE_STATE_MANUAL) {
				/*
				 * MANUAL -> AUTO
				 * 수동으로 옮겨 놓은 위치를 UART/PID 쪽 현재값에 넘겨주어
				 * 전환 순간 서보가 튀지 않게 한다.
				 */
				float pan_now;
				float tilt_now;
				Motor_ManualGetPosition(&pan_now, &tilt_now);

				pan_val = (uint16_t) pan_now;
				tilt_val = (uint16_t) tilt_now;
				pan_pid.current = pan_now;
				tilt_pid.current = tilt_now;

				g_track_state = (uint8_t) MACHINE_STATE_IDLE;
				UART_Printf("Mode -> AUTO\r\n");


			}
			else {
				/*
				 * AUTO -> MANUAL
				 * 현재 서보가 실제로 가 있는 위치를 수동 제어가 이어받는다.
				 */
				Motor_ManualSetPosition(
						(float) __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1),
						(float) __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2));

				g_track_state = (uint8_t) MACHINE_STATE_MANUAL;
				UART_Printf("Mode -> MANUAL\r\n");
			}
		}


		if (g_track_state == MACHINE_STATE_MANUAL) {
			/* 속도형 제어. 중립이면 증분이 0 이라 그 자리에 멈춘다. */
			Motor_ManualProcess(&joy, &htim2, MOTOR_TASK_PERIOD_MS);
		} else {

			osStatus_t status = osMessageQueueGet(Queue1Handle, &rx_msg,
			NULL, 20);

			if (status == osOK && rx_msg.detected > 0.5f) {

// 객체가 정상 감지되었을 때 PID 연산 수행
				Motor_PID_Process_With_Error(&pan_pid, &tilt_pid, rx_msg.x,
						rx_msg.y, &htim2);

				UART_Printf(
						"Target(%.1f, %.1f) | Pan_Cur: %d | Tilt_Cur: %d\r\n",
						rx_msg.x, rx_msg.y, (int) pan_pid.current,
						(int) tilt_pid.current);
			}
		}


//
//		AUTO 경로를 되살릴 때는 위 블록을 아래처럼 감싼다.
//		MANUAL 중에는 조이스틱이 서보를 소유하므로 PID 가 끼어들면 안 된다.
//
//		if (g_track_state != MACHINE_STATE_MANUAL) { ... }

		osDelay(MOTOR_TASK_PERIOD_MS);
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

	}
}

}
