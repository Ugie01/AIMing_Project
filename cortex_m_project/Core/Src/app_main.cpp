#include "app_main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h" // osDelay 등을 쓰기 위해 포함
#include "edge-impulse-sdk/classifier/ei_run_classifier.h" // Edge Impulse 핵심 헤더
#include <string.h>
#include <stdarg.h> // va_list 사용을 위해 추가
#include <stdio.h>  // vsprintf 사용을 위해 추가

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;



int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        // 실제 카메라 센서가 없으므로, 테스트를 위해 모든 픽셀을 0으로 채웁니다.
        out_ptr[i] = 0.0f;
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

void UART_Printf(const char* format, ...) {
    char loc_buf[256];
    va_list args;

    va_start(args, format);
    // 가변 인자를 포맷팅하여 버퍼에 문자열로 저장
    int len = vsnprintf(loc_buf, sizeof(loc_buf), format, args);
    va_end(args);

    if (len > 0) HAL_UART_Transmit(&huart1, (uint8_t*)loc_buf, (uint16_t)len, HAL_MAX_DELAY);

}


extern "C" {

void VisionTask(void) {
	UART_Printf(" VisionTask Running inference...\r\n");
	UBaseType_t vision_stack = uxTaskGetStackHighWaterMark(NULL);


	for (;;) {
		UART_Printf("VisionTask Stack Free: %lu Words (%lu Bytes)\r\n",
				vision_stack, vision_stack * 4);
//       모델에 데이터를 공급할 Signal 구조체 설정
		signal_t signal;
		signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
		signal.get_data = &raw_feature_get_face_data;

//		 결과값을 저장할 구조체 초기화
		ei_impulse_result_t result = { 0 };

//		 추론(Inference)
		EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
		if (res != 0) {
			UART_Printf("Failed to run classifier (Error code: %d)\r\n", res);
			osDelay(1000);
			continue;
		}

//		 실행 시간(타이밍) 출력 (STM32H7의 480MHz 파워를 확인할 수 있습니다)
		UART_Printf("Timing: DSP %d ms, inference %d ms\r\n", result.timing.dsp,
				result.timing.classification);

//		 FOMO 객체 인식 결과(Bounding Boxes) 출력
		bool object_detected = false;
		for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
			auto bb = result.bounding_boxes[ix];
			if (bb.value == 0) {
				continue; // 확신도(Confidence)가 0인 빈 그리드는 건너뜀
			}
			UART_Printf(
					"Found '%s' (%.2f) at x: %ld, y: %ld, w: %ld, h: %ld\r\n",
					bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
			object_detected = true;
		}

		if (!object_detected) {
			UART_Printf("No objects found in this frame.\r\n");
		}

		UART_Printf("---------------------------------------\r\n");
//       카메라 영상 가져오기 -> AI 추론 -> 큐에 좌표 전송
		osDelay(10); // 임시 딜레이
	}
}

void MotorTask(void) {
	UART_Printf(" MotorTask Running inference...\r\n");
	UBaseType_t motor_stack = uxTaskGetStackHighWaterMark(NULL);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	for (;;) {
		UART_Printf("MotorTask Stack Free: %lu Words (%lu Bytes)\r\n",
				motor_stack, motor_stack * 4);
		// 큐에서 좌표 읽기 -> PID 연산 -> 모터 PWM 출력
		osDelay(10); // 10ms 주기
	}
}

}
