#include "app_main.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h" // Edge Impulse 핵심 헤더
#include "test_raw_frame.h" // 새로 생성한 RAW 데이터 헤더 추가
//#include <string.h>
//#include <stdarg.h>
//#include <math.h>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <cmath>


extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
//extern I2C_HandleTypeDef hi2c1;

// -----------------------------------------------------------------------------
// [벤치마크 설정 및 Ground Truth 설정]
// -----------------------------------------------------------------------------
// 테스트 RAW 프레임 내 실제 객체의 정답(Ground Truth) 중심 좌표 설정
#define GT_CENTER_X         48.0f
#define GT_CENTER_Y         32.0f
#define DISTANCE_THRESHOLD  10.0f   // GT와 예측 좌표 간 거리(픽셀)가 이 값 이하여야 Detection 성공으로 판정
//#ifndef EI_CLASSIFIER_RAM_SIZE
#define EI_CLASSIFIER_RAM_SIZE EI_CLASSIFIER_SLICE_SIZE
//#endif


// RAW 이미지 데이터를 Edge Impulse 모델 버퍼로 공급하는 콜백 함수
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
//    for (size_t i = 0; i < length; i++) {
//        // 실제 카메라 센서가 없으므로, 테스트를 위해 모든 픽셀을 0으로 채웁니다.
//        out_ptr[i] = 0.0f;
//    }
	for (size_t i = 0; i < length; i++) {
	        // offset + i 위치의 RAW 픽셀 데이터를 float 형태로 할당
	        if ((offset + i) < TEST_RAW_PIXEL_COUNT) {
	            out_ptr[i] = (float)TEST_RAW_DATA[offset + i];
	        } else {
	            out_ptr[i] = 0.0f; // 범위 초과 시 예외 처리
	        }
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

    if (len > 0){
    	HAL_UART_Transmit(&huart1, (uint8_t*)loc_buf, (uint16_t)len, HAL_MAX_DELAY);
    }
}


// -----------------------------------------------------------------------------
// [성능 평가 및 시각화 리포트 출력 함수]
// -----------------------------------------------------------------------------
// [Main Application Entry Point]
// app_main.h의 extern "C" 선언에 의해 C-linkage가 자동 적용됨
// -----------------------------------------------------------------------------
void app_main(void) {
    HAL_UART_Transmit(&huart1, (uint8_t*)"[System Ready]\r\n", 16, HAL_MAX_DELAY);

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &raw_feature_get_data;

    ei_impulse_result_t result = { 0 };

    // 10회 추론 평균 Latency 측정
    uint32_t total_latency = 0;
    const int TEST_LOOPS = 10;

    for (int i = 0; i < TEST_LOOPS; i++) {
        run_classifier(&signal, &result, false);
        total_latency += result.timing.classification;
    }

    float avg_latency_ms = static_cast<float>(total_latency) / TEST_LOOPS;
    float fps = (avg_latency_ms > 0.0f) ? (1000.0f / avg_latency_ms) : 0.0f;

    bool detection_success = false;
    float best_confidence = 0.0f;
    float pred_x = 0.0f, pred_y = 0.0f;
    float pos_error = 999.0f;

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
        auto bb = result.bounding_boxes[ix];
        if (bb.value == 0) continue;

        float cx = bb.x + (bb.width / 2.0f);
        float cy = bb.y + (bb.height / 2.0f);
        float dist = std::sqrt(std::pow(cx - GT_CENTER_X, 2) + std::pow(cy - GT_CENTER_Y, 2));

        if (dist < pos_error) {
            pos_error = dist;
            pred_x = cx;
            pred_y = cy;
            best_confidence = bb.value;
        }
    }
    if (pos_error <= DISTANCE_THRESHOLD) {
        detection_success = true;
    }
#endif

    float success_rate = detection_success ? 100.0f : 0.0f;
    float ram_kb = static_cast<float>(EI_CLASSIFIER_RAM_SIZE) / 1024.0f;

    // PC 자동 수신 스크립트 트리거용 데이터 출력
    UART_Printf("[CSV_DATA]%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.4f,%.2f,%.2f,%.2f\r\n",
                success_rate, GT_CENTER_X, GT_CENTER_Y, pred_x, pred_y,
                pos_error, best_confidence, avg_latency_ms, fps, ram_kb);

    while (1) {
        HAL_Delay(5000);
    }
}

