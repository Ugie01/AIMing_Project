#include "task_vision.h"
#include "app_globals.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ov2640.h"
#include "camera.h"
#include "display.h"

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

extern DCMI_HandleTypeDef hdcmi;

// AI 입력 및 크롭 버퍼 전역 변수 은닉화
ALIGN_32BYTES(static float ai_input_features[CROP_W * CROP_H]);
ALIGN_32BYTES(static uint16_t crop_buffer[CROP_W * CROP_H]);
static volatile float current_fps = 0.0f;

// 기본 설정 모드 (원하는 모드로 초기값 설정 가능)
static volatile PreprocessMode_t current_prep_mode = PREPROCESS_HSV;

// Getter 구현
float Vision_GetCurrentFPS(void) {
    return current_fps;
}

// AI 데이터 추출 함수
static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = ai_input_features[offset + i];
    }
    return 0;
}

// [Mode 0] 전처리 없음: 원본 RGB888 유지
static inline void Preprocess_None(uint8_t r, uint8_t g, uint8_t b, uint16_t raw_pixel, uint8_t *out_r, uint8_t *out_g, uint8_t *out_b,
        uint16_t *out_crop) {
    *out_r = r;
    *out_g = g;
    *out_b = b;
    *out_crop = raw_pixel;
}

// [Mode 1] 단순 RGB Threshold 전처리
static inline void Preprocess_SimpleRGB(uint8_t r, uint8_t g, uint8_t b, uint16_t raw_pixel, uint8_t *out_r, uint8_t *out_g, uint8_t *out_b,
        uint16_t *out_crop) {
    if (r > 150 && r > (g + 60) && r > (b + 60)) {
        *out_r = 255;
        *out_g = 0;
        *out_b = 0;
        *out_crop = 0xF800; // RED
    } else {
        *out_r = r;
        *out_g = g;
        *out_b = b;
        *out_crop = raw_pixel;
    }
}

// [Mode 2] HSV 정밀 Red 마스킹 및 배경 무채색화 전처리
static inline void Preprocess_HSV_Red(uint8_t r, uint8_t g, uint8_t b, uint16_t raw_pixel, uint8_t *out_r, uint8_t *out_g, uint8_t *out_b,
        uint16_t *out_crop) {
    uint8_t max_val = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
    uint8_t min_val = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
    uint8_t diff = max_val - min_val;

    uint16_t hue = 0;
    uint8_t sat = (max_val == 0) ? 0 : (255 * diff / max_val);

    if (diff > 0) {
        if (max_val == r) {
            int32_t h_calc = (int32_t) (60 * (g - b)) / diff;
            if (h_calc < 0)
                h_calc += 360;
            hue = (uint16_t) h_calc;
        } else if (max_val == g) {
            hue = (uint16_t) (60 * (b - r) / diff + 120);
        } else {
            hue = (uint16_t) (60 * (r - g) / diff + 240);
        }
    }

    // Red 조건 (Hue: 0~10 or 348~360, Saturation > 110, Brightness > 60)
    if ((hue <= 10 || hue >= 348) && sat > 110 && max_val > 60) {
        *out_r = 255;
        *out_g = 0;
        *out_b = 0;
        *out_crop = 0xF800;
    } else {
        // 배경 무채색(Grayscale) 처리
        uint8_t gray = (uint8_t) (0.299f * r + 0.587f * g + 0.114f * b);
        *out_r = gray;
        *out_g = gray;
        *out_b = gray;
        *out_crop = raw_pixel;
        // 참고: LCD 디스플레이 화면도 흑백으로 보려면 아래 주석을 해제하세요.
        // *out_crop = ((gray >> 3) << 11) | ((gray >> 2) << 5) | (gray >> 3);
    }
}

// ==============================================================================
// 96x96 이미지 크롭 & 전처리 함수
// ==============================================================================
static void Process_Crop_Image_And_Features(const uint16_t *src_image, float *out_features, uint16_t *dst_crop, PreprocessMode_t mode) {
    const uint16_t start_x = (FRAME_W - CROP_W) / 2;
    const uint16_t start_y = (FRAME_H - CROP_H) / 2;

    for (int y = 0; y < CROP_H; y++) {
        const uint32_t src_row_offset = (start_y + y) * FRAME_W + start_x;
        const int target_row_offset = y * CROP_W;

        for (int x = 0; x < CROP_W; x++) {
            uint16_t pixel = src_image[src_row_offset + x];

            // RGB565 -> RGB888 비트 확장 변환
            uint32_t r_5 = (pixel >> 11) & 0x1F;
            uint32_t g_6 = (pixel >> 5) & 0x3F;
            uint32_t b_5 = pixel & 0x1F;

            uint8_t r_8 = (r_5 << 3) | (r_5 >> 2);
            uint8_t g_8 = (g_6 << 2) | (g_6 >> 4);
            uint8_t b_8 = (b_5 << 3) | (b_5 >> 2);

            uint8_t final_r, final_g, final_b;
            uint16_t crop_pixel;

            // 선택된 모드별 서브 함수 호출
            switch (mode) {
            case PREPROCESS_NONE:
                Preprocess_None(r_8, g_8, b_8, pixel, &final_r, &final_g, &final_b, &crop_pixel);
                break;

            case PREPROCESS_SIMPLE:
                Preprocess_SimpleRGB(r_8, g_8, b_8, pixel, &final_r, &final_g, &final_b, &crop_pixel);
                break;

            case PREPROCESS_HSV:
                Preprocess_HSV_Red(r_8, g_8, b_8, pixel, &final_r, &final_g, &final_b, &crop_pixel);
                break;

            default:
                Preprocess_None(r_8, g_8, b_8, pixel, &final_r, &final_g, &final_b, &crop_pixel);
                break;
            }

            // 3. 결과 버퍼 기록
            int target_idx = target_row_offset + x;
            dst_crop[target_idx] = crop_pixel;

            // AI 입력용 RGB888 패킹 (0x00RRGGBB)
            uint32_t hex_val = ((uint32_t) final_r << 16) | ((uint32_t) final_g << 8) | final_b;
            out_features[target_idx] = (float) hex_val;
        }
    }
}

// 파란 점 그리기
static void Draw_5x5_BlueDot(uint16_t *crop_buf, int center_x, int center_y) {
    for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
            int px = center_x + dx;
            int py = center_y + dy;
            if (px >= 0 && px < CROP_W && py >= 0 && py < CROP_H) {
                crop_buf[py * CROP_W + px] = BLUE_DOT_COLOR;
            }
        }
    }
}

extern "C" void VisionTask(void) {
    UART_Printf(" VisionTask Running inference...\r\n");

    HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CAM_RET_GPIO_Port, CAM_RET_Pin, GPIO_PIN_RESET);
    osDelay(30);
    HAL_GPIO_WritePin(CAM_RET_GPIO_Port, CAM_RET_Pin, GPIO_PIN_SET);
    osDelay(20);

    uint16_t pid = ov2640_ReadID(OV2640_I2C_ADDR);
    if (pid != OV2640_ID) {
        UART_Printf("OV2640 Check Failed! Read ID = 0x%02X\r\n", pid);
        return Error_Handler();
    }
    UART_Printf("OV2640 Connected! ID = 0x%02X\r\n", pid);

    ov2640_Init(OV2640_I2C_ADDR, CAMERA_R160x120);
    Camera_SetMode(Camera_GetMode());
    Camera_StartCapture();
    UART_Printf("Start Capture...\r\n");

    if (Display_Init() != HAL_OK) {
        Error_Handler();
    }

    uint32_t last_fps_tick = HAL_GetTick();

    // 루프 진입 전, 측정용 변수 선언 및 초기화
    static uint32_t profiling_start_tick = HAL_GetTick();
    static uint32_t frame_count = 0;
    static uint32_t sum_preprocess = 0;
    static uint32_t sum_inference = 0;
    static uint32_t sum_postprocess = 0;
    static uint32_t sum_display = 0;
    static uint32_t sum_total = 0;

    for (;;) {
        if (Camera_IsFrameReady()) {
            uint32_t current_fps_tick = HAL_GetTick();
            current_fps = 1000.0f / (float) (current_fps_tick - last_fps_tick);
            last_fps_tick = current_fps_tick;

            Camera_ClearFrameReady();
            uint16_t *frame_buf = Camera_GetFrameBuffer();

            // ========================================================
            // 1. Preprocessing 구간 (캐시 무효화 + 크롭/특징 추출)
            // ========================================================
            uint32_t t_start = HAL_GetTick();

            SCB_InvalidateDCache_by_Addr((uint32_t*) frame_buf, FRAME_BYTES);
            Process_Crop_Image_And_Features(frame_buf, ai_input_features, crop_buffer, current_prep_mode);
            SCB_CleanDCache_by_Addr((uint32_t*) ai_input_features, sizeof(ai_input_features));

            uint32_t t_pre = HAL_GetTick();

            // ========================================================
            // 2. AI Inference 구간 (Edge Impulse 추론)
            // ========================================================
            signal_t signal;
            signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
            signal.get_data = &raw_feature_get_data;

            ei_impulse_result_t result = { 0 };
            EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
            if (res != 0) {
                UART_Printf("Failed to run classifier (Error code: %d)\r\n", res);
                continue;
            }

            uint32_t t_infer = HAL_GetTick();

            // ========================================================
            // 3. Postprocessing 구간 (결과 파싱 + UI 그리기 + 큐 전송)
            // ========================================================
            TargetCoord_t target_msg = { 0.0f, 0.0f, 0.0f };

            static uint8_t target_miss_count = 2;
            static float last_valid_x = 48.0f;
            static float last_valid_y = 48.0f;

            if (result.bounding_boxes_count > 0) {
                auto best_bb = result.bounding_boxes[0];
                if (best_bb.value >= 0.60f) {
                    int center_x = best_bb.x + (best_bb.width / 2);
                    int center_y = best_bb.y + (best_bb.height / 2);

                    // 노이즈 필터링 (지수 이동 평균 EMA: 반응이 빠르고 튀는 튀김 방지)
                    // alpha 값이 클수록 최신 값 반영(빠름), 작을수록 부드러움 (0.6 추천)
                    float alpha = 0.6f;
                    if (target_miss_count > 0xFF) { // 초기화 상태 예외 방지용 등
                        last_valid_x = (float) center_x;
                        last_valid_y = (float) center_y;
                    }
                    last_valid_x = (alpha * (float) center_x) + ((1.0f - alpha) * last_valid_x);
                    last_valid_y = (alpha * (float) center_y) + ((1.0f - alpha) * last_valid_y);

                    Draw_5x5_BlueDot(crop_buffer, (int) last_valid_x, (int) last_valid_y);

                    target_msg.x = last_valid_x;
                    target_msg.y = last_valid_y;
                    target_msg.detected = 1.0f;

                    target_miss_count = 0; // 타겟 발견했으므로 미스 카운트 리셋
                }
            }

            // 타겟을 이번 프레임에 못 찾았을 경우 처리
            if (target_msg.detected == 0.0f) {
                if (target_miss_count < 2) { // 🚀 2프레임 동안은 없는 것으로 바로 치지 않음
                    target_miss_count++;
                    target_msg.x = last_valid_x;
                    target_msg.y = last_valid_y;
                    target_msg.detected = 1.0f; // 기존 타겟이 유지되는 것처럼 속임
                } else {
                    // 2프레임 이상 완전히 안 보일 때만 진짜 소실 처리
                    target_msg.detected = 0.0f;
                }
            }

            osMessageQueuePut(Queue1Handle, &target_msg, 0, 0);

            uint32_t t_post = HAL_GetTick();

            // ========================================================
            // 4. Display Update 구간 (LCD 출력 및 캐시 클린)
            // ========================================================
            SCB_CleanDCache_by_Addr((uint32_t*) crop_buffer, sizeof(crop_buffer));

            if (Display_UpdateImage(crop_buffer, CROP_W, CROP_H) != HAL_OK) {
                Error_Handler();
            }

            uint32_t t_disp = HAL_GetTick();

            // ========================================================
            // 5. 측정 결과 누적 및 30초마다 평균 출력
            // ========================================================
            sum_preprocess += (t_pre - t_start);
            sum_inference += (t_infer - t_pre);
            sum_postprocess += (t_post - t_infer);
            sum_display += (t_disp - t_post);
            sum_total += (t_disp - t_start);
            frame_count++;

            // 30초(30000ms) 경과 확인
            if ((HAL_GetTick() - profiling_start_tick) >= 30000U) {
                if (frame_count > 0) {
                    UART_Printf("\r\n=== Vision Task Profiling (Avg over %lu frames) ===\r\n", frame_count);
                    UART_Printf(" 1. Preprocessing : %lu ms\r\n", sum_preprocess / frame_count);
                    UART_Printf(" 2. AI Inference  : %lu ms\r\n", sum_inference / frame_count);
                    UART_Printf(" 3. Postprocess   : %lu ms\r\n", sum_postprocess / frame_count);
                    UART_Printf(" 4. Display Update: %lu ms\r\n", sum_display / frame_count);
                    UART_Printf("---------------------------------------------------\r\n");
                    UART_Printf(" -> Total Time    : %lu ms per frame\r\n", sum_total / frame_count);
                    UART_Printf("===================================================\r\n\r\n");
                }

                // 다음 30초 측정을 위해 변수 초기화
                sum_preprocess = 0;
                sum_inference = 0;
                sum_postprocess = 0;
                sum_display = 0;
                sum_total = 0;
                frame_count = 0;
                profiling_start_tick = HAL_GetTick();
            }
        }

        if (hdcmi.State == HAL_DCMI_STATE_ERROR) {
            UART_Printf("** DCMI ERROR 발생 **\r\n");
            HAL_DCMI_Stop(&hdcmi);
            hdcmi.State = HAL_DCMI_STATE_READY;
            Camera_ClearFrameReady();
            Camera_StartCapture();
        }

        osDelay(1);
    }
}
