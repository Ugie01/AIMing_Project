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

// 96x96 크롭 & 전처리 함수
static void Process_Crop_Image_And_Features(const uint16_t *src_image, float *out_features, uint16_t *dst_crop, uint8_t enable_preprocessing) {
    const uint16_t start_x = (FRAME_W - CROP_W) / 2;
    const uint16_t start_y = (FRAME_H - CROP_H) / 2;

    for (int y = 0; y < CROP_H; y++) {
        const uint32_t src_row_offset = (start_y + y) * FRAME_W + start_x;
        const int target_row_offset = y * CROP_W;

        for (int x = 0; x < CROP_W; x++) {
            uint16_t pixel = src_image[src_row_offset + x];

            uint32_t r_5 = (pixel >> 11) & 0x1F;
            uint32_t g_6 = (pixel >> 5) & 0x3F;
            uint32_t b_5 = pixel & 0x1F;

            uint32_t r_8 = (r_5 << 3) | (r_5 >> 2);
            uint32_t g_8 = (g_6 << 2) | (g_6 >> 4);
            uint32_t b_8 = (b_5 << 3) | (b_5 >> 2);

            uint16_t crop_pixel = pixel;

            if (enable_preprocessing) {
                if (r_8 > 150 && r_8 > (g_8 + 60) && r_8 > (b_8 + 60)) {
                    r_8 = 255;
                    g_8 = 0;
                    b_8 = 0;
                    crop_pixel = 0xF800;
                }
            }

            int target_idx = target_row_offset + x;
            dst_crop[target_idx] = crop_pixel;

            uint32_t hex_val = (r_8 << 16) | (g_8 << 8) | b_8;
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

    for (;;) {
        if (Camera_IsFrameReady()) {
            uint32_t current_fps_tick = HAL_GetTick();
            current_fps = 1000.0f / (float) (current_fps_tick - last_fps_tick);
            last_fps_tick = current_fps_tick;

            Camera_ClearFrameReady();
            uint16_t *frame_buf = Camera_GetFrameBuffer();

            SCB_InvalidateDCache_by_Addr((uint32_t*) frame_buf, FRAME_BYTES);

            Process_Crop_Image_And_Features(frame_buf, ai_input_features, crop_buffer, 1);

            SCB_CleanDCache_by_Addr((uint32_t*) ai_input_features, sizeof(ai_input_features));

            signal_t signal;
            signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
            signal.get_data = &raw_feature_get_data;

            ei_impulse_result_t result = { 0 };
            EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
            if (res != 0) {
                UART_Printf("Failed to run classifier (Error code: %d)\r\n", res);
                continue;
            }

            TargetCoord_t target_msg = { 0.0f, 0.0f, 0.0f };

            if (result.bounding_boxes_count > 0) {
                auto best_bb = result.bounding_boxes[0];
                if (best_bb.value >= 0.35f) {
                    UART_Printf("Best Target '%s' (%.2f) at x: %ld, y: %ld\r\n", best_bb.label, best_bb.value, best_bb.x, best_bb.y);

                    int center_x = best_bb.x + (best_bb.width / 2);
                    int center_y = best_bb.y + (best_bb.height / 2);

                    Draw_5x5_BlueDot(crop_buffer, center_x, center_y);

                    target_msg.x = (float) center_x;
                    target_msg.y = (float) center_y;
                    target_msg.detected = 1.0f;
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
