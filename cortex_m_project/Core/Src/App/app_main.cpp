#include "app_main.h"
#include "app_globals.h"
#include "camera.h"
#include <stdarg.h>
#include <stdio.h>

// HAL 핸들 선언
extern UART_HandleTypeDef huart1;
extern DCMI_HandleTypeDef hdcmi;
extern osSemaphoreId_t cameraFrameSemHandle;

// UART1 시리얼 프린트 (전역 유틸리티)
void UART_Printf(const char *format, ...) {
    char loc_buf[256];
    va_list args;

    va_start(args, format);
    int len = vsnprintf(loc_buf, sizeof(loc_buf), format, args);
    va_end(args);

    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t*) loc_buf, (uint16_t) len, 1000);
    }
}

extern "C" {

// C++ 애플리케이션 엔트리 포인트
void app_main(void) {
    UART_Printf("System Initialized.\r\n");
}

// DCMI 프레임 수신 완료 콜백 함수
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi) {
    if (cameraFrameSemHandle != NULL) {
        // 🚀 인터럽트 안에서 세마포어 릴리즈 (Task를 깨움)
        osSemaphoreRelease (cameraFrameSemHandle);
    }
}

// 에러 발생 시 강제 복구 콜백 함수
void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi) {
	hdcmi->State = HAL_DCMI_STATE_READY;
    Camera_ClearFrameReady();
}



}
