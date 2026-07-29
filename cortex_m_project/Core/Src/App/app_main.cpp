#include "app_main.h"
#include "app_globals.h"
#include "camera.h"
#include <stdarg.h>
#include <stdio.h>

// HAL 핸들 선언
extern UART_HandleTypeDef huart1;
extern DCMI_HandleTypeDef hdcmi;

// UART1 시리얼 프린트 (전역 유틸리티)
void UART_Printf(const char *format, ...) {
    char loc_buf[256];
    va_list args;

    va_start(args, format);
    int len = vsnprintf(loc_buf, sizeof(loc_buf), format, args);
    va_end(args);

    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t*) loc_buf, (uint16_t) len, HAL_MAX_DELAY);
    }
}

extern "C" {

// C++ 애플리케이션 엔트리 포인트
void app_main(void) {
    UART_Printf("System Initialized.\r\n");
}

// DCMI 프레임 수신 완료 콜백 함수
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi) {
    Camera_SetFrameReady();
}

// 에러 발생 시 강제 복구 콜백 함수
void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi) {
	hdcmi->State = HAL_DCMI_STATE_READY;
    Camera_ClearFrameReady();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {

	}
}

}
