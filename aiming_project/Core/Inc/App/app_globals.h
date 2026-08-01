#ifndef APP_GLOBALS_H_
#define APP_GLOBALS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"

// 1. 공용 상태 (MachineState_t)
typedef enum {
    MACHINE_STATE_IDLE = 0, /*!< 목표 탐색 중          : 초록 */
    MACHINE_STATE_TRACKING = 1, /*!< 목표 포착/추적        : 노랑 */
    MACHINE_STATE_LOCKON = 2, /*!< 목표 사격 중          : 빨강 */
    MACHINE_STATE_MANUAL = 3 /*!< 조이스틱 수동 조작 중 : 시안 */
} MachineState_t;

// 2. 태스크간 통신 메시지 구조체
typedef struct {
    float x;
    float y;
    float detected; // 1.0f: 객체 있음, 0.0f: 객체 없음
} TargetCoord_t;

// 3. 공용 큐 핸들
extern osMessageQueueId_t Queue1Handle;

// 4. UART Printf 공용 유틸리티
void UART_Printf(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* APP_GLOBALS_H_ */
