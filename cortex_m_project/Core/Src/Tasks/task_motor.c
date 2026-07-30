#include "task_motor.h"
#include "app_globals.h"
#include "motor.h"
#include "joystick.h"
#include <math.h>

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

#define MOTOR_TASK_PERIOD_MS   20U

// 서보 제어 및 상태 변수 은닉화
static PID_Controller pan_pid;
static PID_Controller tilt_pid;
static uint16_t pan_val = ANGLE_MID;
static uint16_t tilt_val = ANGLE_MID;
static volatile uint8_t track_state = MACHINE_STATE_IDLE;

uint8_t tilt_toggle_state = 0;

// pid 상수 조절 커맨드
uint8_t rx_data = 0;

// Getter 구현
uint8_t Motor_GetTrackState(void) {
    return track_state;
}

void Raser_ON() {
    HAL_GPIO_WritePin(RASER_GPIO_Port, RASER_Pin, GPIO_PIN_SET);
}

void Raser_OFF() {
    HAL_GPIO_WritePin(RASER_GPIO_Port, RASER_Pin, GPIO_PIN_RESET);
}

void MotorTask(void) {
    UART_Printf(" MotorTask Started...\r\n");

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    PID_Init(&pan_pid, 1.2f, 0.001f, 0.01f, 48.0f);
    PID_Init(&tilt_pid, 1.2f, 0.001f, 0.01f, 48.0f);

    UART_Printf("Pan(360)  -> P: %.2f | I: %.2f | D: %.2f\r\n", pan_pid.kp, pan_pid.ki, pan_pid.kd);
    UART_Printf("Tilt(180) -> P: %.2f | I: %.2f | D: %.2f\r\n", tilt_pid.kp, tilt_pid.ki, tilt_pid.kd);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pan_val);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, tilt_val);

    Joystick_Init();
    Motor_ManualSetPosition((float) ANGLE_MID, (float) ANGLE_MID);

    TargetCoord_t rx_msg;
    JoystickInput_t joy;

    // 순찰(Patrol) 및 락온용 로컬 상태 변수
    float patrol_pan = (float) ANGLE_MID;
    float patrol_dir = 1.0f; // 1.0: 우측 이동, -1.0: 좌측 이동
    uint8_t lockon_counter = 0;

    HAL_UART_Receive_IT(&huart1, &rx_data, 1);

    for (;;) {
        Joystick_Read(&joy);

        // [조이스틱 모드 전환 로직]
        if (joy.button_pressed) {
            if (track_state == MACHINE_STATE_MANUAL) {
                float pan_now, tilt_now;
                Motor_ManualGetPosition(&pan_now, &tilt_now);
                patrol_pan = pan_now; // 수동 제어 끝난 위치부터 순찰 재시작

                pan_pid.integral = 0.0f;
                tilt_pid.integral = 0.0f;
                lockon_counter = 0;
                Raser_OFF();

                UART_Printf("MACHINE_STATE_IDLE\r\n");
                track_state = MACHINE_STATE_IDLE;
            } else {

                Raser_OFF();
                UART_Printf("MACHINE_STATE_MANUAL\r\n");
                track_state = MACHINE_STATE_MANUAL;
            }
        }

        // [상태별 모터 제어 실행]
        if (track_state == MACHINE_STATE_MANUAL) {
            Motor_ManualProcess(&joy, &htim2, MOTOR_TASK_PERIOD_MS);
        } else {
            // 🚀 핵심 변경 1: 큐를 기다리지 않고(Timeout 0) 프레임 도착 여부만 즉시 확인
            osStatus_t status = osMessageQueueGet(Queue1Handle, &rx_msg, NULL, 0);

            // 📷 카메라 프레임이 도착했을 때만 (약 140ms 마다 1번씩 실행됨)
            if (status == osOK) {
                if (rx_msg.detected) {
                    // 상태 2 & 3: 타겟 발견 (TRACKING or LOCKON)
                    float err_x = rx_msg.x - LASER_TARGET_X;
                    float err_y = rx_msg.y - LASER_TARGET_Y;
                    float distance = sqrtf((err_x * err_x) + (err_y * err_y));

                    if (distance <= LOCKON_ERROR_MARGIN) {
                        if (lockon_counter < LOCKON_MAINTAIN_COUNT) {
                            lockon_counter++;
                        }
                    } else {
                        lockon_counter = 0;
                    }

                    if (lockon_counter >= LOCKON_MAINTAIN_COUNT) {
                        track_state = MACHINE_STATE_LOCKON;
                        Raser_ON();
                    } else {
                        track_state = MACHINE_STATE_TRACKING;
                        Raser_OFF();
                    }

                    // PID 연산은 새 프레임이 갱신되었을 때만 1회 실행하여 튀는 현상 방지
                    Motor_PID_Process_With_Error(&pan_pid, &tilt_pid, rx_msg.x, rx_msg.y, &htim2);
                    Motor_ManualGetPosition(&patrol_pan, NULL);

                } else {
                    // 타겟 미발견 시 상태만 IDLE로 변경
                    track_state = MACHINE_STATE_IDLE;
                    lockon_counter = 0;
                    Raser_OFF();
                    pan_pid.integral = 0.0f;
                    tilt_pid.integral = 0.0f;
                }
            }

            // 🚀 핵심 변경 2: 순찰 모드는 큐 수신 여부와 관계없이 매 루프(20ms마다) 부드럽게 실행
            if (track_state == MACHINE_STATE_IDLE) {
                patrol_pan += (patrol_dir * PATROL_STEP_ANGLE);

                if (patrol_pan >= PATROL_PAN_MAX) {
                    patrol_pan = PATROL_PAN_MAX;
                    patrol_dir = -1.0f;
                } else if (patrol_pan <= PATROL_PAN_MIN) {
                    patrol_pan = PATROL_PAN_MIN;
                    patrol_dir = 1.0f;
                }

                Motor_ManualSetPosition(patrol_pan, (float) ANGLE_MID);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t )patrol_pan);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)ANGLE_MID);
            }
        }

        osDelay(MOTOR_TASK_PERIOD_MS);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {

        // ==========================================
        // 1. Pan 모터 (360도) PID 튜닝 (P, I, D 키)
        // ==========================================
        if (rx_data == 'q')
            pan_pid.kp += 0.15f;
        else if (rx_data == 'a')
            pan_pid.kp -= 0.15f;

        else if (rx_data == 'w')
            pan_pid.ki += 0.001f;
        else if (rx_data == 's')
            pan_pid.ki -= 0.001f;

        else if (rx_data == 'e')
            pan_pid.kd += 0.1f;
        else if (rx_data == 'd')
            pan_pid.kd -= 0.1f;

        // ==========================================
        // 2. Tilt 모터 (180도) PID 튜닝 (Q, W, E 키)
        // ==========================================
        else if (rx_data == 'Q')
            tilt_pid.kp += 0.1f;  // Tilt는 민감하므로 0.1씩 조절
        else if (rx_data == 'A')
            tilt_pid.kp -= 0.1f;

        else if (rx_data == 'W')
            tilt_pid.ki += 0.001f;
        else if (rx_data == 'S')
            tilt_pid.ki -= 0.001f;

        else if (rx_data == 'E')
            tilt_pid.kd += 0.05f;
        else if (rx_data == 'D')
            tilt_pid.kd -= 0.05f;

        // ==========================================
        // 방어 로직: PID 상수가 음수가 되지 않도록 클램핑
        // ==========================================
        if (pan_pid.kp < 0.0f)
            pan_pid.kp = 0.0f;
        if (pan_pid.ki < 0.0f)
            pan_pid.ki = 0.0f;
        if (pan_pid.kd < 0.0f)
            pan_pid.kd = 0.0f;

        if (tilt_pid.kp < 0.0f)
            tilt_pid.kp = 0.0f;
        if (tilt_pid.ki < 0.0f)
            tilt_pid.ki = 0.0f;
        if (tilt_pid.kd < 0.0f)
            tilt_pid.kd = 0.0f;

        // 현재 설정된 PID 값 콘솔 출력 (디버깅용)
        UART_Printf("\r\n[PID TUNING]\r\n");
        UART_Printf("Pan(360)  -> P: %.2f | I: %.3f | D: %.2f\r\n", pan_pid.kp, pan_pid.ki, pan_pid.kd);
        UART_Printf("Tilt(180) -> P: %.2f | I: %.3f | D: %.2f\r\n", tilt_pid.kp, tilt_pid.ki, tilt_pid.kd);

        // 다음 1바이트 수신을 위해 인터럽트 재활성화
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}
