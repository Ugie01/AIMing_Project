#include "task_motor.h"
#include "app_globals.h"
#include "motor.h"
#include "joystick.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

#define MOTOR_TASK_PERIOD_MS   20U

// 서보 제어 및 상태 변수 은닉화
static PID_Controller pan_pid;
static PID_Controller tilt_pid;
static uint16_t pan_val = ANGLE_MID;
static uint16_t tilt_val = ANGLE_MID;
static volatile uint8_t track_state = MACHINE_STATE_IDLE;

#define STEP_SIZE  25;
uint8_t tilt_toggle_state = 0;
uint8_t rx_data = 0;

// Getter 구현
uint8_t Motor_GetTrackState(void) {
    return track_state;
}

void MotorTask(void) {
    UART_Printf(" MotorTask Started...\r\n");

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    PID_Init(&pan_pid, 6.0f, 0.00f, 0.5f, 48.0f);
    PID_Init(&tilt_pid, 0.6f, 0.00f, 0.1f, 48.0f);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pan_val);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, tilt_val);

    Joystick_Init();
    Motor_ManualSetPosition((float) PAN_STOP_PWM, (float) 2000.0f);

    TargetCoord_t rx_msg;
    JoystickInput_t joy;

    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    for (;;) {
        Joystick_Read(&joy);

        if (joy.button_pressed) {
            if (track_state == MACHINE_STATE_MANUAL) {
                float pan_now, tilt_now;
                Motor_ManualGetPosition(&pan_now, &tilt_now);

                pan_val = (uint16_t) pan_now;
                tilt_val = (uint16_t) tilt_now;

                pan_pid.integral = 0.0f;
                tilt_pid.integral = 0.0f;

                track_state = MACHINE_STATE_IDLE;
            } else {
                Motor_ManualSetPosition((float) __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1), (float) __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2));

                track_state = MACHINE_STATE_MANUAL;
            }
        }

        if (track_state == MACHINE_STATE_MANUAL) {
            Motor_ManualProcess(&joy, &htim2, MOTOR_TASK_PERIOD_MS);
        } else {
            osStatus_t status = osMessageQueueGet(Queue1Handle, &rx_msg, NULL, 20);

            if (status == osOK && rx_msg.detected > 0.5f) {
                // 새로운 비전 데이터가 들어왔을 때만 PID 수행
                Motor_PID_Process_With_Error(&pan_pid, &tilt_pid, rx_msg.x, rx_msg.y, &htim2);
                // UART_Printf("Target(%.1f, %.1f) | Pan_Cur: %d | Tilt_Cur: %d\r\n", rx_msg.x, rx_msg.y, (int) pan_pid.current, (int) tilt_pid.current);

            } else if (track_state != MACHINE_STATE_MANUAL) {
                // 객체를 놓쳤을 때 Pan 모터(360도)가 계속 도는 것을 방지
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t )PAN_STOP_PWM);
            }
        }

        osDelay(1);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {

        // ==========================================
        // 1. Pan 모터 (360도) PID 튜닝 (P, I, D 키)
        // ==========================================
        if (rx_data == 'q')
            pan_pid.kp += 0.5f;
        else if (rx_data == 'a')
            pan_pid.kp -= 0.5f;

        else if (rx_data == 'w')
            pan_pid.ki += 0.01f;
        else if (rx_data == 's')
            pan_pid.ki -= 0.01f;

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
            tilt_pid.ki += 0.01f;
        else if (rx_data == 'S')
            tilt_pid.ki -= 0.01f;

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
        UART_Printf("Pan(360)  -> P: %.2f | I: %.2f | D: %.2f\r\n", pan_pid.kp, pan_pid.ki, pan_pid.kd);
        UART_Printf("Tilt(180) -> P: %.2f | I: %.2f | D: %.2f\r\n", tilt_pid.kp, tilt_pid.ki, tilt_pid.kd);

        // 다음 1바이트 수신을 위해 인터럽트 재활성화
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}
