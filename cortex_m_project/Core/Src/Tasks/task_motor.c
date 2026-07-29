#include "task_motor.h"
#include "app_globals.h"
#include "motor.h"
#include "joystick.h"

extern TIM_HandleTypeDef htim2;

#define MOTOR_TASK_PERIOD_MS   20U

// 서보 제어 및 상태 변수 은닉화
static PID_Controller pan_pid;
static PID_Controller tilt_pid;
static uint16_t pan_val = ANGLE_MID;
static uint16_t tilt_val = ANGLE_MID;
static volatile uint8_t track_state = MACHINE_STATE_IDLE;

// Getter 구현
uint8_t Motor_GetTrackState(void) {
    return track_state;
}

void MotorTask(void) {
    UART_Printf(" MotorTask Started...\r\n");

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    PID_Init(&pan_pid, 0.5f, 0.00f, 0.0f, ANGLE_MID);
    PID_Init(&tilt_pid, 0.5f, 0.00f, 0.0f, ANGLE_MID);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pan_val);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, tilt_val);

    Joystick_Init();
    Motor_ManualSetPosition((float) ANGLE_MID, (float) ANGLE_MID);

    TargetCoord_t rx_msg;
    JoystickInput_t joy;

    for (;;) {
        Joystick_Read(&joy);

        if (joy.button_pressed) {
            if (track_state == MACHINE_STATE_MANUAL) {
                float pan_now, tilt_now;
                Motor_ManualGetPosition(&pan_now, &tilt_now);

                pan_val = (uint16_t) pan_now;
                tilt_val = (uint16_t) tilt_now;
                pan_pid.current = pan_now;
                tilt_pid.current = tilt_now;

                track_state = MACHINE_STATE_IDLE;
                UART_Printf("Mode -> AUTO\r\n");
            } else {
                Motor_ManualSetPosition((float) __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1), (float) __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2));

                track_state = MACHINE_STATE_MANUAL;
                UART_Printf("Mode -> MANUAL\r\n");
            }
        }

        if (track_state == MACHINE_STATE_MANUAL) {
            Motor_ManualProcess(&joy, &htim2, MOTOR_TASK_PERIOD_MS);
        } else {
            osStatus_t status = osMessageQueueGet(Queue1Handle, &rx_msg, NULL, 20);

            if (status == osOK && rx_msg.detected > 0.5f) {
                Motor_PID_Process_With_Error(&pan_pid, &tilt_pid, rx_msg.x, rx_msg.y, &htim2);
                UART_Printf("Target(%.1f, %.1f) | Pan_Cur: %d | Tilt_Cur: %d\r\n", rx_msg.x, rx_msg.y, (int) pan_pid.current, (int) tilt_pid.current);
            }
        }

        osDelay(MOTOR_TASK_PERIOD_MS);
    }
}
