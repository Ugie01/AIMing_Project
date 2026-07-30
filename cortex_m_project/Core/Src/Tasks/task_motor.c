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
static volatile bool pid_updated_flag = false;

uint8_t tilt_toggle_state = 0;
#define STEP_SIZE 25

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

    // --- 프로파일링용 변수 추가 ---
    uint32_t motor_profiling_start = HAL_GetTick();
    uint32_t motor_loop_count = 0;
    uint32_t sum_joystick_time = 0;
    uint32_t sum_motor_logic_time = 0;
    uint8_t miss_counter = 0;
    const uint8_t MISS_TOLERANCE_COUNT = 10; // 10프레임 동안은 현재 위치 유지

    for (;;) {
        // PID 테스트 프린트
        if (pid_updated_flag) {
            pid_updated_flag = false;
            UART_Printf("\r\n[PID TUNING]\r\n");
            UART_Printf("Pan(360)  -> P: %.2f | I: %.3f | D: %.2f\r\n", pan_pid.kp, pan_pid.ki, pan_pid.kd);
            UART_Printf("Tilt(180) -> P: %.2f | I: %.3f | D: %.2f\r\n", tilt_pid.kp, tilt_pid.ki, tilt_pid.kd);
        }

        uint32_t t_start = HAL_GetTick(); // 시작 시간 기록
        Joystick_Read(&joy);
        uint32_t t_joy = HAL_GetTick(); // 조이스틱 완료 시간 기록

        // [조이스틱 모드 전환 로직]
        if (joy.button_pressed) {
            if (track_state == MACHINE_STATE_MANUAL) {
                float pan_now, tilt_now;
                Motor_ManualGetPosition(&pan_now, &tilt_now);
                patrol_pan = pan_now;

                pan_pid.integral = 0.0f;
                tilt_pid.integral = 0.0f;
                lockon_counter = 0;
                Raser_OFF();

                // 🔥 수동 모드 종료 시 조이스틱 DMA 정지
                Joystick_Stop_DMA();

                UART_Printf("MACHINE_STATE_IDLE\r\n");
                track_state = MACHINE_STATE_IDLE;
            } else {
                Raser_OFF();
                UART_Printf("MACHINE_STATE_MANUAL\r\n");

                // 🔥 수동 모드 진입 시 조이스틱 DMA 시작 (약 40ms 소요되지만 1회성이므로 무방)
                Joystick_Start_DMA();

                Motor_ManualSetPosition((float) ANGLE_MID, (float) ANGLE_MID);
                track_state = MACHINE_STATE_MANUAL;
            }
        }

        // [상태별 모터 제어 실행]
        if (track_state == MACHINE_STATE_MANUAL) {
            Motor_ManualProcess(&joy, &htim2, MOTOR_TASK_PERIOD_MS);
        } else {
            //  큐를 기다리지 않고(Timeout 0) 프레임 도착 여부만 즉시 확인
            osStatus_t status = osMessageQueueGet(Queue1Handle, &rx_msg, NULL, 0);

            // 카메라 프레임이 도착했을 때만 (약 140ms 마다 1번씩 실행됨)
            if (status == osOK) {
                if (rx_msg.detected) {
                    // 🎯 타겟을 찾은 경우: miss_counter 초기화
                    miss_counter = 0;

                    // 상태 2 & 3: 타겟 발견 (TRACKING or LOCKON)
                    float err_x = rx_msg.x - LASER_TARGET_X;
                    float err_y = rx_msg.y - LASER_TARGET_Y;
                    float distance_sq = (err_x * err_x) + (err_y * err_y);
                    float margin_sq = (float) (LOCKON_ERROR_MARGIN) * (float) (LOCKON_ERROR_MARGIN);

                    if (distance_sq <= margin_sq) {
                        if (lockon_counter < LOCKON_MAINTAIN_COUNT) {
                            lockon_counter++;
                        }
                    } else {
                        lockon_counter = 0;
                    }

                    if (lockon_counter >= LOCKON_MAINTAIN_COUNT) {
                        // 상태가 변경되는 '최초 1회'에만 UART로 FIRE 전송
                        if (track_state != MACHINE_STATE_LOCKON) {
                            UART_Printf("FIRE\r\n");
                        }

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
                    // ❌ 타겟 미발견 시 로직
                    if (miss_counter < MISS_TOLERANCE_COUNT) {
                        // 1. 유예 기간 중: 카운터만 증가시키고, 현재 모터 위치를 그대로 유지 (아무것도 안 함)
                        // 상태는 추적(TRACKING) 상태를 유지하여 UI상에서도 타겟을 찾고 있음을 표시
                        miss_counter++;
                        lockon_counter = 0; // 타겟을 놓쳤으므로 락온은 즉시 풀림
                        Raser_OFF();

                        // (선택) UI에서 잠시 잃어버렸음을 명확히 하려면 상태를 IDLE로 바꿔도 됩니다.
                        // 여기서는 "가만히 대기"하는 상태를 TRACKING의 연장선으로 보았습니다.
                    } else {
                        // 2. 유예 기간 초과 (10프레임 이상 미검출): 순찰(IDLE) 모드로 완전 전환
                        track_state = MACHINE_STATE_IDLE;
                        lockon_counter = 0;
                        Raser_OFF();
                        pan_pid.integral = 0.0f;
                        tilt_pid.integral = 0.0f;
                    }
                }
            }

            // 순찰 모드는 큐 수신 여부와 관계없이 매 루프(20ms마다) 부드럽게 실행
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
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)ANGLE_MID - 100.0f);
            }
        }

        // 모터 로직 완료 시간 기록
        uint32_t t_logic = HAL_GetTick();

        // --- 측정 시간 누적 ---
        sum_joystick_time += (t_joy - t_start);
        sum_motor_logic_time += (t_logic - t_joy);
        motor_loop_count++;
        if ((HAL_GetTick() - motor_profiling_start) >= 30000U) {
            if (motor_loop_count > 0) {
                UART_Printf("\r\n=== Motor Task Profiling (Avg over %lu loops) ===\r\n", motor_loop_count);
                UART_Printf(" 1. Joystick Read : %lu ms\r\n", sum_joystick_time / motor_loop_count);
                UART_Printf(" 2. Motor Logic   : %lu ms\r\n", sum_motor_logic_time / motor_loop_count);
                UART_Printf("===================================================\r\n");
            }

            // 변수 초기화
            sum_joystick_time = 0;
            sum_motor_logic_time = 0;
            motor_loop_count = 0;
            motor_profiling_start = HAL_GetTick();
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
    tilt_pid.kp += 0.1f;// Tilt는 민감하므로 0.1씩 조절
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

    pid_updated_flag = true;
    // 다음 1바이트 수신을 위해 인터럽트 재활성화
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);

    // 입력받은 문자(rx_data)에 따른 Pan/Tilt 값 조정
    //            if (rx_data == 'w' || rx_data == 'W') {
//                tilt_val += STEP_SIZE; // 위쪽 (Tilt 증가)
//            } else if (rx_data == 's' || rx_data == 'S') {
//                tilt_val -= STEP_SIZE; // 아래쪽 (Tilt 감소)
//            } else if (rx_data == 'a' || rx_data == 'A') {
//                pan_val += STEP_SIZE;  // 오른쪽 (Pan 증가)
//            } else if (rx_data == 'd' || rx_data == 'D') {
//
//                pan_val -= STEP_SIZE;  // 왼쪽 (Pan 감소)
//            } else if (rx_data == 't' || rx_data == 'T') {
//                if (tilt_toggle_state == 0) {
//                    tilt_val = 500;
//                    tilt_toggle_state = 1;
//                } else {
//                    tilt_val = 1250;
//                    tilt_toggle_state = 0;
//                }
//            }
//
//            // 서보모터 안전 범위 제한 (500 ~ 2500) 강제 적용
//            if (pan_val > ANGLE_MAX)
//                pan_val = ANGLE_MAX;
//            if (pan_val < ANGLE_MIN)
//                pan_val = ANGLE_MIN;
//            if (tilt_val > ANGLE_MAX)
//                tilt_val = ANGLE_MAX;
//            if (tilt_val < ANGLE_MIN)
//                tilt_val = ANGLE_MIN;
//
//            // 실제 타이머 CCR 값 갱신 (TIM2 채널 1: Pan, 채널 2: Tilt)
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pan_val);
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, tilt_val);
//
//            // 디버깅용 현재 값 출력
//        UART_Printf("WASD Input [%c] -> Pan: %d, Tilt: %d\r\n", rx_data, pan_val, tilt_val);
//
//            // 다음 1바이트 수신을 위해 인터럽트 재활성화
//            HAL_UART_Receive_IT(&huart1, &rx_data, 1);
//
    }
}
