
## 💻 3. `main.c` 활용 예시 코드

분리된 모듈을 STM32 메인 루프에서 어떻게 호출하는지 보여주는 예시입니다.

```c
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "motor.h" // 모듈 포함

PID_Controller pan_pid;
PID_Controller tilt_pid;
uint8_t rx_data;

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  
  // 타이머 및 PWM 시작
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);

  // PID 초기화 (초기 위치 0도)
  PID_Init(&pan_pid, 0.3f, 0.01f, 0.05f, 0.0f);
  PID_Init(&tilt_pid, 0.3f, 0.01f, 0.05f, 0.0f);

  while (1)
  {
      // 매 20ms마다 각도 단위(예: 중앙 0도, 0도)를 타겟으로 주고 PID 제어 실행
      Motor_Angle_Process(&pan_pid, &tilt_pid, &htim2, 0.0f, 0.0f);

      // 서보모터 표준 제어 주기 (20ms)
      HAL_Delay(20);
  }
}

```

---


