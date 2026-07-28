/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "motor.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_Controller pan_pid;
PID_Controller tilt_pid;

uint32_t last_random_tick = 0;
uint8_t rx_data;             // 시리얼 수신 버퍼 1바이트
uint16_t pan_val = ANGLE_MID;  // 초기 Pan 각도 (중앙)
uint16_t tilt_val = ANGLE_MID; // 초기 Tilt 각도 (중앙)


float target_x, target_y;

// uint8_t frame_buffer[IMG_SIZE];
extern UART_HandleTypeDef huart1; // 사용 중인 UART 핸들러 (CubeMX 설정에 맞게 변경)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

void Update_Target_Values(PID_Controller *pan, PID_Controller *tilt)
{
    // 예시 1: 카메라가 없을 때, 시간이 지남에 따라 타겟을 좌우로 흔들어 테스트하고 싶다면?
    // uint32_t tick = HAL_GetTick();
    
    // // 소수점 연산으로 명확하게 처리
    // pan->target  = 1500.0f + 500.0f  * sinf((float)tick / 500.0f); 
    // tilt->target = 1500.0f + 1000.0f * sinf(2.0f * (float)tick / 500.0f);
    // printf("pan : %d\r\n", (int)pan->target);
    // printf("tilt : %d\r\n", (int)tilt->target);
    // 마지막으로 타겟이 갱신된 시간을 기억하는 변수
    static uint32_t last_update_tick = 0;
    uint32_t current_tick = HAL_GetTick();

    // 0.3초(300ms)가 경과했을 때만 타겟 값 변경
    //if (current_tick - last_update_tick >= 300) {
        last_update_tick = current_tick; // 시간 갱신

        // 예시: 300ms마다 새로운 타겟으로 갱신 (원하시는 동작으로 변경 가능)
        pan->target = 1500.0f + 500.0f * sinf((float)current_tick / 500.0f); 
        tilt->target = 1500.0f + 1000.0f * sinf((float)current_tick / 500.0f);
        
        printf("pan : %d\r\n", (int)pan->target);
        printf("tilt : %d\r\n", (int)tilt->target);
    //}
}

void Gimbal_Sweep_Step(void)
{
    // 정적(static) 변수로 현재 위치와 목표 방향을 유지
    static uint16_t current_pan = 1250;
    static uint16_t current_tilt = 1250;
    static uint16_t target_pan = 500;
    static uint16_t target_tilt = 500;

    // 1. 목표 지점(오른쪽 위 500 또는 왼쪽 아래 1250)에 도달했는지 확인
    if (current_pan == target_pan && current_tilt == target_tilt)
    {
        // 도달했다면 다음 목표를 반대편으로 토글
        if (target_pan == ANGLE_MIN) // 현재 500이면 -> 1250으로 복귀 설정
        {
            target_pan = ANGLE_MAX;
            target_tilt = ANGLE_MAX;
        }
        else // 현재 1250이면 -> 500으로 이동 설정
        {
            target_pan = ANGLE_MIN;
            target_tilt = ANGLE_MIN;
        }
    }

    // 2. Pan 축 1 CCR씩 증감
    if (current_pan > target_pan) {
        current_pan--;
    } else if (current_pan < target_pan) {
        current_pan++;
    }

    // 3. Tilt 축 1 CCR씩 증감
    if (current_tilt > target_tilt) {
        current_tilt--;
    } else if (current_tilt < target_tilt) {
        current_tilt++;
    }

    printf("Pan_CCR: %4d | Tilt_CCR: %4d | Target(P/T): %4d\r\n", 
           current_pan, current_tilt, target_pan);

    // 4. 하드웨어 PWM 레지스터에 반영 (사용 중인 타이머 채널에 맞게 수정)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pan);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_tilt);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_DCMI_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  
  PID_Init(&pan_pid, 0.3f, 0.01f, 0.05f, (float)ANGLE_MID);
  PID_Init(&tilt_pid, 0.3f, 0.01f, 0.05f, (float)ANGLE_MID);
  
  printf("Started!\r\n"  );
  last_random_tick = HAL_GetTick();
  
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pan_val);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, tilt_val);

  
  
 
  
  // --- 선형 보간을 위한 실시간 상태 변수들 ---
  static float current_x = 500.0f;
  static float current_y = 500.0f;
  int random_num;
  int ai_delay_counter=0;
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    // 1. 랜덤한 타이밍(2~6틱 = 40ms~120ms)마다 새로운 AI(더미) 타겟 좌표를 가져옴
    if (ai_delay_counter == 0)
    {
        Get_Dummy_Target_Coords(&target_x, &target_y);
        random_num = rand() % (6 - 2 + 1) + 2; // 다음 갱신까지 대기할 틱 수 (2~6)
    }

    ai_delay_counter++;
    if (ai_delay_counter >= random_num)
    {
        ai_delay_counter = 0;
    }

    // 2. 현재 위치와 타겟 간의 거리 계산
    float distance = sqrtf(powf(target_x - current_x, 2) + powf(target_y - current_y, 2));

    // 3. 거리에 따른 동적 smoothing_alpha 설정
    float smoothing_alpha;
    if (distance > 50.0f) {
        smoothing_alpha = 0.4f;  // 멀리 떨어져 있을 때는 빠르게 추종
    } else {
        smoothing_alpha = 0.15f; // 가까워지면 부드럽게 안착 (진동 방지)
    }

    // 4. EMA 필터 적용
    current_x = current_x + smoothing_alpha * (target_x - current_x);
    current_y = current_y + smoothing_alpha * (target_y - current_y);

    // 5. 모니터링 출력
    printf("Target(%.1f, %.1f) | EMA(%.1f, %.1f) | Alpha: %.2f | Pan: %d | Tilt: %d\r\n", 
           target_x, target_y, 
           current_x, current_y, 
           smoothing_alpha,
           (int)pan_pid.current, (int)tilt_pid.current);

    // 6. PID 제어기로 투입
    Motor_PID_Process_With_Error(&pan_pid, &tilt_pid, (int)current_x, (int)current_y, &htim2);

    // 7. 20ms 주기 유지
    HAL_Delay(20);
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 48;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV4;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 키보드 입력에 따라 조정할 델타(증감) 값 설정 (원하는 만큼 조절 가능)
#define STEP_SIZE 25 
static uint8_t tilt_toggle_state = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 입력받은 문자(rx_data)에 따른 Pan/Tilt 값 조정
        if (rx_data == 'w' || rx_data == 'W')
        {
            tilt_val += STEP_SIZE; // 위쪽 (Tilt 증가)
        }
        else if (rx_data == 's' || rx_data == 'S')
        {
            tilt_val -= STEP_SIZE; // 아래쪽 (Tilt 감소)
        }
        else if (rx_data == 'a' || rx_data == 'A')
        {
            pan_val += STEP_SIZE;  // 오른쪽 (Pan 증가)
        }
        else if (rx_data == 'd' || rx_data == 'D')
        {
            
            pan_val -= STEP_SIZE;  // 왼쪽 (Pan 감소)
        }
        else if (rx_data == 't' || rx_data == 'T')
        {
            if (tilt_toggle_state == 0) {
                tilt_val = 500;
                tilt_toggle_state = 1;
            } else {
                tilt_val = 1250;
                tilt_toggle_state = 0;
            }
        }

        // 서보모터 안전 범위 제한 (500 ~ 2500) 강제 적용
        if (pan_val > ANGLE_MAX) pan_val = ANGLE_MAX;
        if (pan_val < ANGLE_MIN) pan_val = ANGLE_MIN;
        if (tilt_val > ANGLE_MAX) tilt_val = ANGLE_MAX;
        if (tilt_val < ANGLE_MIN) tilt_val = ANGLE_MIN;

        // 실제 타이머 CCR 값 갱신 (TIM2 채널 1: Pan, 채널 2: Tilt)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pan_val);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, tilt_val);

        // 디버깅용 현재 값 출력
        printf("WASD Input [%c] -> Pan: %d, Tilt: %d\r\n", rx_data, pan_val, tilt_val);

        // 다음 1바이트 수신을 위해 인터럽트 재활성화
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
