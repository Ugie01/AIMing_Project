/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAM_D6_Pin GPIO_PIN_6
#define CAM_D6_GPIO_Port GPIOI
#define CAM_VSYNC_Pin GPIO_PIN_5
#define CAM_VSYNC_GPIO_Port GPIOI
#define CAM_D5_Pin GPIO_PIN_4
#define CAM_D5_GPIO_Port GPIOI
#define LASER_Pin GPIO_PIN_4
#define LASER_GPIO_Port GPIOD
#define SERVO_PAN_Pin GPIO_PIN_15
#define SERVO_PAN_GPIO_Port GPIOA
#define CAM_D7_Pin GPIO_PIN_7
#define CAM_D7_GPIO_Port GPIOI
#define CAM_SCL_Pin GPIO_PIN_6
#define CAM_SCL_GPIO_Port GPIOB
#define CAM_D4_Pin GPIO_PIN_11
#define CAM_D4_GPIO_Port GPIOC
#define CAM_SDA_Pin GPIO_PIN_7
#define CAM_SDA_GPIO_Port GPIOB
#define SERVO_TILT_Pin GPIO_PIN_3
#define SERVO_TILT_GPIO_Port GPIOB
#define TFT_LED_Pin GPIO_PIN_12
#define TFT_LED_GPIO_Port GPIOC
#define TFT_CS_Pin GPIO_PIN_9
#define TFT_CS_GPIO_Port GPIOB
#define SERIAL_RX_Pin GPIO_PIN_10
#define SERIAL_RX_GPIO_Port GPIOA
#define SERIAL_TX_Pin GPIO_PIN_9
#define SERIAL_TX_GPIO_Port GPIOA
#define CAM_D2_Pin GPIO_PIN_8
#define CAM_D2_GPIO_Port GPIOC
#define CAM_D1_Pin GPIO_PIN_7
#define CAM_D1_GPIO_Port GPIOC
#define CAM_D0_Pin GPIO_PIN_6
#define CAM_D0_GPIO_Port GPIOC
#define JOY_SW_Pin GPIO_PIN_2
#define JOY_SW_GPIO_Port GPIOA
#define JOY_Y_Pin GPIO_PIN_1
#define JOY_Y_GPIO_Port GPIOA
#define JOY_X_Pin GPIO_PIN_0
#define JOY_X_GPIO_Port GPIOA
#define TFT_RST_Pin GPIO_PIN_10
#define TFT_RST_GPIO_Port GPIOB
#define TFT_DC_Pin GPIO_PIN_11
#define TFT_DC_GPIO_Port GPIOB
#define CAM_PIXCLK_Pin GPIO_PIN_6
#define CAM_PIXCLK_GPIO_Port GPIOA
#define CAM_D3_Pin GPIO_PIN_12
#define CAM_D3_GPIO_Port GPIOH
#define CAM_HSYNC_Pin GPIO_PIN_8
#define CAM_HSYNC_GPIO_Port GPIOH
#define CAM_PWDN_Pin GPIO_PIN_12
#define CAM_PWDN_GPIO_Port GPIOB
#define TFT_MOSI_Pin GPIO_PIN_15
#define TFT_MOSI_GPIO_Port GPIOB
#define CAM_RET_Pin GPIO_PIN_7
#define CAM_RET_GPIO_Port GPIOH
#define TFT_SCK_Pin GPIO_PIN_13
#define TFT_SCK_GPIO_Port GPIOB
#define TFT_MISO_Pin GPIO_PIN_14
#define TFT_MISO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
