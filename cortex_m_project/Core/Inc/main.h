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
#define SERVO_PAN_Pin GPIO_PIN_15
#define SERVO_PAN_GPIO_Port GPIOA
#define SERVO_TILT_Pin GPIO_PIN_3
#define SERVO_TILT_GPIO_Port GPIOB
#define TFT_LED_Pin GPIO_PIN_12
#define TFT_LED_GPIO_Port GPIOC
#define TFT_CS_Pin GPIO_PIN_9
#define TFT_CS_GPIO_Port GPIOB
#define TFT_RST_Pin GPIO_PIN_10
#define TFT_RST_GPIO_Port GPIOB
#define TFT_DC_Pin GPIO_PIN_11
#define TFT_DC_GPIO_Port GPIOB
#define CAM_PWDN_Pin GPIO_PIN_12
#define CAM_PWDN_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_15
#define LCD_MOSI_GPIO_Port GPIOB
#define CAM_RET_Pin GPIO_PIN_7
#define CAM_RET_GPIO_Port GPIOH
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOB
#define LCD_MISO_Pin GPIO_PIN_14
#define LCD_MISO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
