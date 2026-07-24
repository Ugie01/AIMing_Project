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
#define SG90_PAN_Pin GPIO_PIN_15
#define SG90_PAN_GPIO_Port GPIOA
#define Camera_I2C_SCL_Pin GPIO_PIN_6
#define Camera_I2C_SCL_GPIO_Port GPIOB
#define Camera_I2C_SDA_Pin GPIO_PIN_7
#define Camera_I2C_SDA_GPIO_Port GPIOB
#define SG90_TILT_Pin GPIO_PIN_3
#define SG90_TILT_GPIO_Port GPIOB
#define Serial_RX_Pin GPIO_PIN_10
#define Serial_RX_GPIO_Port GPIOA
#define Serial_TX_Pin GPIO_PIN_9
#define Serial_TX_GPIO_Port GPIOA
#define LED_PIN_Pin GPIO_PIN_13
#define LED_PIN_GPIO_Port GPIOC
#define RASER_PIN_Pin GPIO_PIN_7
#define RASER_PIN_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
