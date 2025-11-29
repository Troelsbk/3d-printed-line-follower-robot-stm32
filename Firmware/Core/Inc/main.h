/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define led_hard_left_Pin GPIO_PIN_0
#define led_hard_left_GPIO_Port GPIOA
#define sensor_hard_right_Pin GPIO_PIN_1
#define sensor_hard_right_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define sensor_right_Pin GPIO_PIN_3
#define sensor_right_GPIO_Port GPIOA
#define led_left_Pin GPIO_PIN_4
#define led_left_GPIO_Port GPIOA
#define led_right_Pin GPIO_PIN_5
#define led_right_GPIO_Port GPIOA
#define sensor_left_Pin GPIO_PIN_6
#define sensor_left_GPIO_Port GPIOA
#define led_hard_right_Pin GPIO_PIN_7
#define led_hard_right_GPIO_Port GPIOA
#define sensor_hard_left_Pin GPIO_PIN_1
#define sensor_hard_left_GPIO_Port GPIOB
#define pwm_left_Pin GPIO_PIN_9
#define pwm_left_GPIO_Port GPIOA
#define pwm_right_Pin GPIO_PIN_10
#define pwm_right_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define yellow_led_Pin GPIO_PIN_4
#define yellow_led_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
