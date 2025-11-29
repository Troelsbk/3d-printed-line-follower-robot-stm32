/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
# define left_ch TIM_CHANNEL_2
# define right_ch TIM_CHANNEL_3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef enum
{
    straight   = 0,
    right      = 1,
    hard_right = 2,
    left       = 3,
    hard_left  = 4
} MotorState_t;

typedef struct
{
    uint8_t left_base;
    uint8_t right_base;

    uint8_t left_slow;
    uint8_t right_slow;

    uint8_t left_extra_slow;
    uint8_t right_extra_slow;

    uint8_t left_fast;
    uint8_t right_fast;

    uint8_t left_boost;
    uint8_t right_boost;
} MotorPwm_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static MotorState_t get_direction(void)
{
    /* all LEDs off */
    HAL_GPIO_WritePin(led_left_GPIO_Port,       led_left_Pin,       GPIO_PIN_RESET);
    HAL_GPIO_WritePin(led_right_GPIO_Port,      led_right_Pin,      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(led_hard_left_GPIO_Port,  led_hard_left_Pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(led_hard_right_GPIO_Port, led_hard_right_Pin, GPIO_PIN_RESET);

    if (HAL_GPIO_ReadPin(sensor_right_GPIO_Port, sensor_right_Pin) == GPIO_PIN_SET)
    {
        //HAL_GPIO_WritePin(led_right_GPIO_Port, led_right_Pin, GPIO_PIN_SET);
        return left;
    }
    else if (HAL_GPIO_ReadPin(sensor_hard_right_GPIO_Port, sensor_hard_right_Pin) == GPIO_PIN_SET)
    {
    	//HAL_GPIO_WritePin(led_hard_right_GPIO_Port, led_hard_right_Pin, GPIO_PIN_SET);
        return hard_left;
    }
    else if (HAL_GPIO_ReadPin(sensor_left_GPIO_Port, sensor_left_Pin) == GPIO_PIN_SET)
    {
    	//HAL_GPIO_WritePin(led_left_GPIO_Port, led_left_Pin, GPIO_PIN_SET);
        return right;
    }
    else if (HAL_GPIO_ReadPin(sensor_hard_left_GPIO_Port, sensor_hard_left_Pin) == GPIO_PIN_SET)
    {
    	//HAL_GPIO_WritePin(led_hard_left_GPIO_Port, led_hard_left_Pin, GPIO_PIN_SET);
        return hard_right;
    }
    else
    {
        /* straight: all LEDs stay off */
        return straight;
    }
}

static void apply_motor_pwm(MotorState_t state, const MotorPwm_t *pwm)
{
    switch (state)
    {
        case straight:
            __HAL_TIM_SET_COMPARE(&htim1, left_ch,  pwm->left_base);
            __HAL_TIM_SET_COMPARE(&htim1, right_ch, pwm->right_base);
            break;

        case left:
            __HAL_TIM_SET_COMPARE(&htim1, left_ch,  pwm->left_slow);
            __HAL_TIM_SET_COMPARE(&htim1, right_ch, pwm->right_fast);
            break;

        case right:
            __HAL_TIM_SET_COMPARE(&htim1, left_ch,  pwm->left_fast);
            __HAL_TIM_SET_COMPARE(&htim1, right_ch, pwm->right_slow);
            break;

        case hard_left:
            __HAL_TIM_SET_COMPARE(&htim1, left_ch,  pwm->left_extra_slow);
            __HAL_TIM_SET_COMPARE(&htim1, right_ch, pwm->right_boost);
            break;

        case hard_right:
            __HAL_TIM_SET_COMPARE(&htim1, left_ch,  pwm->left_boost);
            __HAL_TIM_SET_COMPARE(&htim1, right_ch, pwm->right_extra_slow);
            break;

        default:
            break;
    }
}

// 5 volt settings
//MotorPwm_t pwm = {
//	  .left_base = 			55,
//	  .right_base = 		42,
//	  .left_slow = 			45, // -10
//	  .right_slow = 		32, // -10
//	  .left_extra_slow = 	20, // -35
//	  .right_extra_slow = 	7, // -35
//	  .left_fast = 			65, // +10
//	  .right_fast = 		52, // +10
//	  .left_boost = 		90, // base +35
//	  .right_boost = 		77 // base + 35
//};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, left_ch);
  HAL_TIM_PWM_Start(&htim1, right_ch);

  MotorPwm_t pwm = {
	  .left_base = 			55,
	  .right_base = 		42,
	  .left_slow = 			45, // -10
	  .right_slow = 		32, // -10
	  .left_extra_slow = 	20, // -25
	  .right_extra_slow = 	7, // -25
	  .left_fast = 			65, // +10
	  .right_fast = 		52, // +10
	  .left_boost = 		90, // base +25
	  .right_boost = 		77 // base + 25
  };


  	MotorState_t state = straight;

    const uint32_t control_period_ms = 2U;    /* motor update rate   */
    const uint32_t blink_period_ms   = 100U;  /* PB4 blink period    */

    uint32_t last_control_ms = 0U;
    uint32_t last_blink_ms   = 0U;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      uint32_t now = HAL_GetTick();


      if ((now - last_control_ms) >= control_period_ms)
      {
          state = get_direction();
          apply_motor_pwm(state, &pwm);
          last_control_ms = now;
      }


      if ((now - last_blink_ms) >= blink_period_ms)
      {
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
          last_blink_ms = now;
      }
  }//while(1) end
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1599-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led_hard_left_Pin|led_left_Pin|led_right_Pin|led_hard_right_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(yellow_led_GPIO_Port, yellow_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : led_hard_left_Pin led_left_Pin led_right_Pin led_hard_right_Pin */
  GPIO_InitStruct.Pin = led_hard_left_Pin|led_left_Pin|led_right_Pin|led_hard_right_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : sensor_hard_right_Pin sensor_right_Pin sensor_left_Pin */
  GPIO_InitStruct.Pin = sensor_hard_right_Pin|sensor_right_Pin|sensor_left_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : sensor_hard_left_Pin */
  GPIO_InitStruct.Pin = sensor_hard_left_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(sensor_hard_left_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : yellow_led_Pin */
  GPIO_InitStruct.Pin = yellow_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(yellow_led_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
