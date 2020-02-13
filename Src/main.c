/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

#define __LED_ON          	HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET)
#define __LED_OFF           HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET)
#define __DRV_EN_ON       	HAL_GPIO_WritePin(GPIOA, DRV_EN_Pin, GPIO_PIN_RESET)
#define __DRV_EN_OFF        HAL_GPIO_WritePin(GPIOA, DRV_EN_Pin, GPIO_PIN_SET)
#define __DRV_STEP_ON		HAL_GPIO_WritePin(GPIOA, DRV_STEP_Pin, GPIO_PIN_SET)
#define __DRV_STEP_OFF      HAL_GPIO_WritePin(GPIOA, DRV_STEP_Pin, GPIO_PIN_RESET)
#define __DRV_DIR_UP     	HAL_GPIO_WritePin(GPIOA, DRV_DIR_Pin, GPIO_PIN_SET)
#define __DRV_DIR_DOWN      HAL_GPIO_WritePin(GPIOA, DRV_DIR_Pin, GPIO_PIN_RESET)

#define __MOTOR_START		HAL_TIM_PWM_Start_IT(&htim16, TIM_CHANNEL_1)
#define __MOTOR_STOP		HAL_TIM_PWM_Stop_IT(&htim16, TIM_CHANNEL_1)

/*
NEMA17 		1.8 deg / step
TMC2208 	16 microsteps / step
(360 / 1.8) * 16 = 3200 - number of pulses to make 1 revolution

GT2-20T
P = 40 mm
3200 / 40 = 80 - number of pulses to move carriage 1 mm
*/


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
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t txBufUART[10], rxBufUART[10];
uint8_t rxByteUART, rxDataLength;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float str2float(uint8_t *buf) {
	float result = 0;
	uint32_t digit = 1, divRatio = 1;
	for (uint8_t i=(rxDataLength-1); i>3; i--) {
		if ((buf[i] == 0x2E) && (divRatio == 1))
			divRatio = digit;
		else if ((buf[i] >= 0x30) && (buf[i] <= 0x39)) {
			result += (buf[i] - 0x30) * digit;
			digit *= 10;
		}
		else {
			HAL_UART_Transmit(&huart1, (uint8_t*)&"PARSE ERR\r\n", 11, 100);
			break;
		}
	}
	return (result / divRatio);
}

void M17_Handler(void) {														// Enable Steppers
	HAL_UART_Transmit(&huart1, (uint8_t*)&"OK M17\r\n", 8, 100);
}

void M18_Handler(void) {														// Disable Steppers
	HAL_UART_Transmit(&huart1, (uint8_t*)&"OK M18\r\n", 8, 100);
}

void G28_Handler(void) {														// Move to Origin
	HAL_UART_Transmit(&huart1, (uint8_t*)&"OK G28\r\n", 8, 100);
}

void G1_Handler(void) {															// Move
	float z = str2float(rxBufUART);
	HAL_UART_Transmit(&huart1, (uint8_t*)&"OK G1\r\n", 7, 100);
}

void UART_RxMessageHandler(void) {
	if (rxDataLength == 3) {
		if (!strcmp("M17", (char*)&rxBufUART))
			M17_Handler();
		else if (!strcmp("M18", (char*)&rxBufUART))
			M18_Handler();
		else if (!strcmp("G28", (char*)&rxBufUART))
			G28_Handler();
		else HAL_UART_Transmit(&huart1, (uint8_t*)&"MSG ERR\r\n", 9, 100);
	}
	else {
		if (!strncmp("G1 Z", (char*)&rxBufUART, 4))
			G1_Handler();
		else HAL_UART_Transmit(&huart1, (uint8_t*)&"MSG ERR\r\n", 9, 100);
	}

	for (uint8_t i=0; i<10; i++)
		rxBufUART[i] = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if ((rxByteUART != '\r') && (rxByteUART != '\n')) {
		if (rxDataLength < 9) {
			rxBufUART[rxDataLength] = rxByteUART;
			rxDataLength++;
		}
		else {
			HAL_UART_Transmit(&huart1, (uint8_t*)&"MSG ERR\r\n", 9, 100);
			rxDataLength = 0;
		}
		HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxByteUART, 1);
	}
	else if (rxByteUART == '\r') {
		rxDataLength *= 10;
		HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxByteUART, 1);
	}
	else if (rxByteUART == '\n') {
		if (rxDataLength >= 10) {
			rxDataLength /= 10;
			UART_RxMessageHandler();
		}
		else
			HAL_UART_Transmit(&huart1, (uint8_t*)&"MSG ERR\r\n", 9, 100);
		rxDataLength = 0;
		HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxByteUART, 1);
	}
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
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart1, (uint8_t*)&"HELLO BLYAD\r\n", 13, 100);

  rxDataLength = 0;
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxByteUART, 1);

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 47;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 500;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|DRV_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DRV_DIR_GPIO_Port, DRV_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ENDSTOP_IN_Pin */
  GPIO_InitStruct.Pin = ENDSTOP_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENDSTOP_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV_EN_Pin DRV_DIR_Pin */
  GPIO_InitStruct.Pin = DRV_EN_Pin|DRV_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
