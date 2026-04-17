/* USER CODE BEGIN Header */

/**



 ******************************************************************************



 * @file : main.c



 * @brief : Main program body



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

/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include <stdio.h>

#include <string.h>

#include <stdlib.h>

#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

#define ENCODER_PPR 11 // 모터 축 1회전당 펄스

#define GEAR_RATIO 30 // 감속비

#define QUADRATURE 4 // 4체배

#define COUNTS_PER_REV (ENCODER_PPR * GEAR_RATIO * QUADRATURE) // 1320

#define WHEEL_DIAMETER_MM 65.0f

#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3.14159f) // 204.2mm

#define SAMPLE_PERIOD_MS 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;

TIM_HandleTypeDef htim2;

TIM_HandleTypeDef htim3;

TIM_HandleTypeDef htim4;

TIM_HandleTypeDef htim5;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile uint8_t rx_data; // 1바이트 수신용

char rx_buf[20]; // 패킷 저장 버퍼

int rx_idx = 0; // 버퍼 인덱스

int error = 0;

int32_t target_pwm_value_L = 0;

int32_t target_pwm_value_R = 0;

// 빌드 에러 해결을 위한 선언

volatile int cmd_complete = 0; // 패킷 수신 완료 플래그

float target_rpm_L = 0.0; // 왼쪽 목표 속도 (에러 해결)

float target_rpm_R = 0.0; // 오른쪽 목표 속도 (에러 해결)

int16_t cnt_now_L = 0;

int16_t cnt_prev_L = 0;

int16_t cnt_now_R = 0;

int16_t cnt_prev_R = 0;

int16_t delta_cnt_L = 0;

int16_t delta_cnt_R = 0;

float RPM_L = 0.0;

float RPM_R = 0.0;

//제어기 게인

float Kp = 1.4f, ki = 1.42f, kd = 0.3f;

float error_L = 0, error_L_prev = 0, error_L_sum = 0;

float error_R = 0, error_R_prev = 0, error_R_sum = 0;

float pid_output_L = 0, pid_output_R = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void
SystemClock_Config(void);

static void
MX_GPIO_Init(void);

static void
MX_USART1_UART_Init(void);

static void
MX_TIM3_Init(void);

static void
MX_TIM1_Init(void);

static void
MX_TIM2_Init(void);

static void
MX_TIM4_Init(void);

static void
MX_USART2_UART_Init(void);

static void
MX_TIM5_Init(void);

static void
MX_TIM10_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)

{

	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);

	return len;

}

// 엔코더 카운터 읽기 (오버플로 처리 포함)

int32_t read_encoder(TIM_HandleTypeDef *htim)

{

	return (int16_t) __HAL_TIM_GET_COUNTER(htim);

}

/* USER CODE END 0 */

/**=



 * @brief The application entry point.



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

	MX_USART1_UART_Init();

	MX_TIM3_Init();

	MX_TIM1_Init();

	MX_TIM2_Init();

	MX_TIM4_Init();

	MX_USART2_UART_Init();

	MX_TIM5_Init();

	MX_TIM10_Init();

	/* USER CODE BEGIN 2 */

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Left

	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Right

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

// 카운터 초기화

	__HAL_TIM_SET_COUNTER(&htim3, 0);

	__HAL_TIM_SET_COUNTER(&htim4, 0);

	printf("\r\n===== Encoder Test Start =====\r\n");

	HAL_TIM_Base_Start_IT(&htim10);

	HAL_UART_Receive_IT(&huart1, &rx_data, 1); // UART1 수신 인터럽트 시작 (1바이트씩)

	printf("UART1 Interrupt Started!\r\n");

	/* USER CODE END 2 */

	/* Infinite loop */

	/* USER CODE BEGIN WHILE */

	while (1)

	{

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

// echo -e "M30.50,-25.30\n" > /dev/ttyS0
// cat /dev/ttyS0
//sprintf: 변수 -> 버퍼 (쓰기)
// sscanf: 버퍼 -> 변수 (읽기 + 쓰기)
		if (cmd_complete)

		{

//STM32가 파이로부터 받은 데이터 출력

			printf("Raw rx_buf: [%s]\r\n", rx_buf);

//제대로 데이터 받아왔는지 sscanf 반환값으로 체크

			int count = sscanf(rx_buf, "M%f,%f", &target_rpm_L, &target_rpm_R);

			if (count == 2)

			{

				printf(">>> SUCCESS! L:%.2f, R:%.2f\r\n", target_rpm_L,
						target_rpm_R);

			}

			else

			{

				printf(">>> [FAIL] Direct access failed! Count: %d\r\n", count);

			}

			cmd_complete = 0; //한바이트 데이터 출력 받았으니 다시 플레그변수0으로 초기화

		}

	}

	/* USER CODE END 3 */

}

/**



 * @brief System Clock Configuration



 * @retval None



 */

void SystemClock_Config(void)

{

	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };

	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage



	 */

	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters



	 * in the RCC_OscInitTypeDef structure.



	 */

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

	RCC_OscInitStruct.HSIState = RCC_HSI_ON;

	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

	RCC_OscInitStruct.PLL.PLLM = 16;

	RCC_OscInitStruct.PLL.PLLN = 336;

	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;

	RCC_OscInitStruct.PLL.PLLQ = 4;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)

	{

		Error_Handler();

	}

	/** Initializes the CPU, AHB and APB buses clocks



	 */

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK

	| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)

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

	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };

	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };

	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig =
	{ 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */

	htim1.Instance = TIM1;

	htim1.Init.Prescaler = 83;

	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;

	htim1.Init.Period = 999;

	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	htim1.Init.RepetitionCounter = 0;

	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

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

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)

	{

		Error_Handler();

	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)

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



 * @brief TIM2 Initialization Function



 * @param None



 * @retval None



 */

static void MX_TIM2_Init(void)

{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };

	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };

	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */

	htim2.Instance = TIM2;

	htim2.Init.Prescaler = 0;

	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;

	htim2.Init.Period = 4294967295;

	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)

	{

		Error_Handler();

	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)

	{

		Error_Handler();

	}

	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)

	{

		Error_Handler();

	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)

	{

		Error_Handler();

	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;

	sConfigOC.Pulse = 0;

	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;

	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)

	{

		Error_Handler();

	}

	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

	HAL_TIM_MspPostInit(&htim2);

}

/**



 * @brief TIM3 Initialization Function



 * @param None



 * @retval None



 */

static void MX_TIM3_Init(void)

{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig =
	{ 0 };

	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */

	htim3.Instance = TIM3;

	htim3.Init.Prescaler = 0;

	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;

	htim3.Init.Period = 65535;

	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;

	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;

	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;

	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;

	sConfig.IC1Filter = 0;

	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;

	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;

	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;

	sConfig.IC2Filter = 0;

	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)

	{

		Error_Handler();

	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)

	{

		Error_Handler();

	}

	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**



 * @brief TIM4 Initialization Function



 * @param None



 * @retval None



 */

static void MX_TIM4_Init(void)

{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig =
	{ 0 };

	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */

	htim4.Instance = TIM4;

	htim4.Init.Prescaler = 0;

	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;

	htim4.Init.Period = 65535;

	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;

	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;

	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;

	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;

	sConfig.IC1Filter = 0;

	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;

	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;

	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;

	sConfig.IC2Filter = 0;

	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)

	{

		Error_Handler();

	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)

	{

		Error_Handler();

	}

	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**



 * @brief TIM5 Initialization Function



 * @param None



 * @retval None



 */

static void MX_TIM5_Init(void)

{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };

	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */

	htim5.Instance = TIM5;

	htim5.Init.Prescaler = 0;

	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;

	htim5.Init.Period = 4294967295;

	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)

	{

		Error_Handler();

	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)

	{

		Error_Handler();

	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;

	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)

	{

		Error_Handler();

	}

	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**



 * @brief TIM10 Initialization Function



 * @param None



 * @retval None



 */

static void MX_TIM10_Init(void)

{

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */

	htim10.Instance = TIM10;

	htim10.Init.Prescaler = 8399;

	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;

	htim10.Init.Period = 499;

	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim10) != HAL_OK)

	{

		Error_Handler();

	}

	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */

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

	huart1.Init.BaudRate = 115200;

	huart1.Init.WordLength = UART_WORDLENGTH_8B;

	huart1.Init.StopBits = UART_STOPBITS_1;

	huart1.Init.Parity = UART_PARITY_NONE;

	huart1.Init.Mode = UART_MODE_TX_RX;

	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;

	huart1.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart1) != HAL_OK)

	{

		Error_Handler();

	}

	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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

	huart2.Init.BaudRate = 115200;

	huart2.Init.WordLength = UART_WORDLENGTH_8B;

	huart2.Init.StopBits = UART_STOPBITS_1;

	huart2.Init.Parity = UART_PARITY_NONE;

	huart2.Init.Mode = UART_MODE_TX_RX;

	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;

	huart2.Init.OverSampling = UART_OVERSAMPLING_16;

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

	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */

	__HAL_RCC_GPIOC_CLK_ENABLE();

	__HAL_RCC_GPIOH_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */

	HAL_GPIO_WritePin(GPIOB, MOTOR_L_DIR_Pin | MOTOR_R_DIR_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */

	GPIO_InitStruct.Pin = B1_Pin;

	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;

	GPIO_InitStruct.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */

	GPIO_InitStruct.Pin = LD2_Pin;

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

	GPIO_InitStruct.Pull = GPIO_NOPULL;

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MOTOR_L_DIR_Pin MOTOR_R_DIR_Pin */

	GPIO_InitStruct.Pin = MOTOR_L_DIR_Pin | MOTOR_R_DIR_Pin;

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

	GPIO_InitStruct.Pull = GPIO_NOPULL;

	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

{

//ENCL A/B:PA67

//ENCR A/B:PB67

	if (htim->Instance == TIM10) //TIM10이 불렀다면

	{

		cnt_now_L = read_encoder(&htim3);

		cnt_now_R = read_encoder(&htim4);

		delta_cnt_L = (int16_t) (cnt_now_L - cnt_prev_L);

		delta_cnt_R = (int16_t) (cnt_now_R - cnt_prev_R);

		RPM_L = ((float) delta_cnt_L / COUNTS_PER_REV)
				* (60000.0f / SAMPLE_PERIOD_MS);

		RPM_R = ((float) delta_cnt_R / COUNTS_PER_REV)
				* (60000.0f / SAMPLE_PERIOD_MS);

		/*------------엔코더 읽기&&피드백RPM 계산(UART)-------------------*/

		error_L = (target_rpm_L) - (RPM_L);

		error_R = (target_rpm_R) - (RPM_R);

// I항 누적

		error_L_sum += error_L;

		error_R_sum += error_R;

		if (error_L_sum > 2000)
			error_L_sum = 2000;

		if (error_L_sum < -2000)
			error_L_sum = -2000;

		if (error_R_sum > 2000)
			error_R_sum = 2000;

		if (error_R_sum < -2000)
			error_R_sum = -2000;

		pid_output_L = (Kp * error_L) + (ki * error_L_sum*0.05)
				+ (kd * (error_L - error_L_prev)/0.05);

		pid_output_R = (Kp * error_R) + (ki * error_R_sum*0.05)
				+ (kd * (error_R - error_R_prev)/0.05);

		float final_pwm_L = (((target_rpm_L) / 333.0f) * 999.0f)
				+ pid_output_L;

		float final_pwm_R = (((target_rpm_R) / 333.0f) * 999.0f)
				+ pid_output_R;

		if (final_pwm_L > 999)
			final_pwm_L = 999;

		if (final_pwm_L < 0)
			final_pwm_L = 0;

		if (final_pwm_R > 999)
			final_pwm_R = 999;

		if (final_pwm_R < 0)
			final_pwm_R = 0;

		target_pwm_value_L = (uint32_t) final_pwm_L;

		target_pwm_value_R = (uint32_t) final_pwm_R;

		HAL_GPIO_WritePin(GPIOB, MOTOR_L_DIR_Pin,
				(target_rpm_L >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

//모터드라이버의 h-브릿지: 3.3V->정방향(SET) 0V->역방향(RESET)

		HAL_GPIO_WritePin(GPIOB, MOTOR_R_DIR_Pin,
				(target_rpm_R >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

//rpm -> pwm 변환: 최대 rpm 333, 최대 pwm값 999 => 비율을 구한다음에 999에 곱해서

//흐르는 전류값(모터속도 제어)

		if (target_pwm_value_L > 999)
			target_pwm_value_L = 999;

		if (target_pwm_value_R > 999)
			target_pwm_value_R = 999;

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, target_pwm_value_L);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, target_pwm_value_R);

//방향 핀에 값 인가

		char tx_buf[32];

		int len = sprintf(tx_buf, "F%.2f,%.2f\n", RPM_L, RPM_R);

		HAL_UART_Transmit(&huart1, (uint8_t*) tx_buf, len, 10);

		error_L_prev = error_L;

		error_R_prev = error_R;

		cnt_prev_L = cnt_now_L;

		cnt_prev_R = cnt_now_R;

	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{

	if (huart->Instance == USART1)

	{

		if (rx_data == '\n' || rx_data == '\r')

		{

			if (rx_idx > 0)
			{

				rx_buf[rx_idx] = '\0';

				cmd_complete = 1;

				rx_idx = 0;

			}

		}

		else
		{

			if (rx_idx < 19)

			{

				rx_buf[rx_idx++] = (char) rx_data;

			}

		}

		HAL_UART_Receive_IT(huart, &rx_data, 1);

	}

}

/* USER CODE END 4 */

/**



 * @brief This function is executed in case of error occurrence.



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



* @brief Reports the name of the source file and the source line number



* where the assert_param error has occurred.



* @param file: pointer to the source file name



* @param line: assert_param error line source number



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
