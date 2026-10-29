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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "motor_control.h"
#include "robot_control.h"

#include <stdlib.h>
#include <string.h>
#include <errno.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMMAND_TIMEOUT_MS 500U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Motor motor1 = {
	.config = {
		.encoder_timer = &htim2,
		.pwm_timer = &htim1,
		.pwm_channel = TIM_CHANNEL_1,

		.dir_port = M1_DIR_GPIO_Port,
		.dir_pin = M1_DIR_Pin,

		.encoder_bits = 32U,
		.encoder_sign = 1,

		.counts_per_turn = 2465.2f,
		.wheel_circumference_mm = 345.6f
	}
};

Motor motor2 = {
	.config = {
		.encoder_timer = &htim3,
		.pwm_timer = &htim1,
		.pwm_channel = TIM_CHANNEL_2,

		.dir_port = M2_DIR_GPIO_Port,
		.dir_pin = M2_DIR_Pin,

		.encoder_bits = 16U,
		.encoder_sign = 1,

		.counts_per_turn = 2475.6f,
		.wheel_circumference_mm = 345.6f
	}
};

Robot robot = {
	.left_motor = &motor1,
	.right_motor = &motor2,
	.wheel_track_mm = WHEEL_TRACK_MM
};
float linear_speed_mm_s = 0.0f;
float angular_speed_rad_s = 0.0f;

static char uart_message[96];
static volatile uint8_t uart_tx_busy = 0U;

HAL_StatusTypeDef uart_tx_status = HAL_OK;

#define UART_RX_BUFFER_SIZE 64U

static uint8_t uart_rx_byte;
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

static volatile uint16_t uart_rx_head = 0U;
static volatile uint16_t uart_rx_tail = 0U;

static volatile uint8_t uart_rx_overflow = 0U;

HAL_StatusTypeDef uart_rx_status = HAL_OK;

static char uart_line[64];
static uint16_t uart_line_length = 0U;
static uint8_t uart_line_ready = 0U;
static uint8_t uart_discard_line = 0U;

static uint32_t last_command_tick = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2) {
		uart_tx_busy = 0U;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART2) {
		const uint16_t next_head =
			(uart_rx_head + 1U) % UART_RX_BUFFER_SIZE;

		if (next_head != uart_rx_tail) {
			/* 문자를 저장한 뒤 저장 위치를 공개합니다. */
			uart_rx_buffer[uart_rx_head] = uart_rx_byte;
			uart_rx_head = next_head;
		}
		else {
			/* 해석이나 모터 제어는 하지 않고 상태만 표시합니다. */
			uart_rx_overflow = 1U;
		}

		/* 다음 문자도 받도록 다시 요청합니다. */
		uart_rx_status = HAL_UART_Receive_IT(
			&huart2,
			&uart_rx_byte,
			1U
		);
	}
}

static void UART_ReceiveTask(void)
{
    /* 완성된 명령은 메인이 해석할 때까지 유지합니다. */
    if (uart_line_ready != 0U) {
        return;
    }

    if (uart_rx_overflow != 0U) {
        /* 넘친 버퍼를 비우는 짧은 구간만 인터럽트를 막습니다. */
        const uint32_t saved_primask = __get_PRIMASK();
        __disable_irq();

        uart_rx_tail = uart_rx_head;
        uart_rx_overflow = 0U;

        __set_PRIMASK(saved_primask);

        uart_line_length = 0U;
        uart_discard_line = 1U;
    }

    /* 한 번의 호출에서 처리량을 제한해 모터 제어로 돌아갑니다. */
    for (uint16_t i = 0U; i < UART_RX_BUFFER_SIZE; i++) {
        if (uart_rx_tail == uart_rx_head) {
            break;
        }

        const uint8_t received = uart_rx_buffer[uart_rx_tail];

        uart_rx_tail =
            (uart_rx_tail + 1U) % UART_RX_BUFFER_SIZE;

        if (received == '\n') {
            if (uart_discard_line == 0U &&
                uart_line_length > 0U) {

                uart_line[uart_line_length] = '\0';
                uart_line_length = 0U;
                uart_line_ready = 1U;
                return;
            }

            uart_line_length = 0U;
            uart_discard_line = 0U;
        }
        else if (received == '\r') {
            /* CRLF 줄바꿈의 CR은 제외합니다. */
        }
        else if (uart_discard_line == 0U) {
            if (received >= 32U && received <= 126U &&
                uart_line_length < sizeof(uart_line) - 1U) {

                uart_line[uart_line_length++] = (char)received;
            }
            else {
                /* 잘린 명령을 적용하지 않도록 줄 전체를 버립니다. */
                uart_line_length = 0U;
                uart_discard_line = 1U;
            }
        }
    }
}

static void UART_ApplyCommand(void)
{
    if (strncmp(uart_line, "V=", 2U) != 0) {
        return;
    }

    char *end;
    const char *start = uart_line + 2;

    errno = 0;
    const long linear = strtol(start, &end, 10);

    if (end == start || errno == ERANGE ||
        strncmp(end, " W=", 3U) != 0) {
        return;
    }

    start = end + 3;

    errno = 0;
    const long angular = strtol(start, &end, 10);

    if (end == start || errno == ERANGE || *end != '\0') {
        return;
    }

    /* 이번 명령 인터페이스의 허용 범위입니다. */
    if (linear < -200L || linear > 200L ||
        angular < -2000L || angular > 2000L) {
        return;
    }

    /* 두 값이 모두 정상일 때만 함께 적용합니다. */
    linear_speed_mm_s = (float)linear;
    angular_speed_rad_s = (float)angular / 1000.0f;

    last_command_tick = HAL_GetTick();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t previous_tick = 0;

	uint32_t previous_uart_tick = 0;
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  if (Motor_Init(&motor1) != HAL_OK){
	  Error_Handler();
  }
  if (Motor_Init(&motor2) != HAL_OK){
  	  Error_Handler();
  }

  previous_tick = HAL_GetTick();

  uart_rx_status = HAL_UART_Receive_IT(
	  &huart2, &uart_rx_byte, 1U
  );

  if (uart_rx_status != HAL_OK){
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  UART_ReceiveTask();

	  if (uart_line_ready != 0U) {
	      UART_ApplyCommand();

	      /* 정상·잘못된 명령 모두 처리가 끝나면 다음 줄을 받습니다. */
	      uart_line_ready = 0U;
	  }

	  const uint32_t now = HAL_GetTick();

	  if ((uint32_t)(now - last_command_tick) >= COMMAND_TIMEOUT_MS) {
	      linear_speed_mm_s = 0.0f;
	      angular_speed_rad_s = 0.0f;
	  }


	  if ((uint32_t)(now - previous_tick) >= 100U){
		  const uint32_t sample_dt_ms = now - previous_tick;

		  Robot_SetVelocity(
			  &robot,
			  linear_speed_mm_s,
			  angular_speed_rad_s
		  );
		  // 모터 측정과 제어 수행
		  Robot_Update(&robot, sample_dt_ms);

		  previous_tick = now;
	  }

	  if ((uint32_t)(now - previous_uart_tick) >= 1000U) {
		  previous_uart_tick = now;


		  if (uart_tx_busy == 0U){
			  const int length = snprintf(
				  uart_message,
				  sizeof(uart_message),
				  "L=%ld R=%ld  X=%ld Y=%ld A=%ld\r\n",
				  (long)motor1.state.speed_mm_s,
				  (long)motor2.state.speed_mm_s,
				  (long)robot.state.x_mm,
				  (long)robot.state.y_mm,
				  (long)(robot.state.yaw_rad * 1000.0f)
			  );

			  if(length > 0 && (size_t)length < sizeof(uart_message)){

				  uart_tx_busy = 1U;

				  uart_tx_status = HAL_UART_Transmit_IT(
					  &huart2,
					  (uint8_t *)uart_message,
					  (uint16_t)length
				  );

				  if (uart_tx_status != HAL_OK){
					  uart_tx_busy = 0U;
				  }
			  }
		  }
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M1_DIR_Pin|M2_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M1_DIR_Pin M2_DIR_Pin */
  GPIO_InitStruct.Pin = M1_DIR_Pin|M2_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
	if (htim1.Instance == TIM1)
	{
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	}
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
