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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	float target_speed_mm_s;
	float control_target_speed_mm_s;
	float speed_mm_s;
	float delta_distance_mm;
	float rpm;

	float duty_percent;
	float speed_error;
	float i_output;

	float kp;
	float ki;

	uint32_t previous_count;
	int32_t delta_count;

	int8_t direction;
	uint8_t stop_samples;
} MotorState;

typedef struct{
	TIM_HandleTypeDef *encoder_timer;
	TIM_HandleTypeDef *pwm_timer;
	uint32_t pwm_channel;

	GPIO_TypeDef *dir_port;
	uint16_t dir_pin;

	uint32_t encoder_bits;
	int8_t encoder_sign;

	float counts_per_turn;
	float wheel_circumference_mm;
} MotorConfig;

typedef struct{
	MotorConfig config;
	MotorState state;
} Motor;


typedef struct {
	float delta_s_mm;
	float delta_yaw_rad;

	float x_mm;
	float y_mm;
	float yaw_rad;
} RobotState;

typedef struct {
	Motor *left_motor;
	Motor *right_motor;

	float wheel_track_mm;

	RobotState state;
} Robot;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_DUTY_LIMIT_PERCENT 50.0f
#define MOTOR_SPEED_LIMIT_MM_S 200.0f
#define MOTOR_ACCELERATION_LIMIT_MM_S2 100.0f
#define WHEEL_TRACK_MM 234.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;


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

		.counts_per_turn = 2475.6f,
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

		.counts_per_turn = 2465.2f,
		.wheel_circumference_mm = 345.6f
	}
};

Robot robot = {
	.left_motor = &motor1,
	.right_motor = &motor2,
	.wheel_track_mm = WHEEL_TRACK_MM
};
/* 디버거가 외부에서 변경하는 명령값이므로 volatile을 사용합니다. */
volatile float linear_speed_mm_s = 0.0f;
volatile float angular_speed_rad_s = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef Motor_Init(Motor *motor);

void Motor_SetTargetSpeed(Motor *motor, float target_speed_mm_s);

void Motor_Update(Motor *motor, uint32_t sample_dt_ms);

void Motor_Stop(Motor *motor);


void Robot_SetVelocity(
	Robot *robot,
	float linear_speed_mm_s,
	float angular_speed_rad_s
);

void Robot_Update(
	Robot *robot,
	uint32_t sample_dt_ms
);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Motor_ApplyDuty(Motor *motor, float duty_percent)
{
    if (duty_percent < 0.0f){
        duty_percent = 0.0f;
    }
    else if (duty_percent > MOTOR_DUTY_LIMIT_PERCENT){
        duty_percent = MOTOR_DUTY_LIMIT_PERCENT;
    }

    motor->state.duty_percent = duty_percent;

    const uint32_t period_counts =
    		__HAL_TIM_GET_AUTORELOAD(motor->config.pwm_timer) + 1U;
    const uint32_t compare =
    		(uint32_t)((float)period_counts * duty_percent / 100.0f);
    __HAL_TIM_SET_COMPARE(
    		motor->config.pwm_timer,
			motor->config.pwm_channel,
			compare
	);
}

void Motor_Stop(Motor *motor){
	motor->state.target_speed_mm_s = 0.0f;
	motor->state.control_target_speed_mm_s = 0.0f;
	motor->state.speed_error = -motor->state.speed_mm_s;
	motor->state.i_output = 0.0f;
	motor->state.stop_samples = 0;

	Motor_ApplyDuty(motor, 0.0f);
}

HAL_StatusTypeDef Motor_Init(Motor *motor){
	motor->state = (MotorState){
		/* 반복한 바닥 시험에서 목표 부근의 속도 변동이 작았던 값입니다.
		 * Kp가 크면 속도 변화에 따른 듀티 변화도 커집니다. */
		.kp = 0.1f,
		.ki = 0.1f,
		.direction = 1
	};

	Motor_ApplyDuty(motor, 0.0f);

	HAL_GPIO_WritePin(
			motor->config.dir_port,
			motor->config.dir_pin,
			GPIO_PIN_SET
		);

	__HAL_TIM_SET_COUNTER(motor->config.encoder_timer, 0);

	if (HAL_TIM_Encoder_Start(
			motor->config.encoder_timer,
			TIM_CHANNEL_ALL) != HAL_OK)
	{
		return HAL_ERROR;
	}

	motor->state.previous_count =
			__HAL_TIM_GET_COUNTER(motor->config.encoder_timer);

	if (HAL_TIM_PWM_Start(
			motor->config.pwm_timer,
			motor->config.pwm_channel) != HAL_OK)
	{
		Motor_Stop(motor);
		return HAL_ERROR;
	}

	return HAL_OK;
}

void Motor_Update(Motor *motor, uint32_t sample_dt_ms){
	if (sample_dt_ms == 0U){
		return;
	}
	const uint32_t current_count =
			__HAL_TIM_GET_COUNTER(motor->config.encoder_timer);

	int32_t delta_count;

	if (motor->config.encoder_bits == 16U){
		delta_count =
				(int32_t)current_count - (int32_t)motor->state.previous_count;

		if (delta_count > 32767){
			delta_count -= 65536;
		}
		else if (delta_count < -32768){
			delta_count += 65536;
		}
	}
	else {
		delta_count =
				(int32_t)(current_count - motor->state.previous_count);
	}
	motor->state.previous_count = current_count;

	delta_count *= motor->config.encoder_sign;
	motor->state.delta_count = delta_count;

	const float turns =
			(float)delta_count / motor->config.counts_per_turn;

	motor->state.delta_distance_mm =
			turns * motor->config.wheel_circumference_mm;

	motor->state.speed_mm_s =
			motor->state.delta_distance_mm
			* 1000.0f / (float)sample_dt_ms;

	motor->state.rpm =
			turns * 60000.0f / (float)sample_dt_ms;

	const int8_t requested_direction =
			(motor->state.target_speed_mm_s > 0.0f) ? 1 :
			(motor->state.target_speed_mm_s < 0.0f) ? -1 : 0;

	if (requested_direction == 0){
		Motor_Stop(motor);
		return;
	}
	if (requested_direction != motor->state.direction){
		Motor_ApplyDuty(motor, 0.0f);
		motor->state.control_target_speed_mm_s = 0.0f;
		motor->state.i_output = 0.0f;

		if (delta_count == 0){
			motor->state.stop_samples++;
		}
		else{
			motor->state.stop_samples = 0;
		}
		if (motor->state.stop_samples >= 3U){
			HAL_GPIO_WritePin(
				motor->config.dir_port,
				motor->config.dir_pin,
				requested_direction > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET
			);

			motor->state.direction = requested_direction;
			motor->state.stop_samples = 0;
		}

		return;
	}

	motor->state.stop_samples = 0;

	const float dt_s = (float)sample_dt_ms / 1000.0f;

	const float max_change = MOTOR_ACCELERATION_LIMIT_MM_S2 * dt_s;

	float target_change =
			motor->state.target_speed_mm_s - motor->state.control_target_speed_mm_s;

	if (target_change > max_change){
		target_change = max_change;
	}

	else if (target_change < -max_change){
		target_change = -max_change;
	}

	motor->state.control_target_speed_mm_s += target_change;

	motor->state.speed_error =
			motor->state.control_target_speed_mm_s - motor->state.speed_mm_s;

	const float control_error =
			(float)motor->state.direction * motor->state.speed_error;

	const float p_output =
		motor->state.kp * control_error;

	const float i_candidate =
		motor->state.i_output
		+ motor->state.ki * control_error * dt_s;

	const float output_candidate = p_output + i_candidate;

	/* 출력 제한을 더 벗어나게 하는 적분은 저장하지 않습니다. */
	if (!((output_candidate > MOTOR_DUTY_LIMIT_PERCENT &&
		   control_error > 0.0f) ||
		  (output_candidate < 0.0f &&
		   control_error < 0.0f)))
	{
		motor->state.i_output = i_candidate;
	}

	/* 공통 출력 함수에서 듀티 제한과 실제 CCR 갱신을 수행합니다. */
	Motor_ApplyDuty(
		motor,
		p_output + motor->state.i_output
	);
}

void Motor_SetTargetSpeed(Motor *motor, float target_speed_mm_s){
	if (!isfinite(target_speed_mm_s)){ // isfinite()는 NaN·무한대 같은 잘못된 계산값을 걸러냄
		Motor_Stop(motor);
		return;
	}

	if (target_speed_mm_s > MOTOR_SPEED_LIMIT_MM_S){
		target_speed_mm_s = MOTOR_SPEED_LIMIT_MM_S;
	}
	else if (target_speed_mm_s < -MOTOR_SPEED_LIMIT_MM_S){
		target_speed_mm_s = -MOTOR_SPEED_LIMIT_MM_S;
	}

	if (target_speed_mm_s == 0){
		Motor_Stop(motor);
		return;
	}
	motor->state.target_speed_mm_s = target_speed_mm_s;
}

void Robot_SetVelocity(
	Robot *robot,
	float linear_speed_mm_s,
	float angular_speed_rad_s)
{
	const float turn_speed =
			angular_speed_rad_s * robot->wheel_track_mm * 0.5f;

	float left_target = linear_speed_mm_s - turn_speed;
	float right_target = linear_speed_mm_s + turn_speed;

	if (!isfinite(left_target) || !isfinite(right_target)) {
		Motor_Stop(robot->left_motor);
		Motor_Stop(robot->right_motor);
		return;
	}

	const float max_target =
		fmaxf(fabsf(left_target), fabsf(right_target));

	if (max_target > MOTOR_SPEED_LIMIT_MM_S) {
		const float scale =
			MOTOR_SPEED_LIMIT_MM_S / max_target;

		left_target *= scale;
		right_target *= scale;
	}

    Motor_SetTargetSpeed(robot->left_motor, left_target);
    Motor_SetTargetSpeed(robot->right_motor, right_target);
}

void Robot_Update(Robot *robot, uint32_t sample_dt_ms){
	if (sample_dt_ms == 0U){
		return;
	}

	Motor_Update(robot->left_motor, sample_dt_ms);
	Motor_Update(robot->right_motor, sample_dt_ms);

	const float left_mm =
		robot->left_motor->state.delta_distance_mm;

	const float right_mm =
		robot->right_motor->state.delta_distance_mm;

	robot->state.delta_s_mm =
		(left_mm + right_mm) * 0.5f;

	robot->state.delta_yaw_rad =
		(right_mm - left_mm) / robot->wheel_track_mm;

	const float mid_yaw_rad =
		robot->state.yaw_rad
		+ robot->state.delta_yaw_rad * 0.5f;

    robot->state.x_mm +=
        robot->state.delta_s_mm * cosf(mid_yaw_rad);

    robot->state.y_mm +=
        robot->state.delta_s_mm * sinf(mid_yaw_rad);

    robot->state.yaw_rad += robot->state.delta_yaw_rad;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    uint32_t previous_tick = 0U;
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
  /* USER CODE BEGIN 2 */
  if (Motor_Init(&motor1) != HAL_OK){
	  Error_Handler();
  }
  if (Motor_Init(&motor2) != HAL_OK){
  	  Error_Handler();
  }

  previous_tick = HAL_GetTick();
/* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    const uint32_t now = HAL_GetTick();

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
