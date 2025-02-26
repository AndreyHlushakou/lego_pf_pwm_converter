/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdlib.h" // abs()
#include "stdio.h"  // snprintf()
#include "string.h" // strlen()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*transition_callback)(void);

typedef enum {
    HIGH,
	PWM,
	LOW
} STATE_PWM_PIN;

typedef struct {
    transition_callback worker;
} TRANSITION;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PERIOD_MAX (htim4.Init.Period)
#define size_arr (100)                  //pwm   in arr
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/*
volatile uint8_t flag_irq = 0; //button exti
volatile uint32_t time_irq = 0;//button exti
volatile uint8_t flag_read = 0;//button exti
void button_hnd(void);         //button
*/

uint8_t dataReceived=0;    //uart признак данное получено
uint8_t dataTransmitted=1; //uart признак данное передано
uint8_t str_uart_buffer[3];//uart buffer


volatile uint32_t falling1_R = 0; //pwm R in
volatile uint32_t falling1_L = 0; //pwm L in
uint32_t falling_buffer = 0;

uint8_t counter1_R = 0;
uint8_t counter1_L = 0;
uint32_t arr_falling1_R = 0;
uint32_t arr_falling1_L = 0;

volatile uint8_t flag_pwm1_R = 0;
volatile uint8_t flag_pwm1_L = 0;
volatile uint32_t last_capture_time1_R = 0;
volatile uint32_t last_capture_time1_L = 0;

STATE_PWM_PIN statepwm1_R = LOW;
STATE_PWM_PIN statepwm1_L = LOW;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

//inits begin
void led_init(void);
void all_led_on(void);
void all_led_off(void);
void driver_motor1_ports_init();
void tim1_for_motor1_ports_init();
//inits end

//main handles begin
void hnd_state_for_motor1(void);
//main handlers end

//hnd_state_for_motor begin
STATE_PWM_PIN hnd_state_in_pwm_R(void);
STATE_PWM_PIN hnd_state_in_pwm_L(void);
STATE_PWM_PIN hnd_state_in_pwm_i(
		GPIO_PinState state, STATE_PWM_PIN *statepwm,
		volatile uint8_t *flag_pwm, volatile uint32_t last_capture_time,
		char type);
void led_state_pwm(STATE_PWM_PIN *statepwm, char type);

void hal_tim_set_compare(uint16_t pwm);
void enable_R(void);
void enable_L(void);
uint16_t falling_to_pwm(volatile uint32_t falling, char *type);

void state_zero(void);
void state_PWM_L(void);
void state_MAX_L(void);
void state_PWM_R(void);
void state_MAX_R(void);

TRANSITION FSM_table[3][3] = {
   //LEFT//RIGHT//
    [HIGH][HIGH] = {state_zero},
    [HIGH][PWM]  = {state_PWM_R},
    [HIGH][LOW]  = {state_MAX_R},
    [PWM][HIGH]  = {state_PWM_L},
    [LOW][HIGH]  = {state_MAX_L},
	//
    [PWM][PWM] = {NULL},
    [PWM][LOW] = {NULL},
    [LOW][PWM] = {NULL},
    [LOW][LOW] = {NULL}
    //
};
//hnd_state_for_motor end

void fill_arr(uint8_t *counter, uint32_t *arr_falling, volatile uint32_t *falling, uint32_t falling0, char *type);

void print_uniq_uart_data(volatile uint32_t falling, char *str);
void print_uart_data(volatile uint32_t falling, char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void led_init(void) {
	char str1[7] = "\nINIT;\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)str1, strlen(str1), 1000);

    all_led_on();
    HAL_Delay(1000);
    all_led_off();
}

void all_led_on(void) {
	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET); //revers t.k pc13
	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
}

void all_led_off(void) {
	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET); //revers t.k pc13
	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
}

void driver_motor1_ports_init() {
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	  HAL_GPIO_WritePin(an1_GPIO_Port, an1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(an2_GPIO_Port, an2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(stby_GPIO_Port, stby_Pin, GPIO_PIN_RESET);
}

void tim1_for_motor1_ports_init() {
	  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
	  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  led_init();
  driver_motor1_ports_init();
  tim1_for_motor1_ports_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  hnd_state_for_motor1();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(2);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65000-1;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 62600;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led1_Pin|led2_Pin|led3_Pin|led4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, stby_Pin|an2_Pin|an1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led1_Pin led2_Pin led3_Pin led4_Pin */
  GPIO_InitStruct.Pin = led1_Pin|led2_Pin|led3_Pin|led4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : stby_Pin an2_Pin an1_Pin */
  GPIO_InitStruct.Pin = stby_Pin|an2_Pin|an1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : in1_R_Pin in1_L_Pin */
  GPIO_InitStruct.Pin = in1_R_Pin|in1_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  if (huart == &huart3)
  {
    dataReceived=1;

    if( dataTransmitted != 0 )
    {
      HAL_UART_Transmit_IT(&huart3, str_uart_buffer, 1);
      dataReceived=0;
      dataTransmitted=0;
    }

    HAL_UART_Receive_IT (&huart3, str_uart_buffer, 1);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

  if(huart == &huart3)
  {
    dataTransmitted=1;

    if( dataReceived != 0 )
    {
      HAL_UART_Transmit_IT(&huart3, str_uart_buffer, 1);
      dataReceived=0;
      dataTransmitted=0;
    }
  }
}

/*
void button_hnd(void)
{
	if(flag_irq && ((HAL_GetTick() - time_irq) > 200))
	{
		__HAL_GPIO_EXTI_CLEAR_IT(button_Pin);  // очищаем бит EXTI_PR
		NVIC_ClearPendingIRQ(EXTI15_10_IRQn); // очищаем бит NVIC_ICPRx
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);   // включаем внешнее прерывание
		flag_irq = 0;
		flag_read = !flag_read;

		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
	}
	if (flag_read) {
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == button_Pin && !flag_irq)
	{
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); // сразу же отключаем прерывания на этом пине
		// либо выполняем какое-то действие прямо тут, либо поднимаем флажок
		flag_irq = 1;
		time_irq = HAL_GetTick();
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
		if(!flag_read) HAL_UART_Transmit(&huart3, (uint8_t*) "read mode ON\n", strlen( "read mode ON\n" ), 1000);
		else HAL_UART_Transmit(&huart3, (uint8_t*) "read mode OFF\n", strlen( "read mode OFF\n" ), 1000);
	}

}
*/

void hnd_state_for_motor1(void) {
	STATE_PWM_PIN current_state_R = hnd_state_in_pwm_R();
	STATE_PWM_PIN current_state_L = hnd_state_in_pwm_L();
	transition_callback worker = FSM_table[current_state_L][current_state_R].worker;
	if (worker != NULL) {
		worker();
	}
}

STATE_PWM_PIN hnd_state_in_pwm_R(void) {
	GPIO_PinState stateR = HAL_GPIO_ReadPin(in1_R_GPIO_Port, in1_R_Pin);
	return hnd_state_in_pwm_i(
			stateR, &statepwm1_R,
			&flag_pwm1_R, last_capture_time1_R,
			'R');
}

STATE_PWM_PIN hnd_state_in_pwm_L(void) {
	GPIO_PinState stateL = HAL_GPIO_ReadPin(in1_L_GPIO_Port, in1_L_Pin);
	return hnd_state_in_pwm_i(
			stateL, &statepwm1_L,
			&flag_pwm1_L, last_capture_time1_L,
			'L');
}


STATE_PWM_PIN hnd_state_in_pwm_i(
		GPIO_PinState state, STATE_PWM_PIN *statepwm,
		volatile uint8_t *flag_pwm, volatile uint32_t last_capture_time,
		char type) {

    uint32_t current_time = HAL_GetTick();
    // Если прошло больше 5 мс с последнего захвата фронта - считаем, что ШИМ отсутствует
    if (*flag_pwm == 1 && (current_time - last_capture_time > 5)) {
    	*flag_pwm = 0; // ШИМ отсутствует
    }

	if (*flag_pwm) *statepwm = PWM;
	else *statepwm = (state == GPIO_PIN_SET) ? HIGH : LOW;

	led_state_pwm(statepwm, type);

	return *statepwm;
}

void led_state_pwm(STATE_PWM_PIN *statepwm, char type) {
	switch(*statepwm) {
	    case HIGH: {
	    	switch (type) {
				case 'L': {
			    	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
			    	HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
				}
				break;
				case 'R': {
			    	HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
		            HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
				}
				break;
			}

	    }
	    break;
	    case PWM: {
	    	switch (type) {
				case 'L': {
			    	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
			    	HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
				}
				break;
				case 'R': {
			    	HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_RESET);
		            HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
				}
				break;
			}

	    }
	    break;
	    case LOW: {
	    	switch (type) {
				case 'L': {
			    	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
			    	HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
				}
				break;
				case 'R': {
			    	HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
		            HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
				}
				break;
			}

	    }
	    break;
	}
}

void hal_tim_set_compare(uint16_t pwm) {
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);
}

void state_zero(void) {
	HAL_GPIO_WritePin(stby_GPIO_Port, stby_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(an1_GPIO_Port, an1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(an2_GPIO_Port, an2_Pin, GPIO_PIN_RESET);
	hal_tim_set_compare(0);
	//print_uart_data(0, "STATE ZERO");
}

void enable_L(void) {
	HAL_GPIO_WritePin(an1_GPIO_Port, an1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(an2_GPIO_Port, an2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(stby_GPIO_Port, stby_Pin, GPIO_PIN_SET);
}

void enable_R(void) {
	HAL_GPIO_WritePin(an1_GPIO_Port, an1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(an2_GPIO_Port, an2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(stby_GPIO_Port, stby_Pin, GPIO_PIN_SET);
}

void state_MAX_L(void) {
	enable_L();
	hal_tim_set_compare(PERIOD_MAX);
	//print_uart_data(0, "STATE L MAX");
}

void state_MAX_R(void) {
	enable_R();
	hal_tim_set_compare(PERIOD_MAX);
	//print_uart_data(0, "STATE R MAX");
}

void state_PWM_L(void) {
	enable_L();
	uint16_t pwm1 = falling_to_pwm(falling1_L, "STATE L PWM");
	hal_tim_set_compare(pwm1);
}

void state_PWM_R(void) {
	enable_R();
	uint16_t pwm1 = falling_to_pwm(falling1_R, "STATE R PWM");
	hal_tim_set_compare(pwm1);
}


uint16_t falling_to_pwm(volatile uint32_t falling, char *type) {
	uint16_t pwm1 = PERIOD_MAX;

	//print_uart_data(falling, type);
    print_uniq_uart_data(falling, type);

	if(1600 >= falling && falling >= 1400) {
		pwm1 = PERIOD_MAX * 266 / 1000;
	} else if(1350 >= falling && falling >= 1150) {
		pwm1 = PERIOD_MAX * 389 / 1000;
	} else if(1100 >= falling && falling >= 900) {
		pwm1 = PERIOD_MAX * 512 / 1000;
	} else if(850 >= falling && falling >= 650) {
		pwm1 = PERIOD_MAX * 630 / 1000;
	} else if(600 >= falling && falling >= 400) {
		pwm1 = PERIOD_MAX * 750 / 1000;
	} else if(350 >= falling && falling >= 150) {
		pwm1 = PERIOD_MAX * 872 / 1000;
	}
	return pwm1;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1 ) {
		uint32_t current_time = HAL_GetTick(); // Получаем текущее время в мс

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			__HAL_TIM_SET_COUNTER(&htim1, 0x0000);
		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			uint32_t falling0 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
			fill_arr(&counter1_R, &arr_falling1_R, &falling1_R, falling0, "TIM1  R");

            flag_pwm1_R = 1; // ШИМ обнаружен
            last_capture_time1_R = current_time;
		 }

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			__HAL_TIM_SET_COUNTER(&htim1, 0x0000);
		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			uint32_t falling0 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);
			fill_arr(&counter1_L, &arr_falling1_L, &falling1_L, falling0, "TIM1  L");

            flag_pwm1_L = 1; // ШИМ обнаружен
            last_capture_time1_L = current_time;
		}

	 }

}

void fill_arr(uint8_t *counter, uint32_t *arr_falling, volatile uint32_t *falling, uint32_t falling0, char *type) {

	*arr_falling += falling0;
	(*counter)++;
    if (*counter == size_arr) {
        *falling = *arr_falling / size_arr;

        *counter = 0;
        *arr_falling = 0;

        //print_uart_data(*falling, type);
        print_uniq_uart_data(*falling, type);
    }
}

void print_uniq_uart_data(volatile uint32_t falling, char *str) {
	if (falling != falling_buffer) {
		print_uart_data(falling, str);
		falling_buffer = falling;
	}
}

void print_uart_data(volatile uint32_t falling, char *str) {
	char str1[63] = {0,};
    snprintf(str1, 63, "\nResult %s= %lu\n", str, falling);
    HAL_UART_Transmit(&huart3, (uint8_t*)str1, strlen(str1), 1000);
}

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
