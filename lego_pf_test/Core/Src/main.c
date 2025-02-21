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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint8_t flag_irq = 0; //button exti
volatile uint32_t time_irq = 0;//button exti
volatile uint8_t flag_read = 0;//button exti


uint8_t dataReceived=0;    //uart признак данное получено
uint8_t dataTransmitted=1; //uart признак данное передано
uint8_t str_uart_buffer[3];//uart buffer

#define size_arr (10)                  //pwm   in arr
volatile uint32_t falling11 = 0;       //pwm l in
volatile uint8_t flag_falling11 = 0;   //pwm l in
volatile uint8_t counter1 = 0;         //pwm l in arr
volatile uint32_t arr_fall1[size_arr]; //pwm l in arr

volatile uint32_t falling12 = 0;       //pwm r in
volatile uint8_t flag_falling12 = 0;   //pwm r in
volatile uint8_t counter2 = 0;         //pwm r in arr
volatile uint32_t arr_fall2[size_arr]; //pwm r in arr

/*
volatile uint32_t falling21 = 0;
volatile uint32_t falling22 = 0;
volatile uint8_t flag_falling21 = 0;
volatile uint8_t flag_falling22 = 0;
*/


enum state_pwm {
    HIGH,
	PWM,
	LOW
};

uint32_t last_time1 = 0;
uint32_t last_time2 = 0;
GPIO_PinState last_state1 = GPIO_PIN_RESET;
GPIO_PinState last_state2 = GPIO_PIN_RESET;
volatile uint8_t changes_count1 = 0;
volatile uint8_t changes_count2 = 0;
enum state_pwm statepwm1 = LOW;
enum state_pwm statepwm2 = LOW;


volatile uint8_t flag_irq1 = 0;
volatile uint8_t flag_irq2 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void pwm_generate_hnd(void);
uint16_t falling_to_pwm(uint32_t falling);
void button_hnd(void);
void select_print_data(void);
void print_uart_data(uint32_t falling, uint8_t num);

void fill_arr(uint8_t *counter, uint8_t *flag_falling, uint32_t *falling,
              uint32_t arr_fall[], uint32_t falling0);

void pwm_input_hnd(void);
void pwm_in_hnd_i(GPIO_PinState state, GPIO_PinState *last_state,
		uint8_t *changes_count, uint32_t *last_time, enum state_pwm *statepwm,
		uint8_t type);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET); //revers t.k pc13
  HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);


  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);                  //servo
  HAL_GPIO_WritePin(an1_GPIO_Port, an1_Pin, GPIO_PIN_RESET); //servo
  HAL_GPIO_WritePin(an2_GPIO_Port, an2_Pin, GPIO_PIN_RESET); //servo
  HAL_GPIO_WritePin(stby_GPIO_Port, stby_Pin, GPIO_PIN_RESET); //servo


  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);

/*
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  pwm_input_hnd();
	  pwm_generate_hnd();
	  //select_print_data();
	  //button_hnd();
	  //HAL_Delay(1);
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
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 62600;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 30000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOC, led_Pin|led1_Pin|led2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led3_Pin|led4_Pin|an1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, an2_Pin|stby_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : led_Pin led1_Pin led2_Pin */
  GPIO_InitStruct.Pin = led_Pin|led1_Pin|led2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : led3_Pin led4_Pin an1_Pin */
  GPIO_InitStruct.Pin = led3_Pin|led4_Pin|an1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : in1_Pin in2_Pin button_Pin */
  GPIO_InitStruct.Pin = in1_Pin|in2_Pin|button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : an2_Pin stby_Pin */
  GPIO_InitStruct.Pin = an2_Pin|stby_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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


	if(GPIO_Pin == in1_Pin)
	{

	}

	if(GPIO_Pin == in2_Pin)
	{

	}

}

void pwm_input_hnd(void) {
	GPIO_PinState state1 = HAL_GPIO_ReadPin(in1_GPIO_Port, in1_Pin);
	pwm_in_hnd_i(state1, &last_state1, &changes_count1, &last_time1, &statepwm1, 1);

	GPIO_PinState state2 = HAL_GPIO_ReadPin(in2_GPIO_Port, in2_Pin);
	pwm_in_hnd_i(state2, &last_state2, &changes_count2, &last_time2, &statepwm2, 2);
}

void pwm_in_hnd_i(GPIO_PinState state, GPIO_PinState *last_state,
		uint8_t *changes_count, uint32_t *last_time, enum state_pwm *statepwm,
		uint8_t type) {
	uint32_t current_time = HAL_GetTick();

    // Проверка изменения состояния входов
    if (state != *last_state) {
        *last_state = state;
        (*changes_count)++;
    }

    // Проверяем обновление состояния сигнала каждые 10 мс
    if ((int32_t)(current_time - *last_time) > 10) {

		// Если был зафиксирован Ш�?М, не сбрасываем его до статичного сигнала
        if (*changes_count > 5) {
        	*statepwm = PWM;
        } else if (*changes_count == 0) {  // Если сигнал полностью стабилен, сбрасываем PWM
        	*statepwm = (state == GPIO_PIN_SET) ? HIGH : LOW;
        }


        *last_time = current_time;
        // Сбрасываем счетчик изменений
        *changes_count = 0;
	}

	switch(*statepwm)
	{
	    case HIGH: {
	    	switch (type) {
				case 1: {
			    	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
			    	HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
				}
				break;
				case 2: {
			    	HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
		            HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);
				}
				break;
			}

	    }
	    break;
	    case PWM: {
	    	switch (type) {
				case 1: {
			    	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
			    	HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
				}
				break;
				case 2: {
			    	HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_RESET);
		            HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
				}
				break;
			}

	    }
	    break;
	    case LOW: {
	    	switch (type) {
				case 1: {
			    	HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
			    	HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
				}
				break;
				case 2: {
			    	HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
		            HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);
				}
				break;
			}

	    }
	    break;
	}

}

/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t current_time = HAL_GetTick();

    if (GPIO_Pin == in1_Pin) {
        handle_pwm_input(GPIO_Pin, &last_time1, &statepwm1, led1_GPIO_Port, led1_Pin, led2_GPIO_Port, led2_Pin);
    } else if (GPIO_Pin == in2_Pin) {
        handle_pwm_input(GPIO_Pin, &last_time2, &statepwm2, led3_GPIO_Port, led3_Pin, led4_GPIO_Port, led4_Pin);
    }
}

void handle_pwm_input(uint16_t GPIO_Pin, volatile uint32_t *last_time, volatile enum state_pwm *statepwm,
                      GPIO_TypeDef *ledHigh_Port, uint16_t ledHigh_Pin,
                      GPIO_TypeDef *ledPwm_Port, uint16_t ledPwm_Pin) {
    static uint32_t prev_time = 0;
    GPIO_PinState state = HAL_GPIO_ReadPin(GPIOA, GPIO_Pin);
    uint32_t current_time = HAL_GetTick();
    uint32_t time_diff = current_time - *last_time;

    if (state == GPIO_PIN_SET) {
        if (time_diff > 10) {
            *statepwm = PWM;
        }
    } else {
        if (time_diff > 50) {
            *statepwm = LOW;
        }
    }

    if (time_diff < 10) {
        *statepwm = HIGH;
    }

    *last_time = current_time;

    // Управление светодиодами
    switch (*statepwm) {
        case HIGH:
            HAL_GPIO_WritePin(ledHigh_Port, ledHigh_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ledPwm_Port, ledPwm_Pin, GPIO_PIN_RESET);
            break;
        case PWM:
            HAL_GPIO_WritePin(ledHigh_Port, ledHigh_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ledPwm_Port, ledPwm_Pin, GPIO_PIN_SET);
            break;
        case LOW:
            HAL_GPIO_WritePin(ledHigh_Port, ledHigh_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ledPwm_Port, ledPwm_Pin, GPIO_PIN_SET);
            break;
    }
}

*/


//#define PERIOD (htim2.Init.Period)


void pwm_generate_hnd(void) {
	if(statepwm1 == HIGH && statepwm2 == HIGH) {
		HAL_GPIO_WritePin(stby_GPIO_Port, stby_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(an1_GPIO_Port, an1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(an2_GPIO_Port, an2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	} else {
		uint16_t pwm1 = 0;

		if ( (statepwm1 == PWM || statepwm1 == LOW) && statepwm2 == HIGH ) {
			HAL_GPIO_WritePin(an1_GPIO_Port, an1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(an2_GPIO_Port, an2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(stby_GPIO_Port, stby_Pin, GPIO_PIN_SET);

			if (statepwm1 == PWM) pwm1 = falling_to_pwm(falling11);

		}
		else if ( (statepwm2 == PWM || statepwm2 == LOW) && statepwm1 == HIGH  ) {
			HAL_GPIO_WritePin(an1_GPIO_Port, an1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(an2_GPIO_Port, an2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stby_GPIO_Port, stby_Pin, GPIO_PIN_SET);

			if (statepwm2 == PWM) pwm1 = falling_to_pwm(falling12);
		}

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm1);
	}

}

uint16_t falling_to_pwm(uint32_t falling) {
	uint16_t pwm1 = 0;

	print_uart_data(falling, 1);

	if(1700 >= falling && falling >= 1500) {
		pwm1 = 62600 * 389 / 1000;
	} else if(1450 >= falling && falling >= 1250) {
		pwm1 = 62600 * 512 / 1000;
	} else if(1150 >= falling && falling >= 950) {
		pwm1 = 62600 * 630 / 1000;
	} else if(900 >= falling && falling >= 700) {
		pwm1 = 62600 * 750 / 1000;
	} else if(600 >= falling && falling >= 400) {
		pwm1 = 62600 * 872 / 1000;
	} else if(350 >= falling && falling >= 150) {
		pwm1 = 62600 * 950 / 1000;
	}
	return pwm1;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1 ) {

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			//TIM1->CNT = 0;
			__HAL_TIM_SET_COUNTER(&htim1, 0x0000);
		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			uint32_t falling0 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
			//falling11 = falling0;
			//flag_falling11 = 1;

			fill_arr(&counter1, &flag_falling11, &falling11, arr_fall1, falling0);
		 }

		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			//TIM1->CNT = 0;
			__HAL_TIM_SET_COUNTER(&htim1, 0x0000);
		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			uint32_t falling0 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);
			//falling12 = falling0;
			//flag_falling12 = 1;

			fill_arr(&counter2, &flag_falling12, &falling12, arr_fall2, falling0);

		}

	 }


/*
	 if(htim->Instance == TIM2 )
	 {
		 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && !flag_falling21)
		 {
			 TIM2->CNT = 0;

			 falling21 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
			 flag_falling21 =1;

		 }

		 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3 && !flag_falling22)
		 {
			 TIM2->CNT = 0;

			 falling22 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);
			 flag_falling22 =1;

		 }
	 }
*/

}

void fill_arr(uint8_t *counter, uint8_t *flag_falling, uint32_t *falling, uint32_t arr_fall[], uint32_t falling0) {

    arr_fall[(*counter)++] = falling0;

    if (*counter == size_arr && !(*flag_falling)) {
        *falling = 0;
        for (uint8_t var = 0; var < size_arr; var++) {
            *falling += arr_fall[var];
        }
        *falling /= size_arr;
        *counter = 0;
        *flag_falling = 1;
    }
}

void select_print_data(void) {
	if (flag_falling11) {
		print_uart_data(falling11, 1);
        flag_falling11 = 0;
	}

	if (flag_falling12) {
		print_uart_data(falling12, 2);
        flag_falling12 = 0;
	}

}

void print_uart_data(uint32_t falling, uint8_t num) {
	char str1[63] = {0,};
    snprintf(str1, 63, "\nPulse%d %lu\n", num, falling);
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
