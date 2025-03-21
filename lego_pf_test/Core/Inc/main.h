/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define led_Pin GPIO_PIN_13
#define led_GPIO_Port GPIOC
#define in2_L_Pin GPIO_PIN_1
#define in2_L_GPIO_Port GPIOA
#define in2_R_Pin GPIO_PIN_3
#define in2_R_GPIO_Port GPIOA
#define in1_L_Pin GPIO_PIN_9
#define in1_L_GPIO_Port GPIOA
#define in1_R_Pin GPIO_PIN_11
#define in1_R_GPIO_Port GPIOA
#define bn2_Pin GPIO_PIN_15
#define bn2_GPIO_Port GPIOA
#define bn1_Pin GPIO_PIN_3
#define bn1_GPIO_Port GPIOB
#define an2_Pin GPIO_PIN_4
#define an2_GPIO_Port GPIOB
#define an1_Pin GPIO_PIN_5
#define an1_GPIO_Port GPIOB
#define stby_Pin GPIO_PIN_6
#define stby_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
