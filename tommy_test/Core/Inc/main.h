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
#include "stm32g4xx_hal.h"

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
#define FR_PWM_Pin GPIO_PIN_2
#define FR_PWM_GPIO_Port GPIOE
#define C_IN1_Pin GPIO_PIN_3
#define C_IN1_GPIO_Port GPIOE
#define C_IN2_Pin GPIO_PIN_4
#define C_IN2_GPIO_Port GPIOE
#define FL_PWM_Pin GPIO_PIN_5
#define FL_PWM_GPIO_Port GPIOE
#define D_IN1_Pin GPIO_PIN_6
#define D_IN1_GPIO_Port GPIOE
#define RL_PWM_Pin GPIO_PIN_3
#define RL_PWM_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_7
#define LED4_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOE
#define D_IN2_Pin GPIO_PIN_13
#define D_IN2_GPIO_Port GPIOE
#define A_IN2_Pin GPIO_PIN_8
#define A_IN2_GPIO_Port GPIOA
#define A_IN1_Pin GPIO_PIN_9
#define A_IN1_GPIO_Port GPIOA
#define B_IN1_Pin GPIO_PIN_10
#define B_IN1_GPIO_Port GPIOA
#define B_IN2_Pin GPIO_PIN_11
#define B_IN2_GPIO_Port GPIOA
#define LINEAR_ACT_2_2_Pin GPIO_PIN_1
#define LINEAR_ACT_2_2_GPIO_Port GPIOD
#define LINEAR_ACT_1_1_Pin GPIO_PIN_2
#define LINEAR_ACT_1_1_GPIO_Port GPIOD
#define LINEAR_ACT_1_2_Pin GPIO_PIN_4
#define LINEAR_ACT_1_2_GPIO_Port GPIOD
#define LINEAR_ACT_2_1_Pin GPIO_PIN_6
#define LINEAR_ACT_2_1_GPIO_Port GPIOD
#define RR_PWM_Pin GPIO_PIN_7
#define RR_PWM_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
