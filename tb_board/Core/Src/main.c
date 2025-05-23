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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "movement.h"
#include "pid-mecanum.h"
#include "controller.h"
#include <string.h>

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

/* USER CODE BEGIN PV */
int stage = 0;
// BaseVelocity test_target_vel = {0, 0, 0};
WheelVelocity test_target_vel = {0, 0, 0, 0};
WheelPWM test_read_pwm = {0, 0, 0, 0};
WheelVelocity test_read_vel = {0, 0, 0, 0};
BaseVelocity test_base_vel = {0, 0, 0};
WheelVelocity test_pid_value = {0, 0, 0, 0};
BaseVelocity target_base_vel = {0, 0, 0};
float test_var_1 = 0;
char test_buffer[100] = "testing\n";
int start_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM2_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  if (HAL_GPIO_ReadPin(USER_SWITCH_GPIO_Port, USER_SWITCH_Pin) == GPIO_PIN_RESET) {
    HAL_Delay(100);
    if (HAL_GPIO_ReadPin(USER_SWITCH_GPIO_Port, USER_SWITCH_Pin) == GPIO_PIN_RESET) {
      HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);

      stage++;
      if (stage > 6)
        stage = 0;

      // if (stage == 1) {
      //   test_base_vel.x_vel = 0;
      //   test_base_vel.y_vel = ROBOT_MAX_Y_VELOCITY * 0.75;
      //   test_base_vel.z_vel = 0;
      //   movement_control(test_base_vel);
      //   HAL_Delay(375);
      //   test_base_vel.x_vel = 0;
      //   test_base_vel.y_vel = 0;
      //   test_base_vel.z_vel = 0;
      //   movement_control(test_base_vel);
      // } else if (stage == 2) {
      //   test_base_vel.x_vel = 0;
      //   test_base_vel.y_vel = -ROBOT_MAX_Y_VELOCITY * 0.75;
      //   test_base_vel.z_vel = 0;
      //   movement_control(test_base_vel);
      //   HAL_Delay(375);
      //   test_base_vel.x_vel = 0;
      //   test_base_vel.y_vel = 0;
      //   test_base_vel.z_vel = 0;
      //   movement_control(test_base_vel);
      // } else if (stage == 3) {
      //   test_base_vel.x_vel = ROBOT_MAX_X_VELOCITY * 0.75;
      //   test_base_vel.y_vel = 0;
      //   test_base_vel.z_vel = 0;
      //   movement_control(test_base_vel);
      //   HAL_Delay(375);
      //   test_base_vel.x_vel = 0;
      //   test_base_vel.y_vel = 0;
      //   test_base_vel.z_vel = 0;
      //   movement_control(test_base_vel);
      // } else if (stage == 4) {
      //   test_base_vel.x_vel = -ROBOT_MAX_X_VELOCITY * 0.75;
      //   test_base_vel.y_vel = 0;
      //   test_base_vel.z_vel = 0;
      //   movement_control(test_base_vel);
      //   HAL_Delay(375);
      //   test_base_vel.x_vel = 0;
      //   test_base_vel.y_vel = 0;
      //   test_base_vel.z_vel = 0;
      //   movement_control(test_base_vel);
      // } else if (stage == 5) {
      //   test_base_vel.x_vel = 0;
      //   test_base_vel.y_vel = 0;
      //   test_base_vel.z_vel = ROBOT_MAX_Z_VELOCITY * 0.5;
      //   movement_control(test_base_vel);
      //   HAL_Delay(375);
      //   test_base_vel.x_vel = 0;
      //   test_base_vel.y_vel = 0;
      //   test_base_vel.z_vel = 0;
      //   movement_control(test_base_vel);
      // } else if (stage == 6) {
      //   test_base_vel.x_vel = 0;
      //   test_base_vel.y_vel = 0;
      //   test_base_vel.z_vel = -ROBOT_MAX_Z_VELOCITY * 0.5;
      //   movement_control(test_base_vel);
      //   HAL_Delay(375);
      //   test_base_vel.x_vel = 0;
      //   test_base_vel.y_vel = 0;
      //   test_base_vel.z_vel = 0;
      //   movement_control(test_base_vel);
      // }
      if (stage == 0) {
        test_base_vel.x_vel = 0;
        test_base_vel.y_vel = 0;
        test_base_vel.z_vel = 0;
      } else if (stage == 1) { // forward
        test_base_vel.x_vel = ROBOT_MAX_X_VELOCITY * 0.75;
        test_base_vel.y_vel = 0;
        test_base_vel.z_vel = 0;
      } else if (stage == 2) { // backward
        test_base_vel.x_vel = -ROBOT_MAX_X_VELOCITY * 0.75;
        test_base_vel.y_vel = 0;
        test_base_vel.z_vel = 0;
      } else if (stage == 3) { // right 
        test_base_vel.x_vel = 0;
        test_base_vel.y_vel = ROBOT_MAX_Y_VELOCITY * 0.75;
        test_base_vel.z_vel = 0;
      } else if (stage == 4) { // left
        test_base_vel.x_vel = 0;
        test_base_vel.y_vel = -ROBOT_MAX_Y_VELOCITY * 0.75;
        test_base_vel.z_vel = 0;
      } else if (stage == 5) { // clockwise
        test_base_vel.x_vel = 0;
        test_base_vel.y_vel = 0;
        test_base_vel.z_vel = ROBOT_MAX_Z_VELOCITY * 0.5;
      } else if (stage == 6) { // anticlockwise
        test_base_vel.x_vel = 0;
        test_base_vel.y_vel = 0;
        test_base_vel.z_vel = -ROBOT_MAX_Z_VELOCITY * 0.5;
      }

      movement_control(test_base_vel);
    }
  }
  test_read_vel = read_current_velocity(encoders);
  // switch (stage) {
  //   case 1:
  //     target_base_vel.x_vel = ROBOT_MAX_X_VELOCITY * 0.75;
  //     target_base_vel.y_vel = ROBOT_MAX_Y_VELOCITY * 0.5;
  //     target_base_vel.z_vel = 0;
  //     break;
  //   default:
  //     target_base_vel.x_vel = 0;
  //     target_base_vel.y_vel = 0;
  //     target_base_vel.z_vel = 0;
  //     break;
  // }
//
//    movement_control(target_base_vel);
//
//    test_read_vel = read_current_velocity(encoders);
//    test_read_pwm = wheel2pwm(test_read_vel);
//    test_base_vel = wheel2base(test_read_vel);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  while (1) {
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
