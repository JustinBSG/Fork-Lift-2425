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

#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "controller.h"
#include "movement.h"
#include "servo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int test_encoder[4] = {0, 0, 0, 0};
BaseVelocity test_base_vel = {0, 0, 0};
WheelVelocity test_wheel_vel = {0, 0, 0, 0};
WheelPWM test_pwm = {16800, 16800, 16800, 16800};
WheelVelocity test_read_vel = {0, 0, 0, 0};
int test_count = 0;
int stage = 0;
int test_duration = 0;
int test_var_1 = 0;
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
int main(void) {
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_ALL);
  // enable both sides of motor driver IC
  HAL_GPIO_WritePin(MOTOR_LEFT_ENABLE_GPIO_Port, MOTOR_LEFT_ENABLE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_RIGHT_ENABLE_GPIO_Port, MOTOR_RIGHT_ENABLE_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);

  servo_reset_all();
  HAL_Delay(5000);
  servo_move(&(servos[4]), SERVO_ID5_MAX_POS, SHORTEST_TIME_ROTATE(5, 390));
  HAL_Delay(5000);
  servo_move(&(servos[4]), SERVO_ID5_MIN_POS, SHORTEST_TIME_ROTATE(5, 85));
  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1);
#if (TEST == 0)
    HAL_UART_Transmit(&huart1, controller_buffer, sizeof(controller_buffer), 0xFFFF);
    parse_controller_data(controller_buffer, &controller_state);
    if (controller_state.options_button) {  // turn on/off the robot
      turn_on = !turn_on;
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, turn_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    if (turn_on) {
      float rotation_vel = (controller_state.l2_pressure / 1024.0 + controller_state.r2_pressure / -1024.0) * 100.0;

      if (controller_state.ps_button) {  // auto, line following
        continue;
      }

      if (controller_state.l_stick_x == 0 && controller_state.r_stick_x == 0 && !controller_state.r1 && !controller_state.l1 && rotation_vel != 0) {  // rotate, r2 or l2
        BaseVelocity target_vel = {0, 0, rotation_vel / 100.0 * ROBOT_MAX_Z_VELOCITY};
        movement_control(target_vel);
      } else if (controller_state.r_stick_x == 0 && controller_state.r_stick_y == 0 && !controller_state.r1 && !controller_state.l1) {  // move fastly, left joy stick

      } else if (!controller_state.r1 && !controller_state.l1) {  // move slowly, right joy stick

      } else {  // rotate slowly, l1 or r1
      }
    }
#else
    // test_base_vel.x_vel = 0;
    // test_base_vel.y_vel = 0;
    // test_base_vel.z_vel = ROBOT_MAX_Z_VELOCITY;
    // test_base_vel.x_vel = ROBOT_MAX_X_VELOCITY;
    // test_base_vel.y_vel = 0;
    // test_base_vel.z_vel = 0;
    // test_base_vel.x_vel = ROBOT_MAX_X_VELOCITY;
    // test_base_vel.y_vel = 0;
    // test_base_vel.z_vel = 0;
    // movement_control(test_base_vel);
    // if (HAL_GetTick() - test_duration > 5000) {
    //   test_duration = HAL_GetTick();
    //   test_count++;
    //   if (test_count == 3)
    //     test_count = 0;
    // }
    // if (test_count == 1) {
    //   servo_move(&(servos[4]), 500 + 380, SHORTEST_TIME_ROTATE(5, 380));
    //   test_var_1 = SHORTEST_TIME_ROTATE(5, 380);
    // }
    // else if (test_count == 2)
    //   servo_reset_all();
    test_wheel_vel = read_current_velocity(encoders);
#endif
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
