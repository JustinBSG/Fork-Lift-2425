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
#include "auto_path.h"
#include "controller.h"
#include "encoder.h"
#include "mech.h"
#include "movement.h"
#include "robot.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int test_encoder[4] = {0, 0, 0, 0};
BaseVelocity test_target_base_vel = {0, 0, 0};
int test_time = 0;
int test_stage = 0;
int test_ccr = 0;
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_ALL);

  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);

  // HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(MOTOR_FR_IN1_GPIO_Port, MOTOR_FR_IN1_Pin, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(MOTOR_FR_IN2_GPIO_Port, MOTOR_FR_IN2_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(MOTOR_RL_IN1_GPIO_Port, MOTOR_RL_IN1_Pin, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(MOTOR_RL_IN2_GPIO_Port, MOTOR_RL_IN2_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(MOTOR_RR_IN1_GPIO_Port, MOTOR_RR_IN1_Pin, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(MOTOR_RR_IN2_GPIO_Port, MOTOR_RR_IN2_Pin, GPIO_PIN_SET);

  // TIM3->CCR2 = 65535/2;
  // TIM2->CCR1 = 65535/2;
  // TIM2->CCR2 = 65535/2;
  // TIM3->CCR1 = 65535/2;

  big_wheel_move_up();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if (TEST == 0)
    HAL_UART_Receive(&huart1, controller_buffer, sizeof(controller_buffer), 0xFFFF);
    parse_controller_data(controller_buffer, &controller_state);

    if (controller_state.options_button && !prev_turn_on) {  // turn on/off the robot
      turn_on = !turn_on;
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, turn_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    prev_turn_on = controller_state.options_button;

    if (turn_on) {
      if (auto_path_selection == LEFT_PATH) {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
      } else if (auto_path_selection == MID_PATH) {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
      } else if (auto_path_selection == RIGHT_PATH) {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
      }

      float rotation_vel = (controller_state.l2_pressure / 1024.0 + controller_state.r2_pressure / -1024.0) * 100.0;

      if (controller_state.ps_button && !prev_auto_path_enable) {  // auto, line following
        auto_path_enable = !auto_path_enable;
        if (auto_path_enable)
          follow_auto_path(auto_path_selection);
      }
      prev_auto_path_enable = controller_state.ps_button;

      if (controller_state.cross && !prev_auto_path_switch) {
        auto_path_switch = !auto_path_switch;
        if (auto_path_selection == LEFT_PATH)
          auto_path_selection = MID_PATH;
        else if (auto_path_selection == MID_PATH)
          auto_path_selection = RIGHT_PATH;
        else if (auto_path_selection == RIGHT_PATH)
          auto_path_selection = LEFT_PATH;
      }  // auto, choose path, toggle left / right / straight forward
      prev_auto_path_switch = controller_state.cross;

      if (controller_state.left && !controller_state.right && !controller_state.up && !controller_state.down) {
        BaseVelocity target_vel = {0, ROBOT_MAX_X_VELOCITY * 0.15, 0};
        movement_control(target_vel);  // move to the left
      } else if (!controller_state.left && controller_state.right && !controller_state.up && !controller_state.down) {
        BaseVelocity target_vel = {0, -ROBOT_MAX_X_VELOCITY * 0.15, 0};
        movement_control(target_vel);  // move to the right
      } else if (!controller_state.left && !controller_state.right && controller_state.up && !controller_state.down) {
        BaseVelocity target_vel = {ROBOT_MAX_Y_VELOCITY * 0.15, 0, 0};
        movement_control(target_vel);  // move forward
      } else if (!controller_state.left && !controller_state.right && !controller_state.up && controller_state.down) {
        BaseVelocity target_vel = {-ROBOT_MAX_Y_VELOCITY * 0.15, 0, 0};
        movement_control(target_vel);                                                                                                                        // move backward
      } else if (controller_state.l_stick_x == 0 && controller_state.l_stick_y == 0 && rotation_vel != 0 && !controller_state.r1 && !controller_state.l1) {  // rotate
        BaseVelocity target_vel = {0, 0, rotation_vel / 100.0 * ROBOT_MAX_Z_VELOCITY * 0.35};
        movement_control(target_vel);
      } else if (controller_state.r_stick_x == 0 && controller_state.r_stick_y == 0 && !controller_state.r1 && !controller_state.l1) {  // move fastly
        BaseVelocity target_vel = {controller_state.l_stick_y / 100.0 * ROBOT_MAX_Y_VELOCITY * 0.5,
                                   controller_state.l_stick_x / 100.0 * ROBOT_MAX_X_VELOCITY * 0.5,
                                   0};
        movement_control(target_vel);
      } else if (!controller_state.r1 && !controller_state.l1) {  // move slowly
        BaseVelocity target_vel = {controller_state.r_stick_y / 100.0 * ROBOT_MAX_Y_VELOCITY * 0.25,
                                   controller_state.r_stick_x / 100.0 * ROBOT_MAX_X_VELOCITY * 0.25,
                                   0};
        movement_control(target_vel);
      } else if (controller_state.l1 || controller_state.r1) {  // rotate slowly
        BaseVelocity target_vel = {0, 0, 0};
        if (controller_state.l1)
          target_vel.z_vel = ROBOT_MAX_Z_VELOCITY * 0.15;
        else if (controller_state.r1)
          target_vel.z_vel = ROBOT_MAX_Z_VELOCITY * -0.15;
        movement_control(target_vel);
      }

      if (controller_state.triangle && !prev_big_wheel_state) {  // lift up / down the big wheel
        big_wheel_state = !big_wheel_state;
        if (big_wheel_pos == BIG_WHEEL_DOWN) {
          big_wheel_move_up();
          big_wheel_pos = BIG_WHEEL_UP;
        } else if (big_wheel_pos == BIG_WHEEL_UP) {
          big_wheel_move_down();
          big_wheel_pos = BIG_WHEEL_DOWN;
        }
      }
      prev_big_wheel_state = controller_state.triangle;

      if (controller_state.square && !controller_state.circle)  // collect ball
        big_wheel_rotate(BIG_WHEEL_ROTATE_CLOCKWISE);
      else if (controller_state.circle && !controller_state.square)  // release ball
        big_wheel_rotate(BIG_WHEEL_ROTATE_ANTICLOCKWISE);
      else if (!controller_state.circle && !controller_state.square)  // stop the big wheel
        big_wheel_rotate(BIG_WHEEL_ROTATE_STOP);
    }
#else
    // switch (test_stage) {
    //   case 1:
    //     test_target_base_vel.x_vel = ROBOT_MAX_Y_VELOCITY;
    //     test_target_base_vel.y_vel = 0;
    //     test_target_base_vel.z_vel = 0;
    //     break;
    //   case 2:
    //     test_target_base_vel.x_vel = 0;
    //     test_target_base_vel.y_vel = ROBOT_MAX_X_VELOCITY;
    //     test_target_base_vel.z_vel = 0;
    //     break;
    //   case 3:
    //     test_target_base_vel.x_vel = 0;
    //     test_target_base_vel.y_vel = 0;
    //     test_target_base_vel.z_vel = ROBOT_MAX_Z_VELOCITY;
    //     break;
    //   case 4:
    //     test_target_base_vel.x_vel = 0;
    //     test_target_base_vel.y_vel = 0;
    //     test_target_base_vel.z_vel = 0;
    //     test_stage = 0;
    //     break;
    // }

    test_target_base_vel.x_vel = ROBOT_MAX_Y_VELOCITY / 2.0;
    test_target_base_vel.y_vel = 0;
    test_target_base_vel.z_vel = 0;
    movement_control(test_target_base_vel);
    // if (HAL_GetTick() - test_time > 3000) {
    //   test_time = HAL_GetTick();
    //   HAL_GPIO_TogglePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin);
    //   HAL_GPIO_TogglePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin);
    //   HAL_GPIO_TogglePin(MOTOR_FR_IN1_GPIO_Port, MOTOR_FR_IN1_Pin);
    //   HAL_GPIO_TogglePin(MOTOR_FR_IN2_GPIO_Port, MOTOR_FR_IN2_Pin);
    //   HAL_GPIO_TogglePin(MOTOR_RL_IN1_GPIO_Port, MOTOR_RL_IN1_Pin);
    //   HAL_GPIO_TogglePin(MOTOR_RL_IN2_GPIO_Port, MOTOR_RL_IN2_Pin);
    //   HAL_GPIO_TogglePin(MOTOR_RR_IN1_GPIO_Port, MOTOR_RR_IN1_Pin);
    //   HAL_GPIO_TogglePin(MOTOR_RR_IN2_GPIO_Port, MOTOR_RR_IN2_Pin);
    // }
#endif
    HAL_Delay(1);
    read_current_velocity(encoders);
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
