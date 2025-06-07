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
BaseVelocity test_base_vel = {0, 0, 0};
int time_stamp = 0;
int test_stage = 0;
int test_var = 0;
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
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
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

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1);
#if (TEST == 0)
    HAL_UART_Receive(&huart1, controller_buffer, sizeof(controller_buffer), 0xFFFF);
    parse_controller_data(controller_buffer, &controller_state);

    if (controller_state.options_button && !prev_turn_on) {  // turn on/off the robot
      turn_on = !turn_on;
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, turn_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    prev_turn_on = controller_state.options_button;

    if (turn_on) {
      if (auto_path_selection == LEFT_PATH) {
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
      } else if (auto_path_selection == MID_PATH) {
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
      } else if (auto_path_selection == RIGHT_PATH) {
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
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
        BaseVelocity target_vel = {0, ROBOT_MAX_X_VELOCITY * 0.25, 0};  // move to the left
        movement_control(target_vel);
      } else if (!controller_state.left && controller_state.right && !controller_state.up && !controller_state.down) {
        BaseVelocity target_vel = {0, -ROBOT_MAX_X_VELOCITY * 0.25, 0};  // move to the right
        movement_control(target_vel);
      } else if (!controller_state.left && !controller_state.right && controller_state.up && !controller_state.down) {
        BaseVelocity target_vel = {ROBOT_MAX_Y_VELOCITY * 0.25, 0, 0};  // move forward
        movement_control(target_vel);
      } else if (!controller_state.left && !controller_state.right && !controller_state.up && controller_state.down) {
        BaseVelocity target_vel = {-ROBOT_MAX_Y_VELOCITY * 0.25, 0, 0};  // move backward
        movement_control(target_vel);
      } else if (controller_state.l_stick_x == 0 && controller_state.l_stick_y == 0 && rotation_vel != 0 && !controller_state.r1 && !controller_state.l1) {  // rotate
        BaseVelocity target_vel = {0, 0, rotation_vel / 100.0 * ROBOT_MAX_Z_VELOCITY * 0.5};
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
          target_vel.z_vel = ROBOT_MAX_Z_VELOCITY * 0.25;
        else if (controller_state.r1)
          target_vel.z_vel = ROBOT_MAX_Z_VELOCITY * -0.25;
        movement_control(target_vel);
      }

      if (controller_state.triangle && !prev_vertical_linear_actuator_extend) {  // extend / retract vertical linear actuator
        vertical_linear_actuator_extend = !vertical_linear_actuator_extend;
        if (vertical_linear_actuator_extend)
          linear_actuator_extend(&linear_actuator[0]);
        else
          linear_actuator_retract(&linear_actuator[0]);
      }
      prev_vertical_linear_actuator_extend = controller_state.triangle;

      if (controller_state.square && !horizontal_linear_actuator_extend) {  // extend / retract horizontal linear actuator
        horizontal_linear_actuator_extend = !horizontal_linear_actuator_extend;
        if (horizontal_linear_actuator_extend)
          linear_actuator_extend(&linear_actuator[1]);
        else
          linear_actuator_retract(&linear_actuator[1]);
      }
      prev_horizontal_linear_actuator_extend = controller_state.square;
    }
#else
    if (HAL_GetTick() - time_stamp > 3000) {
      time_stamp = HAL_GetTick();
      test_stage++;
      // HAL_GPIO_TogglePin(LINEAR_ACT_1_1_GPIO_Port, LINEAR_ACT_1_1_Pin);
      // HAL_GPIO_TogglePin(LINEAR_ACT_1_2_GPIO_Port, LINEAR_ACT_1_2_Pin);
      // HAL_GPIO_TogglePin(LINEAR_ACT_2_1_GPIO_Port, LINEAR_ACT_2_1_Pin);
      // HAL_GPIO_TogglePin(LINEAR_ACT_2_2_GPIO_Port, LINEAR_ACT_2_2_Pin);
      // HAL_GPIO_TogglePin(A_IN1_GPIO_Port, A_IN1_Pin);
      // HAL_GPIO_TogglePin(A_IN2_GPIO_Port, A_IN2_Pin);
      // HAL_GPIO_TogglePin(B_IN1_GPIO_Port, B_IN1_Pin);
      // HAL_GPIO_TogglePin(B_IN2_GPIO_Port, B_IN2_Pin);
      // HAL_GPIO_TogglePin(C_IN1_GPIO_Port, C_IN1_Pin);
      // HAL_GPIO_TogglePin(C_IN2_GPIO_Port, C_IN2_Pin);
      // HAL_GPIO_TogglePin(D_IN1_GPIO_Port, D_IN1_Pin);
      // HAL_GPIO_TogglePin(D_IN2_GPIO_Port, D_IN2_Pin);
    }

    switch (test_stage%8) {
      case 0:
        test_base_vel.x_vel = ROBOT_MAX_Y_VELOCITY * 0.5;
        test_base_vel.y_vel = 0;
        test_base_vel.z_vel = 0;
        break;
      case 1:
        test_base_vel.x_vel = ROBOT_MAX_Y_VELOCITY * -0.5;
        test_base_vel.y_vel = 0;
        test_base_vel.z_vel = 0;
        break;
      case 2:
        test_base_vel.x_vel = 0;
        test_base_vel.y_vel = ROBOT_MAX_X_VELOCITY * 0.5;
        test_base_vel.z_vel = 0;
        break;
      case 3:
        test_base_vel.x_vel = 0;
        test_base_vel.y_vel = ROBOT_MAX_X_VELOCITY * -0.5;
        test_base_vel.z_vel = 0;
        break;
      case 4:
        test_base_vel.x_vel = 0;
        test_base_vel.y_vel = 0;
        test_base_vel.z_vel = ROBOT_MAX_Z_VELOCITY * 0.25;
        break;
      case 5:
        test_base_vel.x_vel = 0;
        test_base_vel.y_vel = 0;
        test_base_vel.z_vel = ROBOT_MAX_Z_VELOCITY * -0.25;
        break;
      case 6:
        test_base_vel.x_vel = 0;
        test_base_vel.y_vel = 0;
        test_base_vel.z_vel = 0;
        // linear_actuator_extend(&(linear_actuator[0]));
        // linear_actuator_extend(&(linear_actuator[1]));
        break;
      case 7:
        test_base_vel.x_vel = 0;
        test_base_vel.y_vel = 0;
        test_base_vel.z_vel = 0;
        // linear_actuator_retract(&(linear_actuator[0]));
        // linear_actuator_retract(&(linear_actuator[1]));
        break;
      default:
        break;
    }
    movement_control(test_base_vel);
    read_current_velocity(encoders);

#endif
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
