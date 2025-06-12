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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "auto_path.h"
#include "controller.h"
#include "mech.h"
#include "movement.h"
#include "robot.h"
#include "servo.h"
#include "test.h"

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
int time_stamp = 0;
BaseVelocity test_base_vel = {0, 0, 0};
WheelPWM test_pwm = {0, 0, 0, 0};
WheelVelocity test_wheel_vel = {0, 0, 0, 0};
int test_var = 0;
uint8_t test_buffer[3] = {0, 0, 0};

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_ALL);

  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
  // HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);

  hmc5883l_init();
  HAL_Delay(10);

  hmc5883l_read(HMC5883L_REG_ADDR_IDA, &(test_buffer[0]));
  hmc5883l_read(HMC5883L_REG_ADDR_IDB, &(test_buffer[1]));
  hmc5883l_read(HMC5883L_REG_ADDR_IDC, &(test_buffer[2]));

  // BaseVelocity target_vel = {ROBOT_MAX_X_VELOCITY * 0.5,
  //                            0,
  //                            0};
  // rotate_motor(target_vel);

  // servo_move(&(servos[0]), SERVO_ID1_INITIAL_POS);
  // servo_move(&(servos[1]), SERVO_ID2_INITIAL_POS);
  // servo_move(&(servos[2]), SERVO_ID3_INITIAL_POS);
  // servo_move(&(servos[3]), SERVO_ID4_INITIAL_POS);

  // fan_operation(false);
  // HAL_Delay(1000);

  // servo_reset_all();
  // HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1);
    read_current_velocity(encoders);
#if (TEST == 0)
    HAL_UART_Receive(&huart1, controller_buffer, sizeof(controller_buffer), 0xFFFF);
    parse_controller_data(controller_buffer, &controller_state);
    if (controller_state.options_button && !prev_turn_on) {  // turn on/off the robot
      turn_on = !turn_on;
      // HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, turn_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
      if (!turn_on) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
      }
    }
    prev_turn_on = controller_state.options_button;

    if (turn_on) {
      if (auto_path_selection == LEFT_PATH) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
      } else if (auto_path_selection == MID_PATH) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
      } else if (auto_path_selection == RIGHT_PATH) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
      }

      float rotation_vel = (controller_state.l2_pressure / 1024.0 + controller_state.r2_pressure / -1024.0) * 100.0;

      if (controller_state.ps_button && !prev_auto_path_enable) {  // auto, line following
        auto_path_enable = !auto_path_enable;
        if (auto_path_enable)
          follow_auto_path(auto_path_selection);
      }
      prev_auto_path_enable = controller_state.ps_button;

      if (controller_state.square && !prev_auto_path_switch) {
        auto_path_switch = !auto_path_switch;
        if (auto_path_selection == LEFT_PATH)
          auto_path_selection = MID_PATH;
        else if (auto_path_selection == MID_PATH)
          auto_path_selection = RIGHT_PATH;
        else if (auto_path_selection == RIGHT_PATH)
          auto_path_selection = LEFT_PATH;
      }  // auto, choose path, toggle left / right / straight forward
      prev_auto_path_switch = controller_state.square;

      if (controller_state.r2 || controller_state.l2) {
        BaseVelocity target_vel = {0, 0, rotation_vel / 100.0 * ROBOT_MAX_Z_VELOCITY * 0.35};
        movement_control(target_vel);
      } else if (controller_state.r1) {
        BaseVelocity target_vel = {0, 0, ROBOT_MAX_Z_VELOCITY * -0.2};
        movement_control(target_vel);
      } else if (controller_state.l1) {
        BaseVelocity target_vel = {0, 0, ROBOT_MAX_Z_VELOCITY * 0.2};
        movement_control(target_vel);
      } else if (controller_state.up) {
        BaseVelocity target_vel = {0,
                                   ROBOT_MAX_Y_VELOCITY * 0.5,
                                   0};
        movement_control(target_vel);
      } else if (controller_state.down) {
        BaseVelocity target_vel = {0,
                                   ROBOT_MAX_Y_VELOCITY * -0.5,
                                   0};
        movement_control(target_vel);
      } else if (controller_state.left) {
        BaseVelocity target_vel = {ROBOT_MAX_X_VELOCITY * 0.5,
                                   0,
                                   0};
        movement_control(target_vel);
      } else if (controller_state.right) {
        BaseVelocity target_vel = {ROBOT_MAX_X_VELOCITY * -0.5,
                                   0,
                                   0};
        movement_control(target_vel);
      } else {
        WheelPWM target_pwm = {0, 0, 0, 0};
        wheels_control(target_pwm);
      }

      if (controller_state.triangle && !prev_turn_on_fan) {
        turn_on_fan = !turn_on_fan;
        fan_operation(turn_on_fan);
      }
      prev_turn_on_fan = controller_state.triangle;
    }
#else
    HAL_UART_Receive(&huart1, controller_buffer, sizeof(controller_buffer), 0xFFFF);
    parse_controller_data(controller_buffer, &controller_state);
    if (controller_state.options_button && !prev_turn_on) {  // turn on/off the robot
      turn_on = !turn_on;
      // HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, turn_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
      if (!turn_on) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
      }
    }
    prev_turn_on = controller_state.options_button;

    // if (HAL_GetTick() - time_stamp > 5000) {
    //   time_stamp = HAL_GetTick();
    //   // HAL_GPIO_TogglePin(C_IN1_GPIO_Port, C_IN1_Pin);
    //   // HAL_GPIO_TogglePin(C_IN2_GPIO_Port, C_IN2_Pin);
    //   // HAL_GPIO_TogglePin(D_IN1_GPIO_Port, D_IN1_Pin);
    //   // HAL_GPIO_TogglePin(D_IN2_GPIO_Port, D_IN2_Pin);
    //   // HAL_GPIO_TogglePin(A_IN1_GPIO_Port, A_IN1_Pin);
    //   // HAL_GPIO_TogglePin(A_IN2_GPIO_Port, A_IN2_Pin);
    //   // HAL_GPIO_TogglePin(B_IN1_GPIO_Port, B_IN1_Pin);
    //   // HAL_GPIO_TogglePin(B_IN2_GPIO_Port, B_IN2_Pin);
    //   uint8_t send_buffer[10];
    //   send_buffer[0] = send_buffer[1] = 0x55;
    //   send_buffer[2] = 0x08;
    //   send_buffer[3] = 0x03;
    //   send_buffer[4] = 0x01;
    //   send_buffer[5] = 0xE8;
    //   send_buffer[6] = 0x03;
    //   send_buffer[7] = 0x01;
    //   send_buffer[8] = 0x20;
    //   send_buffer[9] = 0x03;
    //   HAL_UART_Transmit(&huart4, send_buffer, sizeof(send_buffer), 0xFFFF);
    // }
    // if (HAL_GetTick() - time_stamp > 500) {
    //   time_stamp = HAL_GetTick();
    //   test_var += 50;
    // }

    // TIM3->CCR2 = test_var; // PB5
    // TIM3->CCR3 = test_var; // PC8
    // TIM3->CCR4 = test_var; // PC9
    // TIM9->CCR1 = test_var; // PE5
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
