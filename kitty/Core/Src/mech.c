#include "mech.h"

bool big_wheel_state = false;
bool prev_big_wheel_state = false;

BigWheelPositionState big_wheel_pos = BIG_WHEEL_DOWN;

void big_wheel_move_up(void) {
  TIM2->CCR4 = 65535/14;
}

void big_wheel_move_down(void) {
  TIM2->CCR4 = 65535/7;
}

void big_wheel_rotate(BigWheelRotateState direction) {
  switch (direction) {
    case BIG_WHEEL_ROTATE_CLOCKWISE:
      HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, GPIO_PIN_SET);
      TIM3->CCR4 = 65535 / 4;
      break;
    case BIG_WHEEL_ROTATE_ANTICLOCKWISE:
      HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, GPIO_PIN_RESET);
      TIM3->CCR4 = 65535 / 4;
      break;
    default:
      TIM3->CCR4 = 0;
      break;
  }
}