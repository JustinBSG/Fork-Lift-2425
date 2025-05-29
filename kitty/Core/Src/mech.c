#include "mech.h"

bool big_wheel_state = false;
bool prev_big_wheel_state = false;

BigWheelPositionState big_wheel_pos = BIG_WHEEL_DOWN;

void big_wheel_move_up(void) {
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
}

void big_wheel_move_down(void) {
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
}

void big_wheel_rotate(BigWheelRotateState direction) {
  switch (direction) {
    case BIG_WHEEL_ROTATE_CLOCKWISE:
      // set DIR pin to low
      // toggle STEP pin to rotate the big wheel
      // find a var to store the time stamp
      // if current - time stamp > 500, toggle
      // faster frequency => faster rotation
      HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
      break;
    case BIG_WHEEL_ROTATE_ANTICLOCKWISE:
      // set DIR pin to high
      // toggle STEP pin to rotate the big wheel
      // find a var to store the time stamp
      // if current - time stamp > 500, toggle
      // faster frequency => faster rotation
      HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
      break;
    default:
      // reset all things
      HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
      break;
  }

  // EN pin to high => enable the motor
  // m1, m2, m3 to low => full step mode
}