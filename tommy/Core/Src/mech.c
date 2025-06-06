#include "mech.h"

LinearActuator linear_actuator[2] = {{.id = 0, .pos = LINEAR_ACUATOR_DOWN},
                                     {.id = 1, .pos = LINEAR_ACUATOR_DOWN}};

bool vertical_linear_actuator_extend = false;

bool prev_vertical_linear_actuator_extend = false;

bool horizontal_linear_actuator_extend = false;

bool prev_horizontal_linear_actuator_extend = false;

void linear_actuator_extend(LinearActuator *actuator) {
  if (actuator->pos == LINEAR_ACUATOR_UP)
    return;

  actuator->pos = LINEAR_ACUATOR_UP;
  switch (actuator->id) {
    case 0:
      HAL_GPIO_WritePin(LINEAR_ACT_1_1_GPIO_Port, LINEAR_ACT_1_1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LINEAR_ACT_1_2_GPIO_Port, LINEAR_ACT_1_2_Pin, GPIO_PIN_RESET);
      break;
    case 1:
      HAL_GPIO_WritePin(LINEAR_ACT_2_1_GPIO_Port, LINEAR_ACT_2_1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LINEAR_ACT_2_2_GPIO_Port, LINEAR_ACT_2_2_Pin, GPIO_PIN_RESET);
      break;
  }
}

void linear_actuator_retract(LinearActuator *actuator) {
  if (actuator->pos == LINEAR_ACUATOR_DOWN)
    return;

  actuator->pos = LINEAR_ACUATOR_DOWN;
  switch (actuator->id) {
    case 0:
      HAL_GPIO_WritePin(LINEAR_ACT_1_1_GPIO_Port, LINEAR_ACT_1_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LINEAR_ACT_1_2_GPIO_Port, LINEAR_ACT_1_2_Pin, GPIO_PIN_SET);
      break;
    case 1:
      HAL_GPIO_WritePin(LINEAR_ACT_2_1_GPIO_Port, LINEAR_ACT_2_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LINEAR_ACT_2_2_GPIO_Port, LINEAR_ACT_2_2_Pin, GPIO_PIN_SET);
      break;
  }
}