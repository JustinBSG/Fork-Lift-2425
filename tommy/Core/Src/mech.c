#include "mech.h"

LinearActuator linear_actuator[2] = {{.id = 0},
                                     {.id = 1}};

void linear_actuator_operation(LinearActuator *actuator, LinearActuatorOperation operation) {
  switch (operation) {
    case LINEAR_ACUATOR_EXTEND:
      if (actuator->id == 0) {
        HAL_GPIO_WritePin(LINEAR_ACT_1_1_GPIO_Port, LINEAR_ACT_1_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LINEAR_ACT_1_2_GPIO_Port, LINEAR_ACT_1_2_Pin, GPIO_PIN_RESET);
      } else if (actuator->id == 1) {
        HAL_GPIO_WritePin(LINEAR_ACT_2_1_GPIO_Port, LINEAR_ACT_2_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LINEAR_ACT_2_2_GPIO_Port, LINEAR_ACT_2_2_Pin, GPIO_PIN_RESET);
      }
      break;
    case LINEAR_ACUATOR_RETRACT:
      if (actuator->id == 0) {
        HAL_GPIO_WritePin(LINEAR_ACT_1_1_GPIO_Port, LINEAR_ACT_1_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LINEAR_ACT_1_2_GPIO_Port, LINEAR_ACT_1_2_Pin, GPIO_PIN_SET);
      } else if (actuator->id == 1) {
        HAL_GPIO_WritePin(LINEAR_ACT_2_1_GPIO_Port, LINEAR_ACT_2_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LINEAR_ACT_2_2_GPIO_Port, LINEAR_ACT_2_2_Pin, GPIO_PIN_SET);
      }
      break;
    case LINEAR_ACUATOR_IDLE:
      if (actuator->id == 0) {
        HAL_GPIO_WritePin(LINEAR_ACT_1_1_GPIO_Port, LINEAR_ACT_1_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LINEAR_ACT_1_2_GPIO_Port, LINEAR_ACT_1_2_Pin, GPIO_PIN_RESET);
      } else if (actuator->id == 1) {
        HAL_GPIO_WritePin(LINEAR_ACT_2_1_GPIO_Port, LINEAR_ACT_2_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LINEAR_ACT_2_2_GPIO_Port, LINEAR_ACT_2_2_Pin, GPIO_PIN_RESET);
      }
      break;
    default:
      if (actuator->id == 0) {
        HAL_GPIO_WritePin(LINEAR_ACT_1_1_GPIO_Port, LINEAR_ACT_1_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LINEAR_ACT_1_2_GPIO_Port, LINEAR_ACT_1_2_Pin, GPIO_PIN_RESET);
      } else if (actuator->id == 1) {
        HAL_GPIO_WritePin(LINEAR_ACT_2_1_GPIO_Port, LINEAR_ACT_2_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LINEAR_ACT_2_2_GPIO_Port, LINEAR_ACT_2_2_Pin, GPIO_PIN_RESET);
      }
      break;
  }
}