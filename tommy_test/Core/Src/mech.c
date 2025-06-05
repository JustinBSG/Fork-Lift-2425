#include "mech.h"

LinearActuator linear_actuator[2] = {{.id = 0, .pos = LINEAR_ACUATOR_DOWN},
                                     {.id = 1, .pos = LINEAR_ACUATOR_DOWN}};

void linear_actuator_extend(LinearActuator *actuator) {
  if (actuator->pos == LINEAR_ACUATOR_UP)
    return;

  actuator->pos = LINEAR_ACUATOR_UP;
  // TODO: Implement the actual extension logic here
}

void linear_actuator_retract(LinearActuator *actuator) {
  if (actuator->pos == LINEAR_ACUATOR_DOWN)
    return;

  actuator->pos = LINEAR_ACUATOR_DOWN;
  // TODO: Implement the actual extension logic here
}