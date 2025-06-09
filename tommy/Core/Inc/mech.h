#ifndef __MECH_H__
#define __MECH_H__

#include <stdbool.h>

#include "main.h"

typedef enum {
  LINEAR_ACUATOR_EXTEND,
  LINEAR_ACUATOR_RETRACT,
  LINEAR_ACUATOR_IDLE
} LinearActuatorOperation;

typedef struct {
    int id;
} LinearActuator;

void linear_actuator_operation(LinearActuator *actuator, LinearActuatorOperation operation);

extern LinearActuator linear_actuator[2];

extern bool vertical_linear_actuator_extend;
extern bool prev_vertical_linear_actuator_extend;
extern bool horizontal_linear_actuator_extend;
extern bool prev_horizontal_linear_actuator_extend;

#endif  // __MECH_H__