#ifndef __MECH_H__
#define __MECH_H__

#include <stdbool.h>

#include "main.h"

typedef enum {
  LINEAR_ACUATOR_UP,
  LINEAR_ACUATOR_DOWN,
} LinearActuatorPosition;

typedef struct {
  int id;
  LinearActuatorPosition pos;
} LinearActuator;

void linear_actuator_extend(LinearActuator *actuator);

void linear_actuator_retract(LinearActuator *actuator);

extern LinearActuator linear_actuator[2];

#endif // __MECH_H__