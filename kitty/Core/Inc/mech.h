#ifndef __MECH_H__
#define __MECH_H__

#include <stdbool.h>

#include "main.h"

typedef enum {
  BIG_WHEEL_UP,
  BIG_WHEEL_DOWN,
} BigWheelPositionState;

typedef enum {
  BIG_WHEEL_ROTATE_CLOCKWISE,
  BIG_WHEEL_ROTATE_ANTICLOCKWISE,
  BIG_WHEEL_ROTATE_STOP  // stop the big wheel
} BigWheelRotateState;

void big_wheel_move_down(void);

void big_wheel_move_up(void);

void big_wheel_rotate(BigWheelRotateState direction);

extern BigWheelPositionState big_wheel_pos;
extern bool big_wheel_state;
extern bool prev_big_wheel_state;

#endif // __MECH_H__