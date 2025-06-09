#ifndef __MECH_H__
#define __MECH_H__

#include <stdbool.h>

#include "servo.h"

typedef enum {
  FRONT_BACK,
  LEFT_RIGHT, 
  ROTATE
} Direction_Encoder;

extern Direction_Encoder direction_encoder;
extern bool turn_on_fan;  
extern bool prev_turn_on_fan;

#endif // __MECH_H__