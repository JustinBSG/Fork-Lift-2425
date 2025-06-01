#ifndef __MECH_H__
#define __MECH_H__

#include "servo.h"

typedef enum {
  FRONT_BACK,
  LEFT_RIGHT, 
  ROTATE
} Direction_Encoder;

extern Direction_Encoder direction_encoder;

#endif // __MECH_H__