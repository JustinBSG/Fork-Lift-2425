#ifndef __MECH_H__
#define __MECH_H__

#include <stdbool.h> 

#include "servo.h"

typedef enum {
  FRONT_BACK,
  LEFT_RIGHT, 
  ROTATE
} Direction_Encoder;

void catch_move_down(void);

void catch_move_up(void);

void catch_reset(void);

void container_move_down(void);

void container_reset(void);

extern Direction_Encoder direction_encoder;

extern bool container_down;
extern bool prev_container_down;

#endif // __MECH_H__