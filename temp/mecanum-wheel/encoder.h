#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>

#include "main.h"

typedef struct {
  float velocity; // use number of rotations to find total rotation degree to find angular velocity in rad/s
  float displacement; // use counter value to find number of rotations
  float position; // total number of rotations
  uint32_t last_tick;
  uint32_t last_counter_value;
  TIM_HandleTypeDef* htim;
} Encoder;

void update_encoder(Encoder* encoder);

void update_all_encoders(Encoder** encoder);

void reset_encoder(Encoder* encoder);

#endif  // __ENCODER_H__