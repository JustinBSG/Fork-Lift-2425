#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

typedef struct {
  bool r1;
  bool r2;
  bool r3;
  bool l1;
  bool l2;
  bool l3;
  bool cross;
  bool circle;
  bool triangle;
  bool square;
  bool up;
  bool down;
  bool left;
  bool right;
  int8_t l_stick_x;
  int8_t l_stick_y;
  int8_t r_stick_x;
  int8_t r_stick_y;
  uint16_t l2_pressure;
  uint16_t r2_pressure;
  bool ps_button;
  bool share_button;
  bool options_button;
} ControllerState;

uint8_t parse_controller_data(const char* input, ControllerState* data);

extern char controller_buffer[41];
extern ControllerState controller_state;

#endif // __CONTROLLER_H__