#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  int8_t stick_x;
  int8_t stick_y;
  uint8_t r2;
  uint8_t l2;
  bool r1;
  bool l1;
  bool cross;
  bool circle;
  bool triangle;
  bool square;
  bool up;
  bool down;
  bool left;
  bool right;
} Controller_data;

uint8_t parse_controller_data(const char* input, Controller_data* data);

#endif // CONTROLLER_H