#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>

#include "controller.h"

uint8_t parse_controller_data(const char* input, Controller_data* data) {
  int result = sscanf(input, "c%" SCNd8 ",%" SCNd8 ",%" SCNu8 ",%" SCNu8 ",%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu",
                          &data->stick_x, &data->stick_y, &data->r2, &data->l2,
                          (uint8_t *)&data->r1, (uint8_t *)&data->l1, (uint8_t *)&data->cross, (uint8_t *)&data->circle, (uint8_t *)&data->triangle, (uint8_t *)&data->square,
                          (uint8_t *)&data->up, (uint8_t *)&data->down, (uint8_t *)&data->left, (uint8_t *)&data->right);
  if (result != 14)
    return 1;
  
  return 0;
}