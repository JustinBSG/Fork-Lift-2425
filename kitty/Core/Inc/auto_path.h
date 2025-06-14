#ifndef __AUTO_PATH_H__
#define __AUTO_PATH_H__

#include <stdbool.h>

#include "movement.h"

// TODO: need to slightly reduce the time
#define LEFT_PATH_TIME_1 2000
#define LEFT_PATH_TIME_2 1100
#define MID_PATH_TIME_1 3000 
#define RIGHT_PATH_TIME_1 2000
#define RIGHT_PATH_TIME_2 1100

typedef enum {
  LEFT_PATH,
  MID_PATH,
  RIGHT_PATH
} AutoPathSelection;

void follow_auto_path(AutoPathSelection auto_path_selection);

extern bool auto_path_enable;
extern bool prev_auto_path_enable;
extern bool auto_path_switch;
extern bool prev_auto_path_switch;
extern AutoPathSelection auto_path_selection;

#endif // __AUTO_PATH_H__
