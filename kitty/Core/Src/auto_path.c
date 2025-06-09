#include "auto_path.h"

bool auto_path_enable = false;

bool prev_auto_path_enable = false;

bool auto_path_switch = false;

bool prev_auto_path_switch = false;

AutoPathSelection auto_path_selection = LEFT_PATH;

void follow_auto_path(AutoPathSelection auto_path_selection) {
  switch (auto_path_selection) {
    case LEFT_PATH: {
      BaseVelocity target_vel = {0, ROBOT_MAX_X_VELOCITY * 0.25, 0};
      movement_control(target_vel);
      HAL_Delay(LEFT_PATH_TIME_1);
      target_vel.x_vel = -ROBOT_MAX_Y_VELOCITY * 0.25;
      target_vel.y_vel = 0;
      target_vel.z_vel = 0;
      movement_control(target_vel);
      HAL_Delay(LEFT_PATH_TIME_2);
      break;
    }
    case MID_PATH: {
      BaseVelocity target_vel = {0, ROBOT_MAX_X_VELOCITY * 0.3, 0};
      movement_control(target_vel);
      HAL_Delay(MID_PATH_TIME_1);
      break;
    }
    case RIGHT_PATH: {
      BaseVelocity target_vel = {0, ROBOT_MAX_X_VELOCITY * 0.25, 0};
      movement_control(target_vel);
      HAL_Delay(RIGHT_PATH_TIME_1);
      target_vel.x_vel = ROBOT_MAX_Y_VELOCITY * 0.25;
      target_vel.y_vel = 0;
      target_vel.z_vel = 0;
      movement_control(target_vel);
      HAL_Delay(RIGHT_PATH_TIME_2);
      break;
    }
    default: {
      BaseVelocity target_vel = {0, 0, 0};
      movement_control(target_vel);
      break;
    }
  }
  BaseVelocity target_vel = {0, 0, 0};
  movement_control(target_vel);
}