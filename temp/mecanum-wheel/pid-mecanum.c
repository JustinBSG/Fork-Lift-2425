#include "pid-mecanum.h"

float calculate_pid(MecanumWheel wheel, float target_vel, float current_vel) {
  switch (wheel) {
    default:
      // TODO: pid algorithm
      break;
  }
}

WheelVelocity pid_system(WheelVelocity target_vel, WheelVelocity current_vel) {
  WheelVelocity result_vel = {.front_left = 0, .front_right = 0, .rear_left = 0, .rear_right = 0};
  result_vel.front_left = calculate_pid(FRONT_LEFT, target_vel.front_left, current_vel.front_left);
  result_vel.front_right = calculate_pid(FRONT_RIGHT, target_vel.front_right, current_vel.front_right);
  result_vel.rear_left = calculate_pid(REAR_LEFT, target_vel.rear_left, current_vel.rear_left);
  result_vel.rear_right = calculate_pid(REAR_RIGHT, target_vel.rear_right, current_vel.rear_right);

  return result_vel;
}