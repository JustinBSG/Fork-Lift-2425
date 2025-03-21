#include "pid-mecanum.h"

float calculate_pid(PID_Data* pid_data, float target_vel, float current_vel) {
  if (pid_data == NULL)
    return 0;

  uint32_t current_time = HAL_GetTick();
  uint32_t dt = (current_time - pid_data->last_time) / 1000;

  if (dt == 0)
    return 0;

  float error = target_vel - current_vel;  // need to test if this is correct -/+
  pid_data->sum_error += error * dt;

  float pid_value = pid_data->kp * error + pid_data->ki * pid_data->sum_error + pid_data->kd * (error - pid_data->previous_error) / dt;

  pid_data->previous_error = error;
  pid_data->last_time = current_time;

  return pid_value;
}

WheelVelocity pid_system(WheelVelocity target_vel, WheelVelocity current_vel) {
  WheelVelocity result_vel = {.front_left = 0, .front_right = 0, .rear_left = 0, .rear_right = 0};
  result_vel.front_left = calculate_pid(FRONT_LEFT, target_vel.front_left, current_vel.front_left);
  result_vel.front_right = calculate_pid(FRONT_RIGHT, target_vel.front_right, current_vel.front_right);
  result_vel.rear_left = calculate_pid(REAR_LEFT, target_vel.rear_left, current_vel.rear_left);
  result_vel.rear_right = calculate_pid(REAR_RIGHT, target_vel.rear_right, current_vel.rear_right);

  return result_vel;
}