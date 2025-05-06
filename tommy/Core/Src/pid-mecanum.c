#include "pid-mecanum.h"

PID_Data pid_data[4] = {
  {.kp = 100, .ki = 0, .kd = 10, .sum_error = 0, .previous_error = 0, .last_time = 0},
  {.kp = 100, .ki = 0, .kd = 10, .sum_error = 0, .previous_error = 0, .last_time = 0},
  {.kp = 100, .ki = 0, .kd = 10, .sum_error = 0, .previous_error = 0, .last_time = 0},
  {.kp = 100, .ki = 0, .kd = 10, .sum_error = 0, .previous_error = 0, .last_time = 0}};

void calculate_pid(MecanumWheel wheel, float target_vel, float current_vel, float *target_pid_value) {
  if (wheel < FRONT_LEFT || wheel > REAR_RIGHT)
    return 0;

  PID_Data* target_pid = &(pid_data[wheel]);

  uint32_t current_time = HAL_GetTick();
  uint32_t dt = current_time - target_pid->last_time;

  if (dt <= 1)
    return;

  float error = target_vel - current_vel;  // need to test if this is correct -/+
  pid_data->sum_error += error * dt;
  // target_pid->sum_error += error * (dt / 1000.0);

  // float pid_value = target_pid->kp * error + 
  //                   target_pid->ki * target_pid->sum_error + 
  //                   target_pid->kd * (error - target_pid->previous_error) * 1000.0 / dt;
  float pid_value = target_pid->kp * error + 
                    target_pid->ki * target_pid->sum_error + 
                    target_pid->kd * (error - target_pid->previous_error) / dt;

  target_pid->previous_error = error;
  target_pid->last_time = current_time;

  *target_pid_value = pid_value;
}

WheelVelocity pid_system(WheelVelocity target_vel, WheelVelocity current_vel) {
  static WheelVelocity result_vel = {.front_left = 0, .front_right = 0, .rear_left = 0, .rear_right = 0};
  calculate_pid(FRONT_LEFT, target_vel.front_left, current_vel.front_left, &(result_vel.front_left));
  calculate_pid(FRONT_RIGHT, target_vel.front_right, current_vel.front_right, &(result_vel.front_right));
  calculate_pid(REAR_LEFT, target_vel.rear_left, current_vel.rear_left, &(result_vel.rear_left));
  calculate_pid(REAR_RIGHT, target_vel.rear_right, current_vel.rear_right, &(result_vel.rear_right));

  return result_vel;
}