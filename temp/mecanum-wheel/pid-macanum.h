#ifndef __PID_MACANUM_H__
#define __PID_MACANUM_H__

void cal_pid_macanum(float target_x_vel, float target_y_vel, float target_z_vel, float* result_x_vel, float* result_y_vel, float* result_z_vel);

void reset_pid_mecanum(void);

#endif  // __PID_MACANUM_H__