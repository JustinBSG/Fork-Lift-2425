#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__

#include "robot.h"

typedef enum {
  FRONT_LEFT,
  FRONT_RIGHT,
  REAR_LEFT,
  REAR_RIGHT
} MacanumWheel;

/**
 * @brief Calculate angular velocities for the four wheels of the robot
 *
 * @param x_vel base linear velocity in x direction, in m/s
 * @param y_vel base linear velocity in y direction, in m/s
 * @param z_vel base angular velocity around z-axis, in rad/s
 * @param wheel_vel angular velocities for the front left, front right, rear left and rear right wheel respectively, in rad/s
 */
void cal_wheel_velocity(const float x_vel, const float y_vel, const float z_vel, float* wheel_vel);

/**
 * @brief Compute the robot's base velocity when given the individual wheel velocities
 *
 * @param x_vel base linear velocity in x direction, in m/s
 * @param y_vel base linear velocity in y direction, in m/s
 * @param z_vel base angular velocity around z-axis, in rad/s
 * @param wheel_vel angular velocities for the front left, front right, rear left and rear right wheel respectively, in rad/s
 */
void forward_kinematics(float* x_vel, float* y_vel, float* z_vel, const float* const wheel_vel);

// angular velocity of the wheel (rad/s) => PWM signal (full rpm * pwm ratio) (rpm)
void wheel_control(float ang_vel, MacanumWheel wheel);

#endif  // __MOVEMENT_H__