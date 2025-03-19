#ifndef __PID_MECANUM_H__
#define __PID_MECANUM_H__

#include "movement.h"

/**
 * @brief Calculate PID control for a single mecanum wheel
 *
 * Note: MecanumWheel wheel is for sperating tuning for each wheel, not necessary
 *
 * @param wheel mecanum wheel to calculate PID for
 * @param target_vel target angular velocity for the wheel, in rad/s
 * @param current_vel current angular velocity for the wheel, in rad/s
 * @return float angular velocity for the wheel, in rad/s
 */
float calculate_pid(MecanumWheel wheel, float target_vel, float current_vel);

/**
 * @brief Perform PID control for the mecanum wheel system
 *
 * @param target_vel target angular velocities for all wheels, in rad/s
 * @param current_vel velocities read by encoders
 * @return WheelVelocity angular velocities for all wheels, in rad/s
 */
WheelVelocity pid_system(WheelVelocity target_vel, WheelVelocity current_vel);

#endif  // __PID_MECANUM_H__