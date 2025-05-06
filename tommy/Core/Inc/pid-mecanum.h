#ifndef __PID_MECANUM_H__
#define __PID_MECANUM_H__

#include <stdint.h>

#include "movement.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float sum_error;
    float previous_error;
    uint32_t last_time;  // in ms
} PID_Data;

/**
 * @brief Calculate the PID control output for a single mecanum wheel.
 *
 * This function computes the PID control output based on the target angular velocity,
 * current angular velocity, and the PID parameters associated with the specified mecanum wheel.
 *
 * @param wheel The mecanum wheel.
 * @param target_vel The desired angular velocity for the wheel, in radians per second (rad/s).
 * @param current_vel The current angular velocity of the wheel, in radians per second (rad/s).
 * @return float The computed control output for the wheel, in radians per second (rad/s).
 */
void calculate_pid(MecanumWheel wheel , float target_vel, float current_vel, float *target_pid_value);

/**
 * @brief Perform PID control for the mecanum wheel system
 *
 * @param target_vel target angular velocities for all wheels, in rad/s
 * @param current_vel velocities read by encoders
 * @return WheelVelocity angular velocities for all wheels, in rad/s
 */
WheelVelocity pid_system(WheelVelocity target_vel, WheelVelocity current_vel);

extern PID_Data pid_data[4];

#endif /* __PID_MECANUM_H__ */
