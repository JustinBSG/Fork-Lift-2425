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
 * @brief Calculate PID control for a single mecanum wheel
 *
 * @param pid_data data of pid calculation
 * @param target_vel target angular velocity for the wheel, in rad/s
 * @param current_vel current angular velocity for the wheel, in rad/s
 * @return float angular velocity for the wheel, in rad/s
 */
float calculate_pid(PID_Data* pid_data, float target_vel, float current_vel);

/**
 * @brief Perform PID control for the mecanum wheel system
 *
 * @param target_vel target angular velocities for all wheels, in rad/s
 * @param current_vel velocities read by encoders
 * @return WheelVelocity angular velocities for all wheels, in rad/s
 */
WheelVelocity pid_system(WheelVelocity target_vel, WheelVelocity current_vel);

#endif  // __PID_MECANUM_H__