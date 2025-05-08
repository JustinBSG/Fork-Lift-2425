#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <stdbool.h>

// TODO: Change these values to the actual values of the robot, in meters
#define LENGTH_CENTER_WHEEL_X 0.097   // distance from the robot's center to the wheels on the x-axis
#define LENGTH_CENTER_WHEEL_Y 0.090  // distance from the robot's center to the wheels on the y-axis
#define RADIUS_WHEEL 0.0375         // radius of the mecanum wheel

#define ROBOT_MAX_X_VELOCITY 7.4625 * 0.5 // max linear velocity of the robot, in m/s
#define ROBOT_MAX_Y_VELOCITY 7.4625 * 0.5 // max linear velocity of the robot, in m/s
#define ROBOT_MAX_Z_VELOCITY 72.1100 * 0.3 // max angular velocity of the robot, in rad/s

#define PID_MODE 0

extern bool turn_on;

#endif /* __ROBOT_H__ */