#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <stdbool.h>

#define LENGTH_CENTER_WHEEL_X 0.097  // distance from the robot's center to the wheels on the x-axis
#define LENGTH_CENTER_WHEEL_Y 0.090  // distance from the robot's center to the wheels on the y-axis
#define RADIUS_WHEEL 0.022           // radius of the mecanum wheel

#define ROBOT_MAX_X_VELOCITY 6.842 * 0.7  // max linear velocity of the robot, in m/s
#define ROBOT_MAX_Y_VELOCITY 6.842 * 0.7  // max linear velocity of the robot, in m/s
#define ROBOT_MAX_Z_VELOCITY 36.58 * 0.15    // max angular velocity of the robot, in rad/s

#define PID_MODE 0

extern bool turn_on;

#endif /* __ROBOT_H__ */