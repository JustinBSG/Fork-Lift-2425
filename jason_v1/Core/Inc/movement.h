#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__

#include <math.h>
#include <stdlib.h>

#include "main.h"
#include "robot.h"
#include "servo.h"

#define FL_MOTOR_TIMER TIM3
#define FR_MOTOR_TIMER TIM3
#define RL_MOTOR_TIMER TIM3
#define RR_MOTOR_TIMER TIM3

#define FL_MOTOR_ARR FL_MOTOR_TIMER->ARR
#define FR_MOTOR_ARR FR_MOTOR_TIMER->ARR
#define RL_MOTOR_ARR RL_MOTOR_TIMER->ARR
#define RR_MOTOR_ARR RR_MOTOR_TIMER->ARR

#define FL_MOTOR_CCR FL_MOTOR_TIMER->CCR4
#define FR_MOTOR_CCR FR_MOTOR_TIMER->CCR1
#define RL_MOTOR_CCR RL_MOTOR_TIMER->CCR3
#define RR_MOTOR_CCR RR_MOTOR_TIMER->CCR2

#define MOTOR_MAX_CCR FL_MOTOR_ARR
#define MOTOR_MAX_VELOCITY 288.0  // max angular velocity of the motor, in rad/s

/**
 * (x,y,z): base velocity of robot
 * x: forward(+ve), backward(-ve)
 * y: right(+ve), left(-ve)
 * z: clockwise(+ve), anticlockwise(-ve)
 */

typedef enum {
  FRONT_LEFT,
  FRONT_RIGHT,
  REAR_LEFT,
  REAR_RIGHT,
  TEST
} Wheel;

typedef struct {
    float x_vel;  // base linear velocity in x direction, in m/s
    float y_vel;  // base linear velocity in y direction, in m/s
    float z_vel;  // base angular velocity around z-axis, in rad/s
} BaseVelocity;

typedef struct {
    float front_left;
    float front_right;
    float rear_left;
    float rear_right;
} WheelVelocity;  // angular velocity of 4 wheels, in rad/s

typedef struct {
    int front_left;
    int front_right;
    int rear_left;
    int rear_right;
} WheelPWM;  // CCR value for adjusting duty cycle of PWM signal

/**
 * @brief Calculate angular velocities for the four wheels of the robot
 *
 * @param base_vel base linear velocity in x, y direction in m/s and base angular velocity in z direction in rad/s
 * @return WheelVelocity angular velocities for all wheels, in rad/s
 */
WheelVelocity base2wheel(BaseVelocity base_vel);

/**
 * @brief Compute the robot's base velocity when given the individual wheel velocities
 *
 * @param wheel_vel angular velocities for all wheels, in rad/s
 * @return BaseVelocity base linear velocity in x, y direction in m/s and base angular velocity in z direction in rad/s
 */
BaseVelocity wheel2base(WheelVelocity wheel_vel);

/**
 * @brief Compute CCR value for adjusting duty cycle of PWM signal when given the angular velocities for all wheels
 *
 * @param wheel_vel angular velocities for all wheels, in rad/s
 * @return WheelPWM CCR value for adjusting duty cycle of PWM signal
 */
WheelPWM wheel2pwm(WheelVelocity wheel_vel);

/**
 * @brief control the speed of a wheel
 *
 * @param wheel indicate which wheel to control
 * @param speed CCR value for adjusting duty cycle of PWM signal
 */
void wheel_control(Wheel wheel, int speed);

/**
 * @brief control the speed of all wheels
 *
 * @param pwm CCR value for adjusting duty cycle of PWM signal
 */
void wheels_control(WheelPWM pwm);

void rotate_motor(BaseVelocity base_vel);

/**
 * @brief control the movement of the robot
 *
 * @param base_vel base linear velocity in x, y direction in m/s and base angular velocity in z direction in rad/s
 */
void movement_control(BaseVelocity base_vel);

/**
 * @brief Rotate the robot by a certain degree
 *
 * @param degree the degree to rotate, in radians
 */
void movement_rotation(int degree);

#include "encoder.h"

#endif /* __MOVEMENT_H__ */
