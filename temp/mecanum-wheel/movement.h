#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__

#include "encoder.h"
#include "robot.h"

extern EncoderData* encoders[4];

typedef enum {
  FRONT_LEFT,
  FRONT_RIGHT,
  REAR_LEFT,
  REAR_RIGHT
} MecanumWheel;

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
void wheel_control(MecanumWheel wheel, int speed);

/**
 * @brief control the speed of all wheels
 *
 * @param pwm CCR value for adjusting duty cycle of PWM signal
 */
void wheels_control(WheelPWM pwm);

/**
 * @brief control the movement of the robot
 *
 * @param base_vel base linear velocity in x, y direction in m/s and base angular velocity in z direction in rad/s
 */
void movement_control(BaseVelocity base_vel);

void movement_rotation(int degree);

#endif  // __MOVEMENT_H__