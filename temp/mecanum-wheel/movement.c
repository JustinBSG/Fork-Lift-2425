#include "movement.h"

#include "encoder.h"
#include "pid-mecanum.h"

// void cal_wheel_velocity(const float x_vel, const float y_vel, const float z_vel, float* wheel_vel) {
//   wheel_vel[FRONT_LEFT] = (x_vel - y_vel - (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * z_vel) / RADIUS_WHEEL;
//   wheel_vel[FRONT_RIGHT] = (x_vel + y_vel + (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * z_vel) / RADIUS_WHEEL;
//   wheel_vel[REAR_LEFT] = (x_vel + y_vel - (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * z_vel) / RADIUS_WHEEL;
//   wheel_vel[REAR_RIGHT] = (x_vel - y_vel + (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * z_vel) / RADIUS_WHEEL;
// }

WheelVelocity base2wheel(BaseVelocity base_vel) {
  float front_left = (base_vel.x_vel - base_vel.y_vel - (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * base_vel.z_vel) / RADIUS_WHEEL;
  float front_right = (base_vel.x_vel + base_vel.y_vel + (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * base_vel.z_vel) / RADIUS_WHEEL;
  float rear_left = (base_vel.x_vel + base_vel.y_vel - (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * base_vel.z_vel) / RADIUS_WHEEL;
  float rear_right = (base_vel.x_vel - base_vel.y_vel + (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * base_vel.z_vel) / RADIUS_WHEEL;
  return (WheelVelocity){front_left, front_right, rear_left, rear_right};
}

BaseVelocity wheel2base(WheelVelocity wheel_vel) {
  float x_vel = (wheel_vel.front_left + wheel_vel.front_right + wheel_vel.rear_left + wheel_vel.rear_right) * RADIUS_WHEEL / 4.0;
  float y_vel = (-wheel_vel.front_left + wheel_vel.front_right + wheel_vel.rear_left - wheel_vel.rear_right) * RADIUS_WHEEL / 4.0;
  float z_vel = (-wheel_vel.front_left + wheel_vel.front_right - wheel_vel.rear_left + wheel_vel.rear_right) * RADIUS_WHEEL / (4.0 * (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y));
  return (BaseVelocity){x_vel, y_vel, z_vel};
}

WheelPWM wheel2pwm(WheelVelocity wheel_vel) {
  // TODO: need to check timer settings for pwm generation
  // rad/s *2 * pi * 60 = rpm
  // rpm / max rpm * 100 = duty cycle
  // duty cycle * ARR = CCR = pwm signal
}

void wheel_control(MecanumWheel wheel, int speed) {
  // TODO: set pwm signal to the dc motor
  switch (wheel) {
    case FRONT_LEFT:
      // set pwm signal to the dc motor
      break;
    case FRONT_RIGHT:
      // set pwm signal to the dc motor
      break;
    case REAR_LEFT:
      // set pwm signal to the dc motor
      break;
    case REAR_RIGHT:
      // set pwm signal to the dc motor
      break;
    default:
      break;
  }
}

void wheels_control(WheelPWM pwm) {
  wheel_control(FRONT_LEFT, pwm.front_left);
  wheel_control(FRONT_RIGHT, pwm.front_right);
  wheel_control(REAR_LEFT, pwm.rear_left);
  wheel_control(REAR_RIGHT, pwm.rear_right);
}

void movement_control(BaseVelocity base_vel) {
  WheelVelocity target_vel = base2wheel(base_vel);
  WheelVelocity current_vel = read_current_velocity(encoders);
  WheelVelocity result_vel = pid_system(target_vel, current_vel);
  WheelPWM target_pwm = wheel2pwm(result_vel);
  wheels_control(target_pwm);
}