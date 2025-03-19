#include "movement.h"

void cal_wheel_velocity(const float x_vel, const float y_vel, const float z_vel, float* wheel_vel) {
  wheel_vel[FRONT_LEFT] = (x_vel - y_vel - (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * z_vel) / RADIUS_WHEEL;
  wheel_vel[FRONT_RIGHT] = (x_vel + y_vel + (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * z_vel) / RADIUS_WHEEL;
  wheel_vel[REAR_LEFT] = (x_vel + y_vel - (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * z_vel) / RADIUS_WHEEL;
  wheel_vel[REAR_RIGHT] = (x_vel - y_vel + (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * z_vel) / RADIUS_WHEEL;
}

void forward_kinematics(float* x_vel, float* y_vel, float* z_vel, const float* const wheel_vel) {
  *x_vel = (wheel_vel[FRONT_LEFT] + wheel_vel[FRONT_RIGHT] + wheel_vel[REAR_LEFT] + wheel_vel[REAR_RIGHT]) * RADIUS_WHEEL / 4.0;
  *y_vel = (-wheel_vel[FRONT_LEFT] + wheel_vel[FRONT_RIGHT] + wheel_vel[REAR_LEFT] - wheel_vel[REAR_RIGHT]) * RADIUS_WHEEL / 4.0;
  *z_vel = (-wheel_vel[FRONT_LEFT] + wheel_vel[FRONT_RIGHT] - wheel_vel[REAR_LEFT] + wheel_vel[REAR_RIGHT]) * RADIUS_WHEEL / (4.0 * (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y));
}

void wheel_control(float ang_vel, MacanumWheel wheel) {}