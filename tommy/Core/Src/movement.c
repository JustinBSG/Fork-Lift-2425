#include "movement.h"

#include "pid-mecanum.h"

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
  int front_left = (int)(wheel_vel.front_left * 60.0 / (2.0 * M_PI) / (MOTOR_MAX_VELOCITY * 60.0 / (2.0 * M_PI)) * 100.0 * FL_MOTOR_ARR / 100.0);
  int front_right = (int)(wheel_vel.front_right * 60.0 / (2.0 * M_PI) / (MOTOR_MAX_VELOCITY * 60.0 / (2.0 * M_PI)) * 100.0 * FR_MOTOR_ARR / 100.0);
  int rear_left = (int)(wheel_vel.rear_left * 60.0 / (2.0 * M_PI) / (MOTOR_MAX_VELOCITY * 60.0 / (2.0 * M_PI)) * 100.0 * RL_MOTOR_ARR / 100.0);
  int rear_right = (int)(wheel_vel.rear_right * 60.0 / (2.0 * M_PI) / (MOTOR_MAX_VELOCITY * 60.0 / (2.0 * M_PI)) * 100.0 * RR_MOTOR_ARR / 100.0);
  return (WheelPWM){front_left, front_right, rear_left, rear_right};
}

void wheel_control(MecanumWheel wheel, int speed) {
  if (speed > 16800)
    speed = 16800;
  else if (speed < -16800)
    speed = -16800;

  switch (wheel) {
    case FRONT_LEFT:
      if (speed > 0) {
        // FL_MOTOR_A_CCR = 0;
        // FL_MOTOR_B_CCR = speed;
        HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_RESET);
      } else if (speed < 0) {
        // FL_MOTOR_A_CCR = -speed;
        // FL_MOTOR_B_CCR = 0;
        HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_SET);
      } else {
        // FL_MOTOR_A_CCR = 0;
        // FL_MOTOR_B_CCR = 0;
        HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_RESET);
      }
      FL_MOTOR_CCR = abs(speed);
      break;
    case FRONT_RIGHT:
      if (speed < 0) {
        // FR_MOTOR_B_CCR = speed;
        // FR_MOTOR_A_CCR = 0;
        HAL_GPIO_WritePin(MOTOR_FR_IN1_GPIO_Port, MOTOR_FR_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_FR_IN2_GPIO_Port, MOTOR_FR_IN2_Pin, GPIO_PIN_RESET);
      } else if (speed > 0) {
        // FR_MOTOR_B_CCR = 0;
        // FR_MOTOR_A_CCR = -speed;
        HAL_GPIO_WritePin(MOTOR_FR_IN1_GPIO_Port, MOTOR_FR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_FR_IN2_GPIO_Port, MOTOR_FR_IN2_Pin, GPIO_PIN_SET);
      } else {
        // FR_MOTOR_A_CCR = 0;
        // FR_MOTOR_B_CCR = 0;
        HAL_GPIO_WritePin(MOTOR_FR_IN1_GPIO_Port, MOTOR_FR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_FR_IN2_GPIO_Port, MOTOR_FR_IN2_Pin, GPIO_PIN_RESET);
      }
      FR_MOTOR_CCR = abs(speed);
      break;
    case REAR_LEFT:
      if (speed > 0) {
        // RL_MOTOR_B_CCR = 0;
        // RL_MOTOR_A_CCR = speed;
        HAL_GPIO_WritePin(MOTOR_RL_IN1_GPIO_Port, MOTOR_RL_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_RL_IN2_GPIO_Port, MOTOR_RL_IN2_Pin, GPIO_PIN_RESET);
      } else if (speed < 0) {
        // RL_MOTOR_B_CCR = -speed;
        // RL_MOTOR_A_CCR = 0;
        HAL_GPIO_WritePin(MOTOR_RL_IN1_GPIO_Port, MOTOR_RL_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_RL_IN2_GPIO_Port, MOTOR_RL_IN2_Pin, GPIO_PIN_SET);
      } else {
        // RL_MOTOR_A_CCR = 0;
        // RL_MOTOR_B_CCR = 0;
        HAL_GPIO_WritePin(MOTOR_RL_IN1_GPIO_Port, MOTOR_RL_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_RL_IN2_GPIO_Port, MOTOR_RL_IN2_Pin, GPIO_PIN_RESET);
      }
      RL_MOTOR_CCR = abs(speed);
      break;
    case REAR_RIGHT:
      if (speed < 0) {
        // RR_MOTOR_A_CCR = speed;
        // RR_MOTOR_B_CCR = 0;
        HAL_GPIO_WritePin(MOTOR_RR_IN1_GPIO_Port, MOTOR_RR_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_RR_IN2_GPIO_Port, MOTOR_RR_IN2_Pin, GPIO_PIN_RESET);
      } else if (speed > 0) {
        // RR_MOTOR_A_CCR = 0;
        // RR_MOTOR_B_CCR = -speed;
        HAL_GPIO_WritePin(MOTOR_RR_IN1_GPIO_Port, MOTOR_RR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_RR_IN2_GPIO_Port, MOTOR_RR_IN2_Pin, GPIO_PIN_SET);
      } else {
        // RR_MOTOR_A_CCR = 0;
        // RR_MOTOR_B_CCR = 0;
        HAL_GPIO_WritePin(MOTOR_RR_IN1_GPIO_Port, MOTOR_RR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_RR_IN2_GPIO_Port, MOTOR_RR_IN2_Pin, GPIO_PIN_RESET);
      }
      RR_MOTOR_CCR = abs(speed);
      break;
    default:
      if (speed > 0) {
        // FL_MOTOR_A_CCR = 0;
        // FL_MOTOR_B_CCR = speed;
        HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_RESET);
      } else if (speed < 0) {
        // FL_MOTOR_A_CCR = -speed;
        // FL_MOTOR_B_CCR = 0;
        HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_SET);
      } else {
        // FL_MOTOR_A_CCR = 0;
        // FL_MOTOR_B_CCR = 0;
        HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_RESET);
      }
      FL_MOTOR_CCR = abs(speed);
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
#if (PID_MODE == 1)
  WheelVelocity current_vel = read_current_velocity(encoders);
  WheelVelocity result_vel = pid_system(target_vel, current_vel);
  WheelPWM target_pwm = wheel2pwm(result_vel);
#else
  WheelPWM target_pwm = wheel2pwm(target_vel);
#endif
  wheels_control(target_pwm);
}

// TODO: need to test
void movement_rotation(int degree) {
  // TODO: rotate the robot by degree
  // if degree is positive, rotate clockwise
  // if degree is negative, rotate counterclockwise
  // if degree is 0, do nothing
  // call movement_control with the appropriate base velocity
  // check current angle of the robot
  // if current angle is close to the target angle, stop the robot by calling movement_control with zero base velocity
  if (degree == 0) 
    return;
  // hmc5883l_read_data(&hmc5883l_data);
  // float current_angle = hmc5883l_cal_xy_angle(&hmc5883l_data, &hmc5883l_cali_data);
  // float start_angle = current_angle;
  // float target_angle = current_angle + degree;
  // int count = 0;
  // while (1) {
  //   hmc5883l_read_data(&hmc5883l_data);
  //   current_angle = hmc5883l_cal_xy_angle(&hmc5883l_data, &hmc5883l_cali_data);

  // }
}