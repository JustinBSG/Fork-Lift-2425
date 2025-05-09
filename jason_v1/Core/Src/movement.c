#include "movement.h"

// #include "pid-mecanum.h"

// TODO: need to update
WheelVelocity base2wheel(BaseVelocity base_vel) {
  // float front_left = (base_vel.x_vel - base_vel.y_vel - (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * base_vel.z_vel) / RADIUS_WHEEL;
  // float front_right = (base_vel.x_vel + base_vel.y_vel + (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * base_vel.z_vel) / RADIUS_WHEEL;
  // float rear_left = (base_vel.x_vel + base_vel.y_vel - (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * base_vel.z_vel) / RADIUS_WHEEL;
  // float rear_right = (base_vel.x_vel - base_vel.y_vel + (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y) * base_vel.z_vel) / RADIUS_WHEEL;
  // return (WheelVelocity){front_left, front_right, rear_left, rear_right};
}

// TODO: need to update
BaseVelocity wheel2base(WheelVelocity wheel_vel) {
  // float x_vel = (wheel_vel.front_left + wheel_vel.front_right + wheel_vel.rear_left + wheel_vel.rear_right) * RADIUS_WHEEL / 4.0;
  // float y_vel = (-wheel_vel.front_left + wheel_vel.front_right + wheel_vel.rear_left - wheel_vel.rear_right) * RADIUS_WHEEL / 4.0;
  // float z_vel = (-wheel_vel.front_left + wheel_vel.front_right - wheel_vel.rear_left + wheel_vel.rear_right) * RADIUS_WHEEL / (4.0 * (LENGTH_CENTER_WHEEL_X + LENGTH_CENTER_WHEEL_Y));
  // return (BaseVelocity){x_vel, y_vel, z_vel};
}

// TODO: need to update
WheelPWM wheel2pwm(WheelVelocity wheel_vel) {
  // int front_left = (int)(wheel_vel.front_left * 60.0 / (2.0 * M_PI) / (MOTOR_MAX_VELOCITY * 60.0 / (2.0 * M_PI)) * 100.0 * FL_MOTOR_ARR / 100.0);
  // int front_right = (int)(wheel_vel.front_right * 60.0 / (2.0 * M_PI) / (MOTOR_MAX_VELOCITY * 60.0 / (2.0 * M_PI)) * 100.0 * FR_MOTOR_ARR / 100.0);
  // int rear_left = (int)(wheel_vel.rear_left * 60.0 / (2.0 * M_PI) / (MOTOR_MAX_VELOCITY * 60.0 / (2.0 * M_PI)) * 100.0 * RL_MOTOR_ARR / 100.0);
  // int rear_right = (int)(wheel_vel.rear_right * 60.0 / (2.0 * M_PI) / (MOTOR_MAX_VELOCITY * 60.0 / (2.0 * M_PI)) * 100.0 * RR_MOTOR_ARR / 100.0);
  // return (WheelPWM){front_left, front_right, rear_left, rear_right};
}

// TODO: need to update
void wheel_control(Wheel wheel, int speed) {
  // if (speed > 16800)
  //   speed = 16800;
  // else if (speed < -16800)
  //   speed = -16800;

  // switch (wheel) {
  //   case FRONT_LEFT:
  //     if (speed > 0) {
  //       // FL_MOTOR_A_CCR = 0;
  //       // FL_MOTOR_B_CCR = speed;
  //       HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_SET);
  //       HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_RESET);
  //     } else if (speed < 0) {
  //       // FL_MOTOR_A_CCR = -speed;
  //       // FL_MOTOR_B_CCR = 0;
  //       HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_RESET);
  //       HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_SET);
  //     } else {
  //       // FL_MOTOR_A_CCR = 0;
  //       // FL_MOTOR_B_CCR = 0;
  //       HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_RESET);
  //       HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_RESET);
  //     }
  //     FL_MOTOR_CCR = abs(speed);
  //     break;
  //   case FRONT_RIGHT:
  //     if (speed < 0) {
  //       // FR_MOTOR_B_CCR = speed;
  //       // FR_MOTOR_A_CCR = 0;
  //       HAL_GPIO_WritePin(MOTOR_FR_IN1_GPIO_Port, MOTOR_FR_IN1_Pin, GPIO_PIN_SET);
  //       HAL_GPIO_WritePin(MOTOR_FR_IN2_GPIO_Port, MOTOR_FR_IN2_Pin, GPIO_PIN_RESET);
  //     } else if (speed > 0) {
  //       // FR_MOTOR_B_CCR = 0;
  //       // FR_MOTOR_A_CCR = -speed;
  //       HAL_GPIO_WritePin(MOTOR_FR_IN1_GPIO_Port, MOTOR_FR_IN1_Pin, GPIO_PIN_RESET);
  //       HAL_GPIO_WritePin(MOTOR_FR_IN2_GPIO_Port, MOTOR_FR_IN2_Pin, GPIO_PIN_SET);
  //     } else {
  //       // FR_MOTOR_A_CCR = 0;
  //       // FR_MOTOR_B_CCR = 0;
  //       HAL_GPIO_WritePin(MOTOR_FR_IN1_GPIO_Port, MOTOR_FR_IN1_Pin, GPIO_PIN_RESET);
  //       HAL_GPIO_WritePin(MOTOR_FR_IN2_GPIO_Port, MOTOR_FR_IN2_Pin, GPIO_PIN_RESET);
  //     }
  //     FR_MOTOR_CCR = abs(speed);
  //     break;
  //   case REAR_LEFT:
  //     if (speed > 0) {
  //       // RL_MOTOR_B_CCR = 0;
  //       // RL_MOTOR_A_CCR = speed;
  //       HAL_GPIO_WritePin(MOTOR_RL_IN1_GPIO_Port, MOTOR_RL_IN1_Pin, GPIO_PIN_SET);
  //       HAL_GPIO_WritePin(MOTOR_RL_IN2_GPIO_Port, MOTOR_RL_IN2_Pin, GPIO_PIN_RESET);
  //     } else if (speed < 0) {
  //       // RL_MOTOR_B_CCR = -speed;
  //       // RL_MOTOR_A_CCR = 0;
  //       HAL_GPIO_WritePin(MOTOR_RL_IN1_GPIO_Port, MOTOR_RL_IN1_Pin, GPIO_PIN_RESET);
  //       HAL_GPIO_WritePin(MOTOR_RL_IN2_GPIO_Port, MOTOR_RL_IN2_Pin, GPIO_PIN_SET);
  //     } else {
  //       // RL_MOTOR_A_CCR = 0;
  //       // RL_MOTOR_B_CCR = 0;
  //       HAL_GPIO_WritePin(MOTOR_RL_IN1_GPIO_Port, MOTOR_RL_IN1_Pin, GPIO_PIN_RESET);
  //       HAL_GPIO_WritePin(MOTOR_RL_IN2_GPIO_Port, MOTOR_RL_IN2_Pin, GPIO_PIN_RESET);
  //     }
  //     RL_MOTOR_CCR = abs(speed);
  //     break;
  //   case REAR_RIGHT:
  //     if (speed < 0) {
  //       // RR_MOTOR_A_CCR = speed;
  //       // RR_MOTOR_B_CCR = 0;
  //       HAL_GPIO_WritePin(MOTOR_RR_IN1_GPIO_Port, MOTOR_RR_IN1_Pin, GPIO_PIN_SET);
  //       HAL_GPIO_WritePin(MOTOR_RR_IN2_GPIO_Port, MOTOR_RR_IN2_Pin, GPIO_PIN_RESET);
  //     } else if (speed > 0) {
  //       // RR_MOTOR_A_CCR = 0;
  //       // RR_MOTOR_B_CCR = -speed;
  //       HAL_GPIO_WritePin(MOTOR_RR_IN1_GPIO_Port, MOTOR_RR_IN1_Pin, GPIO_PIN_RESET);
  //       HAL_GPIO_WritePin(MOTOR_RR_IN2_GPIO_Port, MOTOR_RR_IN2_Pin, GPIO_PIN_SET);
  //     } else {
  //       // RR_MOTOR_A_CCR = 0;
  //       // RR_MOTOR_B_CCR = 0;
  //       HAL_GPIO_WritePin(MOTOR_RR_IN1_GPIO_Port, MOTOR_RR_IN1_Pin, GPIO_PIN_RESET);
  //       HAL_GPIO_WritePin(MOTOR_RR_IN2_GPIO_Port, MOTOR_RR_IN2_Pin, GPIO_PIN_RESET);
  //     }
  //     RR_MOTOR_CCR = abs(speed);
  //     break;
  //   default:
  //     if (speed > 0) {
  //       // FL_MOTOR_A_CCR = 0;
  //       // FL_MOTOR_B_CCR = speed;
  //       HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_SET);
  //       HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_RESET);
  //     } else if (speed < 0) {
  //       // FL_MOTOR_A_CCR = -speed;
  //       // FL_MOTOR_B_CCR = 0;
  //       HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_RESET);
  //       HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_SET);
  //     } else {
  //       // FL_MOTOR_A_CCR = 0;
  //       // FL_MOTOR_B_CCR = 0;
  //       HAL_GPIO_WritePin(MOTOR_FL_IN1_GPIO_Port, MOTOR_FL_IN1_Pin, GPIO_PIN_RESET);
  //       HAL_GPIO_WritePin(MOTOR_FL_IN2_GPIO_Port, MOTOR_FL_IN2_Pin, GPIO_PIN_RESET);
  //     }
  //     FL_MOTOR_CCR = abs(speed);
  //     break;
  // }
}

void wheels_control(WheelPWM pwm) {
  wheel_control(FRONT_LEFT, pwm.front_left);
  wheel_control(FRONT_RIGHT, pwm.front_right);
  wheel_control(REAR_LEFT, pwm.rear_left);
  wheel_control(REAR_RIGHT, pwm.rear_right);
}

void rotate_motor(BaseVelocity base_vel) {
  if (base_vel.z_vel != 0) {
    servo_move(&(servos[0]), SERVO_ID1_ANGLE_TO_POS(45), SHORTEST_TIME_ROTATE(45));
    servo_move(&(servos[1]), SERVO_ID2_ANGLE_TO_POS(-45), SHORTEST_TIME_ROTATE(-45));
    servo_move(&(servos[3]), SERVO_ID4_ANGLE_TO_POS(45), SHORTEST_TIME_ROTATE(45));
    servo_move(&(servos[2]), SERVO_ID3_ANGLE_TO_POS(-45), SHORTEST_TIME_ROTATE(-45));
    return;
  }

  float angle = atan2(base_vel.y_vel, base_vel.x_vel) * 180 / M_PI;

  if (base_vel.x_vel == 0 && base_vel.y_vel != 0) {  // angle = 90 or 270
    servo_move(&(servos[0]), INITIAL_POS, SHORTEST_TIME_ROTATE(90));
    servo_move(&(servos[1]), INITIAL_POS, SHORTEST_TIME_ROTATE(90));
    servo_move(&(servos[2]), INITIAL_POS, SHORTEST_TIME_ROTATE(90));
    servo_move(&(servos[3]), INITIAL_POS, SHORTEST_TIME_ROTATE(90));
  } else if (base_vel.x_vel != 0 && base_vel.y_vel == 0) {  // angle = 0 or 180
    servo_move(&(servos[0]), SERVO_ID1_MAX_POS, SHORTEST_TIME_ROTATE(90));
    servo_move(&(servos[1]), SERVO_ID2_MAX_POS, SHORTEST_TIME_ROTATE(90));
    servo_move(&(servos[2]), SERVO_ID3_MAX_POS, SHORTEST_TIME_ROTATE(90));
    servo_move(&(servos[3]), SERVO_ID4_MAX_POS, SHORTEST_TIME_ROTATE(90));
  } else if (base_vel.x_vel < 0 && base_vel.y_vel > 0 || base_vel.x_vel > 0 && base_vel.y_vel < 0) {    // quadrant 2 or 4
    if (angle < 0)
      angle += 180;
    angle -= 90;
    angle *= -1;

    servo_move(&(servos[0]), SERVO_ID1_ANGLE_TO_POS(angle), SHORTEST_TIME_ROTATE(angle));
    servo_move(&(servos[1]), SERVO_ID2_ANGLE_TO_POS(angle), SHORTEST_TIME_ROTATE(angle));
    servo_move(&(servos[2]), SERVO_ID3_ANGLE_TO_POS(angle), SHORTEST_TIME_ROTATE(angle));
    servo_move(&(servos[3]), SERVO_ID4_ANGLE_TO_POS(angle), SHORTEST_TIME_ROTATE(angle));
  } else if (base_vel.x_vel < 0 && base_vel.y_vel < 0 || base_vel.x_vel > 0 && base_vel.y_vel > 0) {    // quadrant 1 or 3
    if (angle < 0)
      angle += 180;
    angle = 90 - angle;
    
    servo_move(&(servos[0]), SERVO_ID1_ANGLE_TO_POS(angle), SHORTEST_TIME_ROTATE(angle));
    servo_move(&(servos[1]), SERVO_ID2_ANGLE_TO_POS(angle), SHORTEST_TIME_ROTATE(angle));
    servo_move(&(servos[2]), SERVO_ID3_ANGLE_TO_POS(angle), SHORTEST_TIME_ROTATE(angle));
    servo_move(&(servos[3]), SERVO_ID4_ANGLE_TO_POS(angle), SHORTEST_TIME_ROTATE(angle));
  }
}

// TODO: need to update
void movement_control(BaseVelocity base_vel) {
  rotate_motor(base_vel);
  WheelVelocity target_vel = base2wheel(base_vel);
  WheelPWM target_pwm = wheel2pwm(target_vel);
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
}