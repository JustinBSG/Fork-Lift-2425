#include "servo.h"

HTD45H_Servo servos[4] = {
  {0x01, SERVO_ID1_INITIAL_POS, 0},  // Front Left
  {0x02, SERVO_ID2_INITIAL_POS, 0},  // Front Right
  {0x03, SERVO_ID3_INITIAL_POS, 0},  // Rear Left
  {0x04, SERVO_ID4_INITIAL_POS, 0},  // Rear Right
};

void servo_move(HTD45H_Servo* target_servo, uint16_t target_pos) {
  switch (target_servo->servo_id) {
    case 1:
      FL_SERVO_CCR = target_pos;

      if (target_pos == SERVO_ID1_MAX_POS)
        target_servo->current_degree = 90;
      else if (target_pos == SERVO_ID1_MIN_POS)
        target_servo->current_degree = -90;
      else 
        target_servo->current_degree = 0;
        // target_servo->current_degree = SERVO_ID1_POS_TO_ANGLE(target_pos);
      break;
    case 2:
      FR_SERVO_CCR = target_pos;

      if (target_pos == SERVO_ID2_MAX_POS)
        target_servo->current_degree = 90;
      else if (target_pos == SERVO_ID2_MIN_POS)
        target_servo->current_degree = -90;
      else 
        target_servo->current_degree = 0;
        // target_servo->current_degree = SERVO_ID2_POS_TO_ANGLE(target_pos);
      break;
    case 3:
      RL_SERVO_CCR = target_pos;

      if (target_pos == SERVO_ID3_MAX_POS)
        target_servo->current_degree = 90;
      else if (target_pos == SERVO_ID3_MIN_POS)
        target_servo->current_degree = -90;
      else
        target_servo->current_degree = 0;
        // target_servo->current_degree = SERVO_ID3_POS_TO_ANGLE(target_pos);
      break;
    case 4:
      RR_SERVO_CCR = target_pos;

      if (target_pos == SERVO_ID4_MAX_POS)
        target_servo->current_degree = 90;
      else if (target_pos == SERVO_ID4_MIN_POS)
        target_servo->current_degree = -90;
      else  
        target_servo->current_degree = 0;
        // target_servo->current_degree = SERVO_ID4_POS_TO_ANGLE(target_pos);
      break;
    default:
      break;
  }
}

void servo_reset_all(void) {
  servo_move(&(servo[0]), SERVO_ID1_INITIAL_POS);
  servo_move(&(servo[1]), SERVO_ID2_INITIAL_POS);
  servo_move(&(servo[2]), SERVO_ID3_INITIAL_POS);
  servo_move(&(servo[3]), SERVO_ID4_INITIAL_POS);
}