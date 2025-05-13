#include "mech.h"

Direction_Encoder direction_encoder = FRONT_BACK;

void catch_move_down(void) {
  servo_move(&(servos[4]), SERVO_ID5_MAX_POS, SHORTEST_TIME_ROTATE(5, 90));
}

void catch_move_up(void) {
  servo_move(&(servos[4]), SERVO_ID5_MIN_POS, SHORTEST_TIME_ROTATE(5, 100));
}

void catch_reset(void) {
  servo_move(&(servos[4]), INITIAL_POS, 180);
}

void container_move_down(void) {
  if (servos[4].current_pos != SERVO_ID5_MIN_POS)
    servo_move(&(servos[5]), SERVO_ID6_MIN_POS, SHORTEST_TIME_ROTATE(6, 30));
}

void container_reset(void) {
  if (servos[4].current_pos != SERVO_ID5_MIN_POS)
    servo_move(&(servos[5]), INITIAL_POS, SHORTEST_TIME_ROTATE(6, 30));
}