#include "servo.h"

HTD45H_Servo servos[6] = {
  {0x01, INITIAL_POS},  // Front Left
  {0x02, INITIAL_POS},  // Front Right
  {0x03, INITIAL_POS},  // Rear Left
  {0x04, INITIAL_POS},  // Rear Right
  {0x05, INITIAL_POS},  // Catch Ball
  {0x06, INITIAL_POS}   // Unload Ball
};

// TODO: need to test
void servo_update_current_pos(HTD45H_Servo* target_servo) {
  uint8_t send_buffer[6], receive_buffer[8];
  send_buffer[0] = send_buffer[1] = FRAME_HEADER;
  send_buffer[2] = 0x01 + 3;
  send_buffer[3] = CMD_MULT_SERVO_POS_READ;
  send_buffer[4] = 0x01;
  send_buffer[5] = target_servo->servo_id;
  HAL_UART_Transmit(&huart5, send_buffer, sizeof(send_buffer), 0xFFFF);
  HAL_UART_Receive(&huart5, receive_buffer, sizeof(receive_buffer), 0xFFFF);
  target_servo->current_pos = (receive_buffer[8] << 8) + receive_buffer[7];
}

void servo_move(HTD45H_Servo* target_servo, uint16_t target_pos, uint16_t time) {
  uint8_t send_buffer[10];
  send_buffer[0] = send_buffer[1] = FRAME_HEADER;
  send_buffer[2] = 0x01 * 3 + 5;
  send_buffer[3] = CMD_SERVO_MOVE;
  send_buffer[4] = 0x01;
  send_buffer[5] = GET_LOW_BYTE(time);
  send_buffer[6] = GET_HIGH_BYTE(time);
  send_buffer[7] = target_servo->servo_id;
  send_buffer[8] = GET_LOW_BYTE(target_pos);
  send_buffer[9] = GET_HIGH_BYTE(target_pos);
  HAL_UART_Transmit(&huart5, send_buffer, sizeof(send_buffer), 0xFFFF);
  target_servo->current_pos = target_pos;
}

// TODO: need to test
void servo_unload(HTD45H_Servo* target_servo) {
  uint8_t send_buffer[6];
  send_buffer[0] = send_buffer[1] = FRAME_HEADER;
  send_buffer[2] = 0x01 + 3;
  send_buffer[3] = CMD_MULT_SERVO_UNLOAD;
  send_buffer[4] = 0x01;
  send_buffer[5] = target_servo->servo_id;
  HAL_UART_Transmit(&huart5, send_buffer, sizeof(send_buffer), 0xFFFF);
}

// TODO: need to test
uint16_t servo_get_current_pos(HTD45H_Servo* target_servo) {
  servo_update_current_pos(target_servo);
  return target_servo->current_pos;
}

void servo_reset_all(void) {
  for (int i = 0; i < 6; i++)
    servo_move(&(servos[i]), INITIAL_POS, 1000);
}