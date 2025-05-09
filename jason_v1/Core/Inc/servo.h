#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdint.h>
#include <usart.h>

extern UART_HandleTypeDef huart5;

#define GET_LOW_BYTE(x) ((uint8_t)((x) & 0x00FF))
#define GET_HIGH_BYTE(x) ((uint8_t)(((x) >> 8) & 0x00FF))

#define FRAME_HEADER 0x55
#define CMD_SERVO_MOVE 0x03
#define CMD_MULT_SERVO_UNLOAD 0x20
#define CMD_MULT_SERVO_POS_READ 0x21

#define INITIAL_POS 500
// TODO: need to find the shortest time to rotate based on current position
#define SHORTEST_TIME_ROTATE(degree) (180 / 60 * degree)
#define SHORTEST_TIME_ROTATE_POS(pos)
#define SHORTEST_TIME_ROTATE_DEGREE(degrees)

#define SERVO_ID1_MAX_POS 500 + 360
#define SERVO_ID1_MIN_POS 500 - 400
#define SERVO_ID2_MAX_POS 500 + 360
#define SERVO_ID2_MIN_POS 500 - 390
#define SERVO_ID3_MAX_POS 500 + 375
#define SERVO_ID3_MIN_POS 500 - 370
#define SERVO_ID4_MAX_POS 500 + 380
#define SERVO_ID4_MIN_POS 500 - 360
#define SERVO_ID5_MAX_POS 500
#define SERVO_ID5_MIN_POS 500
#define SERVO_ID6_MAX_POS 500
#define SERVO_ID6_MIN_POS 500

#define SERVO_ID1_ANGLE_TO_POS(angle)                                          \
  ((angle) > 0                                                                 \
       ? INITIAL_POS + angle / 90.0 * (SERVO_ID1_MAX_POS - INITIAL_POS)        \
       : INITIAL_POS - angle / 90.0 * (INITIAL_POS - SERVO_ID1_MIN_POS))
#define SERVO_ID2_ANGLE_TO_POS(angle)                                          \
  ((angle) > 0                                                                 \
       ? INITIAL_POS + angle / 90.0 * (SERVO_ID2_MAX_POS - INITIAL_POS)        \
       : INITIAL_POS - angle / 90.0 * (INITIAL_POS - SERVO_ID2_MIN_POS))
#define SERVO_ID3_ANGLE_TO_POS(angle)                                          \
  ((angle) > 0                                                                 \
       ? INITIAL_POS + angle / 90.0 * (SERVO_ID3_MAX_POS - INITIAL_POS)        \
       : INITIAL_POS - angle / 90.0 * (INITIAL_POS - SERVO_ID3_MIN_POS))
#define SERVO_ID4_ANGLE_TO_POS(angle)                                          \
  ((angle) > 0                                                                 \
       ? INITIAL_POS + angle / 90.0 * (SERVO_ID4_MAX_POS - INITIAL_POS)        \
       : INITIAL_POS - angle / 90.0 * (INITIAL_POS - SERVO_ID4_MIN_POS))

typedef struct {
    uint8_t servo_id;
    uint16_t current_pos;
} HTD45H_Servo;

void servo_update_current_pos(HTD45H_Servo* target_servo);
void servo_move(HTD45H_Servo* target_servo, uint16_t target_pos, uint16_t time);
void servo_unload(HTD45H_Servo* target_servo);

uint16_t servo_get_current_pos(HTD45H_Servo* target_servo);
void servo_reset_all(void);

extern HTD45H_Servo servos[6];

#endif  // __SERVO_H__