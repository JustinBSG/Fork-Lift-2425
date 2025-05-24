#ifndef __SERVO_H__
#define __SERVO_H__

#include <math.h>
#include <stdint.h>
#include <usart.h>

typedef struct {
    uint8_t servo_id;
    uint16_t current_pos;
    int16_t current_degree;
} HTD45H_Servo;

extern HTD45H_Servo servos[4];

extern UART_HandleTypeDef huart4;

#define GET_LOW_BYTE(x) ((uint8_t)((x) & 0x00FF))
#define GET_HIGH_BYTE(x) ((uint8_t)(((x) >> 8) & 0x00FF))

#define FRAME_HEADER 0x55
#define CMD_SERVO_MOVE 0x03
#define CMD_MULT_SERVO_UNLOAD 0x20
#define CMD_MULT_SERVO_POS_READ 0x21

#define INITIAL_POS 500
#define SHORTEST_TIME_ROTATE(id, degree) (180 / 60.0 * abs((servos[id - 1].current_degree + 90) - (degree + 90)))

#define SERVO_ID1_MAX_POS 500 + 385
#define SERVO_ID1_MIN_POS 500 - 366
#define SERVO_ID2_MAX_POS 500 + 378
#define SERVO_ID2_MIN_POS 500 - 375
#define SERVO_ID3_MAX_POS 500 + 375
#define SERVO_ID3_MIN_POS 500 - 370
#define SERVO_ID4_MAX_POS 500 + 376
#define SERVO_ID4_MIN_POS 500 - 373

#define SERVO_ID1_ANGLE_TO_POS(angle)                                 \
  ((angle) > 0                                                        \
     ? INITIAL_POS + angle / 90.0 * (SERVO_ID1_MAX_POS - INITIAL_POS) \
     : INITIAL_POS - angle / 90.0 * (INITIAL_POS - SERVO_ID1_MIN_POS))
#define SERVO_ID2_ANGLE_TO_POS(angle)                                 \
  ((angle) > 0                                                        \
     ? INITIAL_POS + angle / 90.0 * (SERVO_ID2_MAX_POS - INITIAL_POS) \
     : INITIAL_POS - angle / 90.0 * (INITIAL_POS - SERVO_ID2_MIN_POS))
#define SERVO_ID3_ANGLE_TO_POS(angle)                                 \
  ((angle) > 0                                                        \
     ? INITIAL_POS + angle / 90.0 * (SERVO_ID3_MAX_POS - INITIAL_POS) \
     : INITIAL_POS - angle / 90.0 * (INITIAL_POS - SERVO_ID3_MIN_POS))
#define SERVO_ID4_ANGLE_TO_POS(angle)                                 \
  ((angle) > 0                                                        \
     ? INITIAL_POS + angle / 90.0 * (SERVO_ID4_MAX_POS - INITIAL_POS) \
     : INITIAL_POS - angle / 90.0 * (INITIAL_POS - SERVO_ID4_MIN_POS))

#define SERVO_ID1_POS_TO_ANGLE(pos)                                     \
  ((pos) > INITIAL_POS                                                  \
     ? ((pos) - INITIAL_POS) * 90.0 / (SERVO_ID1_MAX_POS - INITIAL_POS) \
     : (INITIAL_POS - (pos)) * -90.0 / (INITIAL_POS - SERVO_ID1_MIN_POS))
#define SERVO_ID2_POS_TO_ANGLE(pos)                                     \
  ((pos) > INITIAL_POS                                                  \
     ? ((pos) - INITIAL_POS) * 90.0 / (SERVO_ID2_MAX_POS - INITIAL_POS) \
     : (INITIAL_POS - (pos)) * -90.0 / (INITIAL_POS - SERVO_ID2_MIN_POS))
#define SERVO_ID3_POS_TO_ANGLE(pos)                                     \
  ((pos) > INITIAL_POS                                                  \
     ? ((pos) - INITIAL_POS) * 90.0 / (SERVO_ID3_MAX_POS - INITIAL_POS) \
     : (INITIAL_POS - (pos)) * -90.0 / (INITIAL_POS - SERVO_ID3_MIN_POS))
#define SERVO_ID4_POS_TO_ANGLE(pos)                                     \
  ((pos) > INITIAL_POS                                                  \
     ? ((pos) - INITIAL_POS) * 90.0 / (SERVO_ID4_MAX_POS - INITIAL_POS) \
     : (INITIAL_POS - (pos)) * -90.0 / (INITIAL_POS - SERVO_ID4_MIN_POS))

void servo_update_current_pos(HTD45H_Servo* target_servo);
void servo_move(HTD45H_Servo* target_servo, uint16_t target_pos, uint16_t time);
void servo_unload(HTD45H_Servo* target_servo);

uint16_t servo_get_current_pos(HTD45H_Servo* target_servo);
void servo_reset_all(void);

#endif  // __SERVO_H__