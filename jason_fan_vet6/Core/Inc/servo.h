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

#define FL_SERVO_TIMER TIM3
#define FR_SERVO_TIMER TIM3
#define RL_SERVO_TIMER TIM3
#define RR_SERVO_TIMER TIM9

#define FL_SERVO_ARR FL_SERVO_TIMER->ARR
#define FR_SERVO_ARR FR_SERVO_TIMER->ARR
#define RL_SERVO_ARR RL_SERVO_TIMER->ARR
#define RR_SERVO_ARR RR_SERVO_TIMER->ARR

#define FL_SERVO_CCR FL_SERVO_TIMER->CCR2
#define FR_SERVO_CCR FR_SERVO_TIMER->CCR4
#define RL_SERVO_CCR RL_SERVO_TIMER->CCR3
#define RR_SERVO_CCR RR_SERVO_TIMER->CCR1

// #define SHORTEST_TIME_ROTATE(id, degree) (180 / 60.0 * abs((servos[id - 1].current_degree + 90) - (degree + 90)))

#define SERVO_ID1_MAX_POS 400
#define SERVO_ID1_INITIAL_POS 400
#define SERVO_ID1_MIN_POS 400

#define SERVO_ID2_MAX_POS 400
#define SERVO_ID2_INITIAL_POS 400
#define SERVO_ID2_MIN_POS 400

#define SERVO_ID3_MAX_POS 400
#define SERVO_ID3_INITIAL_POS 400
#define SERVO_ID3_MIN_POS 400

#define SERVO_ID4_MAX_POS 400
#define SERVO_ID4_INITIAL_POS 400
#define SERVO_ID4_MIN_POS 400

// #define SERVO_ID1_ANGLE_TO_POS(angle)                                 \
//   ((angle) > 0                                                        \
//      ? INITIAL_POS + angle / 90.0 * (SERVO_ID1_MAX_POS - INITIAL_POS) \
//      : INITIAL_POS - angle / 90.0 * (INITIAL_POS - SERVO_ID1_MIN_POS))
// #define SERVO_ID2_ANGLE_TO_POS(angle)                                 \
//   ((angle) > 0                                                        \
//      ? INITIAL_POS + angle / 90.0 * (SERVO_ID2_MAX_POS - INITIAL_POS) \
//      : INITIAL_POS - angle / 90.0 * (INITIAL_POS - SERVO_ID2_MIN_POS))
// #define SERVO_ID3_ANGLE_TO_POS(angle)                                 \
//   ((angle) > 0                                                        \
//      ? INITIAL_POS + angle / 90.0 * (SERVO_ID3_MAX_POS - INITIAL_POS) \
//      : INITIAL_POS - angle / 90.0 * (INITIAL_POS - SERVO_ID3_MIN_POS))
// #define SERVO_ID4_ANGLE_TO_POS(angle)                                 \
//   ((angle) > 0                                                        \
//      ? INITIAL_POS + angle / 90.0 * (SERVO_ID4_MAX_POS - INITIAL_POS) \
//      : INITIAL_POS - angle / 90.0 * (INITIAL_POS - SERVO_ID4_MIN_POS))

// #define SERVO_ID1_POS_TO_ANGLE(pos)                                     \
//   ((pos) > INITIAL_POS                                                  \
//      ? ((pos) - INITIAL_POS) * 90.0 / (SERVO_ID1_MAX_POS - INITIAL_POS) \
//      : (INITIAL_POS - (pos)) * -90.0 / (INITIAL_POS - SERVO_ID1_MIN_POS))
// #define SERVO_ID2_POS_TO_ANGLE(pos)                                     \
//   ((pos) > INITIAL_POS                                                  \
//      ? ((pos) - INITIAL_POS) * 90.0 / (SERVO_ID2_MAX_POS - INITIAL_POS) \
//      : (INITIAL_POS - (pos)) * -90.0 / (INITIAL_POS - SERVO_ID2_MIN_POS))
// #define SERVO_ID3_POS_TO_ANGLE(pos)                                     \
//   ((pos) > INITIAL_POS                                                  \
//      ? ((pos) - INITIAL_POS) * 90.0 / (SERVO_ID3_MAX_POS - INITIAL_POS) \
//      : (INITIAL_POS - (pos)) * -90.0 / (INITIAL_POS - SERVO_ID3_MIN_POS))
// #define SERVO_ID4_POS_TO_ANGLE(pos)                                     \
//   ((pos) > INITIAL_POS                                                  \
//      ? ((pos) - INITIAL_POS) * 90.0 / (SERVO_ID4_MAX_POS - INITIAL_POS) \
//      : (INITIAL_POS - (pos)) * -90.0 / (INITIAL_POS - SERVO_ID4_MIN_POS))

void servo_move(HTD45H_Servo* target_servo, uint16_t target_pos);
void servo_reset_all(void);

#endif  // __SERVO_H__