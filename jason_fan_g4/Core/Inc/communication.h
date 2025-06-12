#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "main.h"

extern UART_HandleTypeDef huart4;

typedef struct {
  uint16_t servo_id1_ccr;
  uint16_t servo_id2_ccr;
  uint16_t servo_id3_ccr;
  uint16_t servo_id4_ccr;
} CommunicationState;

void communication_send_data(CommunicationState* data);

void communication_receive_data(CommunicationState* data);

extern char communication_buffer[25];
extern CommunicationState communication_state;

#endif