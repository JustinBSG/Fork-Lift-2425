#include "communication.h"

char communication_buffer[24] = "";

CommunicationState communication_state = {
  .servo_id1_ccr = 0,
  .servo_id2_ccr = 0,
  .servo_id3_ccr = 0,
  .servo_id4_ccr = 0};

void communication_send_data(CommunicationState* data) {
  if (data == NULL) 
    return;
  
  snprintf(communication_buffer, sizeof(communication_buffer),
           "%05u,%05u,%05u,%05u", 
           data->servo_id1_ccr,
           data->servo_id2_ccr,
           data->servo_id3_ccr,
           data->servo_id4_ccr);

  HAL_UART_Transmit(&huart4, (uint8_t*)communication_buffer, strlen(communication_buffer), HAL_MAX_DELAY);
}

void communication_receive_data(CommunicationState* data) {
  if (data == NULL) 
    return;

  // Parse the input string and store the values in the CommunicationState structure
  int parsed_values = sscanf(communication_buffer, "%5hu,%5hu,%5hu,%5hu", 
                             &data->servo_id1_ccr, 
                             &data->servo_id2_ccr, 
                             &data->servo_id3_ccr, 
                             &data->servo_id4_ccr);

  // Ensure all 4 values are successfully parsed
  if (parsed_values != 4) {
    // Handle parsing error (e.g., reset data to default values)
    data->servo_id1_ccr = 0;
    data->servo_id2_ccr = 0;
    data->servo_id3_ccr = 0;
    data->servo_id4_ccr = 0;
  }
}