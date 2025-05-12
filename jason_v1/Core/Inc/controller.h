#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    bool r1;
    bool r2;
    bool r3;
    bool l1;
    bool l2;
    bool l3;
    bool cross;
    bool circle;
    bool triangle;
    bool square;
    bool up;
    bool down;
    bool left;
    bool right;
    int8_t l_stick_x;
    int8_t l_stick_y;
    int8_t r_stick_x;
    int8_t r_stick_y;
    uint16_t l2_pressure;
    uint16_t r2_pressure;
    bool ps_button;
    bool share_button;
    bool options_button;
} ControllerState;

/**
 * @brief Parses the input string to extract controller state data.
 *
 * This function takes an input string and parses it to populate the
 * provided ControllerState structure with relevant data. It returns
 * a status code indicating the success or failure of the operation.
 *
 * @param input A pointer to the input string containing the controller data.
 * @param data A pointer to the ControllerState structure to be populated.
 * @return uint8_t Returns 0 on success, or an error code on failure.
 */
uint8_t parse_controller_data(const char* input, ControllerState* data);

extern char controller_buffer[41];
extern ControllerState controller_state;

#endif  // __CONTROLLER_H__