#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>

#include "main.h"
#include "movement.h"

// use counter to find number of pulse to find number of rotations
// use number of rotations to find distance and velocity

typedef struct {
    uint16_t displacement;        // rad
    uint16_t velocity;            // rad/s
    uint16_t last_counter_value;  // number of pulses
    uint32_t last_tick;
    TIM_HandleTypeDef* htim;
} EncoderData;

/**
 * @brief Reset the encoder data
 *
 * @param encoder pointer to EncoderData
 */
void reset_encoder(EncoderData* encoder);

/**
 * @brief Update the encoder data
 *
 * @param encoder pointer to EncoderData
 */
void update_encoder(EncoderData* encoder);

/**
 * @brief Read the current velocity of the wheel by using encoder
 *
 * @param encoders array of 4 pointers to EncoderData
 * @return WheelVelocity current angular velocity of all wheels, in rad/s
 */
WheelVelocity read_current_velocity(EncoderData** encoders);

#endif  // __ENCODER_H__