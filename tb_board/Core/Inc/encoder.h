#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <math.h>
#include <stdint.h>

#include "main.h"
#include "movement.h"
#include "tim.h"

// pulses per rovolution
#define FL_ENCODER_PPR 1550 
#define FR_ENCODER_PPR 1550
#define RL_ENCODER_PPR 1550
#define RR_ENCODER_PPR 1550

typedef struct {
    float displacement;      // rad
    float velocity;          // rad/s
    int last_counter_value;  // number of pulses
    uint32_t last_tick;
    TIM_HandleTypeDef* htim;
    uint16_t _ppr;
} EncoderData;

/**
 * @brief Reset the encoder data
 *
 * @param encoder pointer to EncoderData
 */
void reset_encoder(EncoderData* encoder);

/**
 * @brief Updates the encoder data structure with the latest values.
 *
 * This function processes and updates the provided encoder data structure.
 * It is typically used to refresh the encoder's state during runtime.
 *
 * @param encoder Pointer to an EncoderData structure that will be updated.
 */
void update_encoder(EncoderData* encoder);

/**
 * @brief Reads the current angular velocity of the wheels using encoder data.
 *
 * This function calculates the angular velocity of all wheels in radians per second
 * based on the data provided by the encoders.
 *
 * @param encoders An array of 4 pointers to EncoderData structures, each representing
 *                 the data for a specific wheel's encoder.
 * @return WheelVelocity A structure containing the current angular velocity of all wheels
 *                       in radians per second.
 */
WheelVelocity read_current_velocity(EncoderData* encoders);

extern EncoderData encoders[4];

#endif /* __ENCODER_H__ */
