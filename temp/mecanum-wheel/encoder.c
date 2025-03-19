#include "encoder.h"

void reset_encoder(EncoderData* encoder) {
  // TODO: HAL function to set counter value to 0
  // set counter value to 0
  encoder->displacement = 0;
  encoder->velocity = 0;
  encoder->last_counter_value = 0;
}

void update_encoder(EncoderData* encoder) {
  // TODO: update encoder
  // get current counter for calculating number of pulses
  // get duration for velocity calculation, read current tick - last tick
  // calculate number of pulses
    // current counter == last counter => number of pulses = 0
    // current counter > last counter
      // if timer is counting down, number of pulses = (ARR - current counter + last counter) * -1
      // else number of pulse = current counter - last counter
    // current counter < last counter
      // if timer is counting down, number of pulses = (last counter - current counter) * -1
      // else number of pulse = ARR - last counter + current counter
  // calculate displacement by checking encoder's datasheet
  // calculate velocity, displacement / (duration * 1000)
  // update last tick and last counter
}   

WheelVelocity read_current_velocity(EncoderData** encoders) {
  if (encoders == NULL)
    return (WheelVelocity){0, 0, 0, 0};

  WheelVelocity velocities = {.front_left = 0, .front_right = 0, .rear_left = 0, .rear_right = 0};
  for (int i = FRONT_LEFT; i <= REAR_RIGHT; i++)
    update_encoder(encoders[i]);

  velocities.front_left = encoders[FRONT_LEFT]->velocity;
  velocities.front_right = encoders[FRONT_RIGHT]->velocity;
  velocities.rear_left = encoders[REAR_LEFT]->velocity;
  velocities.rear_right = encoders[REAR_RIGHT]->velocity;

  return velocities;
}