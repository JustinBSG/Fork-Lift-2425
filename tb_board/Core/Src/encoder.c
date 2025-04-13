#include "encoder.h"

EncoderData encoders[4] = {{.displacement = 0.0, .velocity = 0.0, .last_counter_value = 0, .last_tick = 0, .htim = &htim2, ._ppr = 1550},
                           {.displacement = 0.0, .velocity = 0.0, .last_counter_value = 0, .last_tick = 0, .htim = &htim3, ._ppr = 1550},
                           {.displacement = 0.0, .velocity = 0.0, .last_counter_value = 0, .last_tick = 0, .htim = &htim4, ._ppr = 1550},
                           {.displacement = 0.0, .velocity = 0.0, .last_counter_value = 0, .last_tick = 0, .htim = &htim5, ._ppr = 1550}};

void reset_encoder(EncoderData* encoder) {
  __HAL_TIM_SET_COUNTER(encoder->htim, 0);
  encoder->displacement = 0;
  encoder->velocity = 0;
  encoder->last_counter_value = 0;
}

void update_encoder(EncoderData* encoder) {
  int current_counter = __HAL_TIM_GET_COUNTER(encoder->htim);
  uint32_t duration = HAL_GetTick() - encoder->last_tick;
  int num_pulse = 0;
  if (duration <= 1)  // delay for encode to update, in ms
    return;

  if (current_counter == encoder->last_counter_value) {
    num_pulse = 0;
  } else if (current_counter > encoder->last_counter_value) {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(encoder->htim)) {  // move backward, count down, overflow
      num_pulse = (__HAL_TIM_GET_AUTORELOAD(encoder->htim) - current_counter + encoder->last_counter_value) * -1;
    } else {  // move forward, count up, no overflow
      num_pulse = current_counter - encoder->last_counter_value;
    }
  } else {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(encoder->htim)) {  // move up, count up, overflow
      num_pulse = (encoder->last_counter_value - current_counter) * -1;
    } else {  // move backward, count down, no overflow
      num_pulse = __HAL_TIM_GET_AUTORELOAD(encoder->htim) - encoder->last_counter_value + current_counter;
    }
  }

  float temp_displacement = (float)num_pulse / encoder->_ppr * 2.0 * M_PI;
  encoder->last_counter_value = current_counter;
  encoder->displacement += temp_displacement;
  encoder->velocity = temp_displacement * 1000.0 / duration;
  encoder->last_tick = HAL_GetTick();
}

WheelVelocity read_current_velocity(EncoderData* encoders) {
  if (encoders == NULL)
    return (WheelVelocity){0, 0, 0, 0};

  WheelVelocity velocities = {.front_left = 0, .front_right = 0, .rear_left = 0, .rear_right = 0};
  for (int i = FRONT_LEFT; i <= REAR_RIGHT; i++)
    update_encoder(&(encoders[i]));

  velocities.front_left = encoders[FRONT_LEFT].velocity;
  velocities.front_right = encoders[FRONT_RIGHT].velocity;
  velocities.rear_left = encoders[REAR_LEFT].velocity;
  velocities.rear_right = encoders[REAR_RIGHT].velocity;

  return velocities;
}