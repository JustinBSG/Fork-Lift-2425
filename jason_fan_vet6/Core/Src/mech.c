#include "mech.h"

Direction_Encoder direction_encoder = FRONT_BACK;

bool turn_on_fan = false;
bool prev_turn_on_fan = false;

void fan_operation(bool turn_on) {
    if (turn_on)
      TIM9->CCR1 = 1450;
    else
      TIM9->CCR1 = 1500;
}