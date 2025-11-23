#pragma once

#include <stdbool.h>
#include <stdint.h>


void ui_simple_init(void);


void ui_set_input_state(uint8_t index, bool active);

void ui_set_output_state(uint8_t index, bool active);

void ui_set_temp_setpoint(int16_t value);

int16_t ui_get_temp_setpoint(void);
