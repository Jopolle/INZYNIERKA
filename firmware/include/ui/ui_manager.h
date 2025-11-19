#ifndef HVAC_UI_MANAGER_H
#define HVAC_UI_MANAGER_H

#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "ui_model.h"

int ui_manager_init(const struct device *display, const struct ui_model *initial_state);
void ui_manager_process(void);
int ui_manager_post_update(const struct ui_model *state, k_timeout_t timeout);
bool ui_manager_is_ready(void);

#endif /* HVAC_UI_MANAGER_H */
