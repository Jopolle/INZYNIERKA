#ifndef HVAC_UI_MODEL_H
#define HVAC_UI_MODEL_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum ui_hvac_mode {
	UI_HVAC_MODE_OFF = 0,
	UI_HVAC_MODE_AUTO,
	UI_HVAC_MODE_MANUAL,
	UI_HVAC_MODE_COUNT
};

struct ui_model {
	float zone_temperature_c;
	float zone_humidity_pct;
	float co2_ppm;
	uint8_t analog_output_percent;
	uint8_t fan_speed_percent;
	enum ui_hvac_mode mode;
	bool alarm_active;
	char alarm_text[48];
};

static inline void ui_model_set_default(struct ui_model *model)
{
	memset(model, 0, sizeof(*model));
	model->zone_temperature_c = 21.5f;
	model->zone_humidity_pct = 45.0f;
	model->co2_ppm = 420.0f;
	model->analog_output_percent = 0U;
	model->fan_speed_percent = 0U;
	model->mode = UI_HVAC_MODE_AUTO;
	model->alarm_active = false;
	model->alarm_text[0] = '\0';
}

static inline const char *ui_mode_to_string(enum ui_hvac_mode mode)
{
	switch (mode) {
	case UI_HVAC_MODE_OFF:
		return "Off";
	case UI_HVAC_MODE_AUTO:
		return "Auto";
	case UI_HVAC_MODE_MANUAL:
		return "Manual";
	default:
		return "Unknown";
	}
}

#endif /* HVAC_UI_MODEL_H */
