
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <lvgl.h>

#include "ui_simple.h"

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

int main(void)
{
    const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

    if (!device_is_ready(display)) {
        LOG_ERR("Display device not ready");
        return;
    }

    ui_simple_init();

    lv_task_handler();
    display_blanking_off(display);

    while (1) {
        lv_task_handler();      
        k_sleep(K_MSEC(10));
    }
    return 0;
}
