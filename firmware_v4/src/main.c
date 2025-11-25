#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <lvgl.h>

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *display_dev;

static lv_obj_t *screen_dashboard;
static lv_obj_t *screen_io;
static lv_obj_t *screen_config;

static void nav_to_dashboard(lv_event_t *e);
static void nav_to_io(lv_event_t *e);
static void nav_to_config(lv_event_t *e);


static void create_nav_buttons(lv_obj_t *nav_container)
{
    /* Dashboard button */
    lv_obj_t *btn_dash = lv_btn_create(nav_container);
    lv_obj_t *lbl_dash = lv_label_create(btn_dash);
    lv_label_set_text(lbl_dash, "Dashboard");
    lv_obj_center(lbl_dash);
    lv_obj_add_event_cb(btn_dash, nav_to_dashboard, LV_EVENT_CLICKED, NULL);
    //lv_obj_align(btn_dash, LV_ALIGN_TOP_LEFT, 0, 0);

    /* I/O Visualiser button */
    lv_obj_t *btn_io = lv_btn_create(nav_container);
    lv_obj_t *lbl_io = lv_label_create(btn_io);
    lv_label_set_text(lbl_io, "I/O Visualiser");
    lv_obj_center(lbl_io);
    lv_obj_add_event_cb(btn_io, nav_to_io, LV_EVENT_CLICKED, NULL);
    //lv_obj_align(btn_io, LV_ALIGN_TOP_MID, 0, 0);

    /* Configuration Loader button */
    lv_obj_t *btn_cfg = lv_btn_create(nav_container);
    lv_obj_t *lbl_cfg = lv_label_create(btn_cfg);
    lv_label_set_text(lbl_cfg, "Config Loader");
    lv_obj_center(lbl_cfg);
    lv_obj_add_event_cb(btn_cfg, nav_to_config, LV_EVENT_CLICKED, NULL);
    //lv_obj_align(btn_cfg, LV_ALIGN_TOP_RIGHT, 0, 0);
}

static void create_header(lv_obj_t *parent, const char *title_text)
{

    lv_obj_t *nav = lv_obj_create(parent);
    //lv_obj_clear_flag(nav, LV_OBJ_FLAG_SCROLLABLE);

    //lv_obj_set_style_bg_color(nav, lv_color_hex(0x007bff), LV_PART_MAIN | LV_STATE_DEFAULT);


    lv_obj_set_style_border_width(nav, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(nav, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(nav, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_obj_set_width(nav, LV_PCT(100));
    lv_obj_set_height(nav, LV_PCT(15));
    lv_obj_set_flex_flow(nav, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(nav,
                          LV_FLEX_ALIGN_START,   
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    lv_obj_align(nav, LV_ALIGN_TOP_MID, 0, 0);

    create_nav_buttons(nav);


    lv_obj_t *title = lv_label_create(parent);
    lv_label_set_text(title, title_text);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 20, 42);
}


static void create_dashboard_screen(void)
{
    screen_dashboard = lv_obj_create(NULL);
    create_header(screen_dashboard, "Dashboard");

}

static void create_io_screen(void)
{
    screen_io = lv_obj_create(NULL);
    create_header(screen_io, "I/O Visualiser");


}

static void create_config_screen(void)
{
    screen_config = lv_obj_create(NULL);
    create_header(screen_config, "Configuration Loader");

}


static void nav_to_dashboard(lv_event_t *e)
{
    ARG_UNUSED(e);
    if (screen_dashboard != NULL) {
        lv_scr_load(screen_dashboard);
    }
}

static void nav_to_io(lv_event_t *e)
{
    ARG_UNUSED(e);
    if (screen_io != NULL) {
        lv_scr_load(screen_io);
    }
}

static void nav_to_config(lv_event_t *e)
{
    ARG_UNUSED(e);
    if (screen_config != NULL) {
        lv_scr_load(screen_config);
    }
}


int main(void)
{
    LOG_INF("Starting simple LVGL UI");

    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready");
        return;
    }

    create_dashboard_screen();
    create_io_screen();
    create_config_screen();

    lv_scr_load(screen_dashboard);

    display_blanking_off(display_dev);

    while (1) {
        lv_timer_handler();    
        k_msleep(10);            
    }
    return 0;
}
