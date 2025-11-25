#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/data/json.h>
#include <lvgl.h>

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *display_dev;

static lv_obj_t *screen_dashboard;
static lv_obj_t *screen_io;
static lv_obj_t *screen_config;

static lv_obj_t *config_status_label;


struct hvac_pid_cfg {
    int32_t kp;
    int32_t ki;
    int32_t kd;
};

struct hvac_config {
    int32_t setpoint;        
    struct hvac_pid_cfg pid;  

};

static struct hvac_config g_hvac_cfg;

//TODO: ulepszenie struktur konfiguracji HVAC w przyszłości:
// - dodanie mapowania kanałów I/O
// - dodanie sekwencji grzania/chłodzenia

static char hvac_config1_json[] =
    "{"
    "  \"setpoint\": 21,"
    "  \"pid\": {"
    "    \"kp\": 10,"
    "    \"ki\": 2,"
    "    \"kd\": 1"
    "  }"
    "}";

static char hvac_config2_json[] =
    "{"
    "  \"setpoint\": 24,"
    "  \"pid\": {"
    "    \"kp\": 15,"
    "    \"ki\": 3,"
    "    \"kd\": 2"
    "  }"
    "}";

static const struct json_obj_descr hvac_pid_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct hvac_pid_cfg, kp, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct hvac_pid_cfg, ki, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct hvac_pid_cfg, kd, JSON_TOK_NUMBER),
};

static const struct json_obj_descr hvac_cfg_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct hvac_config, setpoint, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_OBJECT(struct hvac_config, pid, hvac_pid_descr),
};

static void nav_to_dashboard(lv_event_t *e);
static void nav_to_io(lv_event_t *e);
static void nav_to_config(lv_event_t *e);

static int hvac_load_config_from_json(char *json, size_t len,
                                      struct hvac_config *out_cfg);
static void hvac_apply_config(const struct hvac_config *cfg);

static void on_btn_load_cfg1(lv_event_t *e);
static void on_btn_load_cfg2(lv_event_t *e);


static void create_nav_buttons(lv_obj_t *nav_container)
{
    lv_obj_t *btn_dash = lv_btn_create(nav_container);
    lv_obj_t *lbl_dash = lv_label_create(btn_dash);
    lv_label_set_text(lbl_dash, "Dashboard");
    lv_obj_center(lbl_dash);
    lv_obj_add_event_cb(btn_dash, nav_to_dashboard, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_io = lv_btn_create(nav_container);
    lv_obj_t *lbl_io = lv_label_create(btn_io);
    lv_label_set_text(lbl_io, "I/O Visualiser");
    lv_obj_center(lbl_io);
    lv_obj_add_event_cb(btn_io, nav_to_io, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_cfg = lv_btn_create(nav_container);
    lv_obj_t *lbl_cfg = lv_label_create(btn_cfg);
    lv_label_set_text(lbl_cfg, "Config Loader");
    lv_obj_center(lbl_cfg);
    lv_obj_add_event_cb(btn_cfg, nav_to_config, LV_EVENT_CLICKED, NULL);
}

static void create_header(lv_obj_t *parent, const char *title_text)
{
    lv_obj_t *nav = lv_obj_create(parent);

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

    lv_obj_t *cont = lv_obj_create(screen_config);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(80));
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    lv_obj_t *btn1 = lv_btn_create(cont);
    lv_obj_t *lbl1 = lv_label_create(btn1);
    lv_label_set_text(lbl1, "Load Config 1");
    lv_obj_center(lbl1);
    lv_obj_add_event_cb(btn1, on_btn_load_cfg1, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn2 = lv_btn_create(cont);
    lv_obj_t *lbl2 = lv_label_create(btn2);
    lv_label_set_text(lbl2, "Load Config 2");
    lv_obj_center(lbl2);
    lv_obj_add_event_cb(btn2, on_btn_load_cfg2, LV_EVENT_CLICKED, NULL);

    config_status_label = lv_label_create(cont);
    lv_label_set_text(config_status_label, "No config loaded");
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


static int hvac_load_config_from_json(char *json, size_t len,
                                      struct hvac_config *out_cfg)
{
    int ret = json_obj_parse(json, len,
                             hvac_cfg_descr,
                             ARRAY_SIZE(hvac_cfg_descr),
                             out_cfg);

    if (ret < 0) {
        LOG_ERR("JSON parse error: %d", ret);
        return ret;
    }


    LOG_INF("Loaded config: setpoint=%d, kp=%d, ki=%d, kd=%d",
            out_cfg->setpoint,
            out_cfg->pid.kp,
            out_cfg->pid.ki,
            out_cfg->pid.kd);

    return 0;
}

static void hvac_apply_config(const struct hvac_config *cfg)
{
    g_hvac_cfg = *cfg; 


}


static void on_btn_load_cfg1(lv_event_t *e)
{
    ARG_UNUSED(e);

    struct hvac_config tmp;
    int ret = hvac_load_config_from_json(
        hvac_config1_json,
        sizeof(hvac_config1_json),
        &tmp
    );

    if (ret == 0) {
        hvac_apply_config(&tmp);
        lv_label_set_text(config_status_label, "Loaded Config 1");
    } else {
        lv_label_set_text(config_status_label, "Error loading Config 1");
    }
}

static void on_btn_load_cfg2(lv_event_t *e)
{
    ARG_UNUSED(e);

    struct hvac_config tmp;
    int ret = hvac_load_config_from_json(
        hvac_config2_json,
        sizeof(hvac_config2_json),
        &tmp
    );

    if (ret == 0) {
        hvac_apply_config(&tmp);
        lv_label_set_text(config_status_label, "Loaded Config 2");
    } else {
        lv_label_set_text(config_status_label, "Error loading Config 2");
    }
}

int main(void)
{
    LOG_INF("Starting simple LVGL UI with JSON config loader");

    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready");
        return 0;
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
