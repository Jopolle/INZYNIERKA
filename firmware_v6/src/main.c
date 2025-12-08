#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/data/json.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define button_color lv_color_hex(0x0A854A)

/* Ikona nastawy temperatury */
LV_IMG_DECLARE(TP_type1__not_active);

/* Obrazki z wykresami sekwencji (musisz je wygenerować LVGL converterem) */
LV_IMG_DECLARE(seq_img_cool_dead_heat);
LV_IMG_DECLARE(seq_img_cool_rec_dead_rec_heat);

LV_IMG_DECLARE(snowflake);
LV_IMG_DECLARE(heater);
LV_IMG_DECLARE(heat_exchange);

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define HVAC_NUM_AI_CHANNELS 8
#define HVAC_NUM_AO_CHANNELS 8

#define ABSF(x) ((x) < 0.0f ? -(x) : (x))

/* --- Placeholdery I/O --- */

static float read_ai_voltage(int ch)
{
    ARG_UNUSED(ch);
    return 0.0f;
}

static float read_ao_voltage(int ch)
{
    ARG_UNUSED(ch);
    return 0.0f;
}

static void write_ao_voltage(int ch, float voltage)
{
    ARG_UNUSED(ch);
    ARG_UNUSED(voltage);
}

/* --- Struktury konfiguracji --- */

struct hvac_pid_cfg {
    int32_t kp;
    int32_t ki;
    int32_t kd;
};

struct hvac_io_cfg {
    int32_t t_supply_ai;        // temp. nawiewu
    int32_t t_extract_ai;       // temp. wyciągu
    int32_t t_exhaust_ai;       // temp. wyrzutni
    int32_t t_outdoor_ai;       // temp. zewnętrzna / czerpnia
    int32_t frost_ai;           // czujnik przeciwzamrożeniowy

    int32_t bypass_ao;          // sterowanie bypassem wymiennika
    int32_t fan_vfd_ao;         // falownik wentylatora
    int32_t heater_ao;          // grzałka / nagrzewnica
    int32_t cooler_ao;          // chłodnica
};

struct hvac_seq_band {
    int32_t from_percent;
    int32_t to_percent;
};

struct hvac_seq_cfg {
    struct hvac_seq_band cooling;
    struct hvac_seq_band heating;
    struct hvac_seq_band heat_recovery;
    struct hvac_seq_band deadband;
};

struct hvac_config {
    int32_t setpoint;
    struct hvac_pid_cfg pid;
    struct hvac_io_cfg  io;
    struct hvac_seq_cfg seq;
    const char *sequence_type;   /* np. "cool_dead_heat" albo "cool_rec_dead_rec_heat" */
};

struct hvac_pid_state {
    float i_term;
};

enum {
    HVAC_SEQ_IDX_COOLING = 0,
    HVAC_SEQ_IDX_HEATING,
    HVAC_SEQ_IDX_HEAT_RECOVERY,
    HVAC_SEQ_IDX_DEADBAND,
    HVAC_SEQ_IDX_COUNT
};

static const char *const hvac_seq_band_names[HVAC_SEQ_IDX_COUNT] = {
    "Cooling",
    "Heating",
    "Heat recovery",
    "Deadband"
};

/* --- LVGL / ekrany --- */

static const struct device *display_dev;

static lv_obj_t *screen_dashboard;
static lv_obj_t *screen_io;
static lv_obj_t *screen_config;
static lv_obj_t *screen_seq_viewer;   /* nowy ekran */

static lv_obj_t *config_status_label;

static lv_obj_t *ai_value_labels[HVAC_NUM_AI_CHANNELS];
static lv_obj_t *ai_unit_labels[HVAC_NUM_AI_CHANNELS];
static lv_obj_t *ai_name_labels[HVAC_NUM_AI_CHANNELS];

static lv_obj_t *ao_value_labels[HVAC_NUM_AO_CHANNELS];
static lv_obj_t *ao_unit_labels[HVAC_NUM_AO_CHANNELS];
static lv_obj_t *ao_name_labels[HVAC_NUM_AO_CHANNELS];

static lv_obj_t *io_ai_container;
static lv_obj_t *io_ao_container;

static lv_obj_t *dash_screen1_container;
static lv_obj_t *dash_screen2_container;

static lv_obj_t *setpoint_label;

static lv_obj_t *seq_rows[HVAC_SEQ_IDX_COUNT];
static lv_obj_t *seq_from_labels[HVAC_SEQ_IDX_COUNT];
static lv_obj_t *seq_to_labels[HVAC_SEQ_IDX_COUNT];

static lv_obj_t *seq_viewer_image;    /* obrazek na ekranie Sequence Viewer */

struct seq_btn_ctx {
    uint8_t band_index;
    uint8_t is_from;
    int8_t  delta;
};

static struct seq_btn_ctx g_seq_btn_ctx[HVAC_SEQ_IDX_COUNT][2][2];

/* --- Globalna konfiguracja --- */

static struct hvac_config g_hvac_cfg = {
    .setpoint = 0,
    .pid = { .kp = 0, .ki = 0, .kd = 0 },
    .io = {
        .t_supply_ai      = -1,
        .t_extract_ai     = -1,
        .t_exhaust_ai     = -1,
        .t_outdoor_ai     = -1,
        .frost_ai         = -1,
        .bypass_ao        = -1,
        .fan_vfd_ao       = -1,
        .heater_ao        = -1,
        .cooler_ao        = -1,
    },

    .seq = {
        .cooling       = { .from_percent = -100, .to_percent = -30 },
        .deadband      = { .from_percent =  -30, .to_percent =  30 },
        .heating       = { .from_percent =   30, .to_percent = 100 },
        .heat_recovery = { .from_percent =    0, .to_percent =   0 },
    },

    .sequence_type = NULL,
};

static struct hvac_pid_state g_hvac_pid_state;

/* --- Forward declarations --- */

static void nav_to_dashboard(lv_event_t *e);
static void nav_to_io(lv_event_t *e);
static void nav_to_config(lv_event_t *e);
static void nav_to_seq_viewer(lv_event_t *e);

static int  hvac_load_config_from_json(const char *json_src, size_t len,
                                      struct hvac_config *out_cfg);
static void hvac_apply_config(const struct hvac_config *cfg);

static void on_btn_load_cfg1(lv_event_t *e);
static void on_btn_load_cfg2(lv_event_t *e);

static void on_btn_setpoint_minus(lv_event_t *e);
static void on_btn_setpoint_plus(lv_event_t *e);
static void on_seq_band_adjust(lv_event_t *e);

static void on_dash_show_screen1(lv_event_t *e);
static void on_dash_show_screen2(lv_event_t *e);

static void on_io_show_ai(lv_event_t *e);
static void on_io_show_ao(lv_event_t *e);

static void hvac_refresh_dashboard(void);
static void hvac_refresh_dashboard_setpoint(void);
static void hvac_refresh_dashboard_seq_labels(void);
static void hvac_refresh_sequence_viewer(void);

static void hvac_pid_reset(struct hvac_pid_state *st);
static float hvac_pid_step(const struct hvac_pid_cfg *cfg,
                           struct hvac_pid_state *st,
                           float error,
                           float dt_sec);

static void hvac_apply_sequence(float pid_out_pct,
                                const struct hvac_config *cfg,
                                float *heater_pct,
                                float *cooler_pct,
                                float *bypass_pct);

static float hvac_get_extract_temp_c(void);
static void hvac_control_step(float dt_sec);

/* --- Pomocnicze --- */

static struct hvac_seq_band *hvac_get_seq_band_by_index(struct hvac_seq_cfg *seq, int index)
{
    switch (index) {
    case HVAC_SEQ_IDX_COOLING:       return &seq->cooling;
    case HVAC_SEQ_IDX_HEATING:       return &seq->heating;
    case HVAC_SEQ_IDX_HEAT_RECOVERY: return &seq->heat_recovery;
    case HVAC_SEQ_IDX_DEADBAND:      return &seq->deadband;
    default:                         return NULL;
    }
}

/* --- JSON config przykładowy --- */

static char hvac_config1_json[] =
    "{"
    "  \"setpoint\": 21,"
    "  \"pid\": {"
    "    \"kp\": 10,"
    "    \"ki\": 2,"
    "    \"kd\": 1"
    "  },"
    "  \"seq\": {"
    "    \"cooling\": {"
    "      \"from_percent\": -100,"
    "      \"to_percent\": -30"
    "    },"
    "    \"deadband\": {"
    "      \"from_percent\": -30,"
    "      \"to_percent\": 30"
    "    },"
    "    \"heating\": {"
    "      \"from_percent\": 30,"
    "      \"to_percent\": 100"
    "    },"
    "    \"heat_recovery\": {"
    "      \"from_percent\": 0,"
    "      \"to_percent\": 0"
    "    }"
    "  },"
    "  \"sequence_type\": \"cool_dead_heat\","
    "  \"io\": {"
    "    \"t_supply_ai\": 0,"
    "    \"t_extract_ai\": 1,"
    "    \"t_exhaust_ai\": 2,"
    "    \"t_outdoor_ai\": 3,"
    "    \"frost_ai\": -1,"
    "    \"bypass_ao\": -1,"
    "    \"fan_vfd_ao\": 1,"
    "    \"heater_ao\": 2,"
    "    \"cooler_ao\": 3"
    "  }"
    "}";

static char hvac_config2_json[] =
    "{"
    "  \"setpoint\": 23,"
    "  \"pid\": {"
    "    \"kp\": 15,"
    "    \"ki\": 3,"
    "    \"kd\": 2"
    "  },"
    "  \"seq\": {"
    "    \"cooling\": {"
    "      \"from_percent\": -100,"
    "      \"to_percent\": -60"
    "    },"
    "    \"heating\": {"
    "      \"from_percent\": 60,"
    "      \"to_percent\": 100"
    "    },"
    "    \"heat_recovery\": {"
    "      \"from_percent\": -60,"
    "      \"to_percent\": 60"
    "    },"
    "    \"deadband\": {"
    "      \"from_percent\": -10,"
    "      \"to_percent\": 10"
    "    }"
    "  },"
    "  \"sequence_type\": \"cool_rec_dead_rec_heat\","
    "  \"io\": {"
    "    \"t_supply_ai\": 1,"
    "    \"t_extract_ai\": 0,"
    "    \"t_exhaust_ai\": 2,"
    "    \"t_outdoor_ai\": 3,"
    "    \"frost_ai\": 4,"
    "    \"bypass_ao\": 0,"
    "    \"fan_vfd_ao\": 1,"
    "    \"heater_ao\": 2,"
    "    \"cooler_ao\": 3"
    "  }"
    "}";

/* --- JSON deskriptory --- */

static const struct json_obj_descr hvac_pid_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct hvac_pid_cfg, kp, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct hvac_pid_cfg, ki, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct hvac_pid_cfg, kd, JSON_TOK_NUMBER),
};

static const struct json_obj_descr hvac_io_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct hvac_io_cfg, t_supply_ai,  JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct hvac_io_cfg, t_extract_ai, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct hvac_io_cfg, t_exhaust_ai, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct hvac_io_cfg, t_outdoor_ai, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct hvac_io_cfg, frost_ai,     JSON_TOK_NUMBER),

    JSON_OBJ_DESCR_PRIM(struct hvac_io_cfg, bypass_ao,    JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct hvac_io_cfg, fan_vfd_ao,   JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct hvac_io_cfg, heater_ao,    JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct hvac_io_cfg, cooler_ao,    JSON_TOK_NUMBER),
};

static const struct json_obj_descr hvac_seq_band_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct hvac_seq_band, from_percent, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct hvac_seq_band, to_percent,   JSON_TOK_NUMBER),
};

static const struct json_obj_descr hvac_seq_descr[] = {
    JSON_OBJ_DESCR_OBJECT(struct hvac_seq_cfg, cooling,       hvac_seq_band_descr),
    JSON_OBJ_DESCR_OBJECT(struct hvac_seq_cfg, heating,       hvac_seq_band_descr),
    JSON_OBJ_DESCR_OBJECT(struct hvac_seq_cfg, heat_recovery, hvac_seq_band_descr),
    JSON_OBJ_DESCR_OBJECT(struct hvac_seq_cfg, deadband,      hvac_seq_band_descr),
};

static const struct json_obj_descr hvac_cfg_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct hvac_config, setpoint, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_OBJECT(struct hvac_config, pid, hvac_pid_descr),
    JSON_OBJ_DESCR_OBJECT(struct hvac_config, io,  hvac_io_descr),
    JSON_OBJ_DESCR_OBJECT(struct hvac_config, seq, hvac_seq_descr),
    //JSON_OBJ_DESCR_PRIM(struct hvac_config, sequence_type, JSON_TOK_STRING),
};

/* --- Opisy ról kanałów --- */

static const char *const hvac_ai_role_names[] = {
    "T supply",
    "T extract",
    "T exhaust",
    "T outdoor",
    "Frost protect",
};

static const char *const hvac_ao_role_names[] = {
    "Bypass",
    "Fan VFD",
    "Heater",
    "Cooler",
};

static const char *hvac_ai_role_name_for_channel(int ch)
{
    const struct hvac_io_cfg *io = &g_hvac_cfg.io;

    if (ch == io->t_supply_ai)      return "T supply";
    if (ch == io->t_extract_ai)     return "T extract";
    if (ch == io->t_exhaust_ai)     return "T exhaust";
    if (ch == io->t_outdoor_ai)     return "T outdoor";
    if (ch == io->frost_ai)         return "Frost protect";

    return "Unused";
}

static const char *hvac_ao_role_name_for_channel(int ch)
{
    const struct hvac_io_cfg *io = &g_hvac_cfg.io;

    if (ch == io->bypass_ao)   return "Bypass";
    if (ch == io->fan_vfd_ao)  return "Fan VFD";
    if (ch == io->heater_ao)   return "Heater";
    if (ch == io->cooler_ao)   return "Cooler";

    return "Unused";
}

/* --- Aktualizacja wartości I/O --- */

void hvac_update_io_values(void)
{
    char buf[16];

    for (int i = 0; i < HVAC_NUM_AI_CHANNELS; i++) {
        float v = read_ai_voltage(i);
        snprintf(buf, sizeof(buf), "%.2f", (double)v);
        if (ai_value_labels[i]) {
            lv_label_set_text(ai_value_labels[i], buf);
        }
    }

    for (int i = 0; i < HVAC_NUM_AO_CHANNELS; i++) {
        float v = read_ao_voltage(i);
        snprintf(buf, sizeof(buf), "%.2f", (double)v);
        if (ao_value_labels[i]) {
            lv_label_set_text(ao_value_labels[i], buf);
        }
    }
}

/* --- Odświeżanie nazw ról I/O --- */

static void hvac_refresh_io_role_labels(void)
{
    for (int i = 0; i < HVAC_NUM_AI_CHANNELS; i++) {
        if (ai_name_labels[i]) {
            const char *role = hvac_ai_role_name_for_channel(i);
            lv_label_set_text(ai_name_labels[i], role);
        }
    }

    for (int i = 0; i < HVAC_NUM_AO_CHANNELS; i++) {
        if (ao_name_labels[i]) {
            const char *role = hvac_ao_role_name_for_channel(i);
            lv_label_set_text(ao_name_labels[i], role);
        }
    }
}

/* --- Header + nawigacja --- */

static void create_nav_buttons(lv_obj_t *nav_container)
{
    lv_obj_t *btn_dash = lv_btn_create(nav_container);
    lv_obj_t *lbl_dash = lv_label_create(btn_dash);
    lv_label_set_text(lbl_dash, "Dashboard");
    lv_obj_center(lbl_dash);
    lv_obj_add_event_cb(btn_dash, nav_to_dashboard, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_dash, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *btn_io = lv_btn_create(nav_container);
    lv_obj_t *lbl_io = lv_label_create(btn_io);
    lv_label_set_text(lbl_io, "I/O");
    lv_obj_center(lbl_io);
    lv_obj_add_event_cb(btn_io, nav_to_io, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_io, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *btn_seq = lv_btn_create(nav_container);
    lv_obj_t *lbl_seq = lv_label_create(btn_seq);
    lv_label_set_text(lbl_seq, "Sequence");
    lv_obj_center(lbl_seq);
    lv_obj_add_event_cb(btn_seq, nav_to_seq_viewer, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_seq, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *btn_cfg = lv_btn_create(nav_container);
    lv_obj_t *lbl_cfg = lv_label_create(btn_cfg);
    lv_label_set_text(lbl_cfg, "Config Loader");
    lv_obj_center(lbl_cfg);
    lv_obj_add_event_cb(btn_cfg, nav_to_config, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_cfg, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);
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

static void hvac_refresh_dashboard_setpoint(void)
{
    if (!setpoint_label) {
        return;
    }

    char buf[32];
    snprintf(buf, sizeof(buf), ": %d C", g_hvac_cfg.setpoint);
    lv_label_set_text(setpoint_label, buf);
}

static void hvac_refresh_dashboard_seq_labels(void)
{
    struct hvac_seq_cfg *seq = &g_hvac_cfg.seq;

    for (int i = 0; i < HVAC_SEQ_IDX_COUNT; i++) {
        lv_obj_t *row = seq_rows[i];
        if (!row) {
            continue;
        }

        struct hvac_seq_band *band = hvac_get_seq_band_by_index(seq, i);
        if (!band) {
            continue;
        }

        bool active = (band->from_percent != band->to_percent);

        if (active) {
            lv_obj_clear_flag(row, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(row, LV_OBJ_FLAG_HIDDEN);
        }

        char buf[16];

        if (seq_from_labels[i]) {
            snprintf(buf, sizeof(buf), "%d %%", band->from_percent);
            lv_label_set_text(seq_from_labels[i], buf);
        }
        if (seq_to_labels[i]) {
            snprintf(buf, sizeof(buf), "%d %%", band->to_percent);
            lv_label_set_text(seq_to_labels[i], buf);
        }
    }
}

static void hvac_refresh_dashboard(void)
{
    hvac_refresh_dashboard_setpoint();
    hvac_refresh_dashboard_seq_labels();
}

static void hvac_refresh_sequence_viewer(void)
{
    if (!seq_viewer_image) {
        return;
    }

    const lv_img_dsc_t *img = NULL;

    if (g_hvac_cfg.sequence_type != NULL) {
        if (strcmp(g_hvac_cfg.sequence_type, "cool_dead_heat") == 0) {
            img = &seq_img_cool_dead_heat;
        } else if (strcmp(g_hvac_cfg.sequence_type, "cool_rec_dead_rec_heat") == 0) {
            img = &seq_img_cool_rec_dead_rec_heat;
        }
    }

    if (img) {
        lv_img_set_src(seq_viewer_image, img);
    }
}

/* --- Dashboard handlers --- */

static void on_btn_setpoint_minus(lv_event_t *e)
{
    ARG_UNUSED(e);
    g_hvac_cfg.setpoint -= 1;
    hvac_refresh_dashboard_setpoint();
}

static void on_btn_setpoint_plus(lv_event_t *e)
{
    ARG_UNUSED(e);
    g_hvac_cfg.setpoint += 1;
    hvac_refresh_dashboard_setpoint();
}

static void on_seq_band_adjust(lv_event_t *e)
{
    struct seq_btn_ctx *ctx = lv_event_get_user_data(e);
    if (!ctx) {
        return;
    }

    struct hvac_seq_band *band =
        hvac_get_seq_band_by_index(&g_hvac_cfg.seq, ctx->band_index);
    if (!band) {
        return;
    }

    int32_t *val = ctx->is_from ? &band->from_percent : &band->to_percent;

    *val += ctx->delta;

    if (*val < -100) *val = -100;
    if (*val > 100)  *val = 100;

    if (ctx->is_from && band->from_percent > band->to_percent) {
        band->from_percent = band->to_percent;
    } else if (!ctx->is_from && band->to_percent < band->from_percent) {
        band->to_percent = band->from_percent;
    }

    hvac_refresh_dashboard_seq_labels();
}

/* callback: pokaż Screen 1 (setpoint + cooling + heating) */
static void on_dash_show_screen1(lv_event_t *e)
{
    ARG_UNUSED(e);
    if (!dash_screen1_container || !dash_screen2_container) {
        return;
    }

    lv_obj_clear_flag(dash_screen1_container, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(dash_screen2_container, LV_OBJ_FLAG_HIDDEN);
}

static void on_dash_show_screen2(lv_event_t *e)
{
    ARG_UNUSED(e);
    if (!dash_screen1_container || !dash_screen2_container) {
        return;
    }

    lv_obj_clear_flag(dash_screen2_container, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(dash_screen1_container, LV_OBJ_FLAG_HIDDEN);
}

static void create_dashboard_screen(void)
{
    screen_dashboard = lv_obj_create(NULL);
    lv_obj_clear_flag(screen_dashboard, LV_OBJ_FLAG_SCROLLABLE);
    create_header(screen_dashboard, "Dashboard");

    lv_obj_t *cont = lv_obj_create(screen_dashboard);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(85));
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_END,
                          LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(cont, 8, LV_PART_MAIN);
    lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, LV_PART_MAIN);

    /* wiersz z przyciskami Screen 1 / Screen 2 */
    lv_obj_t *tab_row = lv_obj_create(cont);
    lv_obj_set_width(tab_row, LV_PCT(100));
    lv_obj_set_height(tab_row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(tab_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(tab_row,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(tab_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(tab_row, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(tab_row, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(tab_row, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_obj_t *btn_scr1 = lv_btn_create(tab_row);
    lv_obj_t *lbl_scr1 = lv_label_create(btn_scr1);
    lv_label_set_text(lbl_scr1, "Screen 1");
    lv_obj_center(lbl_scr1);
    lv_obj_add_event_cb(btn_scr1, on_dash_show_screen1, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_scr1, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_height(btn_scr1, 32);

    lv_obj_t *btn_scr2 = lv_btn_create(tab_row);
    lv_obj_t *lbl_scr2 = lv_label_create(btn_scr2);
    lv_label_set_text(lbl_scr2, "Screen 2");
    lv_obj_center(lbl_scr2);
    lv_obj_add_event_cb(btn_scr2, on_dash_show_screen2, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_scr2, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_height(btn_scr2, 32);

    /* kontener Screen 1: setpoint + Cooling + Heating */
    dash_screen1_container = lv_obj_create(cont);
    lv_obj_set_width(dash_screen1_container, LV_PCT(100));
    lv_obj_set_height(dash_screen1_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(dash_screen1_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(dash_screen1_container,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_END,
                          LV_FLEX_ALIGN_END);
    lv_obj_clear_flag(dash_screen1_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(dash_screen1_container, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(dash_screen1_container, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(dash_screen1_container, LV_OPA_TRANSP, LV_PART_MAIN);

    /* kontener Screen 2: Deadband + Heat recovery */
    dash_screen2_container = lv_obj_create(cont);
    lv_obj_set_width(dash_screen2_container, LV_PCT(100));
    lv_obj_set_height(dash_screen2_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(dash_screen2_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(dash_screen2_container,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(dash_screen2_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(dash_screen2_container, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(dash_screen2_container, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(dash_screen2_container, LV_OPA_TRANSP, LV_PART_MAIN);

    /* --- Screen 1: wiersz nastawy temperatury --- */
    lv_obj_t *row_sp = lv_obj_create(dash_screen1_container);
    lv_obj_set_width(row_sp, LV_PCT(100));
    lv_obj_set_height(row_sp, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row_sp, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row_sp,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(row_sp, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(row_sp, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(row_sp, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row_sp, LV_OPA_TRANSP, LV_PART_MAIN);

    /* Ikona nastawy temperatury */
    lv_obj_t *img_sp = lv_img_create(row_sp);
    lv_img_set_src(img_sp, &TP_type1__not_active);
    lv_img_set_zoom(img_sp, 128);
    lv_obj_set_size(img_sp, 32, 32);

    setpoint_label = lv_label_create(row_sp);
    lv_label_set_text(setpoint_label, ": 0 C");

    lv_obj_t *btn_sp_minus = lv_btn_create(row_sp);
    lv_obj_t *lbl_sp_minus = lv_label_create(btn_sp_minus);
    lv_label_set_text(lbl_sp_minus, "-");
    lv_obj_center(lbl_sp_minus);
    lv_obj_add_event_cb(btn_sp_minus, on_btn_setpoint_minus, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_sp_minus, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *btn_sp_plus = lv_btn_create(row_sp);
    lv_obj_t *lbl_sp_plus = lv_label_create(btn_sp_plus);
    lv_label_set_text(lbl_sp_plus, "+");
    lv_obj_center(lbl_sp_plus);
    lv_obj_add_event_cb(btn_sp_plus, on_btn_setpoint_plus, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_sp_plus, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

    /* --- Wiersze sekwencji: Cooling/Heating -> Screen 1, Deadband/HR -> Screen 2 --- */
    for (int i = 0; i < HVAC_SEQ_IDX_COUNT; i++) {
        lv_obj_t *parent_for_row;

        if (i == HVAC_SEQ_IDX_COOLING || i == HVAC_SEQ_IDX_HEATING) {
            parent_for_row = dash_screen1_container;   /* Screen 1 */
        } else {
            parent_for_row = dash_screen2_container;   /* Screen 2 */
        }

        lv_obj_t *row = lv_obj_create(parent_for_row);
        seq_rows[i] = row;

        lv_obj_set_width(row, LV_PCT(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row,
                              LV_FLEX_ALIGN_START,
                              LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(row, 4, LV_PART_MAIN);
        lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, LV_PART_MAIN);

        if (i == HVAC_SEQ_IDX_COOLING) {
            lv_obj_t *img_cool = lv_img_create(row);
            lv_img_set_src(img_cool, &snowflake);   /* nazwa jak w LV_IMG_DECLARE */
            lv_img_set_zoom(img_cool, 128);              /* podobnie jak przy TP_type1__not_active */
            lv_obj_set_size(img_cool, 32, 32);
        } else if (i == HVAC_SEQ_IDX_HEATING) {
            lv_obj_t *img_heat = lv_img_create(row);
            lv_img_set_src(img_heat, &heater);   /* nazwa jak w LV_IMG_DECLARE */
            lv_img_set_zoom(img_heat, 128);          /* podobnie jak przy TP_type1__not_active */
            lv_obj_set_size(img_heat, 32, 32);
        }else if (i == HVAC_SEQ_IDX_HEAT_RECOVERY)
        {
            lv_obj_t *img_heat_ex = lv_img_create(row);
            lv_img_set_src(img_heat_ex, &heat_exchange);   
            lv_img_set_zoom(img_heat_ex, 128);          
            lv_obj_set_size(img_heat_ex, 32, 32);
        }
        
        
        else {
            lv_obj_t *lbl_band = lv_label_create(row);
            lv_label_set_text(lbl_band, hvac_seq_band_names[i]);
        }

        /* FROM */
        lv_obj_t *from_cont = lv_obj_create(row);
        lv_obj_set_width(from_cont, LV_SIZE_CONTENT);
        lv_obj_set_height(from_cont, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(from_cont, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(from_cont,
                              LV_FLEX_ALIGN_START,
                              LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(from_cont, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(from_cont, 2, LV_PART_MAIN);
        lv_obj_set_style_border_width(from_cont, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(from_cont, LV_OPA_TRANSP, LV_PART_MAIN);


        lv_obj_t *btn_from_minus = lv_btn_create(from_cont);
        lv_obj_t *lbl_from_minus = lv_label_create(btn_from_minus);
        lv_label_set_text(lbl_from_minus, "-");
        lv_obj_center(lbl_from_minus);
        lv_obj_set_style_bg_color(btn_from_minus, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

        lv_obj_t *lbl_from_val = lv_label_create(from_cont);
        lv_label_set_text(lbl_from_val, "0 %");
        seq_from_labels[i] = lbl_from_val;

        lv_obj_t *btn_from_plus = lv_btn_create(from_cont);
        lv_obj_t *lbl_from_plus = lv_label_create(btn_from_plus);
        lv_label_set_text(lbl_from_plus, "+");
        lv_obj_center(lbl_from_plus);
        lv_obj_set_style_bg_color(btn_from_plus, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

        /* TO */
        lv_obj_t *to_cont = lv_obj_create(row);
        lv_obj_set_width(to_cont, LV_SIZE_CONTENT);
        lv_obj_set_height(to_cont, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(to_cont, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(to_cont,
                              LV_FLEX_ALIGN_START,
                              LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(to_cont, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(to_cont, 2, LV_PART_MAIN);
        lv_obj_set_style_border_width(to_cont, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(to_cont, LV_OPA_TRANSP, LV_PART_MAIN);

        lv_obj_t *lbl_to_title = lv_label_create(to_cont);
        lv_label_set_text(lbl_to_title, "--");

        lv_obj_t *btn_to_minus = lv_btn_create(to_cont);
        lv_obj_t *lbl_to_minus = lv_label_create(btn_to_minus);
        lv_label_set_text(lbl_to_minus, "-");
        lv_obj_center(lbl_to_minus);
        lv_obj_set_style_bg_color(btn_to_minus, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

        lv_obj_t *lbl_to_val = lv_label_create(to_cont);
        lv_label_set_text(lbl_to_val, "0 %");
        seq_to_labels[i] = lbl_to_val;

        lv_obj_t *btn_to_plus = lv_btn_create(to_cont);
        lv_obj_t *lbl_to_plus = lv_label_create(btn_to_plus);
        lv_label_set_text(lbl_to_plus, "+");
        lv_obj_center(lbl_to_plus);
        lv_obj_set_style_bg_color(btn_to_plus, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

        g_seq_btn_ctx[i][0][0].band_index = i;
        g_seq_btn_ctx[i][0][0].is_from    = 1;
        g_seq_btn_ctx[i][0][0].delta      = -1;

        g_seq_btn_ctx[i][0][1].band_index = i;
        g_seq_btn_ctx[i][0][1].is_from    = 1;
        g_seq_btn_ctx[i][0][1].delta      = +1;

        g_seq_btn_ctx[i][1][0].band_index = i;
        g_seq_btn_ctx[i][1][0].is_from    = 0;
        g_seq_btn_ctx[i][1][0].delta      = -1;

        g_seq_btn_ctx[i][1][1].band_index = i;
        g_seq_btn_ctx[i][1][1].is_from    = 0;
        g_seq_btn_ctx[i][1][1].delta      = +1;

        lv_obj_add_event_cb(btn_from_minus, on_seq_band_adjust, LV_EVENT_CLICKED,
                            &g_seq_btn_ctx[i][0][0]);
        lv_obj_add_event_cb(btn_from_plus,  on_seq_band_adjust, LV_EVENT_CLICKED,
                            &g_seq_btn_ctx[i][0][1]);
        lv_obj_add_event_cb(btn_to_minus,   on_seq_band_adjust, LV_EVENT_CLICKED,
                            &g_seq_btn_ctx[i][1][0]);
        lv_obj_add_event_cb(btn_to_plus,    on_seq_band_adjust, LV_EVENT_CLICKED,
                            &g_seq_btn_ctx[i][1][1]);
    }

    /* domyślnie widoczny Screen 1, Screen 2 ukryty */
    lv_obj_add_flag(dash_screen2_container, LV_OBJ_FLAG_HIDDEN);

    hvac_refresh_dashboard();
}

/* --- I/O Visualiser: przełączane pod-ekrany AI / AO --- */

/* callback: pokaż wejścia (AI) */
static void on_io_show_ai(lv_event_t *e)
{
    ARG_UNUSED(e);
    if (!io_ai_container || !io_ao_container) {
        return;
    }

    lv_obj_clear_flag(io_ai_container, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(io_ao_container, LV_OBJ_FLAG_HIDDEN);
}

/* callback: pokaż wyjścia (AO) */
static void on_io_show_ao(lv_event_t *e)
{
    ARG_UNUSED(e);
    if (!io_ai_container || !io_ao_container) {
        return;
    }

    lv_obj_clear_flag(io_ao_container, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(io_ai_container, LV_OBJ_FLAG_HIDDEN);
}

static void create_io_visualiser(lv_obj_t *parent)
{
    /* brak scrolla na ekranie głównym */
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    /* kontener na całą zawartość pod headerem */
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(80));
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(cont, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, LV_PART_MAIN);

    /* wiersz z przyciskami przełączającymi AI / AO */
    lv_obj_t *tab_row = lv_obj_create(cont);
    lv_obj_set_width(tab_row, LV_PCT(100));
    lv_obj_set_height(tab_row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(tab_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(tab_row,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(tab_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(tab_row, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(tab_row, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(tab_row, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_obj_t *btn_ai = lv_btn_create(tab_row);
    lv_obj_t *lbl_ai = lv_label_create(btn_ai);
    lv_label_set_text(lbl_ai, "Inputs (AI)");
    lv_obj_center(lbl_ai);
    lv_obj_add_event_cb(btn_ai, on_io_show_ai, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_ai, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *btn_ao = lv_btn_create(tab_row);
    lv_obj_t *lbl_ao = lv_label_create(btn_ao);
    lv_label_set_text(lbl_ao, "Outputs (AO)");
    lv_obj_center(lbl_ao);
    lv_obj_add_event_cb(btn_ao, on_io_show_ao, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn_ao, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

    /* --- pod-ekran: wejścia AI --- */
    io_ai_container = lv_obj_create(cont);
    lv_obj_set_width(io_ai_container, LV_PCT(100));
    lv_obj_set_height(io_ai_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(io_ai_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(io_ai_container,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(io_ai_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(io_ai_container, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(io_ai_container, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(io_ai_container, LV_OPA_TRANSP, LV_PART_MAIN);


    /* kontener na dwie kolumny AI */
    lv_obj_t *ai_cols = lv_obj_create(io_ai_container);
    lv_obj_set_width(ai_cols, LV_PCT(100));
    lv_obj_set_height(ai_cols, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(ai_cols, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ai_cols,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ai_cols, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(ai_cols, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(ai_cols, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ai_cols, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_obj_t *ai_col_left  = lv_obj_create(ai_cols);
    lv_obj_t *ai_col_right = lv_obj_create(ai_cols);

    lv_obj_set_width(ai_col_left,  LV_PCT(50));
    lv_obj_set_width(ai_col_right, LV_PCT(50));
    lv_obj_set_height(ai_col_left,  LV_SIZE_CONTENT);
    lv_obj_set_height(ai_col_right, LV_SIZE_CONTENT);

    lv_obj_set_flex_flow(ai_col_left,  LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_flow(ai_col_right, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ai_col_left,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_set_flex_align(ai_col_right,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ai_col_left,  LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(ai_col_right, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(ai_col_left,  0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(ai_col_right, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(ai_col_left,  0, LV_PART_MAIN);
    lv_obj_set_style_border_width(ai_col_right, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ai_col_left,  LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ai_col_right, LV_OPA_TRANSP, LV_PART_MAIN);

    /* AI1–AI4 -> lewa kolumna, AI5–AI8 -> prawa kolumna */
    for (int i = 0; i < HVAC_NUM_AI_CHANNELS; i++) {
        lv_obj_t *col = (i < 4) ? ai_col_left : ai_col_right;

        lv_obj_t *row = lv_obj_create(col);
        lv_obj_set_width(row, LV_PCT(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row,
                              LV_FLEX_ALIGN_START,
                              LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(row, 2, LV_PART_MAIN);
        lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, LV_PART_MAIN);

        char buf[16];

        snprintf(buf, sizeof(buf), "AI%d", i + 1);
        lv_obj_t *lbl_ai_ch = lv_label_create(row);
        lv_label_set_text(lbl_ai_ch, buf);

        lv_obj_t *lbl_ai_val = lv_label_create(row);
        lv_label_set_text(lbl_ai_val, "0.00");
        ai_value_labels[i] = lbl_ai_val;

        lv_obj_t *lbl_ai_unit = lv_label_create(row);
        lv_label_set_text(lbl_ai_unit, "V");
        ai_unit_labels[i] = lbl_ai_unit;

        const char *ai_role = hvac_ai_role_name_for_channel(i);
        lv_obj_t *lbl_ai_name = lv_label_create(row);
        lv_label_set_text(lbl_ai_name, ai_role);
        ai_name_labels[i] = lbl_ai_name;
    }

    /* --- pod-ekran: wyjścia AO --- */
    io_ao_container = lv_obj_create(cont);
    lv_obj_set_width(io_ao_container, LV_PCT(100));
    lv_obj_set_height(io_ao_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(io_ao_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(io_ao_container,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(io_ao_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(io_ao_container, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(io_ao_container, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(io_ao_container, LV_OPA_TRANSP, LV_PART_MAIN);


    /* kontener na dwie kolumny AO */
    lv_obj_t *ao_cols = lv_obj_create(io_ao_container);
    lv_obj_set_width(ao_cols, LV_PCT(100));
    lv_obj_set_height(ao_cols, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(ao_cols, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ao_cols,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ao_cols, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(ao_cols, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(ao_cols, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ao_cols, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_obj_t *ao_col_left  = lv_obj_create(ao_cols);
    lv_obj_t *ao_col_right = lv_obj_create(ao_cols);

    lv_obj_set_width(ao_col_left,  LV_PCT(50));
    lv_obj_set_width(ao_col_right, LV_PCT(50));
    lv_obj_set_height(ao_col_left,  LV_SIZE_CONTENT);
    lv_obj_set_height(ao_col_right, LV_SIZE_CONTENT);

    lv_obj_set_flex_flow(ao_col_left,  LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_flow(ao_col_right, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ao_col_left,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_set_flex_align(ao_col_right,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ao_col_left,  LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(ao_col_right, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(ao_col_left,  0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(ao_col_right, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(ao_col_left,  0, LV_PART_MAIN);
    lv_obj_set_style_border_width(ao_col_right, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ao_col_left,  LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ao_col_right, LV_OPA_TRANSP, LV_PART_MAIN);

    /* AO1–AO4 -> lewa kolumna, AO5–AO8 -> prawa kolumna */
    for (int i = 0; i < HVAC_NUM_AO_CHANNELS; i++) {
        lv_obj_t *col = (i < 4) ? ao_col_left : ao_col_right;

        lv_obj_t *row = lv_obj_create(col);
        lv_obj_set_width(row, LV_PCT(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row,
                              LV_FLEX_ALIGN_START,
                              LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(row, 2, LV_PART_MAIN);
        lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, LV_PART_MAIN);

        char buf[16];

        snprintf(buf, sizeof(buf), "AO%d", i + 1);
        lv_obj_t *lbl_ao_ch = lv_label_create(row);
        lv_label_set_text(lbl_ao_ch, buf);

        lv_obj_t *lbl_ao_val = lv_label_create(row);
        lv_label_set_text(lbl_ao_val, "0.00");
        ao_value_labels[i] = lbl_ao_val;

        lv_obj_t *lbl_ao_unit = lv_label_create(row);
        lv_label_set_text(lbl_ao_unit, "V");
        ao_unit_labels[i] = lbl_ao_unit;

        const char *ao_role = hvac_ao_role_name_for_channel(i);
        lv_obj_t *lbl_ao_name = lv_label_create(row);
        lv_label_set_text(lbl_ao_name, ao_role);
        ao_name_labels[i] = lbl_ao_name;
    }

    /* domyślnie pokazujemy wejścia, wyjścia ukryte */
    lv_obj_add_flag(io_ao_container, LV_OBJ_FLAG_HIDDEN);
}

/* --- Ekran I/O --- */

static void create_io_screen(void)
{
    screen_io = lv_obj_create(NULL);
    lv_obj_clear_flag(screen_io, LV_OBJ_FLAG_SCROLLABLE);
    create_header(screen_io, "I/O Visualiser");

    create_io_visualiser(screen_io);

    hvac_refresh_io_role_labels();
}

/* --- Ekran Config --- */

static void create_config_screen(void)
{
    screen_config = lv_obj_create(NULL);
    lv_obj_clear_flag(screen_config, LV_OBJ_FLAG_SCROLLABLE);
    create_header(screen_config, "Configuration Loader");

    lv_obj_t *cont = lv_obj_create(screen_config);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(80));
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(cont, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_obj_t *btn1 = lv_btn_create(cont);
    lv_obj_t *lbl1 = lv_label_create(btn1);
    lv_label_set_text(lbl1, "Load Config 1");
    lv_obj_center(lbl1);
    lv_obj_add_event_cb(btn1, on_btn_load_cfg1, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn1, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t *btn2 = lv_btn_create(cont);
    lv_obj_t *lbl2 = lv_label_create(btn2);
    lv_label_set_text(lbl2, "Load Config 2");
    lv_obj_center(lbl2);
    lv_obj_add_event_cb(btn2, on_btn_load_cfg2, LV_EVENT_CLICKED, NULL);
    lv_obj_set_style_bg_color(btn2, button_color, LV_PART_MAIN | LV_STATE_DEFAULT);

    config_status_label = lv_label_create(cont);
    lv_label_set_text(config_status_label, "No config loaded");
}

/* --- Ekran Sequence Viewer --- */

static void create_seq_viewer_screen(void)
{
    screen_seq_viewer = lv_obj_create(NULL);
    lv_obj_clear_flag(screen_seq_viewer, LV_OBJ_FLAG_SCROLLABLE);
    create_header(screen_seq_viewer, "Sequence Viewer");

    lv_obj_t *cont = lv_obj_create(screen_seq_viewer);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(80));
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(cont, 8, LV_PART_MAIN);
    lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_obj_t *lbl = lv_label_create(cont);
    lv_label_set_text(lbl, "Current sequence");

    seq_viewer_image = lv_img_create(cont);
    lv_obj_center(seq_viewer_image);

    hvac_refresh_sequence_viewer();
}

/* --- Nawigacja ekranów --- */

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

static void nav_to_seq_viewer(lv_event_t *e)
{
    ARG_UNUSED(e);
    if (screen_seq_viewer != NULL) {
        lv_scr_load(screen_seq_viewer);
    }
}

/* --- JSON loader --- */
/* --- JSON loader --- */

static int hvac_load_config_from_json(const char *json_src, size_t len,
                                      struct hvac_config *out_cfg)
{
    char buf[612];

    /* Ze sizeof() masz zwykle wliczone końcowe '\0', obetnij je */
    if (len > 0 && json_src[len - 1] == '\0') {
        len--;
    }

    if (len >= sizeof(buf)) {
        LOG_ERR("JSON too big for local buffer (%u bytes)", (unsigned)len);
        return -ENOMEM;
    }

    memcpy(buf, json_src, len);
    buf[len] = '\0';

    memset(out_cfg, 0, sizeof(*out_cfg));

    int ret = json_obj_parse(buf, len,
                             hvac_cfg_descr,
                             ARRAY_SIZE(hvac_cfg_descr),
                             out_cfg);

    if (ret < 0) {
        LOG_ERR("JSON parse error: %d", ret);
        return ret;
    }

    LOG_INF("Loaded config: setpoint=%d, kp=%d, ki=%d, kd=%d, "
            "cool=[%d,%d], db=[%d,%d], heat=[%d,%d], HR=[%d,%d]",
            out_cfg->setpoint,
            out_cfg->pid.kp,
            out_cfg->pid.ki,
            out_cfg->pid.kd,
            out_cfg->seq.cooling.from_percent,
            out_cfg->seq.cooling.to_percent,
            out_cfg->seq.deadband.from_percent,
            out_cfg->seq.deadband.to_percent,
            out_cfg->seq.heating.from_percent,
            out_cfg->seq.heating.to_percent,
            out_cfg->seq.heat_recovery.from_percent,
            out_cfg->seq.heat_recovery.to_percent);

    return 0;
}



/* --- Regulator PI + sekwencje --- */

static void hvac_pid_reset(struct hvac_pid_state *st)
{
    memset(st, 0, sizeof(*st));
}

static float hvac_pid_step(const struct hvac_pid_cfg *cfg,
                           struct hvac_pid_state *st,
                           float error,
                           float dt_sec)
{
    float kp = (float)cfg->kp;
    float ki = (float)cfg->ki;

    float p = kp * error;

    st->i_term += ki * error * dt_sec;

    if (st->i_term > 100.0f)  st->i_term = 100.0f;
    if (st->i_term < -100.0f) st->i_term = -100.0f;

    float out = p + st->i_term;

    if (out > 100.0f)  out = 100.0f;
    if (out < -100.0f) out = -100.0f;

    return out;
}

static void hvac_apply_sequence(float pid_out_pct,
                                const struct hvac_config *cfg,
                                float *heater_pct,
                                float *cooler_pct,
                                float *bypass_pct)
{
    const struct hvac_seq_cfg *seq = &cfg->seq;
    float u = pid_out_pct;

    *heater_pct = 0.0f;
    *cooler_pct = 0.0f;
    *bypass_pct = 0.0f;

    const struct hvac_seq_band *db = &seq->deadband;
    if (db->from_percent != db->to_percent &&
        u >= db->from_percent && u <= db->to_percent) {
        return;
    }

    const struct hvac_seq_band *heat = &seq->heating;
    if (heat->from_percent != heat->to_percent &&
        u >= heat->from_percent && u <= heat->to_percent) {

        float span = (float)(heat->to_percent - heat->from_percent);
        if (span < 1.0f) span = 1.0f;
        float rel = (u - heat->from_percent) / span;
        if (rel < 0.0f) rel = 0.0f;
        if (rel > 1.0f) rel = 1.0f;
        *heater_pct = rel * 100.0f;
        return;
    }

    const struct hvac_seq_band *cool = &seq->cooling;
    if (cool->from_percent != cool->to_percent &&
        u >= cool->from_percent && u <= cool->to_percent) {

        float span = (float)(cool->from_percent - cool->to_percent);
        if (span < 1.0f) span = 1.0f;
        float rel = (cool->to_percent - u) / span;
        if (rel < 0.0f) rel = 0.0f;
        if (rel > 1.0f) rel = 1.0f;
        *cooler_pct = rel * 100.0f;
        return;
    }

    const struct hvac_seq_band *hr = &seq->heat_recovery;
    if (hr->from_percent != hr->to_percent &&
        u >= hr->from_percent && u <= hr->to_percent) {

        float max_abs = ABSF((float)hr->from_percent);
        if (ABSF((float)hr->to_percent) > max_abs) {
            max_abs = ABSF((float)hr->to_percent);
        }
        if (max_abs < 1.0f) {
            max_abs = 1.0f;
        }

        float mag = ABSF(u);
        float rel = mag / max_abs;
        if (rel < 0.0f) rel = 0.0f;
        if (rel > 1.0f) rel = 1.0f;
        *bypass_pct = rel * 100.0f;
        return;
    }
}

static float hvac_get_extract_temp_c(void)
{
    int ch = g_hvac_cfg.io.t_extract_ai;
    if (ch < 0 || ch >= HVAC_NUM_AI_CHANNELS) {
        return 0.0f;
    }

    float v = read_ai_voltage(ch);

    /* placeholder: 1 V = 5 C */
    return v * 5.0f;
}

static void hvac_control_step(float dt_sec)
{
    float t_extract = hvac_get_extract_temp_c();
    float sp = (float)g_hvac_cfg.setpoint;
    float error = sp - t_extract;

    float u_pct = hvac_pid_step(&g_hvac_cfg.pid,
                                &g_hvac_pid_state,
                                error,
                                dt_sec);

    float heater_pct = 0.0f;
    float cooler_pct = 0.0f;
    float bypass_pct = 0.0f;

    hvac_apply_sequence(u_pct, &g_hvac_cfg,
                        &heater_pct, &cooler_pct, &bypass_pct);

    const struct hvac_io_cfg *io = &g_hvac_cfg.io;

    if (io->heater_ao >= 0 && io->heater_ao < HVAC_NUM_AO_CHANNELS) {
        float v = (heater_pct / 100.0f) * 10.0f;
        if (v < 0.0f) v = 0.0f;
        if (v > 10.0f) v = 10.0f;
        write_ao_voltage(io->heater_ao, v);
    }

    if (io->cooler_ao >= 0 && io->cooler_ao < HVAC_NUM_AO_CHANNELS) {
        float v = (cooler_pct / 100.0f) * 10.0f;
        if (v < 0.0f) v = 0.0f;
        if (v > 10.0f) v = 10.0f;
        write_ao_voltage(io->cooler_ao, v);
    }

    if (io->bypass_ao >= 0 && io->bypass_ao < HVAC_NUM_AO_CHANNELS) {
        float v = (bypass_pct / 100.0f) * 10.0f;
        if (v < 0.0f) v = 0.0f;
        if (v > 10.0f) v = 10.0f;
        write_ao_voltage(io->bypass_ao, v);
    }

    if (io->fan_vfd_ao >= 0 && io->fan_vfd_ao < HVAC_NUM_AO_CHANNELS) {
        float any_pct = heater_pct;
        if (cooler_pct > any_pct) any_pct = cooler_pct;
        if (bypass_pct > any_pct) any_pct = bypass_pct;

        float v = (any_pct > 0.0f) ? 10.0f : 0.0f;
        write_ao_voltage(io->fan_vfd_ao, v);
    }
}

#define HVAC_CTRL_STACK_SIZE 2048
#define HVAC_CTRL_PRIORITY   5

static void hvac_control_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    hvac_pid_reset(&g_hvac_pid_state);

    int64_t last_ctrl_ms = k_uptime_get();

    while (1) {
        int64_t now_ms  = k_uptime_get();
        int64_t diff_ms = now_ms - last_ctrl_ms;

        if (diff_ms >= 100) {
            float dt_sec = diff_ms / 1000.0f;
            hvac_control_step(dt_sec);
            last_ctrl_ms = now_ms;
        }

        k_msleep(10);
    }
}

K_THREAD_DEFINE(hvac_ctrl_thread_id,
                HVAC_CTRL_STACK_SIZE,
                hvac_control_thread,
                NULL, NULL, NULL,
                HVAC_CTRL_PRIORITY, 0, 0);

/* --- Zastosowanie konfiguracji --- */

static void hvac_apply_config(const struct hvac_config *cfg)
{
    g_hvac_cfg = *cfg;

    hvac_pid_reset(&g_hvac_pid_state);
    hvac_refresh_io_role_labels();
    hvac_refresh_dashboard();
    hvac_refresh_sequence_viewer();
}

/* --- Callbacki ładowania configu --- */

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
        tmp.sequence_type = "cool_dead_heat";  /* ręcznie ustawiamy typ sekwencji */
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
        tmp.sequence_type = "cool_rec_dead_rec_heat";  /* tutaj drugi typ */
        hvac_apply_config(&tmp);
        lv_label_set_text(config_status_label, "Loaded Config 2");
    } else {
        lv_label_set_text(config_status_label, "Error loading Config 2");
    }
}

/* --- main() --- */

int main(void)
{
    LOG_INF("Starting LVGL UI with JSON config loader + HVAC PI controller (placeholders IO)");

    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready");
        return 0;
    }

    create_dashboard_screen();
    create_io_screen();
    create_config_screen();
    create_seq_viewer_screen();

    lv_scr_load(screen_dashboard);

    display_blanking_off(display_dev);

    int64_t last_io_update_ms = k_uptime_get();

    while (1) {
        lv_timer_handler();

        int64_t now_ms  = k_uptime_get();
        int64_t diff_ms = now_ms - last_io_update_ms;

        if (diff_ms >= 1000) {
            hvac_update_io_values();
            last_io_update_ms = now_ms;
        }

        k_msleep(5);
    }

    return 0;
}
