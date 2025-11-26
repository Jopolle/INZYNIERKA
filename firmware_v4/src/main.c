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

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define HVAC_NUM_AI_CHANNELS 8
#define HVAC_NUM_AO_CHANNELS 8


#define ABSF(x) ((x) < 0.0f ? -(x) : (x))





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





static const struct device *display_dev;

static lv_obj_t *screen_dashboard;
static lv_obj_t *screen_io;
static lv_obj_t *screen_config;

static lv_obj_t *config_status_label;

static lv_obj_t *ai_value_labels[HVAC_NUM_AI_CHANNELS];
static lv_obj_t *ai_unit_labels[HVAC_NUM_AI_CHANNELS];
static lv_obj_t *ai_name_labels[HVAC_NUM_AI_CHANNELS];

static lv_obj_t *ao_value_labels[HVAC_NUM_AO_CHANNELS];
static lv_obj_t *ao_unit_labels[HVAC_NUM_AO_CHANNELS];
static lv_obj_t *ao_name_labels[HVAC_NUM_AO_CHANNELS];


static lv_obj_t *setpoint_label;

static lv_obj_t *seq_rows[HVAC_SEQ_IDX_COUNT];
static lv_obj_t *seq_from_labels[HVAC_SEQ_IDX_COUNT];
static lv_obj_t *seq_to_labels[HVAC_SEQ_IDX_COUNT];

struct seq_btn_ctx {
    uint8_t band_index;  
    uint8_t is_from;     
    int8_t  delta;       
};


static struct seq_btn_ctx g_seq_btn_ctx[HVAC_SEQ_IDX_COUNT][2][2];





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
};

static struct hvac_pid_state g_hvac_pid_state;





static void nav_to_dashboard(lv_event_t *e);
static void nav_to_io(lv_event_t *e);
static void nav_to_config(lv_event_t *e);

static int  hvac_load_config_from_json(char *json, size_t len,
                                       struct hvac_config *out_cfg);
static void hvac_apply_config(const struct hvac_config *cfg);

static void on_btn_load_cfg1(lv_event_t *e);
static void on_btn_load_cfg2(lv_event_t *e);

static void on_btn_setpoint_minus(lv_event_t *e);
static void on_btn_setpoint_plus(lv_event_t *e);
static void on_seq_band_adjust(lv_event_t *e);

static void hvac_refresh_dashboard(void);
static void hvac_refresh_dashboard_setpoint(void);
static void hvac_refresh_dashboard_seq_labels(void);

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
};


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





static void hvac_refresh_dashboard_setpoint(void)
{
    if (!setpoint_label) {
        return;
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "Setpoint: %d C", g_hvac_cfg.setpoint);
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

    struct hvac_seq_band *band = hvac_get_seq_band_by_index(&g_hvac_cfg.seq, ctx->band_index);
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





static void create_dashboard_screen(void)
{
    screen_dashboard = lv_obj_create(NULL);
    lv_obj_clear_flag(screen_dashboard, LV_OBJ_FLAG_SCROLLABLE);
    create_header(screen_dashboard, "Dashboard");

    lv_obj_t *cont = lv_obj_create(screen_dashboard);
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(80));
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(cont, 8, LV_PART_MAIN);
    lv_obj_set_style_border_width(cont, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, LV_PART_MAIN);

    

    lv_obj_t *row_sp = lv_obj_create(cont);
    lv_obj_set_width(row_sp, LV_PCT(100));
    lv_obj_set_height(row_sp, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row_sp, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row_sp,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(row_sp, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(row_sp, 4, LV_PART_MAIN);
    lv_obj_set_style_border_width(row_sp, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row_sp, LV_OPA_TRANSP, LV_PART_MAIN);

    lv_obj_t *lbl_sp_title = lv_label_create(row_sp);
    lv_label_set_text(lbl_sp_title, "Temperature setpoint:");

    setpoint_label = lv_label_create(row_sp);
    lv_label_set_text(setpoint_label, "Setpoint: 0 C");

    lv_obj_t *btn_sp_minus = lv_btn_create(row_sp);
    lv_obj_t *lbl_sp_minus = lv_label_create(btn_sp_minus);
    lv_label_set_text(lbl_sp_minus, "-");
    lv_obj_center(lbl_sp_minus);
    lv_obj_add_event_cb(btn_sp_minus, on_btn_setpoint_minus, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_sp_plus = lv_btn_create(row_sp);
    lv_obj_t *lbl_sp_plus = lv_label_create(btn_sp_plus);
    lv_label_set_text(lbl_sp_plus, "+");
    lv_obj_center(lbl_sp_plus);
    lv_obj_add_event_cb(btn_sp_plus, on_btn_setpoint_plus, LV_EVENT_CLICKED, NULL);

    

    for (int i = 0; i < HVAC_SEQ_IDX_COUNT; i++) {
        lv_obj_t *row = lv_obj_create(cont);
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

        
        lv_obj_t *lbl_band = lv_label_create(row);
        lv_label_set_text(lbl_band, hvac_seq_band_names[i]);

        
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

        lv_obj_t *lbl_from_title = lv_label_create(from_cont);
        lv_label_set_text(lbl_from_title, "From");

        lv_obj_t *btn_from_minus = lv_btn_create(from_cont);
        lv_obj_t *lbl_from_minus = lv_label_create(btn_from_minus);
        lv_label_set_text(lbl_from_minus, "-");
        lv_obj_center(lbl_from_minus);

        lv_obj_t *lbl_from_val = lv_label_create(from_cont);
        lv_label_set_text(lbl_from_val, "0 %");
        seq_from_labels[i] = lbl_from_val;

        lv_obj_t *btn_from_plus = lv_btn_create(from_cont);
        lv_obj_t *lbl_from_plus = lv_label_create(btn_from_plus);
        lv_label_set_text(lbl_from_plus, "+");
        lv_obj_center(lbl_from_plus);

        
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
        lv_label_set_text(lbl_to_title, "To");

        lv_obj_t *btn_to_minus = lv_btn_create(to_cont);
        lv_obj_t *lbl_to_minus = lv_label_create(btn_to_minus);
        lv_label_set_text(lbl_to_minus, "-");
        lv_obj_center(lbl_to_minus);

        lv_obj_t *lbl_to_val = lv_label_create(to_cont);
        lv_label_set_text(lbl_to_val, "0 %");
        seq_to_labels[i] = lbl_to_val;

        lv_obj_t *btn_to_plus = lv_btn_create(to_cont);
        lv_obj_t *lbl_to_plus = lv_label_create(btn_to_plus);
        lv_label_set_text(lbl_to_plus, "+");
        lv_obj_center(lbl_to_plus);

        
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

    
    hvac_refresh_dashboard();
}





static void create_io_visualiser(lv_obj_t *parent)
{
    
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    
    lv_obj_t *scroll = lv_obj_create(parent);
    lv_obj_set_size(scroll, LV_PCT(100), LV_PCT(80));
    lv_obj_align(scroll, LV_ALIGN_BOTTOM_MID, 0, 0);

    
    lv_obj_add_flag(scroll, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(scroll, LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM);
    lv_obj_clear_flag(scroll, LV_OBJ_FLAG_SCROLL_CHAIN_HOR | LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_set_scroll_dir(scroll, LV_DIR_VER);

    
    lv_obj_set_scrollbar_mode(scroll, LV_SCROLLBAR_MODE_AUTO);

    
    lv_obj_set_flex_flow(scroll, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(scroll,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);

    
    lv_obj_t *title = lv_label_create(scroll);
    lv_label_set_text(title, "Analog I/O (AI1–AI8 / AO1–AO8)");

    
    for (int i = 0; i < HVAC_NUM_AI_CHANNELS; i++) {
        lv_obj_t *row = lv_obj_create(scroll);
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

        
        lv_obj_t *ai_cell = lv_obj_create(row);
        lv_obj_set_width(ai_cell, LV_PCT(50));
        lv_obj_set_height(ai_cell, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(ai_cell, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(ai_cell,
                              LV_FLEX_ALIGN_START,
                              LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(ai_cell, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(ai_cell, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(ai_cell, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(ai_cell, LV_OPA_TRANSP, LV_PART_MAIN);

        char buf[16];

        
        snprintf(buf, sizeof(buf), "AI%d", i + 1);
        lv_obj_t *lbl_ai_ch = lv_label_create(ai_cell);
        lv_label_set_text(lbl_ai_ch, buf);

        
        lv_obj_t *lbl_ai_val = lv_label_create(ai_cell);
        lv_label_set_text(lbl_ai_val, "0.00");
        ai_value_labels[i] = lbl_ai_val;

        
        lv_obj_t *lbl_ai_unit = lv_label_create(ai_cell);
        lv_label_set_text(lbl_ai_unit, "V");
        ai_unit_labels[i] = lbl_ai_unit;

        
        const char *ai_role = hvac_ai_role_name_for_channel(i);
        lv_obj_t *lbl_ai_name = lv_label_create(ai_cell);
        lv_label_set_text(lbl_ai_name, ai_role);
        ai_name_labels[i] = lbl_ai_name;

        
        lv_obj_t *ao_cell = lv_obj_create(row);
        lv_obj_set_width(ao_cell, LV_PCT(50));
        lv_obj_set_height(ao_cell, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(ao_cell, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(ao_cell,
                              LV_FLEX_ALIGN_START,
                              LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(ao_cell, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(ao_cell, 0, LV_PART_MAIN);
        lv_obj_set_style_border_width(ao_cell, 0, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(ao_cell, LV_OPA_TRANSP, LV_PART_MAIN);

        
        snprintf(buf, sizeof(buf), "AO%d", i + 1);
        lv_obj_t *lbl_ao_ch = lv_label_create(ao_cell);
        lv_label_set_text(lbl_ao_ch, buf);

        
        lv_obj_t *lbl_ao_val = lv_label_create(ao_cell);
        lv_label_set_text(lbl_ao_val, "0.00");
        ao_value_labels[i] = lbl_ao_val;

        
        lv_obj_t *lbl_ao_unit = lv_label_create(ao_cell);
        lv_label_set_text(lbl_ao_unit, "V");
        ao_unit_labels[i] = lbl_ao_unit;

        
        const char *ao_role = hvac_ao_role_name_for_channel(i);
        lv_obj_t *lbl_ao_name = lv_label_create(ao_cell);
        lv_label_set_text(lbl_ao_name, ao_role);
        ao_name_labels[i] = lbl_ao_name;
    }
}

static void create_io_screen(void)
{
    screen_io = lv_obj_create(NULL);
    lv_obj_clear_flag(screen_io, LV_OBJ_FLAG_SCROLLABLE);  
    create_header(screen_io, "I/O Visualiser");

    create_io_visualiser(screen_io);

    
    hvac_refresh_io_role_labels();
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
    
    memset(out_cfg, 0, sizeof(*out_cfg));

    int ret = json_obj_parse(json, len,
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





static void hvac_apply_config(const struct hvac_config *cfg)
{
    g_hvac_cfg = *cfg; 

    hvac_pid_reset(&g_hvac_pid_state);
    hvac_refresh_io_role_labels();
    hvac_refresh_dashboard();
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
    LOG_INF("Starting LVGL UI with JSON config loader + HVAC PI controller (placeholders IO)");

    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready");
        return 0;
    }

    hvac_pid_reset(&g_hvac_pid_state);

    create_dashboard_screen();
    create_io_screen();
    create_config_screen();

    lv_scr_load(screen_dashboard);

    display_blanking_off(display_dev);

    int64_t last_ctrl_ms = k_uptime_get();

    while (1) {
        lv_timer_handler();
        k_msleep(10);

        int64_t now_ms = k_uptime_get();
        int64_t diff_ms = now_ms - last_ctrl_ms;

        
        if (diff_ms >= 100) {
            float dt_sec = diff_ms / 1000.0f;
            hvac_control_step(dt_sec);
            last_ctrl_ms = now_ms;
        }

        
        // hvac_update_io_values();
    }

    return 0;
}
