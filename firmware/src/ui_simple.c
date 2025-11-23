#include "ui_simple.h"
#include <lvgl.h>
#include <stdio.h>


typedef struct
{
    lv_obj_t *slider;
    lv_obj_t *value_label;
} pid_param_ctrl_t;

typedef struct
{
    pid_param_ctrl_t *param;
    int32_t delta;
} fine_tune_ctx_t;

typedef struct
{
    lv_obj_t *label;
    bool state;
} io_indicator_t;

#define UI_NUM_INPUTS   4
#define UI_NUM_OUTPUTS  4

static lv_obj_t *dashboard_screen;
static lv_obj_t *menu_screen;

static lv_obj_t *zone_screen;
static lv_obj_t *supply_screen;
static lv_obj_t *schedule_screen;
static lv_obj_t *sequence_screen;
static lv_obj_t *protection_screen;
static lv_obj_t *visualiser_screen;
static lv_obj_t *pid_tuner_screen;
static lv_obj_t *io_selector_screen;
static lv_obj_t *diag_screen;


static pid_param_ctrl_t p_ctrl;
static pid_param_ctrl_t i_ctrl;
static pid_param_ctrl_t d_ctrl;

static fine_tune_ctx_t p_minus_ctx;
static fine_tune_ctx_t p_plus_ctx;
static fine_tune_ctx_t i_minus_ctx;
static fine_tune_ctx_t i_plus_ctx;
static fine_tune_ctx_t d_minus_ctx;
static fine_tune_ctx_t d_plus_ctx;

static int pid_row_index = 0;


static lv_obj_t *temp_value_label;
static int16_t temp_setpoint = 21; /* domyślna zadana temperatura */

static io_indicator_t input_indicators[UI_NUM_INPUTS];
static io_indicator_t output_indicators[UI_NUM_OUTPUTS];


static void load_screen_event_cb(lv_event_t *e);
static lv_obj_t *create_sub_screen(lv_obj_t *return_screen, const char *back_text);

static void update_value_label(pid_param_ctrl_t *ctrl);
static void slider_value_changed_cb(lv_event_t *e);
static void fine_tune_btn_event_cb(lv_event_t *e);
static void create_pid_param_row(lv_obj_t *parent,
                                 const char *name,
                                 pid_param_ctrl_t *ctrl,
                                 fine_tune_ctx_t *minus_ctx,
                                 fine_tune_ctx_t *plus_ctx);
static void build_pid_tuner_screen(void);

static void temp_update_label(void);
static void temp_minus_event_cb(lv_event_t *e);
static void temp_plus_event_cb(lv_event_t *e);

static void update_io_indicator(io_indicator_t *ind, const char *prefix);
static void build_dashboard_screen(void);
static void build_menu_screen(void);
static void build_zone_screen(void);
static void build_supply_screen(void);
static void build_schedule_screen(void);
static void build_sequence_screen(void);
static void build_protection_screen(void);
static void build_io_screen(void);
static void build_diag_screen(void);

static void load_screen_event_cb(lv_event_t *e)
{
    lv_obj_t *target = lv_event_get_user_data(e);

    if (target != NULL)
    {
        lv_scr_load(target);
    }
}

static void update_value_label(pid_param_ctrl_t *ctrl)
{
    if (ctrl == NULL || ctrl->slider == NULL || ctrl->value_label == NULL)
    {
        return;
    }

    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d", lv_slider_get_value(ctrl->slider));
    lv_label_set_text(ctrl->value_label, buf);
}

static void slider_value_changed_cb(lv_event_t *e)
{
    pid_param_ctrl_t *ctrl = lv_event_get_user_data(e);

    if (ctrl != NULL)
    {
        update_value_label(ctrl);
    }
}

static void fine_tune_btn_event_cb(lv_event_t *e)
{
    fine_tune_ctx_t *ctx = lv_event_get_user_data(e);

    if (ctx == NULL || ctx->param == NULL || ctx->param->slider == NULL)
    {
        return;
    }

    int32_t value = lv_slider_get_value(ctx->param->slider);
    int32_t min_value = lv_slider_get_min_value(ctx->param->slider);
    int32_t max_value = lv_slider_get_max_value(ctx->param->slider);

    value += ctx->delta;
    if (value < min_value)
    {
        value = min_value;
    }
    if (value > max_value)
    {
        value = max_value;
    }

    lv_slider_set_value(ctx->param->slider, value, LV_ANIM_OFF);
    update_value_label(ctx->param);
}

static lv_obj_t *create_sub_screen(lv_obj_t *return_screen, const char *back_text)
{
    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_set_scroll_dir(screen, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(screen, LV_SCROLLBAR_MODE_AUTO);

    lv_obj_t *back_btn = lv_btn_create(screen);
    lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_add_event_cb(back_btn, load_screen_event_cb, LV_EVENT_CLICKED, return_screen);

    lv_obj_t *label = lv_label_create(back_btn);
    lv_label_set_text(label, back_text);
    lv_obj_center(label);

    return screen;
}


static void create_pid_param_row(lv_obj_t *parent,
                                 const char *name,
                                 pid_param_ctrl_t *ctrl,
                                 fine_tune_ctx_t *minus_ctx,
                                 fine_tune_ctx_t *plus_ctx)
{
    const lv_coord_t row_height = 70; 
    lv_coord_t base_y = 40 + pid_row_index * row_height;
    pid_row_index++;

    lv_obj_t *name_label = lv_label_create(parent);
    lv_label_set_text(name_label, name);
    lv_obj_align(name_label, LV_ALIGN_TOP_LEFT, 0, base_y);

    ctrl->value_label = lv_label_create(parent);
    lv_label_set_text(ctrl->value_label, "0");
    lv_obj_align(ctrl->value_label, LV_ALIGN_TOP_RIGHT, 0, base_y);

    ctrl->slider = lv_slider_create(parent);
    lv_obj_set_width(ctrl->slider, lv_pct(100));
    lv_obj_set_height(ctrl->slider, 25);
    lv_slider_set_range(ctrl->slider, 0, 100);
    lv_slider_set_value(ctrl->slider, 0, LV_ANIM_OFF);
    lv_obj_align(ctrl->slider, LV_ALIGN_TOP_MID, 0, base_y + 20);
    lv_obj_add_event_cb(ctrl->slider, slider_value_changed_cb,
                        LV_EVENT_VALUE_CHANGED, ctrl);

    minus_ctx->param = ctrl;
    minus_ctx->delta = -1;
    plus_ctx->param = ctrl;
    plus_ctx->delta = 1;
    lv_obj_t *minus_btn = lv_btn_create(parent);
    lv_obj_add_event_cb(minus_btn, fine_tune_btn_event_cb,
                        LV_EVENT_CLICKED, minus_ctx);
    lv_obj_t *minus_label = lv_label_create(minus_btn);
    lv_label_set_text(minus_label, "- Fine");
    lv_obj_center(minus_label);
    lv_obj_align(minus_btn, LV_ALIGN_TOP_LEFT, 0, base_y + 45);

    lv_obj_t *plus_btn = lv_btn_create(parent);
    lv_obj_add_event_cb(plus_btn, fine_tune_btn_event_cb,
                        LV_EVENT_CLICKED, plus_ctx);
    lv_obj_t *plus_label = lv_label_create(plus_btn);
    lv_label_set_text(plus_label, "+ Fine");
    lv_obj_center(plus_label);
    lv_obj_align(plus_btn, LV_ALIGN_TOP_RIGHT, 0, base_y + 45);
}

static void build_pid_tuner_screen(void)
{
    lv_obj_t *content = lv_obj_create(pid_tuner_screen);
    lv_obj_remove_style_all(content);
    lv_obj_set_width(content, lv_pct(100));
    lv_obj_set_height(content, LV_SIZE_CONTENT);
    lv_obj_align(content, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_pad_all(content, 20, LV_PART_MAIN);
    lv_obj_set_style_pad_row(content, 20, LV_PART_MAIN);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_START);

    lv_obj_t *title_label = lv_label_create(content);
    lv_label_set_text(title_label, "PID Tuner");

    create_pid_param_row(content, "P", &p_ctrl, &p_minus_ctx, &p_plus_ctx);
    create_pid_param_row(content, "I", &i_ctrl, &i_minus_ctx, &i_plus_ctx);
    create_pid_param_row(content, "D", &d_ctrl, &d_minus_ctx, &d_plus_ctx);
}


static void temp_update_label(void)
{
    if (temp_value_label == NULL)
    {
        return;
    }

    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d°C", temp_setpoint);
    lv_label_set_text(temp_value_label, buf);
}

static void temp_minus_event_cb(lv_event_t *e)
{
    (void)e;

    ui_set_temp_setpoint(temp_setpoint - 1);
}

static void temp_plus_event_cb(lv_event_t *e)
{
    (void)e;

    ui_set_temp_setpoint(temp_setpoint + 1);
}

static void update_io_indicator(io_indicator_t *ind, const char *prefix)
{
    if (ind == NULL || ind->label == NULL)
    {
        return;
    }

    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%s %s", prefix, ind->state ? "ON" : "OFF");
    lv_label_set_text(ind->label, buf);
    lv_obj_t *box = lv_obj_get_parent(ind->label);
    if (box != NULL)
    {
        lv_color_t col = ind->state
                         ? lv_palette_main(LV_PALETTE_GREEN)
                         : lv_palette_main(LV_PALETTE_GREY);

        lv_obj_set_style_bg_color(box, col, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(box, LV_OPA_COVER, LV_PART_MAIN);
    }
}

static void build_dashboard_screen(void)
{
    lv_obj_t *title = lv_label_create(dashboard_screen);
    lv_label_set_text(title, "HVAC Dashboard");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    /* Przycisk do menu w prawym górnym rogu */
    lv_obj_t *menu_btn = lv_btn_create(dashboard_screen);
    lv_obj_align(menu_btn, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_add_event_cb(menu_btn, load_screen_event_cb,
                        LV_EVENT_CLICKED, menu_screen);

    lv_obj_t *menu_label = lv_label_create(menu_btn);
    lv_label_set_text(menu_label, "Menu");
    lv_obj_center(menu_label);

    lv_obj_t *temp_cont = lv_obj_create(dashboard_screen);
    lv_obj_set_width(temp_cont, lv_pct(100));
    lv_obj_set_height(temp_cont, 70);
    lv_obj_align(temp_cont, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_set_style_pad_all(temp_cont, 10, LV_PART_MAIN);

    lv_obj_t *temp_title = lv_label_create(temp_cont);
    lv_label_set_text(temp_title, "T_set");
    lv_obj_align(temp_title, LV_ALIGN_LEFT_MID, 10, 0);

    temp_value_label = lv_label_create(temp_cont);
    lv_obj_align(temp_value_label, LV_ALIGN_CENTER, 0, 0);
    temp_update_label();

    lv_obj_t *minus_btn = lv_btn_create(temp_cont);
    lv_obj_align(minus_btn, LV_ALIGN_LEFT_MID, 70, 0);
    lv_obj_add_event_cb(minus_btn, temp_minus_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *minus_label = lv_label_create(minus_btn);
    lv_label_set_text(minus_label, "-");
    lv_obj_center(minus_label);

    lv_obj_t *plus_btn = lv_btn_create(temp_cont);
    lv_obj_align(plus_btn, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_add_event_cb(plus_btn, temp_plus_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *plus_label = lv_label_create(plus_btn);
    lv_label_set_text(plus_label, "+");
    lv_obj_center(plus_label);

    lv_obj_t *io_cont = lv_obj_create(dashboard_screen);
    lv_obj_set_width(io_cont, lv_pct(100));
    lv_obj_set_height(io_cont, 150);
    lv_obj_align(io_cont, LV_ALIGN_BOTTOM_MID, 0, -10);

    lv_obj_set_layout(io_cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(io_cont, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_all(io_cont, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_row(io_cont, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_column(io_cont, 10, LV_PART_MAIN);
    lv_obj_set_flex_align(io_cont,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);

    /* Pierwszy wiersz – nagłówki IN/OUT */
    lv_obj_t *in_header = lv_label_create(io_cont);
    lv_label_set_text(in_header, "Inputs");
    lv_obj_t *out_header = lv_label_create(io_cont);
    lv_label_set_text(out_header, "Outputs");

    for (uint8_t i = 0; i < UI_NUM_INPUTS; ++i)
    {
        lv_obj_t *box = lv_obj_create(io_cont);
        lv_obj_set_size(box, 90, 40);

        input_indicators[i].label = lv_label_create(box);
        input_indicators[i].state = false;
        lv_obj_center(input_indicators[i].label);

        update_io_indicator(&input_indicators[i], "IN");
    }

    for (uint8_t i = 0; i < UI_NUM_OUTPUTS; ++i)
    {
        lv_obj_t *box = lv_obj_create(io_cont);
        lv_obj_set_size(box, 90, 40);

        output_indicators[i].label = lv_label_create(box);
        output_indicators[i].state = false;
        lv_obj_center(output_indicators[i].label);

        update_io_indicator(&output_indicators[i], "OUT");
    }
}


static void build_menu_screen(void)
{
    /* tytuł pod przyciskiem Back */
    lv_obj_t *title_label = lv_label_create(menu_screen);
    lv_label_set_text(title_label, "HVAC Controller");
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 40);

    lv_obj_t *menu_list = lv_obj_create(menu_screen);
    lv_obj_remove_style_all(menu_list);
    lv_obj_set_width(menu_list, lv_pct(100));
    lv_obj_set_height(menu_list, LV_SIZE_CONTENT);
    lv_obj_align(menu_list, LV_ALIGN_TOP_MID, 0, 70);
    lv_obj_set_style_pad_all(menu_list, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_row(menu_list, 10, LV_PART_MAIN);

    lv_obj_set_layout(menu_list, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(menu_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(menu_list,
                          LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_START);

    static const char *menu_captions[] = {
        "Dashboard",
        "Zone / Comfort",
        "Supply / Fan",
        "Schedule",
        "Sequences",
        "PID Tuner",
        "Protections",
        "I/O / Service",
        "Visualiser",
        "Diagnostics",
    };

    lv_obj_t *screens[] = {
        dashboard_screen,
        zone_screen,
        supply_screen,
        schedule_screen,
        sequence_screen,
        pid_tuner_screen,
        protection_screen,
        io_selector_screen,
        visualiser_screen,
        diag_screen,
    };

    for (int i = 0; i < 10; ++i)
    {
        lv_obj_t *btn = lv_btn_create(menu_list);
        lv_obj_set_width(btn, lv_pct(90));
        lv_obj_add_event_cb(btn, load_screen_event_cb,
                            LV_EVENT_CLICKED, screens[i]);

        lv_obj_t *label = lv_label_create(btn);
        lv_label_set_text(label, menu_captions[i]);
        lv_obj_center(label);
    }
}

static void build_zone_screen(void)
{
    lv_obj_t *label = lv_label_create(zone_screen);
    lv_label_set_text(label, "Zone / Comfort (T_room, RH, CO2 setpoint)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

static void build_supply_screen(void)
{
    lv_obj_t *label = lv_label_create(supply_screen);
    lv_label_set_text(label, "Supply / Fan (T_supply, fan speed, fresh air)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

static void build_schedule_screen(void)
{
    lv_obj_t *label = lv_label_create(schedule_screen);
    lv_label_set_text(label, "Schedule (godziny pracy, komfort/eco)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

static void build_sequence_screen(void)
{
    lv_obj_t *label = lv_label_create(sequence_screen);
    lv_label_set_text(label, "Sequences (wymiennik, grzałka, chłodnica)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

static void build_protection_screen(void)
{
    lv_obj_t *label = lv_label_create(protection_screen);
    lv_label_set_text(label, "Protections (frost, alarmy temp, CO2)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

static void build_io_screen(void)
{
    lv_obj_t *label = lv_label_create(io_selector_screen);
    lv_label_set_text(label, "I/O / Serwis (podgląd wejść/wyjść)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

static void build_diag_screen(void)
{
    lv_obj_t *label = lv_label_create(diag_screen);
    lv_label_set_text(label, "Diagnostics (FW, logi, alarmy)");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}


void ui_set_input_state(uint8_t index, bool active)
{
    if (index >= UI_NUM_INPUTS)
    {
        return;
    }

    input_indicators[index].state = active;
    update_io_indicator(&input_indicators[index], "IN");
}

void ui_set_output_state(uint8_t index, bool active)
{
    if (index >= UI_NUM_OUTPUTS)
    {
        return;
    }

    output_indicators[index].state = active;
    update_io_indicator(&output_indicators[index], "OUT");
}

void ui_set_temp_setpoint(int16_t value)
{
    if (value < 5)
    {
        value = 5;
    }
    if (value > 35)
    {
        value = 35;
    }

    temp_setpoint = value;
    temp_update_label();
}

int16_t ui_get_temp_setpoint(void)
{
    return temp_setpoint;
}

void ui_simple_init(void)
{

    dashboard_screen = lv_obj_create(NULL);
    lv_obj_set_scroll_dir(dashboard_screen, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(dashboard_screen, LV_SCROLLBAR_MODE_AUTO);

    menu_screen        = create_sub_screen(dashboard_screen, "Back");
    zone_screen        = create_sub_screen(dashboard_screen, "Back");
    supply_screen      = create_sub_screen(dashboard_screen, "Back");
    schedule_screen    = create_sub_screen(dashboard_screen, "Back");
    sequence_screen    = create_sub_screen(dashboard_screen, "Back");
    protection_screen  = create_sub_screen(dashboard_screen, "Back");
    visualiser_screen  = create_sub_screen(dashboard_screen, "Back");
    pid_tuner_screen   = create_sub_screen(dashboard_screen, "Back");
    io_selector_screen = create_sub_screen(dashboard_screen, "Back");
    diag_screen        = create_sub_screen(dashboard_screen, "Back");

    /* budowa zawartości ekranów */
    build_dashboard_screen();
    build_menu_screen();
    build_pid_tuner_screen();
    build_zone_screen();
    build_supply_screen();
    build_schedule_screen();
    build_sequence_screen();
    build_protection_screen();
    build_io_screen();
    build_diag_screen();

    /* start od dashboardu */
    lv_scr_load(dashboard_screen);
}
