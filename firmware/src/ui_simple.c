#include "ui_simple.h"
#include <lvgl.h>

static lv_obj_t *screen1;
static lv_obj_t *screen2;

static void btn_to_screen2_event_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_scr_load(screen2);
}

static void btn_to_screen1_event_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    lv_scr_load(screen1);
}

void ui_simple_init(void)
{
    screen1 = lv_obj_create(NULL);

    lv_obj_t *label1 = lv_label_create(screen1);
    lv_label_set_text(label1, "Screen 1");
    lv_obj_align(label1, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t *btn1 = lv_btn_create(screen1);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(btn1, btn_to_screen2_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn1_label = lv_label_create(btn1);
    lv_label_set_text(btn1_label, "Go to screen 2");
    lv_obj_center(btn1_label);

    screen2 = lv_obj_create(NULL);

    lv_obj_t *label2 = lv_label_create(screen2);
    lv_label_set_text(label2, "Screen 2");
    lv_obj_align(label2, LV_ALIGN_TOP_MID, 0, 10);

    lv_obj_t *btn2 = lv_btn_create(screen2);
    lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(btn2, btn_to_screen1_event_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn2_label = lv_label_create(btn2);
    lv_label_set_text(btn2_label, "Back to screen 1");
    lv_obj_center(btn2_label);

    lv_scr_load(screen1);
}
