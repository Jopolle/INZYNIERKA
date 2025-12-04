cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lvgl_fs_button)

target_sources(app PRIVATE
    src/main.c
    src/ikony/tp_type1_not_active.c
    src/ikony/TP_type1__not_active.c
    src/ikony/seq_img_cool_rec_dead_rec_heat.c
    src/ikony/seq_img_cool_dead_heat.c
    src/ikony/snowflake.c
    src/ikony/heater.c
    src/ikony/heat_exchange.c
)
