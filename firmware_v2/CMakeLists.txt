# CMakeLists.txt

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hvac_sd_config)

add_definitions(-DLV_LVGL_H_INCLUDE_SIMPLE)

target_sources(app
    PRIVATE
        src/main.c
        src/gui.c
)

target_include_directories(app PRIVATE
    ${ZEPHYR_BASE}/../modules/fs/fatfs/include
)