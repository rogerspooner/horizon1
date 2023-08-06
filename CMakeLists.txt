# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(horizon1)

# Some ESP-IDF include files were not found by LVGL.
# include_directories(${ESP_IDF}/components/driver/ledc/include)

# Borrowed from https://docs.lvgl.io/8.2/get-started/cmake.html
# Specify path to own LVGL config header
set(LV_CONF_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/components/lv_conf.h
    CACHE STRING "" FORCE)

# The target "horizon1" depends on LVGL
# target_link_libraries(horizon1 PRIVATE lvgl)