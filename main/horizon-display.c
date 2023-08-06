#include "lvgl.h"
#include "esp_err.h"
#include "esp_log.h"

esp_err_t init_horizon(void)
{
    lv_obj_t * slider1 = lv_slider_create(lv_scr_act());
    lv_obj_set_x(slider1, 30);
    lv_obj_set_y(slider1, 10);
    lv_obj_set_size(slider1, 200, 50);
    ESP_LOGI("horizon-display", "init_horizon() called");
    return ESP_OK;
}
