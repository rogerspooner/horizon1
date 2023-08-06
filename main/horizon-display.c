#include "lvgl.h"
#include "esp_err.h"
#include "esp_log.h"

static lv_obj_t * horizonSlider1 = NULL;
static lv_obj_t * horizonSlider2 = NULL;
static lv_obj_t * horizonLabel1 = NULL;
static lv_style_t sliderStyle;
static int horizonFrame = 0;


esp_err_t init_horizon(void)
{
    lv_style_init(&sliderStyle);
    horizonSlider1 = lv_slider_create(lv_scr_act());
    lv_obj_set_x(horizonSlider1, 50);
    lv_obj_set_y(horizonSlider1, 60);
    lv_obj_set_size(horizonSlider1, 140, 16);
    lv_slider_set_value(horizonSlider1, 50, LV_ANIM_ON);
    horizonSlider2 = lv_slider_create(lv_scr_act());
    lv_obj_set_x(horizonSlider2, 50);
    lv_obj_set_y(horizonSlider2, 90);
    lv_obj_set_size(horizonSlider2, 140, 16);
    lv_slider_set_value(horizonSlider2, 50, LV_ANIM_ON);
    horizonLabel1 = lv_label_create(lv_scr_act());
    lv_label_set_text(horizonLabel1, "Horizon");
    lv_obj_set_x(horizonLabel1, 50);
    lv_obj_set_y(horizonLabel1, 130);
    lv_style_set_anim_time(&sliderStyle, 500);
    lv_obj_add_style(horizonSlider1, &sliderStyle, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_style(horizonSlider2, &sliderStyle, LV_PART_MAIN | LV_STATE_DEFAULT);
    ESP_LOGI("horizon-display", "init_horizon() called");
    return ESP_OK;
}

esp_err_t update_horizon(void)
{
     horizonFrame += 1;
     int slider1Pos = ((horizonFrame % 10) * 10);
     lv_slider_set_value(horizonSlider1, slider1Pos, LV_ANIM_ON);
     int slider2Pos = 100 - ((horizonFrame % 20) * 5);
     lv_slider_set_value(horizonSlider2, slider2Pos, LV_ANIM_ON);

     return ESP_OK;
}