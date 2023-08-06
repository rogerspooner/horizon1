/* Display related features for Roger's artificial horizon app on ESP32 + GC9A01 + MPU6050 */

#include <inttypes.h>
#include "esp_system.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "../lvgl/lvgl.h"

#define LV_TICK_PERIOD_MS 50

/* Interrupt timer driven graphics tasks.
   Since we are on ESP32 FreeRTOS, we could use a task for this. 
*/
static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
    lv_timer_handler();
}

esp_err_t init_displays(void)
{
    lv_init();
    // initialise drivers
    // register display driver with LVGL
    // Call lv_tick_inc(x) every x milliseconds in an interrupt to report the elapsed time to LVGL. Learn more.
    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    // Call lv_timer_handler() every few milliseconds to handle LVGL related tasks. Learn more.}
    return ESP_OK;
}
