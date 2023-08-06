/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lvgl.h"

#include "display.h"
#include "horizon-display.h"


void app_main(void)
{
    init_displays(); // using LVGL
    init_horizon();
    int frame = 0;
    
    // Expect some stuff to happen here
    while (true)
    {
        display_tick();
        vTaskDelay(5);
        if (frame % 20 == 0)
            update_horizon();
        frame ++;
    }
    deinit_displays();
}
