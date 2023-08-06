#include "esp_log.h"

// example of use: (TAG, "lcd_send_i2c_4bit", data4bit, data4bit_size);
static void logDumpBytes(const char *tag, const char *msg, uint8_t *data, size_t size)
{
    static char buf[1024];
    char *p = buf;
    sprintf(p, "%s: ", msg);
    p += strlen(p);
    if (size > 32)
        size = 32;
    for (int i = 0; i < size; i++)
    {
        sprintf(p, "%02x ", data[i]);
        p += strlen(p);
    }
    ESP_LOGI(tag, "%s", buf);
}
