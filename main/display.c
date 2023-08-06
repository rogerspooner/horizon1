/* Display related features for Roger's artificial horizon app on ESP32 + GC9A01 + MPU6050 */

#include <inttypes.h>
#include "esp_system.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lcd_gc9a01_roger.h"

#define LV_TICK_PERIOD_MS 1000
#define PIN_NUM_MISO 35 // not used, but don't want it to interfere
#define PIN_NUM_MOSI 33
#define PIN_NUM_CLK  32
#define PIN_NUM_CS_DISP0   13
#define PIN_NUM_DC   25
#define PIN_NUM_RST  14
#define PIN_NUM_BCKL 27 // of st7735s display. The gc9a01 has a software command 0x53, apparently
#define UPDATE_STRIPE_HEIGHT 32


#define INIT_CMD_DELAY_AFTER 0x80 // long delay after sending SPI command. This should be ORed with databytes field in lcd_init_cmd_t struct
#define INIT_CMD_END_SEQUENCE 0xFF
#define LCD_HOST    HSPI_HOST

static int lv_screenWidth = 240;
static int lv_screenHeight = 240;
static spi_device_handle_t spi_dev0;

static void lv_tick_task(void *arg); // hoist
static void logDumpBytes(const char *tag, const char *msg, uint8_t *data, size_t size); // hoist

/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef enum {
    LCD_TYPE_ILI = 1,
    LCD_TYPE_ST,
    LCD_TYPE_MAX,
} type_lcd_t;

/* Initialise GC9A01. Based on manufacturer's recommendation but copied from AdaFruit */
DRAM_ATTR static const lcd_init_cmd_t gc9a01_InitSequence[] = {
  { GC9A01A_INREGEN2, { }, 0 },
  { 0xEB, { 0x14 }, 1 },// ?
  { GC9A01A_INREGEN1, { }, 0} ,
  { GC9A01A_INREGEN2, { }, 0} ,
  { 0xEB, { 0x14 }, 1 }, // ?
  { 0x84, { 0x40 }, 1 }, // ?
  { 0x85, { 0xFF }, 1 }, // ?
  { 0x86, { 0xFF }, 1 }, // ?
  { 0x87, { 0xFF }, 1 }, // ?
  { 0x88, { 0x0A }, 1 }, // ?
  { 0x89, { 0x21 }, 1 }, // ?
  { 0x8A, { 0x00 }, 1 }, // ?
  { 0x8B, { 0x80 }, 1 }, // ?
  { 0x8C, { 0x01 }, 1 }, // ?
  { 0x8D, { 0x01 }, 1 }, // ?
  { 0x8E, { 0xFF }, 1 }, // ?
  { 0x8F, { 0xFF }, 1 }, // ?
  { 0xB6, { 0x00, 0x00 }, 2 }, // ?
  { GC9A01A_MADCTL, { MADCTL_MX | MADCTL_BGR }, 1 },
  { GC9A01A_COLMOD, { 0x05 }, 1 },
  { 0x90, { 0x08, 0x08, 0x08, 0x08}, 4}, // ?
  { 0xBD, { 0x06 }, 1 }, // ?
  { 0xBC, { 0x00 }, 1 }, // ?
  { 0xFF, { 0x60, 0x01, 0x04 }, 3 }, // ?
  { GC9A01A1_POWER2, { 0x13 }, 1 },
  { GC9A01A1_POWER3, { 0x13 }, 1 },
  { GC9A01A1_POWER4, { 0x22 }, 1 },
  { 0xBE, { 0x11 }, 1 }, // ?
  { 0xE1, { 0x10, 0x0E }, 2 }, // ?
  { 0xDF, { 0x21, 0x0c, 0x02 }, 3 }, // ?
  { GC9A01A_GAMMA1, { 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A }, 6 },
  { GC9A01A_GAMMA2, { 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F }, 6 },
  { GC9A01A_GAMMA3, { 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A }, 6 },
  { GC9A01A_GAMMA4, { 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F }, 6 },
  { 0xED, { 0x1B, 0x0B }, 2}, // ?
  { 0xAE, { 0x77 }, 1 }, // ?
  { 0xCD, { 0x63 }, 1 }, // ?
  // Unsure what this line (from manufacturer's boilerplate code) is
  // meant to do, but users reported issues, seems to work OK without:
  //0x70, 9, 0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03, // ?
  { GC9A01A_FRAMERATE, { 0x34 }, 1 },
  { 0x62, { 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, // ?
            0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70 }, 12 },
  { 0x63, { 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, // ?
            0x18, 0x13, 0x71, 0xF3, 0x70, 0x70} , 12 },
  { 0x64, { 0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07 }, 7 }, // ?
  { 0x66, { 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00 }, 10 }, // ?
  { 0x67, { 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98 }, 10 }, // ?
  { 0x74, { 0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00 }, 7 }, // ?
  { 0x98, { 0x3e, 0x07 }, 2 }, // ?
  { GC9A01A_TEON, { }, 0 },
  { GC9A01A_INVON, { }, 0 },
  { GC9A01A_SLPOUT, { }, 0 | INIT_CMD_DELAY_AFTER }, // Exit sleep
  { GC9A01A_DISPON, { }, 0 | INIT_CMD_DELAY_AFTER }, // Display on
  { 0, { }, 0 | INIT_CMD_END_SEQUENCE }                 // End of list
};

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd, bool keep_cs_active)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    if (keep_cs_active) {
      t.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    }
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

/*
 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

/*
static void send_line_finish(spi_device_handle_t spi)
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x=0; x<6; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}
*/

/* To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
 * before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
 * because the D/C line needs to be toggled in the middle.)
 * This routine queues these commands up as interrupt transactions so they get
 * sent faster (compared to calling spi_device_transmit several times), and at
 * the mean while the lines for next transactions can get calculated.
 */
void disp0_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    static spi_transaction_t trans[6]; // from esp32 version
    esp_err_t ret;

    // ESP_LOGI("display", "disp0_flush() called, area (%d,%d) > (%d,%d)", area->x1, area->y1, area->x2, area->y2);
    for (int x=0; x<6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags = SPI_TRANS_USE_TXDATA;
    }

	size_t datalength = lv_area_get_width(area) * lv_area_get_height(area) * sizeof(lv_color_t) * 8;
    trans[0].tx_data[0]=0x2A;           //Column Address Set
	trans[1].tx_data[0] = (area->x1 >> 8) & 0xFF;
	trans[1].tx_data[1] = area->x1 & 0xFF;
	trans[1].tx_data[2] = (area->x2 >> 8) & 0xFF;
	trans[1].tx_data[3] = area->x2 & 0xFF;
    // length of command and data register were defined above

    trans[2].tx_data[0]=0x2B;           //Page address set
	trans[3].tx_data[0] = (area->y1 >> 8) & 0xFF;
	trans[3].tx_data[1] = area->y1 & 0xFF;
	trans[3].tx_data[2] = (area->y2 >> 8) & 0xFF;
	trans[3].tx_data[3] = area->y2 & 0xFF;

    trans[4].tx_data[0]=0x2C;           //memory write
    trans[5].tx_buffer=color_map;        //finally send the line data
    trans[5].length=datalength;          //Data length, in bits
    trans[5].flags=0; //undo SPI_TRANS_USE_TXDATA flag

    for (int x=0; x<6; x++) {

        ret = spi_device_transmit(spi_dev0, &trans[x]);
        if (ret != ESP_OK)  {
            ESP_LOGE("display", "Error x%02x sending transaction %d", ret, x);
        }
        ESP_ERROR_CHECK(ret);
        // ret=spi_device_queue_trans(spi_dev0, &trans[x], portMAX_DELAY);
    }
    lv_disp_flush_ready(drv);       /* Indicate you are ready with the flushing*/
}

//Initialize the display written by Roger from LVGL notes
void lcd_init_spi()
{
    int cmd=0;
    esp_err_t ret;
    const lcd_init_cmd_t* lcd_init_cmds;

    spi_bus_config_t buscfg={
        .miso_io_num= PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=UPDATE_STRIPE_HEIGHT * 240 * 2+8
    };
    spi_device_interface_config_t devcfg_gc={
        .clock_speed_hz= 1*1000*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS_DISP0,         //CS pin for round LCD
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    ret=spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(LCD_HOST, &devcfg_gc, &spi_dev0);
    ESP_ERROR_CHECK(ret);
    //Initialize the LCD.

    //Initialize non-SPI GPIOs
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL<<PIN_NUM_DC) | (1ULL<<PIN_NUM_RST) | (1ULL<<PIN_NUM_BCKL));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = true;
    gpio_config(&io_conf);
    // Backlight off for st7735s
    gpio_set_level(PIN_NUM_BCKL, 0);
    // reset both devices
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(2);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(2);
    
    ESP_LOGI("lcd","Initialising GC9A01 LCD\n");
    lcd_init_cmds = gc9a01_InitSequence;
    
    while (lcd_init_cmds[cmd].databytes != INIT_CMD_END_SEQUENCE ) {
        lcd_cmd(spi_dev0, lcd_init_cmds[cmd].cmd, false);
        lcd_data(spi_dev0, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes&0x1F);
        if (lcd_init_cmds[cmd].databytes & INIT_CMD_DELAY_AFTER ) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        cmd++;
    }
}

esp_err_t init_displays(void)
{

    ESP_LOGI("display", "init_displays() called");
    xGuiSemaphore = xSemaphoreCreateMutex();
    // Disable st7735s display by de-asserting CS on GPIO 26
    gpio_set_direction(26,GPIO_MODE_OUTPUT); // CS of st7735s
    gpio_set_level(26,1);
    
    lcd_init_spi(); // initialise display(s)
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

    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t *buf1 = NULL;
    buf1 = heap_caps_malloc( lv_screenWidth * UPDATE_STRIPE_HEIGHT * sizeof(lv_color_t) + 32, MALLOC_CAP_DMA);
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, lv_screenWidth * UPDATE_STRIPE_HEIGHT ); // measured in pixels

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = lv_screenWidth;
    disp_drv.ver_res = lv_screenHeight;
    disp_drv.flush_cb = disp0_flush; // we will use GC9A01 driver
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    return ESP_OK;
}

esp_err_t deinit_displays(void)
{
    // TODO release resources
    ESP_LOGI("display", "deinit_displays() called");
    return ESP_OK;
}

/* Interrupt timer driven graphics tasks.
   Since we are on ESP32 FreeRTOS, we could use a task for this. 
*/
static void lv_tick_task(void *arg) {
    (void) arg;
    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, 0)) {
        ESP_LOGI("display", "lv_tick_inc() called");
        lv_tick_inc(LV_TICK_PERIOD_MS);
        xSemaphoreGive(xGuiSemaphore);
    }
    else {
        ESP_LOGI("display", "Denied xGuiSemaphore, didn't call lv_tick_inc()");
    }
}

void display_tick(void)
{
    /* Try to take the semaphore, call lvgl related function on success */
    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
        ESP_LOGI("display", "display_tick() got semaphore");
        lv_task_handler();
        xSemaphoreGive(xGuiSemaphore);
    }
    else {
            ESP_LOGI("display", "Denied xGuiSemaphore to display_tick().");

    }
}


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
