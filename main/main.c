#include <stdio.h>
#include "esp_timer.h"
#include "time.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_cst816s.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

#include "esp_log.h"

#include "lvgl.h"
#include "demos/lv_demos.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "esp_sntp.h"

#define CONFIG_I2C_BUS_BACKWARD_CONFIG 1
#include "bme280.h"

#define EXAMPLE_PIN_NUM_SCLK 39
#define EXAMPLE_PIN_NUM_MOSI 38
#define EXAMPLE_PIN_NUM_MISO 40

#define EXAMPLE_SPI_HOST SPI2_HOST

#define EXAMPLE_I2C_NUM 0  // Use the same I2C port as BME280
#define EXAMPLE_PIN_NUM_I2C_SDA 48
#define EXAMPLE_PIN_NUM_I2C_SCL 47

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (80 * 1000 * 1000)

#define EXAMPLE_PIN_NUM_LCD_DC 42
#define EXAMPLE_PIN_NUM_LCD_RST -1
#define EXAMPLE_PIN_NUM_LCD_CS 45

#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

#define EXAMPLE_LCD_H_RES 240
#define EXAMPLE_LCD_V_RES 320

#define EXAMPLE_PIN_NUM_BK_LIGHT 1

#define LCD_BL_LEDC_TIMER LEDC_TIMER_0
#define LCD_BL_LEDC_MODE LEDC_LOW_SPEED_MODE

#define LCD_BL_LEDC_CHANNEL LEDC_CHANNEL_0
#define LCD_BL_LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define LCD_BL_LEDC_DUTY (1024)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LCD_BL_LEDC_FREQUENCY (10000)          // Frequency in Hertz. Set frequency at 5 kHz

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1

#define WIFI_SSID "DIGIFIBRA-bPKy"
#define WIFI_PASS "3eFuN5GYbE"

static const char *TAG = "lvgl_example";
static lv_indev_drv_t indev_drv; // Input device driver (Touch)
static lv_disp_drv_t disp_drv;   /*Descriptor of a display driver*/
static SemaphoreHandle_t lvgl_api_mux = NULL;

esp_lcd_panel_handle_t panel_handle;
esp_lcd_touch_handle_t tp;
bool lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_api_mux, timeout_ticks) == pdTRUE;
}

void lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_api_mux);
}

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_flush_ready(&disp_drv);
    return false;
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display

    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;
    esp_lcd_touch_read_data(tp);
    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0)
    {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void lv_port_disp_init(void)
{
    static lv_disp_draw_buf_t draw_buf;
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf2);
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2,
                          EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES); /*Initialize the display buffer*/

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    lv_disp_drv_init(&disp_drv); /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = example_lvgl_flush_cb;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf;

    /*Required for Example 3)*/
    disp_drv.full_refresh = 1;
    // disp_drv.direct_mode = 1;

    /* Fill a memory array with a color if you have GPU.
     * Note that, in lv_conf.h you can enable GPUs that has built-in support in LVGL.
     * But if you have a different GPU you can use with this callback.*/
    // disp_drv.gpu_fill_cb = gpu_fill;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}

void lv_port_indev_init(void)
{
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    // indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;

    lv_indev_drv_register(&indev_drv);
}

void display_init(void)
{
    ESP_LOGI(TAG, "SPI BUS init");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(EXAMPLE_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "Install panel IO");

    esp_lcd_panel_io_handle_t io_handle = NULL;

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_SPI_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_LOGI(TAG, "Install ST7789 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
}

void touch_init(void)
{
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    ESP_LOGI(TAG, "Initialize touch IO (I2C)");
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)EXAMPLE_I2C_NUM, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_V_RES,
        .y_max = EXAMPLE_LCD_H_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller CST816");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp));
}

void bsp_brightness_init(void)
{
    gpio_set_direction(EXAMPLE_PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, 1);

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LCD_BL_LEDC_MODE,
        .timer_num = LCD_BL_LEDC_TIMER,
        .duty_resolution = LCD_BL_LEDC_DUTY_RES,
        .freq_hz = LCD_BL_LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LCD_BL_LEDC_MODE,
        .channel = LCD_BL_LEDC_CHANNEL,
        .timer_sel = LCD_BL_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = EXAMPLE_PIN_NUM_BK_LIGHT,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void bsp_brightness_set_level(uint8_t level)
{
    if (level > 100)
    {
        ESP_LOGE(TAG, "Brightness value out of range");
        return;
    }

    uint32_t duty = (level * (LCD_BL_LEDC_DUTY - 1)) / 100;

    ESP_ERROR_CHECK(ledc_set_duty(LCD_BL_LEDC_MODE, LCD_BL_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LCD_BL_LEDC_MODE, LCD_BL_LEDC_CHANNEL));

    ESP_LOGI(TAG, "LCD brightness set to %d%%", level);
}

void lvgl_tick_timer_init(uint32_t ms)
{
    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, ms * 1000));
}

static void task(void *param)
{
    // ESP_LOGI(TAG, "run");
    while (1)
    {
        uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        while (1)
        {
            // Lock the mutex due to the LVGL APIs are not thread-safe
            if (lvgl_lock(-1))
            {
                task_delay_ms = lv_timer_handler();
                // Release the mutex
                lvgl_unlock();
            }
            if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS)
            {
                task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
            }
            else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS)
            {
                task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
            }
            vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
        }
    }
}

static lv_obj_t *time_label;
static lv_obj_t *date_label;
static lv_obj_t *wifi_dot = NULL;
static volatile bool wifi_connected = false;
static bool sntp_started = false;

static void initialize_sntp(void)
{
    if (sntp_started) return;
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    /* Set timezone to Valencia, Spain (Europe/Madrid) */
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
    tzset();

    sntp_started = true;
}

// Event handler for Wi-Fi and IP events
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            wifi_connected = false;
            esp_wifi_connect(); // auto-reconnect
            break;
        default:
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        wifi_connected = true;
        initialize_sntp();
    }
}

static void update_time_task(void *pvParameters) {
    // Set the current time manually
    struct tm timeinfo = {
        .tm_year = 124,  // 2024 - 1900
        .tm_mon = 5,     // June (0-11)
        .tm_mday = 9,    // 9th
        .tm_hour = 8,    // 8 AM
        .tm_min = 40,    // 40 minutes
        .tm_sec = 0      // 0 seconds
    };
    time_t now;

    while (1) {
        if (lvgl_lock(-1)) {
            // Update time
            time(&now);
            localtime_r(&now, &timeinfo);

            // Update time label
            char time_str[32];
            strftime(time_str, sizeof(time_str), "%H:%M", &timeinfo);
            lv_label_set_text(time_label, time_str);

            // Update date label with day of week and day of month
            char date_str[32];
            strftime(date_str, sizeof(date_str), "%a %d", &timeinfo);
            lv_label_set_text(date_label, date_str);

            // Update Wi-Fi indicator color
            lv_color_t dot_col = wifi_connected ? lv_color_hex(0xFFD700) /*yellow*/ : lv_color_hex(0xFF0000);
            lv_obj_set_style_bg_color(wifi_dot, dot_col, 0);

            lvgl_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
    }
}

static int active_button = 0; // Start with button 1 (index 0) active

static void box_event_cb(lv_event_t *e) {
    lv_obj_t *box = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    
    // Find the index of the button
    int index = -1;
    for (int i = 0; i < 4; i++) {
        if (box == lv_obj_get_child(lv_obj_get_parent(box), i)) {
            index = i;
            break;
        }
    }
    
    if (index != -1 && code == LV_EVENT_PRESSED) {
        // Only switch if pressing a different button
        if (index != active_button) {
            // Deactivate previously active button
            lv_obj_t *prev_box = lv_obj_get_child(lv_obj_get_parent(box), active_button);
            lv_obj_set_style_bg_color(prev_box, lv_color_hex(0x000000), 0);
            
            // Activate current button
            lv_obj_set_style_bg_color(box, lv_color_hex(0x222222), 0); // Dark gray when highlighted
            active_button = index;
        }
    }
}

static void create_clock_display(void) {
    // Create a container for the clock display at the top
    lv_obj_t *clock_cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(clock_cont, EXAMPLE_LCD_H_RES, 120);  // Increased height from 90 to 120
    lv_obj_align(clock_cont, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_flex_flow(clock_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(clock_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(clock_cont, 5, 0);  // Increased padding from 2 to 5
    lv_obj_set_style_border_width(clock_cont, 0, 0);
    lv_obj_set_style_bg_color(clock_cont, lv_color_hex(0x000000), 0);

    // Create time label with larger font
    time_label = lv_label_create(clock_cont);
    lv_obj_set_style_text_font(time_label, &lv_font_montserrat_32, 0);  // Increased from 28 to 32
    lv_label_set_text(time_label, "08:44");
    lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFD700), 0);
    
    // Create date label
    date_label = lv_label_create(clock_cont);
    lv_obj_set_style_text_font(date_label, &lv_font_montserrat_20, 0);  // Increased from 16 to 20
    lv_label_set_text(date_label, "2025-06-09");
    lv_obj_set_style_text_color(date_label, lv_color_hex(0xFFD700), 0);

    // Set screen background to black
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), 0);

    // Create container for boxes - adjust height to account for clock container
    lv_obj_t *boxes_cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(boxes_cont, EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES - 120);  // Adjusted for new clock height
    lv_obj_align(boxes_cont, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_flex_flow(boxes_cont, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(boxes_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(boxes_cont, 15, 0);
    lv_obj_set_style_pad_top(boxes_cont, 25, 0);
    lv_obj_set_style_border_width(boxes_cont, 0, 0);
    lv_obj_set_style_bg_color(boxes_cont, lv_color_hex(0x000000), 0);

    // Create 4 boxes with labels
    for(int i = 1; i <= 4; i++) {
        lv_obj_t *box = lv_obj_create(boxes_cont);
        lv_obj_set_size(box, 70, 70);  // Restored to original size
        // Set initial color - highlight first box, others black
        lv_obj_set_style_bg_color(box, (i == 1) ? lv_color_hex(0x222222) : lv_color_hex(0x000000), 0);
        lv_obj_set_style_radius(box, 12, 0);  // Restored to original radius
        lv_obj_set_style_border_width(box, 0, 0);
        
        // Add press and release events
        lv_obj_add_event_cb(box, box_event_cb, LV_EVENT_PRESSED, NULL);
        
        // Create label inside box
        lv_obj_t *label = lv_label_create(box);
        char label_text[2];
        snprintf(label_text, sizeof(label_text), "%d", i);
        lv_label_set_text(label, label_text);
        // Set font size for the label
        lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);  // Restored to original font size
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFD700), 0);
        lv_obj_center(label);
    }

    // Create Wi-Fi status dot in top-left corner
    wifi_dot = lv_obj_create(lv_scr_act());
    lv_obj_set_size(wifi_dot, 12, 12);
    lv_obj_set_style_radius(wifi_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(wifi_dot, 0, 0);
    lv_obj_align(wifi_dot, LV_ALIGN_TOP_LEFT, 5, 5);
    lv_obj_set_style_bg_color(wifi_dot, lv_color_hex(0xFF0000), 0); // start red
}

static void i2c_master_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = EXAMPLE_PIN_NUM_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(EXAMPLE_I2C_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(EXAMPLE_I2C_NUM, i2c_conf.mode, 0, 0, 0));
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        }
    };
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi init done. Connecting to %s", WIFI_SSID);
}

void app_main(void)
{
    lvgl_api_mux = xSemaphoreCreateRecursiveMutex();
    lv_init();

    /* Initialize I2C master for touch controller */
    i2c_master_init();

    /* Initialize Wi-Fi (testing only, no connection) */
    wifi_init();

    display_init();
    touch_init(); 
    lv_port_disp_init();
    lv_port_indev_init();
    lvgl_tick_timer_init(EXAMPLE_LVGL_TICK_PERIOD_MS);
    bsp_brightness_init();
    bsp_brightness_set_level(80);

    if (lvgl_lock(-1)) {
        create_clock_display();
        lvgl_unlock();
    }
    
    // Create task to update time
    xTaskCreate(update_time_task, "update_time_task", 2048, NULL, 5, NULL);
    
    xTaskCreatePinnedToCore(task, "bsp_lv_port_task", 1024 * 20, NULL, 5, NULL, 1);
}
