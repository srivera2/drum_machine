/**
 * @file      main.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xinyuan Electronic Technology Co., Ltd
 * @date      2024-01-07
 *
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "amoled_driver.h"
#include "touch_driver.h"
#include "i2c_driver.h"
#include "power_driver.h"
#include "demos/lv_demos.h"
#include "tft_driver.h"
#include "product_pins.h"
#include "sd_card.h"
#include "pcf8575.h"


static const char *TAG = "main";

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY 2

static SemaphoreHandle_t lvgl_mux = NULL;

static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)

extern "C" {
    lv_disp_drv_t disp_drv;      // contains callback functions
}


static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
#if DISPLAY_FULLRESH
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );
    display_push_colors(area->x1, area->y1, w, h, (uint16_t *)color_map);
    lv_disp_flush_ready( drv );
#else
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    display_push_colors(offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, (uint16_t *)color_map);
#endif
}


#if BOARD_HAS_TOUCH
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    int16_t touchpad_x[1] = {0};
    int16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Get coordinates */
    touchpad_cnt = touch_get_data(touchpad_x, touchpad_y, 1);

    if (touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}
extern "C" {
    void example_lvgl_demo_ui(lv_disp_t *disp);
}

// ---------------------------------------------------------------------------
// Button test: 8 buttons on PCF8575 P00-P07, active low (pressed = 0).
// ---------------------------------------------------------------------------
#define NUM_BUTTONS        8
#define BUTTON_POLL_MS     2    // needs CONFIG_FREERTOS_HZ=1000 to actually be 2 ms
#define BUTTON_RELEASE_MS  20   // must read released this long before it can trigger again
#define BUTTON_UI_MS       20   // how often the screen picks up button state
#define BUTTON_FLASH_MS    100  // minimum time an LED stays lit per hit, so short taps are visible
#define BUTTON_TASK_PRIO   5    // above LVGL / app tasks so screen work can't delay polling

static lv_obj_t *s_btn_label = NULL;
static lv_obj_t *s_btn_led[NUM_BUTTONS];
static lv_obj_t *s_btn_count[NUM_BUTTONS];

// Poll task -> UI. The poll task never touches LVGL or the log, so neither a
// slow screen redraw nor a blocked USB console can make it miss a hit.
static QueueHandle_t s_btn_events = NULL;   // uint8_t button index, one per hit
static volatile uint8_t s_btn_held = 0;     // debounced: bit i set = button i held

// Written only by the poll task, read by the UI and the stats logger.
static struct {
    volatile uint32_t hits[NUM_BUTTONS];         // debounced hits
    volatile uint32_t raw_lows[NUM_BUTTONS];     // raw high->low transitions (includes bounce)
    volatile uint32_t min_press_us[NUM_BUTTONS]; // shortest hit -> release time
    volatile uint32_t max_gap_us;                // longest time between two polls
    volatile uint32_t read_errors;
} s_stats;

static void button_ui_timer_cb(lv_timer_t *timer);

#define LED_OFF_COLOR  lv_palette_darken(LV_PALETTE_GREY, 3)
#define LED_ON_COLOR   lv_palette_main(LV_PALETTE_GREEN)

// Full-screen button test (320x170):
//   status line   (top)
//   "Button N"    (center, large)
//   8 LEDs 1-8 with hit counters underneath (bottom)
// Must be called with the LVGL lock held.
static void button_test_screen_create(esp_err_t exp_result)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_text_color(scr, lv_color_white(), 0);

    lv_obj_t *status = lv_label_create(scr);
    lv_obj_align(status, LV_ALIGN_TOP_MID, 0, 6);

    if (exp_result != ESP_OK) {
        lv_label_set_text_fmt(status, "PCF8575 not found (%s)", esp_err_to_name(exp_result));
        lv_obj_set_style_text_color(status, lv_palette_main(LV_PALETTE_RED), 0);
        lv_obj_t *hint = lv_label_create(scr);
        lv_label_set_text(hint, "Check wiring - see serial log");
        lv_obj_center(hint);
        return;
    }
    lv_label_set_text_fmt(status, "PCF8575 @ 0x%02X", pcf8575_address());
    lv_obj_set_style_text_color(status, lv_palette_main(LV_PALETTE_GREY), 0);

    s_btn_label = lv_label_create(scr);
    lv_obj_set_style_text_font(s_btn_label, &lv_font_montserrat_28, 0);
    lv_label_set_text(s_btn_label, "Press a button");
    lv_obj_align(s_btn_label, LV_ALIGN_CENTER, 0, -18);

    // Row of 8 columns (LED + hit counter). Plain flex containers with all
    // theme styling removed, so default padding/borders don't push things around.
    lv_obj_t *row = lv_obj_create(scr);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, LV_HOR_RES, LV_SIZE_CONTENT);
    lv_obj_align(row, LV_ALIGN_BOTTOM_MID, 0, -6);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    for (int i = 0; i < NUM_BUTTONS; i++) {
        lv_obj_t *col = lv_obj_create(row);
        lv_obj_remove_style_all(col);
        lv_obj_set_size(col, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(col, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_row(col, 4, 0);
        lv_obj_clear_flag(col, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *led = lv_obj_create(col);
        lv_obj_remove_style_all(led);
        lv_obj_set_size(led, 30, 30);
        lv_obj_set_style_radius(led, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(led, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(led, LED_OFF_COLOR, 0);
        lv_obj_clear_flag(led, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *num = lv_label_create(led);
        lv_label_set_text_fmt(num, "%d", i + 1);
        lv_obj_center(num);

        lv_obj_t *count = lv_label_create(col);
        lv_obj_set_style_text_font(count, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(count, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_label_set_text(count, "0");

        s_btn_led[i] = led;
        s_btn_count[i] = count;
    }

    s_btn_events = xQueueCreate(32, sizeof(uint8_t));
    assert(s_btn_events);
    lv_timer_create(button_ui_timer_cb, BUTTON_UI_MS, NULL);
}

// Runs inside lv_timer_handler(), so the LVGL lock is already held.
static void button_ui_timer_cb(lv_timer_t *timer)
{
    static uint32_t flash_until[NUM_BUTTONS];
    static uint32_t shown_hits[NUM_BUTTONS];
    static bool shown_on[NUM_BUTTONS];
    uint32_t now = lv_tick_get();

    uint8_t idx;
    while (xQueueReceive(s_btn_events, &idx, 0) == pdTRUE) {   // last hit wins the big label
        flash_until[idx] = now + BUTTON_FLASH_MS;
        lv_obj_set_style_text_font(s_btn_label, &lv_font_montserrat_40, 0);
        lv_label_set_text_fmt(s_btn_label, "Button %d", idx + 1);
        lv_obj_align(s_btn_label, LV_ALIGN_CENTER, 0, -18);   // re-center for new width
    }

    uint8_t held = s_btn_held;
    for (int i = 0; i < NUM_BUTTONS; i++) {
        bool on = (held & (1 << i)) || (int32_t)(flash_until[i] - now) > 0;
        if (on != shown_on[i]) {
            lv_obj_set_style_bg_color(s_btn_led[i], on ? LED_ON_COLOR : LED_OFF_COLOR, 0);
            shown_on[i] = on;
        }
        uint32_t hits = s_stats.hits[i];
        if (hits != shown_hits[i]) {
            lv_label_set_text_fmt(s_btn_count[i], "%lu", (unsigned long)hits);
            shown_hits[i] = hits;
        }
    }
}

// Debounce tuned for drum triggers (arcade microswitches):
//  - A hit fires on the FIRST read that sees the button down -> latency is
//    at most one poll interval, and short taps can't be missed.
//  - After that the button is "held" and ignores all bounce until it has read
//    released continuously for BUTTON_RELEASE_MS. Only then can it fire again.
// No logging in here: a console write can block for tens of ms when no
// serial monitor is attached, which would stall polling.
static void button_poll_task(void *arg)
{
    uint8_t held = 0;                       // bit i set = button i held
    uint8_t prev_down = 0;                  // last raw reading, for raw edge stats
    int64_t up_since[NUM_BUTTONS] = {0};    // held buttons: when they started reading released (0 = reading down)
    int64_t hit_at[NUM_BUTTONS] = {0};
    int64_t last_poll = 0;
    TickType_t last_wake = xTaskGetTickCount();

    for (int i = 0; i < NUM_BUTTONS; i++) {
        s_stats.min_press_us[i] = UINT32_MAX;
    }

    while (1) {
        int64_t now = esp_timer_get_time();
        if (last_poll && (uint32_t)(now - last_poll) > s_stats.max_gap_us) {
            s_stats.max_gap_us = now - last_poll;
        }
        last_poll = now;

        uint16_t port;
        if (pcf8575_read(&port) == ESP_OK) {
            uint8_t down = ~port & 0xFF;    // active low: 0 on the pin = pressed
            uint8_t new_lows = down & ~prev_down;
            prev_down = down;

            for (int i = 0; i < NUM_BUTTONS; i++) {
                uint8_t bit = 1 << i;
                if (new_lows & bit) {
                    s_stats.raw_lows[i] = s_stats.raw_lows[i] + 1;
                }
                if (!(held & bit)) {
                    if (down & bit) {                       // new hit
                        held |= bit;
                        up_since[i] = 0;
                        hit_at[i] = now;
                        s_stats.hits[i] = s_stats.hits[i] + 1;
                        uint8_t idx = i;
                        if (s_btn_events) {
                            xQueueSend(s_btn_events, &idx, 0);  // drop if UI is behind
                        }
                    }
                } else if (down & bit) {
                    up_since[i] = 0;                        // still down, or bouncing
                } else if (up_since[i] == 0) {
                    up_since[i] = now;                      // first released read
                } else if (now - up_since[i] >= BUTTON_RELEASE_MS * 1000) {
                    held &= ~bit;                           // released for good: re-arm
                    uint32_t press = up_since[i] - hit_at[i];
                    if (press < s_stats.min_press_us[i]) {
                        s_stats.min_press_us[i] = press;
                    }
                }
            }
            s_btn_held = held;
        } else {
            s_stats.read_errors = s_stats.read_errors + 1;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

// Low-priority logger: once there's been any activity, prints the cumulative
// stats every 2 s (repeating, so a monitor attached later still sees them).
// raw > hits is normal (bounce); hits < actual presses means a miss.
static void button_stats_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        uint32_t total = 0;
        for (int i = 0; i < NUM_BUTTONS; i++) {
            total += s_stats.raw_lows[i];
        }
        if (total == 0 && s_stats.read_errors == 0) {
            continue;
        }

        char line[256] = "";
        int n = 0;
        for (int i = 0; i < NUM_BUTTONS && n < (int)sizeof(line); i++) {
            if (!s_stats.raw_lows[i]) {
                continue;
            }
            uint32_t mp = s_stats.min_press_us[i];
            n += snprintf(line + n, sizeof(line) - n, "B%d hits=%lu raw=%lu min=%s%lums  ",
                          i + 1, (unsigned long)s_stats.hits[i], (unsigned long)s_stats.raw_lows[i],
                          mp == UINT32_MAX ? "-" : "", mp == UINT32_MAX ? 0UL : (unsigned long)(mp / 1000));
        }
        ESP_LOGI(TAG, "%s| max_gap=%lu.%lums read_err=%lu", line,
                 (unsigned long)(s_stats.max_gap_us / 1000), (unsigned long)(s_stats.max_gap_us % 1000 / 100),
                 (unsigned long)s_stats.read_errors);
    }
}


extern "C" void app_main(void)
{

    ESP_LOGI(TAG, "------ Initialize I2C.");
    i2c_driver_init();

    ESP_LOGI(TAG, "------ Initialize PMU.");
    if (!power_driver_init()) {
        ESP_LOGE(TAG, "ERROR :No find PMU ....");
    }

    ESP_LOGI(TAG, "------ Initialize TOUCH.");
    touch_init();

    ESP_LOGI(TAG, "------ Initialize DISPLAY.");
    display_init();

    ESP_LOGI(TAG, "------ Initialize SD CARD.");
    sd_card_init();     // mounted at /sdcard; result is logged by the driver

    ESP_LOGI(TAG, "------ Initialize IO EXPANDER.");
    esp_err_t exp_result = pcf8575_init();


    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();


    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
#if CONFIG_SPIRAM

// #if CONFIG_LILYGO_T_RGB
//     // initialize LVGL draw buffers

//     extern void *buf1;
//     extern void *buf2;

//     lv_disp_draw_buf_init(&disp_buf, buf1, buf2, DISPLAY_BUFFER_SIZE);
// #else
    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(DISPLAY_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);

    lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(DISPLAY_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf2);

    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, DISPLAY_BUFFER_SIZE);
// #endif
#else
    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(AMOLED_HEIGHT * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = (lv_color_t *) heap_caps_malloc(AMOLED_HEIGHT * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, AMOLED_HEIGHT * 20);
#endif

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = AMOLED_HEIGHT;
    disp_drv.ver_res = AMOLED_WIDTH;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.full_refresh = DISPLAY_FULLRESH;
    lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick",
        .skip_unhandled_events = false
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

#if BOARD_HAS_TOUCH
    ESP_LOGI(TAG, "Register touch driver to LVGL");
    static lv_indev_drv_t indev_drv; // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = example_lvgl_touch_cb;
    lv_indev_drv_register(&indev_drv);
#endif

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);


    ESP_LOGI(TAG, "Display LVGL");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1)) {

// #if   CONFIG_USE_DEMO_WIDGETS
        // lv_demo_widgets();
// #elif CONFIG_USE_DEMO_BENCHMARK
        // lv_demo_benchmark();
// #elif CONFIG_USE_DEMO_STRESS
//         lv_demo_stress();
// #elif CONFIG_USE_DEMO_MUSIC
        if (exp_result == ESP_ERR_NOT_SUPPORTED) {
            lv_demo_music();    // boards without the expander
        } else {
            button_test_screen_create(exp_result);
        }
// #endif
        // Release the mutex
        example_lvgl_unlock();
    }
    // tskIDLE_PRIORITY,
    ESP_LOGI(TAG, "Create LVGL task");
    // xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

    if (exp_result == ESP_OK) {
        xTaskCreate(button_poll_task, "buttons", 4096, NULL, BUTTON_TASK_PRIO, NULL);
        xTaskCreate(button_stats_task, "btn_stats", 4096, NULL, 1, NULL);
    }

}
