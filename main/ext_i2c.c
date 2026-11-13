/**
 * @file      ext_i2c.c
 * @brief     External I2C bus shared by add-on boards (expanders, ADC, ...).
 */
#include <stdio.h>
#include "ext_i2c.h"
#include "product_pins.h"
#include "esp_log.h"

static const char *TAG = "ext_i2c";

#if defined(BOARD_EXP_SDA) && defined(BOARD_EXP_SCL)

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define EXT_I2C_PORT        I2C_NUM_1      // port 0 is the onboard touch/Qwiic bus
#define EXT_I2C_TIMEOUT_MS  50

static i2c_master_bus_handle_t s_bus = NULL;

// ---------------------------------------------------------------------------
// Diagnostics, run when nothing answers. Logs whether the problem is most
// likely on the breakout (power / pull-ups) or the wiring.
// ---------------------------------------------------------------------------
static void diag_pin(gpio_num_t pin, const char *name)
{
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_INPUT);

    gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);
    vTaskDelay(pdMS_TO_TICKS(5));
    int with_pd = gpio_get_level(pin);

    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(5));
    int with_pu = gpio_get_level(pin);

    gpio_reset_pin(pin);

    const char *verdict;
    if (with_pu == 0) {
        verdict = "held LOW -> breakout unpowered, short to GND, or stuck device";
    } else if (with_pd == 0) {
        verdict = "NO external pull-up -> breakout VCC not connected, or module has no pull-ups";
    } else {
        verdict = "external pull-up present (OK)";
    }
    ESP_LOGW(TAG, "  %s GPIO%-2d  int-pulldown=%d int-pullup=%d  => %s",
             name, pin, with_pd, with_pu, verdict);
}

static void diag_scan(int sda, int scl, const char *label)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = EXT_I2C_PORT,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    if (i2c_new_master_bus(&cfg, &bus) != ESP_OK) {
        ESP_LOGW(TAG, "  scan %s: could not create bus", label);
        return;
    }
    int found = 0, timeouts = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        esp_err_t r = i2c_master_probe(bus, addr, 20);
        if (r == ESP_OK) {
            ESP_LOGW(TAG, "  scan %s: device ACKed at 0x%02X", label, addr);
            found++;
        } else if (r == ESP_ERR_TIMEOUT) {
            timeouts++;
        }
    }
    i2c_del_master_bus(bus);
    ESP_LOGW(TAG, "  scan %s (SDA=%d SCL=%d): %d device(s)%s", label, sda, scl, found,
             timeouts ? ", bus TIMEOUTS -> a line is stuck low" : "");
}

static void ext_i2c_diagnose(void)
{
    ESP_LOGW(TAG, "---- I2C bus diagnostics ----");
    diag_pin(BOARD_EXP_SDA, "SDA");
    diag_pin(BOARD_EXP_SCL, "SCL");
    diag_scan(BOARD_EXP_SDA, BOARD_EXP_SCL, "as wired");
    diag_scan(BOARD_EXP_SCL, BOARD_EXP_SDA, "SDA/SCL swapped");
    ESP_LOGW(TAG, "-----------------------------");
}

esp_err_t ext_i2c_init(void)
{
    if (s_bus) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = EXT_I2C_PORT,
        .sda_io_num = BOARD_EXP_SDA,
        .scl_io_num = BOARD_EXP_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // Weak (~45k) backup; the modules' own pull-ups do the real work.
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        s_bus = NULL;
        return ret;
    }

    char found[128] = "";
    int n = 0, count = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(s_bus, addr, EXT_I2C_TIMEOUT_MS) == ESP_OK) {
            if (n < (int)sizeof(found)) {
                n += snprintf(found + n, sizeof(found) - n, " 0x%02X", addr);
            }
            count++;
        }
    }
    if (!count) {
        ESP_LOGE(TAG, "No devices on bus (SDA=%d SCL=%d)", BOARD_EXP_SDA, BOARD_EXP_SCL);
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        ext_i2c_diagnose();
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Devices on bus (SDA=%d SCL=%d):%s", BOARD_EXP_SDA, BOARD_EXP_SCL, found);
    return ESP_OK;
}

esp_err_t ext_i2c_add_device(uint8_t addr, uint32_t scl_hz, i2c_master_dev_handle_t *out)
{
    *out = NULL;
    if (!s_bus) {
        return ESP_ERR_INVALID_STATE;
    }
    if (i2c_master_probe(s_bus, addr, EXT_I2C_TIMEOUT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "Nothing answering at 0x%02X", addr);
        return ESP_ERR_NOT_FOUND;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = scl_hz,
    };
    return i2c_master_bus_add_device(s_bus, &dev_cfg, out);
}

#else  // no external bus on this board

esp_err_t ext_i2c_init(void)
{
    ESP_LOGW(TAG, "No external I2C pins defined for this board");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ext_i2c_add_device(uint8_t addr, uint32_t scl_hz, i2c_master_dev_handle_t *out)
{
    (void)addr;
    (void)scl_hz;
    *out = NULL;
    return ESP_ERR_NOT_SUPPORTED;
}

#endif
