/**
 * @file      pcf8575.c
 * @brief     Minimal driver for the PCF8575 16-bit I/O expander.
 */
#include "pcf8575.h"
#include "product_pins.h"
#include "esp_log.h"

static const char *TAG = "pcf8575";

#if defined(BOARD_EXP_SDA) && defined(BOARD_EXP_SCL)

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PCF8575_I2C_PORT     I2C_NUM_1      // port 0 is the onboard touch/Qwiic bus
#define PCF8575_I2C_FREQ_HZ  400000         // PCF8575 max; a 2-byte read takes ~70 us
#define PCF8575_ADDR_FIRST   0x20           // A2..A0 select 0x20..0x27
#define PCF8575_ADDR_LAST    0x27
#define PCF8575_TIMEOUT_MS   50

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;
static uint8_t s_addr = 0;

// ---------------------------------------------------------------------------
// Diagnostics, run when the device isn't found. Logs whether the problem is
// most likely on the breakout (power / pull-ups / address) or the wiring.
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
        .i2c_port = PCF8575_I2C_PORT,
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

static void pcf8575_diagnose(void)
{
    ESP_LOGW(TAG, "---- PCF8575 diagnostics ----");
    diag_pin(BOARD_EXP_SDA, "SDA");
    diag_pin(BOARD_EXP_SCL, "SCL");
    diag_scan(BOARD_EXP_SDA, BOARD_EXP_SCL, "as wired");
    diag_scan(BOARD_EXP_SCL, BOARD_EXP_SDA, "SDA/SCL swapped");
    ESP_LOGW(TAG, "-----------------------------");
}

esp_err_t pcf8575_init(void)
{
    if (s_dev) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = PCF8575_I2C_PORT,
        .sda_io_num = BOARD_EXP_SDA,
        .scl_io_num = BOARD_EXP_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // Weak (~45k) backup; the module's own pull-ups do the real work.
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    for (uint8_t addr = PCF8575_ADDR_FIRST; addr <= PCF8575_ADDR_LAST; addr++) {
        if (i2c_master_probe(s_bus, addr, PCF8575_TIMEOUT_MS) == ESP_OK) {
            s_addr = addr;
            break;
        }
    }
    if (!s_addr) {
        ESP_LOGE(TAG, "No PCF8575 found at 0x%02X-0x%02X (SDA=%d SCL=%d)",
                 PCF8575_ADDR_FIRST, PCF8575_ADDR_LAST, BOARD_EXP_SDA, BOARD_EXP_SCL);
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        pcf8575_diagnose();
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_addr,
        .scl_speed_hz = PCF8575_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        s_addr = 0;
        return ret;
    }

    // Release every pin high so all 16 can be read as inputs.
    ret = pcf8575_write(0xFFFF);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PCF8575 ready at 0x%02X", s_addr);
    }
    return ret;
}

esp_err_t pcf8575_read(uint16_t *value)
{
    if (!s_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[2];
    esp_err_t ret = i2c_master_receive(s_dev, buf, sizeof(buf), PCF8575_TIMEOUT_MS);
    if (ret == ESP_OK) {
        *value = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);   // first byte = P0x
    }
    return ret;
}

esp_err_t pcf8575_write(uint16_t value)
{
    if (!s_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[2] = { (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), PCF8575_TIMEOUT_MS);
}

uint8_t pcf8575_address(void)
{
    return s_addr;
}

#else  // no expander on this board

esp_err_t pcf8575_init(void)
{
    ESP_LOGW(TAG, "No PCF8575 pins defined for this board");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t pcf8575_read(uint16_t *value)
{
    (void)value;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t pcf8575_write(uint16_t value)
{
    (void)value;
    return ESP_ERR_NOT_SUPPORTED;
}

uint8_t pcf8575_address(void)
{
    return 0;
}

#endif
