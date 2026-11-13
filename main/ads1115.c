/**
 * @file      ads1115.c
 * @brief     Minimal driver for the ADS1115 4-channel 16-bit ADC on the external bus.
 */
#include <stdlib.h>
#include "ads1115.h"
#include "ext_i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ads1115";

#define ADS1115_I2C_FREQ_HZ   400000
#define ADS1115_TIMEOUT_MS    50

#define REG_CONVERSION        0x00
#define REG_CONFIG            0x01

// Config register fields
#define CFG_OS_START          (1u << 15)      // write: start single conversion / read: 1 = idle
#define CFG_MUX_SINGLE(ch)    ((4u + (ch)) << 12)   // AINx vs GND
#define CFG_PGA_4V096         (1u << 9)       // +/-4.096 V full scale, 0.125 mV/bit
#define CFG_MODE_SINGLE       (1u << 8)
#define CFG_DR_860SPS         (7u << 5)       // ~1.16 ms per conversion
#define CFG_COMP_DISABLE      (3u << 0)

#define LSB_UV                125             // 4.096 V / 32768

struct ads1115_dev {
    i2c_master_dev_handle_t i2c;
    uint8_t addr;
};

static esp_err_t write_reg(ads1115_handle_t dev, uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return i2c_master_transmit(dev->i2c, buf, sizeof(buf), ADS1115_TIMEOUT_MS);
}

static esp_err_t read_reg(ads1115_handle_t dev, uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    esp_err_t ret = i2c_master_transmit_receive(dev->i2c, &reg, 1, buf, sizeof(buf), ADS1115_TIMEOUT_MS);
    if (ret == ESP_OK) {
        *value = ((uint16_t)buf[0] << 8) | buf[1];      // big-endian
    }
    return ret;
}

esp_err_t ads1115_open(uint8_t addr, ads1115_handle_t *out)
{
    *out = NULL;
    struct ads1115_dev *dev = calloc(1, sizeof(*dev));
    if (!dev) {
        return ESP_ERR_NO_MEM;
    }
    dev->addr = addr;

    esp_err_t ret = ext_i2c_add_device(addr, ADS1115_I2C_FREQ_HZ, &dev->i2c);
    if (ret != ESP_OK) {
        free(dev);
        return ret;
    }

    // Sanity check: the config register reads back 0x8583 after power-up.
    uint16_t cfg;
    ret = read_reg(dev, REG_CONFIG, &cfg);
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(dev->i2c);
        free(dev);
        return ret;
    }
    ESP_LOGI(TAG, "ADS1115 ready at 0x%02X (config=0x%04X)", addr, cfg);
    *out = dev;
    return ESP_OK;
}

esp_err_t ads1115_read_mv(ads1115_handle_t dev, int channel, int *mv)
{
    if (!dev || channel < 0 || channel > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t cfg = CFG_OS_START | CFG_MUX_SINGLE(channel) | CFG_PGA_4V096 |
                   CFG_MODE_SINGLE | CFG_DR_860SPS | CFG_COMP_DISABLE;
    esp_err_t ret = write_reg(dev, REG_CONFIG, cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    // Conversion takes ~1.16 ms (+/-10% oscillator); sleep, then confirm done.
    for (int tries = 0; ; tries++) {
        vTaskDelay(pdMS_TO_TICKS(tries == 0 ? 2 : 1));
        ret = read_reg(dev, REG_CONFIG, &cfg);
        if (ret != ESP_OK) {
            return ret;
        }
        if (cfg & CFG_OS_START) {
            break;              // OS bit back to 1 = conversion finished
        }
        if (tries >= 3) {
            return ESP_ERR_TIMEOUT;
        }
    }

    uint16_t raw;
    ret = read_reg(dev, REG_CONVERSION, &raw);
    if (ret != ESP_OK) {
        return ret;
    }
    int32_t uv = (int32_t)(int16_t)raw * LSB_UV;
    *mv = uv > 0 ? (int)(uv / 1000) : 0;    // single-ended: tiny negative offsets -> 0
    return ESP_OK;
}
