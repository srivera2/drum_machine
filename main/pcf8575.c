/**
 * @file      pcf8575.c
 * @brief     Minimal driver for PCF8575 16-bit I/O expanders on the external bus.
 */
#include <stdlib.h>
#include "pcf8575.h"
#include "ext_i2c.h"
#include "esp_log.h"

static const char *TAG = "pcf8575";

#define PCF8575_I2C_FREQ_HZ  400000         // PCF8575 max; a 2-byte read takes ~70 us
#define PCF8575_TIMEOUT_MS   50

struct pcf8575_dev {
    i2c_master_dev_handle_t i2c;
    uint8_t addr;
};

esp_err_t pcf8575_open(uint8_t addr, pcf8575_handle_t *out)
{
    *out = NULL;
    struct pcf8575_dev *dev = calloc(1, sizeof(*dev));
    if (!dev) {
        return ESP_ERR_NO_MEM;
    }
    dev->addr = addr;

    esp_err_t ret = ext_i2c_add_device(addr, PCF8575_I2C_FREQ_HZ, &dev->i2c);
    if (ret != ESP_OK) {
        free(dev);
        return ret;
    }

    // Release every pin high so all 16 can be read as inputs.
    ret = pcf8575_write(dev, 0xFFFF);
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device(dev->i2c);
        free(dev);
        return ret;
    }
    ESP_LOGI(TAG, "PCF8575 ready at 0x%02X", addr);
    *out = dev;
    return ESP_OK;
}

esp_err_t pcf8575_read(pcf8575_handle_t dev, uint16_t *value)
{
    if (!dev) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[2];
    esp_err_t ret = i2c_master_receive(dev->i2c, buf, sizeof(buf), PCF8575_TIMEOUT_MS);
    if (ret == ESP_OK) {
        *value = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);   // first byte = P0x
    }
    return ret;
}

esp_err_t pcf8575_write(pcf8575_handle_t dev, uint16_t value)
{
    if (!dev) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[2] = { (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
    return i2c_master_transmit(dev->i2c, buf, sizeof(buf), PCF8575_TIMEOUT_MS);
}

uint8_t pcf8575_address(pcf8575_handle_t dev)
{
    return dev ? dev->addr : 0;
}
