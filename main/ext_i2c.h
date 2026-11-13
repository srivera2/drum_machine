/**
 * @file      ext_i2c.h
 * @brief     External I2C bus (BOARD_EXP_SDA/SCL, port 1) shared by add-on boards:
 *            PCF8575 expanders, ADS1115 ADC, ...
 *
 * Call ext_i2c_init() once, then each driver attaches its device with
 * ext_i2c_add_device(). The ESP-IDF I2C master driver serializes access, so
 * devices on this bus can be used from different tasks.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Create the bus and log every address that answers. If nothing answers,
 * runs wiring diagnostics (pull-ups / power / swapped lines) to the log.
 * Returns ESP_ERR_NOT_SUPPORTED if the board has no BOARD_EXP_* pins,
 * ESP_ERR_NOT_FOUND if no device answers.
 */
esp_err_t ext_i2c_init(void);

/** Probe `addr` and attach it at `scl_hz`. ESP_ERR_NOT_FOUND if it doesn't answer. */
esp_err_t ext_i2c_add_device(uint8_t addr, uint32_t scl_hz, i2c_master_dev_handle_t *out);

#ifdef __cplusplus
}
#endif
