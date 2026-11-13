/**
 * @file      ads1115.h
 * @brief     Minimal driver for the ADS1115 4-channel 16-bit ADC on the external bus.
 *
 * Single-ended, single-shot reads of AIN0-AIN3 against GND at the +/-4.096 V
 * range (0.125 mV per bit). Inputs must stay between GND and the ADS1115's VDD.
 * Call ext_i2c_init() first.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ads1115_dev *ads1115_handle_t;

/** Attach the ADS1115 at `addr` (0x48-0x4B). ESP_ERR_NOT_FOUND if it doesn't answer. */
esp_err_t ads1115_open(uint8_t addr, ads1115_handle_t *out);

/**
 * Convert one channel (0-3) and return millivolts (clamped at 0).
 * Blocks ~1.5 ms while the conversion runs; the bus is free during the wait.
 */
esp_err_t ads1115_read_mv(ads1115_handle_t dev, int channel, int *mv);

#ifdef __cplusplus
}
#endif
