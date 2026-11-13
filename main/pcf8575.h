/**
 * @file      pcf8575.h
 * @brief     Minimal driver for PCF8575 16-bit quasi-bidirectional I/O expanders.
 *
 * Expanders live on the shared external bus (see ext_i2c.h), each at its own
 * address (0x20-0x27 via A2..A0). Call ext_i2c_init() first.
 *
 * Bit layout of the 16-bit port value: bit 0..7 = P00..P07, bit 8..15 = P10..P17.
 * A pin written as 1 is weakly pulled high and can be used as an input;
 * a switch to GND then reads 0 while closed.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pcf8575_dev *pcf8575_handle_t;

/**
 * Attach the expander at `addr` and release all its pins high (inputs).
 * Returns ESP_ERR_NOT_FOUND if nothing answers at that address.
 */
esp_err_t pcf8575_open(uint8_t addr, pcf8575_handle_t *out);

/** Read all 16 pins. */
esp_err_t pcf8575_read(pcf8575_handle_t dev, uint16_t *value);

/** Write all 16 pins (1 = released/input, 0 = driven low). */
esp_err_t pcf8575_write(pcf8575_handle_t dev, uint16_t value);

/** I2C address of an opened device. */
uint8_t pcf8575_address(pcf8575_handle_t dev);

#ifdef __cplusplus
}
#endif
