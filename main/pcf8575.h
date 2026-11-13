/**
 * @file      pcf8575.h
 * @brief     Minimal driver for the PCF8575 16-bit quasi-bidirectional I/O expander.
 *
 * Bit layout of the 16-bit port value: bit 0..7 = P00..P07, bit 8..15 = P10..P17.
 * A pin written as 1 is weakly pulled high and can be used as an input;
 * a button to GND then reads 0 while pressed.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Create a dedicated I2C bus on BOARD_EXP_SDA/SCL, find the PCF8575 at
 * 0x20-0x27, and release all pins high so they act as inputs.
 * Returns ESP_ERR_NOT_SUPPORTED if the board has no BOARD_EXP_* pins,
 * ESP_ERR_NOT_FOUND if no device answers.
 */
esp_err_t pcf8575_init(void);

/** Read all 16 pins. */
esp_err_t pcf8575_read(uint16_t *value);

/** Write all 16 pins (1 = released/input, 0 = driven low). */
esp_err_t pcf8575_write(uint16_t value);

/** I2C address the device was found at (0 if not initialized). */
uint8_t pcf8575_address(void);

#ifdef __cplusplus
}
#endif
