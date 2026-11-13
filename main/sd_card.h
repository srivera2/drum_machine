/**
 * @file      sd_card.h
 * @brief     SD card over SPI, mounted as a FAT filesystem via the ESP-IDF VFS.
 *
 * Once mounted, use the standard C file APIs (fopen/fread/opendir/readdir...)
 * with paths under SD_MOUNT_POINT, e.g. "/sdcard/kick.wav".
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#define SD_MOUNT_POINT "/sdcard"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the SPI bus and mount the card at SD_MOUNT_POINT.
 * Never formats the card. Returns ESP_ERR_NOT_SUPPORTED on boards without
 * BOARD_SD_* pins defined in product_pins.h.
 */
esp_err_t sd_card_init(void);

/** Unmount the card and free the SPI bus. */
void sd_card_deinit(void);

bool sd_card_is_mounted(void);

#ifdef __cplusplus
}
#endif
