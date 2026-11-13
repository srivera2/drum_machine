/**
 * @file      sd_card.c
 * @brief     SD card over SPI, mounted as a FAT filesystem via the ESP-IDF VFS.
 */
#include <stdio.h>
#include "sd_card.h"
#include "product_pins.h"
#include "esp_log.h"

// Only boards with a dedicated SD SPI bus define all four BOARD_SD_* pins.
#if defined(BOARD_SD_CS) && defined(BOARD_SD_MOSI) && defined(BOARD_SD_MISO) && defined(BOARD_SD_SCLK)
#define SD_CARD_SUPPORTED 1
#else
#define SD_CARD_SUPPORTED 0
#endif

static const char *TAG = "sd_card";

#if SD_CARD_SUPPORTED

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define SD_SPI_HOST SPI2_HOST

static sdmmc_card_t *s_card = NULL;

esp_err_t sd_card_init(void)
{
    if (s_card) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing SD card (SPI) CS=%d MOSI=%d SCLK=%d MISO=%d",
             BOARD_SD_CS, BOARD_SD_MOSI, BOARD_SD_SCLK, BOARD_SD_MISO);

    // Many breakout readers lack a pull-up on MISO; the internal one helps
    // the card respond reliably during the SPI-mode init sequence.
    gpio_set_pull_mode(BOARD_SD_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BOARD_SD_CS, GPIO_PULLUP_ONLY);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = BOARD_SD_MOSI,
        .miso_io_num = BOARD_SD_MISO,
        .sclk_io_num = BOARD_SD_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    esp_err_t ret = spi_bus_initialize(SD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = BOARD_SD_CS;
    slot_config.host_id = SD_SPI_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,   // never wipe the user's samples
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };

    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem (is the card FAT/exFAT formatted?)");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). Check wiring and pull-ups.",
                     esp_err_to_name(ret));
        }
        s_card = NULL;
        spi_bus_free(SD_SPI_HOST);
        return ret;
    }

    ESP_LOGI(TAG, "SD card mounted at %s", SD_MOUNT_POINT);
    sdmmc_card_print_info(stdout, s_card);
    return ESP_OK;
}

void sd_card_deinit(void)
{
    if (!s_card) {
        return;
    }
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    s_card = NULL;
    spi_bus_free(SD_SPI_HOST);
    ESP_LOGI(TAG, "SD card unmounted");
}

bool sd_card_is_mounted(void)
{
    return s_card != NULL;
}

#else  // !SD_CARD_SUPPORTED

esp_err_t sd_card_init(void)
{
    ESP_LOGW(TAG, "SD card not supported on this board configuration");
    return ESP_ERR_NOT_SUPPORTED;
}

void sd_card_deinit(void) {}

bool sd_card_is_mounted(void)
{
    return false;
}

#endif
