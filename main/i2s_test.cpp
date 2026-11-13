
#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s_std.h"
#include "esp_err.h"

// ================= I2S Configuration =================

#define I2S_PORT        I2S_NUM_0

#define I2S_BCLK        GPIO_NUM_13
#define I2S_LRCK        GPIO_NUM_10
#define I2S_DOUT        GPIO_NUM_42

#define SAMPLE_RATE     44100
#define TONE_FREQ       440.0f
#define AMPLITUDE       10000

#define SAMPLES_PER_BUFFER 256

// ================= Global Variables =================

static i2s_chan_handle_t tx_handle = NULL;

static int16_t audio_buffer[SAMPLES_PER_BUFFER * 2];

static float phase = 0.0f;

static const float phase_increment =
    2.0f * M_PI * TONE_FREQ / SAMPLE_RATE;

// ================= I2S Initialization =================

static void init_i2s(void)
{
    // Configure I2S channel
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_PORT,
            I2S_ROLE_MASTER
        );

    chan_cfg.dma_desc_num = 8;
    chan_cfg.dma_frame_num = SAMPLES_PER_BUFFER;

    ESP_ERROR_CHECK(
        i2s_new_channel(
            &chan_cfg,
            &tx_handle,
            NULL
        )
    );

    // Configure I2S standard mode
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),

        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO
        ),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK,
            .ws   = I2S_LRCK,
            .dout = I2S_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false
            }
        }
    };

    ESP_ERROR_CHECK(
        i2s_channel_init_std_mode(
            tx_handle,
            &std_cfg
        )
    );

    ESP_ERROR_CHECK(
        i2s_channel_enable(tx_handle)
    );

    printf("I2S initialized successfully!\n");
}

// ================= Generate Sine Wave =================

static void generate_tone(void)
{
    for (int i = 0; i < SAMPLES_PER_BUFFER; i++)
    {
        // Generate 440 Hz sine wave sample
        int16_t sample = (int16_t)(
            AMPLITUDE * sinf(phase)
        );

        // Left and right channels
        audio_buffer[2 * i]     = sample;
        audio_buffer[2 * i + 1] = sample;

        // Advance phase
        phase += phase_increment;

        if (phase >= 2.0f * M_PI)
        {
            phase -= 2.0f * M_PI;
        }
    }
}

// ================= Main Application =================

extern "C" void app_main(void)
{
    printf("Starting 440 Hz I2S test tone...\n");

    // Initialize I2S peripheral
    init_i2s();

    while (1)
    {
        size_t bytes_written = 0;

        // Generate next audio buffer
        generate_tone();

        // Transmit audio over I2S
        esp_err_t result = i2s_channel_write(
            tx_handle,
            audio_buffer,
            sizeof(audio_buffer),
            &bytes_written,
            portMAX_DELAY
        );

        if (result != ESP_OK)
        {
            printf(
                "I2S write error: %s\n",
                esp_err_to_name(result)
            );
        }
    }
}
