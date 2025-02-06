#include <stdio.h>
#include <inttypes.h> // For PRIu32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_log.h"

// Constants
#define I2S_NUM         I2S_NUM_0
#define I2S_SCK_PIN     37  // I²S Clock Pin
#define I2S_WS_PIN      38  // I²S Word Select Pin
#define I2S_SD_PIN      39  // I²S Data Pin
#define SAMPLE_RATE     16000
#define FFT_SIZE        256

// Logging tag
static const char *TAG = "BeeHiveMonitor";

// Function Prototypes
void i2s_audio_init();
void simple_task(void *arg);

/* I2S Initialization */
void i2s_audio_init() {
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = FFT_SIZE,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN,
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM, &pin_config));
    ESP_LOGI(TAG, "I²S audio initialized.");
}

/* Simple Task for Testing */
void simple_task(void *arg) {
    while (1) {
        ESP_LOGI(TAG, "Simple task is running. Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* Main Function */
void app_main(void) {
    ESP_LOGI(TAG, "Initializing Bee Hive Monitor...");

    // Initialize I2S for audio
    i2s_audio_init();

    // Log free heap memory
    ESP_LOGI(TAG, "Free heap before tasks: %lu bytes", (unsigned long)esp_get_free_heap_size());

    // Start simple task
    if (xTaskCreate(simple_task, "simple_task", 4096, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create simple_task");
    } else {
        ESP_LOGI(TAG, "simple_task created successfully.");
    }

    // Keep the main function alive
    while (1) {
        ESP_LOGI(TAG, "Bee Hive Monitor is running. Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
