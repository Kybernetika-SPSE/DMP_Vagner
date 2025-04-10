#include <stdio.h>
#include "driver/i2s.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2S_NUM         I2S_NUM_0
#define SAMPLE_RATE     16000  // 16kHz sample rate
#define BUFFER_SIZE     512    // Reduce buffer size if needed

// I²S Pin Definitions
#define I2S_SCK_PIN     37  // I²S Clock Pin (BCLK)
#define I2S_WS_PIN      38  // I²S Word Select Pin (LRCLK)
#define I2S_SD_PIN      39  // I²S Data Pin (DATA)

static const char *TAG = "MIC_TEST";

void init_i2s()
{
    esp_err_t err;

    // I2S configuration
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,  // Use standard I2S mode
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = false
    };
    

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_in_num = I2S_SD_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE
    };

    err = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S driver install failed: %s", esp_err_to_name(err));
        return;
    }

    err = i2s_set_pin(I2S_NUM, &pin_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S set pin failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "I2S initialized successfully");
}

void mic_read_task(void *arg)
{
    ESP_LOGI(TAG, "Free heap before allocation: %lu bytes", (unsigned long)esp_get_free_heap_size());

    int16_t *audio_buffer = (int16_t *)heap_caps_malloc(BUFFER_SIZE * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    if (!audio_buffer)
    {
        ESP_LOGE(TAG, "Failed to allocate audio buffer");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Microphone buffer allocated successfully");

    size_t bytes_read;

    while (1)
    {
        i2s_read(I2S_NUM, audio_buffer, BUFFER_SIZE * sizeof(int16_t), &bytes_read, portMAX_DELAY);

        ESP_LOGI(TAG, "Audio Samples:");
        for (int i = 0; i < 10; i++) {
            printf("%d ", audio_buffer[i]);
        }
        printf("\n");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    init_i2s();
    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for I2S to stabilize

    xTaskCreatePinnedToCore(mic_read_task, "mic_read_task", 4096, NULL, 5, NULL, 1);
}
