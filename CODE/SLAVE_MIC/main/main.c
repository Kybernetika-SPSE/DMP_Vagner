#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"

// I2S Configuration (from your working microphone code)
#define I2S_NUM         I2S_NUM_0
#define SAMPLE_RATE     16000   // 16kHz sample rate
#define BUFFER_SIZE     512     // DMA buffer length (not the chunk size for I2C)

#define I2S_SCK_PIN     37      // I2S Clock Pin (BCLK)
#define I2S_WS_PIN      38      // I2S Word Select (LRCLK)
#define I2S_SD_PIN      39      // I2S Data Pin

// I2C Slave Configuration
#define I2C_SLAVE_NUM             I2C_NUM_1
#define I2C_SLAVE_ADDRESS         0x02    // Slave address
#define I2C_SLAVE_SDA_IO          40      // I2C SDA pin (choose pins not conflicting with I2S)
#define I2C_SLAVE_SCL_IO          39      // I2C SCL pin
#define I2C_SLAVE_RX_BUF_LEN      512
#define I2C_SLAVE_TX_BUF_LEN      512
#define I2C_OUT                   38  // Define the GPIO pin number
// Command from master to request audio data
#define CMD_AUDIO_REQUEST         0xA1

static const char *TAG = "MIC_I2C_SLAVE";

/* Initialize I2S for microphone input */
void init_i2s()
{
    esp_err_t err;
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,  // Receive mode
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Mono input
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = false
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_in_num = I2S_SD_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE  // Not used in RX mode
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

/* Initialize I2C as slave */
void i2c_slave_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave = {
            .addr_10bit_en = 0,
            .slave_addr = I2C_SLAVE_ADDRESS
        }
    };
    esp_err_t err = i2c_param_config(I2C_SLAVE_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return;
    }
    err = i2c_driver_install(I2C_SLAVE_NUM, conf.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "I2C slave initialized successfully");
}

void set_gpio_high()
{
    gpio_set_level(I2C_OUT, 1); // Set GPIO high
    ESP_LOGI("GPIO", "GPIO %d set to HIGH.", I2C_OUT);
}
/* I2C Slave Task: Waits for a command from the master.
   When the master sends CMD_AUDIO_REQUEST, read a chunk of audio from I2S and send it over I2C. */
void i2c_slave_task(void *arg)
{
    uint8_t cmd;
    // Define the size of the audio chunk to send over I2C.
    // Since samples are 16-bit, ensure this is an even number (here, 128 bytes).
    #define AUDIO_CHUNK_SIZE 128
    int16_t *audio_buffer = (int16_t *)heap_caps_malloc(AUDIO_CHUNK_SIZE, MALLOC_CAP_INTERNAL);
    if (audio_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate audio buffer");
        vTaskDelete(NULL);
    }
    size_t bytes_read;
    
    while(1) {
        // Wait for a command from the master (1 byte)
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, &cmd, 1, portMAX_DELAY);
        if (len > 0) {
            if (cmd == CMD_AUDIO_REQUEST) {
                // Read a chunk of audio data from the I2S microphone.
                esp_err_t ret = i2s_read(I2S_NUM, (void*)audio_buffer, AUDIO_CHUNK_SIZE, &bytes_read, pdMS_TO_TICKS(1000));
                if (ret == ESP_OK && bytes_read > 0) {
                    ESP_LOGI(TAG, "Read %d bytes from I2S", bytes_read);
                    // Send the audio data chunk over I2C to the master.
                    i2c_slave_write_buffer(I2C_SLAVE_NUM, (uint8_t*)audio_buffer, bytes_read, portMAX_DELAY);
                } else {
                    ESP_LOGW(TAG, "I2S read error or no data available");
                }
            } else {
                ESP_LOGI(TAG, "Received unknown command: 0x%X", cmd);
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting MIC_I2C_SLAVE");
    set_gpio_high();
    init_i2s();
    i2c_slave_init();
    // Allow time for I2S and I2C to stabilize.
    vTaskDelay(pdMS_TO_TICKS(1000));
    // Start the task that handles I2C requests and sends audio data.
    xTaskCreate(i2c_slave_task, "i2c_slave_task", 4096, NULL, 10, NULL);
}

