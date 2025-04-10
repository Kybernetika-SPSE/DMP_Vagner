#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO          38      // GPIO for I2C clock
#define I2C_MASTER_SDA_IO          39      // GPIO for I2C data
#define I2C_MASTER_NUM             I2C_NUM_0 // I2C port number for master
#define I2C_MASTER_FREQ_HZ         100000  // I2C master clock frequency
#define I2C_MASTER_TIMEOUT_MS      1000

#define SLAVE_ADDR                 0x02    // Slave address (must match slave code)
#define CMD_AUDIO_REQUEST          0xA1    // Command to request audio data
#define AUDIO_CHUNK_SIZE           128     // Expected audio data size (bytes)

static const char *TAG = "I2C_MASTER_AUDIO";

/* I2C Master initialization */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
        // .clk_flags = 0, // Optional: use default clock flags
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master parameter configuration failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

/* Task to request audio data from the slave and process it */
void i2c_master_audio_task(void *arg)
{
    uint8_t audio_data[AUDIO_CHUNK_SIZE];

    while (1) {
        esp_err_t ret;
        // Build command to send the audio request command to the slave.
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        // Write slave address with WRITE flag
        i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
        // Write the command byte
        i2c_master_write_byte(cmd, CMD_AUDIO_REQUEST, true);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error sending audio request: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "Audio request sent");

        // Now read the audio data from the slave.
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        // Write slave address with READ flag
        i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | I2C_MASTER_READ, true);
        // Read AUDIO_CHUNK_SIZE bytes and send NACK after the last byte.
        i2c_master_read(cmd, audio_data, AUDIO_CHUNK_SIZE, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error reading audio data: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Received %d bytes of audio data", AUDIO_CHUNK_SIZE);
            // For demonstration, log the first few bytes of the received data.
            ESP_LOGI(TAG, "Data: 0x%02X 0x%02X 0x%02X 0x%02X...",
                     audio_data[0], audio_data[1], audio_data[2], audio_data[3]);
            // Further processing of the audio_data buffer can be done here.
        }
        // Delay before the next request.
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting I2C Master Audio Example");
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C master initialization failed");
        return;
    }
    // Create the task that handles audio data requests.
    xTaskCreate(i2c_master_audio_task, "i2c_master_audio_task", 4096, NULL, 10, NULL);
}
