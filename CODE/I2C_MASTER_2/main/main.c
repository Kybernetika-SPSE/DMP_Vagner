#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "led_strip.h"

#define MAX_HIVE_ID                1
#define interval                   60000  // Interval between requests in milliseconds

#define I2C_MASTER_SCL_IO          38  // GPIO for I2C clock
#define I2C_MASTER_SDA_IO          39  // GPIO for I2C data
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000
#define I2C_MASTER_TIMEOUT_MS      1000


#define RGB_LED_PIN                42  // GPIO for SK6812 RGB LED
#define MOSFET_PIN                 45  // GPIO for MOSFET control

#define WIFI_SSID                  "This"  // Replace with your Wi-Fi SSID
#define WIFI_PASS                  "12345679"  // Replace with your Wi-Fi password
#define WEB_SERVER_URL             "http://smarthives.fun/insert_data.php"  // Replace with your endpoint URL

static const char *TAG = "I2C_MASTER";
static led_strip_handle_t led_strip;

// Function Prototypes
void wifi_init(void);
void send_data_to_server(uint8_t slave_id, float temperature, float humidity, int32_t pressure, int32_t raw_weight);

/* Configure GPIO as output */
void configure_gpio(int gpio_num, int initial_level)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(gpio_num, initial_level);
    ESP_LOGI(TAG, "Configured GPIO %d as output with initial level %d.", gpio_num, initial_level);
}

/* LED Strip Configuration */
void configure_rgb_led()
{
    ESP_LOGI(TAG, "Configuring SK6812 RGB LED...");
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_PIN,
        .max_leds = 1, // At least one LED
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

/* Set RGB LED Color */
void set_led_color(uint8_t red, uint8_t green, uint8_t blue)
{
    led_strip_set_pixel(led_strip, 0, red, green, blue);
    led_strip_refresh(led_strip);
}

/* Initialize I2C Master */
void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C master initialized.");
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Attempting to connect to Wi-Fi...");
        set_led_color(255, 165, 0); // Orange: Connecting
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGW(TAG, "Wi-Fi disconnected. Retrying...");
        set_led_color(255, 0, 0); // Red: Disconnected
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Connected! Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        set_led_color(0, 255, 0); // Green: Connected
    }
}


/* Wi-Fi Initialization */
void wifi_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register the Wi-Fi event handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialization complete.");
}


/* Send Data to Web Server */
void send_data_to_server(uint8_t slave_id, float temperature, float humidity, int32_t pressure, int32_t raw_weight)
{
    char post_data[256];
    snprintf(post_data, sizeof(post_data),
             "{\"slave_id\":%d,\"raw_weight\":%" PRId32 ",\"temperature\":%.2f,\"humidity\":%.2f,\"pressure\":%" PRId32 "}",
             slave_id, raw_weight, temperature, humidity, pressure);

    esp_http_client_config_t config = {
        .url = WEB_SERVER_URL,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d", esp_http_client_get_status_code(client));
        set_led_color(0, 255, 0); // Green: Data sent successfully
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
        set_led_color(255, 0, 0); // Red: HTTP POST failed
    }
    esp_http_client_cleanup(client);
}

/* Read Data from Slave */
void read_from_slaves()
{
    uint8_t buffer[21]; // Updated buffer size for slave ID
    memset(buffer, 0, sizeof(buffer));

    for (uint8_t slave_id = 0; slave_id <= MAX_HIVE_ID; slave_id++) {
        ESP_LOGI(TAG, "Trying to communicate with Slave ID: %d", slave_id);

        set_led_color(0, 0, 255); // Blue: Attempting communication

        uint8_t dummy_request = 0x01; // Request header
        esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, slave_id, &dummy_request, 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send request to Slave ID: %d, Error: %s", slave_id, esp_err_to_name(err));
            continue;
        }

        err = i2c_master_read_from_device(I2C_MASTER_NUM, slave_id, buffer, sizeof(buffer), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        if (err == ESP_OK) {
            uint8_t received_slave_id = buffer[1]; // Extract slave ID
            int32_t temperature_raw;
            uint32_t humidity_raw;
            int32_t pressure, raw_weight;

            memcpy(&temperature_raw, &buffer[2], sizeof(int32_t));
            memcpy(&humidity_raw, &buffer[6], sizeof(uint32_t));
            memcpy(&pressure, &buffer[10], sizeof(int32_t));
            memcpy(&raw_weight, &buffer[14], sizeof(int32_t));

            float temperature = (float)temperature_raw / 100.0;
            float humidity = (float)humidity_raw / 1024.0;

            ESP_LOGI(TAG, "Received from Slave ID: %d, Temperature: %.2f, Humidity: %.2f, Pressure: %" PRId32 ", Raw Weight: %" PRId32,
                     received_slave_id, temperature, humidity, pressure, raw_weight);

            if (buffer[0] == 0xAB && buffer[18] == 0xCD) {
                ESP_LOGI(TAG, "Valid data received from Slave ID: %d", received_slave_id);
                set_led_color(0, 255, 255); // Cyan: Data valid and ready to send
                send_data_to_server(received_slave_id, temperature, humidity, pressure, raw_weight);
            } else {
                ESP_LOGW(TAG, "Invalid data from Slave ID: %d", slave_id);
                set_led_color(255, 255, 0); // Yellow: Data invalid
            }
        } else {
            set_led_color(255, 0, 0); // Red: No response from slave
            ESP_LOGW(TAG, "No response from Slave ID: %d, Error: %s", slave_id, esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(3000)); // Delay between hive ID attempts
    }
}

/* Main Function */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting I2C Master with Wi-Fi, RGB LED, and MOSFET Control...");

    configure_gpio(MOSFET_PIN, 1); // Set MOSFET GPIO HIGH
    configure_rgb_led();

    wifi_init();
    i2c_master_init();

    while (1) {
        read_from_slaves();
        vTaskDelay(pdMS_TO_TICKS(interval)); // Delay between requests
    }
}