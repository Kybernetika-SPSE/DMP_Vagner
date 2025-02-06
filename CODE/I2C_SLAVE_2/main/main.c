#include <stdio.h>
#include <string.h>
#include <inttypes.h> // For PRId32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "led_strip.h"
#include "esp_rom_sys.h"

uint8_t slave_id = 1; // Replace 1 with the unique ID for this slave

#define I2C_SLAVE_SCL_IO          39  // GPIO for I2C clock
#define I2C_SLAVE_SDA_IO          40  // GPIO for I2C data
#define I2C_SLAVE_NUM             I2C_NUM_0
#define I2C_SLAVE_ADDRESS         slave_id
#define I2C_SLAVE_RX_BUF_LEN      512
#define I2C_SLAVE_TX_BUF_LEN      512

#define I2C_OUT                   38  // Define the GPIO pin number

#define HX711_DT_PIN              35  // GPIO for HX711 Data (DT)
#define HX711_SCK_PIN             36  // GPIO for HX711 Clock (SCK)

// BME280 I2C Address and Registers
#define BME280_I2C_ADDRESS  0x76
#define BME280_REG_ID       0xD0
#define BME280_REG_CALIB    0x88
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_DATA     0xF7

static bool bme280_connected = false; // Flag to indicate BME280 connection
int bmeFAIL = 0; // Counter for BME280 failures


// I2C Pins
#define SDA_GPIO 34
#define SCL_GPIO 48



#define SK6812_PIN                5   // GPIO for SK6812 RGB LED

static const char *TAG = "I2C_HX711_SK6812_BME280";

static led_strip_handle_t led_strip;

// Calibration parameters
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} bme280_calib_data_t;

static bme280_calib_data_t calib_data;
static int32_t t_fine = 0;

void i2c_master_init() {
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialized on SDA: %d, SCL: %d", SDA_GPIO, SCL_GPIO);
}

esp_err_t bme280_read_register(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (BME280_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (BME280_I2C_ADDRESS << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK(i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void bme280_write_register(uint8_t reg_addr, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (BME280_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, value, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);
}

void bme280_read_calibration_data() {
    uint8_t calib[26];
    bme280_read_register(BME280_REG_CALIB, calib, 26);

    calib_data.dig_T1 = (uint16_t)(calib[0] | (calib[1] << 8));
    calib_data.dig_T2 = (int16_t)(calib[2] | (calib[3] << 8));
    calib_data.dig_T3 = (int16_t)(calib[4] | (calib[5] << 8));
    calib_data.dig_P1 = (uint16_t)(calib[6] | (calib[7] << 8));
    calib_data.dig_P2 = (int16_t)(calib[8] | (calib[9] << 8));
    calib_data.dig_P3 = (int16_t)(calib[10] | (calib[11] << 8));
    calib_data.dig_P4 = (int16_t)(calib[12] | (calib[13] << 8));
    calib_data.dig_P5 = (int16_t)(calib[14] | (calib[15] << 8));
    calib_data.dig_P6 = (int16_t)(calib[16] | (calib[17] << 8));
    calib_data.dig_P7 = (int16_t)(calib[18] | (calib[19] << 8));
    calib_data.dig_P8 = (int16_t)(calib[20] | (calib[21] << 8));
    calib_data.dig_P9 = (int16_t)(calib[22] | (calib[23] << 8));
    calib_data.dig_H1 = calib[25];
    
    uint8_t h_calib[7];
    bme280_read_register(0xE1, h_calib, 7);
    calib_data.dig_H2 = (int16_t)(h_calib[0] | (h_calib[1] << 8));
    calib_data.dig_H3 = h_calib[2];
    calib_data.dig_H4 = (int16_t)((h_calib[3] << 4) | (h_calib[4] & 0x0F));
    calib_data.dig_H5 = (int16_t)((h_calib[5] << 4) | (h_calib[4] >> 4));
    calib_data.dig_H6 = (int8_t)h_calib[6];
}

int32_t compensate_temperature(int32_t adc_T) {
    int32_t var1 = (((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1)) * ((int32_t)calib_data.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) *
                    ((int32_t)calib_data.dig_T3)) >>
                   14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

uint32_t compensate_pressure(int32_t adc_P) {
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // avoid division by zero
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    return (uint32_t)p / 256;
}

uint32_t compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r = t_fine - 76800;
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib_data.dig_H4) << 20) - (((int32_t)calib_data.dig_H5) * v_x1_u32r)) + 16384) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)calib_data.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib_data.dig_H3)) >> 11) + 32768)) >> 10) + 2097152) *
                   ((int32_t)calib_data.dig_H2) +
                   8192) >>
                  14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib_data.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

void bme280_init() {
    uint8_t chip_id;
    esp_err_t err = bme280_read_register(BME280_REG_ID, &chip_id, 1);
    if (err == ESP_OK && chip_id == 0x60) { // Check for valid BME280 Chip ID
        ESP_LOGI(TAG, "BME280 detected with Chip ID: 0x%X", chip_id);
        bme280_connected = true;

        bme280_read_calibration_data();
        bme280_write_register(BME280_REG_CTRL_HUM, 0x01);  // Humidity oversampling x1
        bme280_write_register(BME280_REG_CTRL_MEAS, 0x27); // Temp and Pressure oversampling x1, Mode normal
    } else {
        ESP_LOGW(TAG, "BME280 not detected! Error: %s", esp_err_to_name(err));
        bme280_connected = false;
    }
}

void configure_gpio_as_output()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2C_OUT), // Set the pin to configure
        .mode = GPIO_MODE_OUTPUT,                 // Set as output mode
        .pull_up_en = GPIO_PULLUP_DISABLE,        // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE            // No interrupt
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf)); // Configure the GPIO
    ESP_LOGI("GPIO", "Configured GPIO %d as output.", I2C_OUT);
}

void set_gpio_high()
{
    gpio_set_level(I2C_OUT, 1); // Set GPIO high
    ESP_LOGI("GPIO", "GPIO %d set to HIGH.", I2C_OUT);
}


/* HX711 Initialization */
void hx711_init()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HX711_DT_PIN) | (1ULL << HX711_SCK_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_direction(HX711_DT_PIN, GPIO_MODE_INPUT); // Set DT as input
    gpio_set_level(HX711_SCK_PIN, 0);                 // Ensure clock starts low
}

/* HX711 Read Function */
int32_t hx711_read()
{
    int32_t data = 0;

    // Wait until the HX711 is ready
    while (gpio_get_level(HX711_DT_PIN) == 1) {
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay
    }

    // Read 24 bits of data from HX711
    for (int i = 0; i < 24; i++) {
        gpio_set_level(HX711_SCK_PIN, 1);
        esp_rom_delay_us(1);
        data = (data << 1) | gpio_get_level(HX711_DT_PIN);
        gpio_set_level(HX711_SCK_PIN, 0);
        esp_rom_delay_us(1);
    }

    // Send the 25th clock pulse to acknowledge the reading
    gpio_set_level(HX711_SCK_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(HX711_SCK_PIN, 0);

    // Convert to signed value
    data ^= 0x800000; // Flip MSB for signed data

    return data;
}

/* I2C Slave Initialization */
void i2c_slave_init()
{
    i2c_config_t conf_slave = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave = {
            .addr_10bit_en = 0,
            .slave_addr = I2C_SLAVE_ADDRESS
        },
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_NUM, &conf_slave));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_SLAVE_NUM, conf_slave.mode,
                                       I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0));
    ESP_LOGI(TAG, "I2C slave initialized at address 0x%X", I2C_SLAVE_ADDRESS);
}

/* LED Strip Configuration */
void configure_led()
{
    ESP_LOGI(TAG, "Configuring SK6812 LED...");
    led_strip_config_t strip_config = {
        .strip_gpio_num = SK6812_PIN,
        .max_leds = 1, // At least one LED
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

/* Set LED Color */
void set_led_color(uint8_t red, uint8_t green, uint8_t blue)
{
    led_strip_set_pixel(led_strip, 0, red, green, blue);
    led_strip_refresh(led_strip);
}

/* I2C Slave Task */
void i2c_slave_task(void *arg) {
    uint8_t buffer[21]; // Header + Slave ID + Data + Footer
    memset(buffer, 0, sizeof(buffer));

    while (1) {
        // Wait for data request from master
        uint8_t dummy[1];
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, dummy, sizeof(dummy), portMAX_DELAY);
        if (len > 0) {
            // Read HX711 for weight
            int32_t weight = hx711_read();

            int32_t temperature = 0;
            uint32_t humidity = 0;
            int32_t pressure = 0;

            if (bme280_connected) {
                uint8_t data[8];
                bme280_read_register(BME280_REG_DATA, data, 8);

                int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
                int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
                int32_t adc_H = (data[6] << 8) | data[7];

                temperature = compensate_temperature(adc_T);
                pressure = compensate_pressure(adc_P);
                humidity = compensate_humidity(adc_H);
            } else {
                
                ESP_LOGW(TAG, "Skipping BME280 data retrieval as it's not connected.");
                set_led_color(128, 0, 128); // Purple LED for missing sensor
                vTaskDelay(pdMS_TO_TICKS(1000));
                bmeFAIL++;
                if (bmeFAIL > 5) {
                    bmeFAIL = 0;
                    bme280_init(); // Try to reconnect
                    set_led_color(100, 100, 0);   // Orange LED for ready state
                }
                set_led_color(0, 0, 255);   // Blue LED for ready state
            }

            ESP_LOGI(TAG, "Slave ID: %d, Temperature: %.2f °C, Pressure: %.2f hPa, Humidity: %.2f%%, Weight: %.2f g",
                     slave_id, temperature / 100.0, pressure / 100.0, humidity / 1024.0, weight / 100.0);

            // Fill buffer with sensor data
            buffer[0] = 0xAB;               // Header
            buffer[1] = slave_id;           // Slave ID
            memcpy(&buffer[2], &temperature, sizeof(int32_t));
            memcpy(&buffer[6], &humidity, sizeof(uint32_t));
            memcpy(&buffer[10], &pressure, sizeof(int32_t));
            memcpy(&buffer[14], &weight, sizeof(int32_t));
            buffer[18] = 0xCD;              // Footer

            // Indicate activity with LED
            set_led_color(0, 255, 0); // Green for active
            vTaskDelay(pdMS_TO_TICKS(500));
            set_led_color(0, 0, 255); // Blue for ready state

            // Write data to I2C buffer
            i2c_slave_write_buffer(I2C_SLAVE_NUM, buffer, sizeof(buffer), portMAX_DELAY);
        }
    }
}



/* Main Function */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting I2C Slave with HX711 and SK6812 LED...");



    // Initialize HX711, I2C, and LED
    hx711_init();
    i2c_slave_init();
    configure_led();
    set_gpio_high();
    i2c_master_init();
    bme280_init();

    // Set initial LED state
    set_led_color(255, 0, 0); // Red for startup
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_led_color(0, 0, 255); // Blue for ready state

    // Start I2C slave task
    xTaskCreate(i2c_slave_task, "i2c_slave_task", 4096, NULL, 10, NULL);
}
