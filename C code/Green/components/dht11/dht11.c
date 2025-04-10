// dht11.c
#include "dht11.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "DHT11";
static gpio_num_t dht11_pin;

/**
 * @brief Initializes the DHT11 sensor.
 */
void dht11_init(gpio_num_t pin) {
    dht11_pin = pin;
    
    // Configure with internal pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << dht11_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    gpio_set_level(dht11_pin, 1);  // Ensure the sensor is in idle state
    vTaskDelay(pdMS_TO_TICKS(1000)); // Give sensor time to stabilize
    ESP_LOGI(TAG, "DHT11 sensor initialized on GPIO %d", pin);
}

/**
 * @brief Reads temperature and humidity from the DHT11 sensor.
 */
esp_err_t dht11_read(dht11_data_t *data) {
    uint8_t buffer[5] = {0, 0, 0, 0, 0};
    
    // Wait at least 1 second between readings as per datasheet
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Start signal
    gpio_set_direction(dht11_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dht11_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(20)); // At least 18ms LOW according to datasheet
    
    // Pull up and wait for DHT11 response
    gpio_set_level(dht11_pin, 1);
    esp_rom_delay_us(40); // 20-40us HIGH according to datasheet
    gpio_set_direction(dht11_pin, GPIO_MODE_INPUT);
    
    // Wait for DHT11 to pull LOW (response signal)
    int timeout = 200; // Increased timeout
    int counter = 0;
    while (gpio_get_level(dht11_pin) == 1) {
        if (++counter > timeout) {
            ESP_LOGE(TAG, "DHT11 not responding (timeout waiting for initial LOW)");
            return ESP_FAIL;
        }
        esp_rom_delay_us(1);
    }
    
    // DHT11 pulls LOW for 80us
    counter = 0;
    while (gpio_get_level(dht11_pin) == 0) {
        if (++counter > timeout) {
            ESP_LOGE(TAG, "DHT11 stuck in LOW state (80us response)");
            return ESP_FAIL;
        }
        esp_rom_delay_us(1);
    }
    
    // DHT11 pulls HIGH for 80us
    counter = 0;
    while (gpio_get_level(dht11_pin) == 1) {
        if (++counter > timeout) {
            ESP_LOGE(TAG, "DHT11 stuck in HIGH state (80us response)");
            return ESP_FAIL;
        }
        esp_rom_delay_us(1);
    }
    
    // Read 40 bits (5 bytes)
    for (int i = 0; i < 40; i++) {
        // Each bit starts with 50us LOW
        counter = 0;
        while (gpio_get_level(dht11_pin) == 0) {
            if (++counter > timeout) {
                ESP_LOGE(TAG, "DHT11 stuck in LOW state (bit start)");
                return ESP_FAIL;
            }
            esp_rom_delay_us(1);
        }
        
        // Duration of HIGH determines bit value
        // ~28us for bit '0', ~70us for bit '1'
        counter = 0;
        while (gpio_get_level(dht11_pin) == 1) {
            if (++counter > timeout) {
                ESP_LOGE(TAG, "DHT11 stuck in HIGH state (bit value)");
                return ESP_FAIL;
            }
            esp_rom_delay_us(1);
        }
        
        // If HIGH pulse is longer than 40us, bit is '1'
        if (counter > 40) {
            buffer[i / 8] |= (1 << (7 - (i % 8)));
        }
    }
    
    // Verify checksum
    uint8_t checksum = buffer[0] + buffer[1] + buffer[2] + buffer[3];
    if (checksum != buffer[4]) {
        ESP_LOGE(TAG, "DHT11 checksum failed: %d != %d", checksum, buffer[4]);
        return ESP_FAIL;
    }
    
    // DHT11 data format: 8bit integral RH + 8bit decimal RH + 8bit integral T + 8bit decimal T + 8bit checksum
    data->humidity = buffer[0];
    data->temperature = buffer[2];
    
    ESP_LOGI(TAG, "Temperature: %d°C, Humidity: %d%%", data->temperature, data->humidity);
    return ESP_OK;
}
