#include "pir.h"
#include "esp_log.h"

static const char *TAG = "PIR";
static gpio_num_t pir_pin;

esp_err_t pir_init(gpio_num_t pin) {
    pir_pin = pin;
    
    // Configure PIR pin as input with pull-down enabled
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pir_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // Enable pull-down
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PIR pin");
        return ret;
    }
    
    ESP_LOGI(TAG, "PIR sensor initialized on GPIO %d", pin);
    return ESP_OK;
}

bool pir_read(void) {
    return gpio_get_level(pir_pin) == 1; // Assumes PIR outputs HIGH on motion
}