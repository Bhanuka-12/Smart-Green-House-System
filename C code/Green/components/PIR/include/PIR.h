#ifndef PIR_H
#define PIR_H

#include "driver/gpio.h"
#include "esp_err.h"

#define PIR_DEFAULT_PIN GPIO_NUM_5

/**
 * @brief Initialize the PIR sensor
 * 
 * @param pin GPIO pin connected to the PIR sensor output
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pir_init(gpio_num_t pin);

/**
 * @brief Read the current state of the PIR sensor
 * 
 * @return true Motion detected (HIGH output)
 * @return false No motion detected (LOW output)
 */
bool pir_read(void);

#endif // PIR_H
