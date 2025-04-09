// dht11.h
#ifndef DHT11_H
#define DHT11_H

#include "esp_err.h"
#include "driver/gpio.h"

typedef struct {
    int temperature;
    int humidity;
} dht11_data_t;

void dht11_init(gpio_num_t pin);
esp_err_t dht11_read(dht11_data_t *data);

#endif // DHT11_H
