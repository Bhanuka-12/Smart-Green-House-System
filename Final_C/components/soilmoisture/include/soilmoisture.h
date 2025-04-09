#ifndef SOIL_MOISTURE_H
#define SOIL_MOISTURE_H

#include "driver/adc.h"
#include "esp_adc_cal.h"

#define SOIL_MOISTURE_PIN ADC1_CHANNEL_6 // GPIO34
#define SOIL_MOISTURE_SAMPLES 64

typedef struct {
    int raw_value;
    float voltage;
    int moisture_percentage;
} soil_moisture_data_t;

void soil_moisture_init(void);
void soil_moisture_read(soil_moisture_data_t *data);

#endif // SOIL_MOISTURE_H
