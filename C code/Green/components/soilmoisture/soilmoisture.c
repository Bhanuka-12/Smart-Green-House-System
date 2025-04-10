#include "soilmoisture.h"
#include "esp_log.h"

static const char *TAG = "SOIL_MOISTURE";
static esp_adc_cal_characteristics_t adc_chars;

void soil_moisture_init(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SOIL_MOISTURE_PIN, ADC_ATTEN_DB_11);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    ESP_LOGI(TAG, "Soil moisture sensor initialized");
}

void soil_moisture_read(soil_moisture_data_t *data) {
    uint32_t adc_reading = 0;
    for (int i = 0; i < SOIL_MOISTURE_SAMPLES; i++) {
        adc_reading += adc1_get_raw(SOIL_MOISTURE_PIN);
    }
    adc_reading /= SOIL_MOISTURE_SAMPLES;

    data->raw_value = adc_reading;
    data->voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars) / 1000.0;
    
    // Convert to moisture percentage (0-100%)
    // Assuming 0.5V is 0% (dry) and 3V is 100% (wet)
    data->moisture_percentage = (data->voltage - 0.5) / (3.0 - 0.5) * 100;
    if (data->moisture_percentage < 0) data->moisture_percentage = 0;
    if (data->moisture_percentage > 100) data->moisture_percentage = 100;

    ESP_LOGI(TAG, "Raw: %d, Voltage: %.2fV, Moisture: %d%%", 
             data->raw_value, data->voltage, data->moisture_percentage);
}
