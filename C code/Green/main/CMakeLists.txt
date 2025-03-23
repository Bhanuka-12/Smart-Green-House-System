idf_component_register(
    SRCS "main.c"               # List source files
    INCLUDE_DIRS "."   # Add public include directories
    PRIV_INCLUDE_DIRS ""        # Private include directories (if any)
    REQUIRES soilmoisture dht11 PIR Servo LCD esp_wifi esp_adc esp_common nvs_flash esp_http_client driver esp_adc esp_common# List required components
    PRIV_REQUIRES               # Private requirements (if any)
)
