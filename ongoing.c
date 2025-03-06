#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_http_server.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"

#define TAG "SMART_FARM"
#define PUMP_PIN GPIO_NUM_23
#define DHT_PIN GPIO_NUM_4
#define SOIL_PIN ADC1_CHANNEL_4
#define PIR_PIN GPIO_NUM_34
#define SERVO_PIN GPIO_NUM_14
#define FAN_PIN1 GPIO_NUM_16
#define FAN_PIN2 GPIO_NUM_17
#define LED_PIN GPIO_NUM_5
#define BUZZER_PIN GPIO_NUM_18
#define LDR_LEFT ADC1_CHANNEL_5
#define LDR_RIGHT ADC1_CHANNEL_0
#define LDR_LED_CONTROL ADC1_CHANNEL_3
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_ADDR 0x27
#define WIFI_SSID "Bhanuka Samsung M31"
#define WIFI_PASS "20011211"

static float temperature = 0;
static float humidity = 0;
static float moisture = 0;
static int ldr_left = 0;
static int ldr_right = 0;
static int ldr_ambient = 0;
static char fan_status[4] = "OFF";
static char pump_status[4] = "OFF";
static char led_status[4] = "OFF";
static bool motion_detected = false;

static httpd_handle_t server = NULL;
static i2c_master_bus_handle_t i2c_bus_handle;

static ledc_channel_config_t servo_channel = {
    .gpio_num = SERVO_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
};

static void initialize_gpio() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_PIN) | (1ULL << FAN_PIN1) | (1ULL << FAN_PIN2) | (1ULL << LED_PIN) | (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_config_t input_conf = {
        .pin_bit_mask = (1ULL << PIR_PIN),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&input_conf);
}

static void initialize_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SOIL_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(LDR_LEFT, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(LDR_RIGHT, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(LDR_LED_CONTROL, ADC_ATTEN_DB_11);
}

static void initialize_servo() {
    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
    };
    ledc_timer_config(&servo_timer);
    ledc_channel_config(&servo_channel);
}

static esp_err_t http_get_handler(httpd_req_t *req) {
    char response[2048];
    snprintf(response, sizeof(response),
        "<html><head><style>"
        "body { font-family: Arial, sans-serif; text-align: center; background-color: #f4f4f4; }"
        "h1 { color: #333; }"
        "button { padding: 10px 20px; margin: 10px; border: none; cursor: pointer; border-radius: 5px; font-size: 16px; }"
        "button.on { background-color: green; color: white; }"
        "button.off { background-color: red; color: white; }"
        "</style></head><body>"
        "<h1>Smart Farm Control Panel</h1>"
        "<p>Temperature: %.1f°C</p>"
        "<p>Humidity: %.1f%%</p>"
        "<p>Soil Moisture: %.1f%%</p>"
        "<p>Motion Detected: %s</p>"
        "<form method='post' action='/control'>"
        "<button class='%s' name='fan' value='ON'>Fan ON</button>"
        "<button class='%s' name='fan' value='OFF'>Fan OFF</button>"
        "<button class='%s' name='pump' value='ON'>Pump ON</button>"
        "<button class='%s' name='pump' value='OFF'>Pump OFF</button>"
        "</form></body></html>",
        temperature, humidity, moisture, motion_detected ? "YES" : "NO",
        strcmp(fan_status, "ON") == 0 ? "on" : "off",
        strcmp(fan_status, "OFF") == 0 ? "on" : "off",
        strcmp(pump_status, "ON") == 0 ? "on" : "off",
        strcmp(pump_status, "OFF") == 0 ? "on" : "off"
    );
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static void start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_uri_t get_uri = { .uri = "/", .method = HTTP_GET, .handler = http_get_handler };
    httpd_start(&server, &config);
    httpd_register_uri_handler(server, &get_uri);
}

static void wifi_init_sta() {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    initialize_gpio();
    initialize_adc();
    initialize_servo();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_sta();
    start_webserver();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}