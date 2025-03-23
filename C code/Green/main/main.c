#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "dht11.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_netif.h"

// WiFi credentials
#define WIFI_SSID "Bhanuka Samsung M31"
#define WIFI_PASSWORD "20011211"

// Firebase credentials
#define FIREBASE_HOST "smart-green-house-system-4ab5d-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "AIzaSyD1z9RLzdvBP4TBtQGNUGIHAjK9oRSNAeM"

// Pin Definitions
#define PUMP_PIN 23
#define DHT_PIN 4
#define SOIL_PIN ADC1_CHANNEL_4  // GPIO32
#define PIR_PIN 34
#define SERVO_PIN 14
#define FAN_PIN1 16
#define FAN_PIN2 17
#define LED_PIN 5
#define BUZZER_PIN 18
#define LDR_LEFT ADC1_CHANNEL_5  // GPIO33
#define LDR_RIGHT ADC1_CHANNEL_0  // GPIO36
#define LDR_LED_CONTROL ADC1_CHANNEL_3  // GPIO39

// I2C Pins for LCD
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_I2C_ADDR 0x27  // Try 0x27 first, if not working try 0x3F

// Servo Configuration
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MAX_DEGREE 180
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY 50

static const char *TAG = "FARM_SYSTEM";

// Sensor Values
float temperature = 0;
float humidity = 0;
float moisture = 0;
int ldrLeft = 0;
int ldrRight = 0;
int ldrAmbient = 0;

// Device States
bool fanStatus = false;
bool pumpStatus = false;
bool ledStatus = false;
bool motionDetected = false;
int servoAngle = 90;

// Function declarations
void init_gpio(void);
void init_adc(void);
void init_servo(void);
void init_i2c(void);
void init_lcd(void);
void lcd_write_byte(uint8_t val, uint8_t mode);
void lcd_clear(void);
void lcd_home(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char *str);
void update_sensors(void);
void update_lcd(void);
void check_auto_modes(void);
void adjust_solar_panel(void);
void control_led(void);
void handle_motion(void);
void set_servo_angle(uint8_t angle);
void wifi_init_sta(void);
void send_to_firebase(void);
esp_err_t wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

esp_err_t wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Retrying connection...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
    }
    return ESP_OK;
}

void wifi_init_sta(void) {
    ESP_LOGI(TAG, "Connecting to WiFi...");
    
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialization completed");
}

void send_to_firebase(void) {
    char json_data[256];
    char url[256];
    
    snprintf(json_data, sizeof(json_data), 
             "{\"Temperature\":\"%.1f\",\"Humidity\":\"%.1f\",\"SoilMoisture\":\"%.1f\",\"Light\":\"%s\",\"MotionDetected\":%s,\"Fan\":\"%s\",\"Pump\":\"%s\"}",
             temperature, humidity, moisture, 
             ledStatus ? "ON" : "OFF", 
             motionDetected ? "true" : "false",
             fanStatus ? "ON" : "OFF",
             pumpStatus ? "ON" : "OFF");
    
    snprintf(url, sizeof(url), "https://%s/greenhouse-data.json?auth=%s", FIREBASE_HOST, FIREBASE_AUTH);
    
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_PUT,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_data, strlen(json_data));
    
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "Firebase update successful, status code: %d", status_code);
    } else {
        ESP_LOGE(TAG, "Firebase update failed: %s", esp_err_to_name(err));
    }
    
    esp_http_client_cleanup(client);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Setup started...");
       // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize TCP/IP and event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize WiFi
    wifi_init_sta();
    
    // Initialize components
    init_gpio();
    init_adc();
    init_i2c();
    init_lcd();
    init_servo();
    
    // Initialize DHT11
    dht11_init(DHT_PIN);
    ESP_LOGI(TAG, "DHT sensor initialized...");
    
    // Initialize LCD
    lcd_clear();
    lcd_print("System Booting...");
    ESP_LOGI(TAG, "LCD initialized...");
    
    // Set initial servo position
    set_servo_angle(servoAngle);
    ESP_LOGI(TAG, "Servo initialized...");
    
    
    uint32_t last_update = 0;
    uint32_t last_firebase_update = 0;
    
    while (1) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if (current_time - last_update >= 1000) {  // Update every 1 second
            last_update = current_time;
            
            // Read all sensors
            update_sensors();
            
            // Update LCD
            update_lcd();
            
            // Automatic controls
            check_auto_modes();
            
            // Solar tracking
            adjust_solar_panel();
            
            // LED control
            control_led();
            
            // Motion detection and alarm
            handle_motion();
            
            send_to_firebase();
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Small delay to prevent watchdog trigger
    }
}

void init_gpio(void) {
    // Configure output pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_PIN) | (1ULL << FAN_PIN1) | (1ULL << FAN_PIN2) | 
                         (1ULL << LED_PIN) | (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Configure input pins
    io_conf.pin_bit_mask = (1ULL << PIR_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
    
    // Initialize outputs to OFF
    gpio_set_level(PUMP_PIN, 0);
    gpio_set_level(FAN_PIN1, 0);
    gpio_set_level(FAN_PIN2, 0);
    gpio_set_level(LED_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);
}

void init_adc(void) {
    esp_err_t ret;
    
    ret = adc1_config_width(ADC_WIDTH_BIT_12);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC width: %s", esp_err_to_name(ret));
    }
    
    ret = adc1_config_channel_atten(SOIL_PIN, ADC_ATTEN_DB_11);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SOIL_PIN: %s", esp_err_to_name(ret));
    }
    
    ret = adc1_config_channel_atten(LDR_LEFT, ADC_ATTEN_DB_11);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LDR_LEFT: %s", esp_err_to_name(ret));
    }
    
    ret = adc1_config_channel_atten(LDR_RIGHT, ADC_ATTEN_DB_11);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LDR_RIGHT: %s", esp_err_to_name(ret));
    }
    
    ret = adc1_config_channel_atten(LDR_LED_CONTROL, ADC_ATTEN_DB_11);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LDR_LED_CONTROL: %s", esp_err_to_name(ret));
    }
}

void init_servo(void) {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER
    };
    ledc_timer_config(&ledc_timer);
    
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_MODE,
        .timer_sel = LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel);
}

void init_i2c(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

void lcd_write_byte(uint8_t val, uint8_t mode) {
    uint8_t high = mode | (val & 0xF0) | 0x08;
    uint8_t low = mode | ((val << 4) & 0xF0) | 0x08;
    
    uint8_t data[4];
    data[0] = high;
    data[1] = high | 0x04;
    data[2] = low;
    data[3] = low | 0x04;
    
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_I2C_ADDR, data, 4, 1000 / portTICK_PERIOD_MS);
}

void init_lcd(void) {
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    lcd_write_byte(0x03, 0);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_write_byte(0x03, 0);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_write_byte(0x03, 0);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_write_byte(0x02, 0);
    
    lcd_write_byte(0x28, 0);
    lcd_write_byte(0x0C, 0);
    lcd_clear();
    lcd_write_byte(0x06, 0);
}

void lcd_clear(void) {
    lcd_write_byte(0x01, 0);
    vTaskDelay(2 / portTICK_PERIOD_MS);
}

void lcd_home(void) {
    lcd_write_byte(0x02, 0);
    vTaskDelay(2 / portTICK_PERIOD_MS);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_write_byte(0x80 | (col + row_offsets[row]), 0);
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_write_byte(*str++, 1);
    }
}

void set_servo_angle(uint8_t angle) {
    if (angle > SERVO_MAX_DEGREE) {
        angle = SERVO_MAX_DEGREE;
    }
    
    uint32_t duty = (SERVO_MIN_PULSEWIDTH_US + 
                     ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle / SERVO_MAX_DEGREE)) * 
                     (1 << LEDC_DUTY_RES) / (1000000 / LEDC_FREQUENCY);
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    
    ESP_LOGI(TAG, "Servo angle set to: %d", angle);
}

void update_sensors(void) {
    // Read DHT11 sensor
    dht11_data_t dht_data;
    if (dht11_read(&dht_data) == ESP_OK) {
        temperature = dht_data.temperature;
        humidity = dht_data.humidity;
    } else {
        ESP_LOGE(TAG, "Failed to read from DHT sensor!");
    }
    
    int soil_raw = adc1_get_raw(SOIL_PIN);
    moisture = (4095 - soil_raw) * 100.0 / 4095;
    
    ldrLeft = adc1_get_raw(LDR_LEFT);
    ldrRight = adc1_get_raw(LDR_RIGHT);
    ldrAmbient = adc1_get_raw(LDR_LED_CONTROL);
    
    ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.1f%%, Moisture: %.1f%%", 
             temperature, humidity, moisture);
    ESP_LOGI(TAG, "LDR Left: %d, LDR Right: %d, LDR Ambient: %d", 
             ldrLeft, ldrRight, ldrAmbient);
}

void update_lcd(void) {
    char buffer[20];
    
    lcd_clear();
    
    snprintf(buffer, sizeof(buffer), "T:%.1fC H:%.1f%%", temperature, humidity);
    lcd_print(buffer);
    
    lcd_set_cursor(0, 1);
    snprintf(buffer, sizeof(buffer), "M:%.1f%% L:%s", moisture, ledStatus ? "ON" : "OFF");
    lcd_print(buffer);
}

void check_auto_modes(void) {
    if (temperature > 25) {
        gpio_set_level(FAN_PIN1, 1);
        gpio_set_level(FAN_PIN2, 1);
        fanStatus = true;
    } else {
        gpio_set_level(FAN_PIN1, 0);
        gpio_set_level(FAN_PIN2, 0);
        fanStatus = false;
    }
    
    if (moisture < 20) {
        gpio_set_level(PUMP_PIN, 1);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        gpio_set_level(PUMP_PIN, 0);
        pumpStatus = true;
    } else {
        pumpStatus = false;
    }
}

void adjust_solar_panel(void) {
    int difference = ldrLeft - ldrRight;
    int adjustment = 2;
    
    if (difference > 200) {
        servoAngle = (servoAngle - adjustment < 0) ? 0 : servoAngle - adjustment;
        set_servo_angle(servoAngle);
    } else if (difference < -200) {
        servoAngle = (servoAngle + adjustment > 180) ? 180 : servoAngle + adjustment;
        set_servo_angle(servoAngle);
    }
}

void control_led(void) {
    if (ldrAmbient < 1500) {
        gpio_set_level(LED_PIN, 1);
        ledStatus = true;
    } else {
        gpio_set_level(LED_PIN, 0);
        ledStatus = false;
    }
}

void handle_motion(void) {
    if (gpio_get_level(PIR_PIN)) {
        motionDetected = true;
        ESP_LOGI(TAG, "Motion Detected! Sounding Alarm...");
        gpio_set_level(BUZZER_PIN, 1);  // Turn on buzzer
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Buzzer stays on for 1 second
        gpio_set_level(BUZZER_PIN, 0);  // Turn off buzzer
    } else {
        motionDetected = false;
    }
}
