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
#include "esp_crt_bundle.h"
#include "freertos/event_groups.h"
#include <math.h>

// WiFi credentials
#define WIFI_SSID "Bhanuka Samsung M31"   // Replace with your router's SSID
#define WIFI_PASS "20011211"         // Replace with your router's password
#define FIREBASE_URL "https://smart-green-house-system-4ab5d-default-rtdb.asia-southeast1.firebasedatabase.app/SmartGreenHouse.json?auth=AIzaSyD1z9RLzdvBP4TBtQGNUGIHAjK9oRSNAeM"
#define MAX_PAYLOAD_SIZE 256
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

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
#define LDR_LEFT ADC1_CHANNEL_5   // GPIO33
#define LDR_RIGHT ADC1_CHANNEL_0  // GPIO36
#define LDR_LED_CONTROL ADC1_CHANNEL_3  // GPIO39

// I2C Pins for LCD
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_I2C_ADDR 0x3F // Try 0x27 if 0x3F doesn't work

// Servo Configuration
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MAX_DEGREE 180
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY 50

// DHT11 read retry parameters
#define DHT11_MAX_RETRIES 5
#define DHT11_RETRY_DELAY_MS 500

// Moisture threshold for pump control
#define MOISTURE_THRESHOLD 20.0

static const char *TAG = "FARM_SYSTEM";

// Sensor Values
float temperature = 0;
float humidity = 0;
float moisture = 0;
int ldrLeft = 0;
int ldrRight = 0;
int ldrAmbient = 0;
uint32_t last_pump_time = 0;

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

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi started, connecting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected, retrying...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP address.");
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register WiFi event handler
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    // Safely copy credentials using strlcpy
    strlcpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization complete. Connecting to %s...", WIFI_SSID);
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                ESP_LOGI(TAG, "%.*s", evt->data_len, (char*)evt->data);
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGI(TAG, "HTTP_EVENT_REDIRECT");
            break;
        default:
            ESP_LOGI(TAG, "Unknown HTTP event id: %d", evt->event_id);
            break;
    }
    return ESP_OK;
}

void send_to_firebase(void) {
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi Connected, starting HTTP POST request");

    // Data to send to Firebase
    char json_payload[MAX_PAYLOAD_SIZE];

    snprintf(json_payload, sizeof(json_payload), 
         "{\"Temperature\":%.1f,\"Humidity\":%.1f,\"SoilMoisture\":%.1f,"
         "\"Light\":\"%s\",\"MotionDetected\":%s,\"Fan\":\"%s\",\"Pump\":\"%s\"}",
         temperature, humidity, moisture, 
         ledStatus ? "ON" : "OFF", 
         motionDetected ? "true" : "false",
         fanStatus ? "ON" : "OFF",
         pumpStatus ? "ON" : "OFF");

    esp_http_client_config_t config = {
        .url = FIREBASE_URL,
        .method = HTTP_METHOD_PUT,
        .event_handler = http_event_handler,
        .cert_pem = NULL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .skip_cert_common_name_check = false,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_post_field(client, json_payload, strlen(json_payload));
    esp_http_client_set_header(client, "Content-Type", "application/json");

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "HTTP Status = %d", status);
    } else {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    esp_http_client_cleanup(client);
}

void app_main(void) {
    ESP_LOGI(TAG, "System initialization started...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize network components
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Initialize hardware components
    init_gpio();
    init_adc();
    init_i2c();
    init_lcd();
    init_servo();
    
    // Initialize DHT11 sensor
    dht11_init(DHT_PIN);
    
    // Initial LCD display
    lcd_clear();
    lcd_print("System Booting...");
    ESP_LOGI(TAG, "LCD initialized");
    
    // Set initial servo position
    set_servo_angle(servoAngle);
    ESP_LOGI(TAG, "Servo initialized");
    
    // Initialize WiFi (after hardware init)
    wifi_init_sta();
    
    uint32_t last_update = 0;
    uint32_t last_firebase_update = 0;
    
    while (1) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if (current_time - last_update >= 2000) {  // Update every 2 seconds
            last_update = current_time;
            
            // Sensor reading and control logic
            update_sensors();
            update_lcd();
            check_auto_modes();
            adjust_solar_panel();
            control_led();
            handle_motion();
        }
        
        // Send to Firebase every 30 seconds
        if (current_time - last_firebase_update >= 30000) {
            send_to_firebase();
            last_firebase_update = current_time;
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void init_gpio(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_PIN) | (1ULL << FAN_PIN1) | (1ULL << FAN_PIN2) | 
                        (1ULL << LED_PIN) | (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << PIR_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
    
    // Initialize relays to inactive state (HIGH for relays)
    gpio_set_level(PUMP_PIN, 1);  // HIGH = relay off
    gpio_set_level(FAN_PIN1, 1);  // HIGH = relay off
    gpio_set_level(FAN_PIN2, 1);  // HIGH = relay off
    gpio_set_level(LED_PIN, 0);   // Regular output, LOW = off
    gpio_set_level(BUZZER_PIN, 0); // Regular output, LOW = off
}


void init_adc(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SOIL_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(LDR_LEFT, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(LDR_RIGHT, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(LDR_LED_CONTROL, ADC_ATTEN_DB_11);
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
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void lcd_write_byte(uint8_t val, uint8_t mode) {
    uint8_t high = mode | (val & 0xF0) | 0x08;
    uint8_t low = mode | ((val << 4) & 0xF0) | 0x08;
    
    uint8_t data[4] = {high, high | 0x04, low, low | 0x04};
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_I2C_ADDR, data, sizeof(data), pdMS_TO_TICKS(1000));
}

void init_lcd(void) {
    vTaskDelay(pdMS_TO_TICKS(50));
    for(int i=0; i<3; i++) {
        lcd_write_byte(0x03, 0);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    lcd_write_byte(0x02, 0);
    lcd_write_byte(0x28, 0);
    lcd_write_byte(0x0C, 0);
    lcd_clear();
    lcd_write_byte(0x06, 0);
}

void lcd_clear(void) {
    lcd_write_byte(0x01, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_write_byte(0x80 | (col + row_offsets[row]), 0);
}

void lcd_print(const char *str) {
    while (*str) lcd_write_byte(*str++, 1);
}

void set_servo_angle(uint8_t angle) {
    angle = (angle > SERVO_MAX_DEGREE) ? SERVO_MAX_DEGREE : angle;
    uint32_t duty = (SERVO_MIN_PULSEWIDTH_US + 
                   ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle / SERVO_MAX_DEGREE)) * 
                   (1 << LEDC_DUTY_RES) / (1000000 / LEDC_FREQUENCY);
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    ESP_LOGI(TAG, "Servo angle: %d", angle);
}

void update_sensors(void) {
    // Improved DHT11 reading with better retry logic
    dht11_data_t dht_data;
    bool dht_success = false;
    
    // Force DHT pin to HIGH state for a moment to reset it
    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Try multiple times with longer delays between attempts
    for (int i = 0; i < 5; i++) {
        esp_err_t result = dht11_read(&dht_data);
        if (result == ESP_OK) {
            // Validate readings are within reasonable range
            if (dht_data.temperature > 0 && dht_data.temperature < 100 && 
                dht_data.humidity > 0 && dht_data.humidity < 100) {
                temperature = dht_data.temperature;
                humidity = dht_data.humidity;
                dht_success = true;
                break;
            }
        }
        
        // If reading failed, wait longer before retry
        ESP_LOGW(TAG, "DHT11 read attempt %d failed, retrying...", i+1);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Reset the DHT11 pin to recover from stuck states
        gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(DHT_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    if (!dht_success) {
        ESP_LOGE(TAG, "DHT11 read failed after multiple attempts");
        // Keep last valid readings if available, otherwise use error values
        if (temperature <= 0 || humidity <= 0) {
            // Only update to error values if current values are invalid
            temperature = -99;
            humidity = -99;
        }
    }
    
    // Read soil moisture with validation
    int soil_raw = adc1_get_raw(SOIL_PIN);
    if (soil_raw >= 0 && soil_raw <= 4095) {
        moisture = (4095 - soil_raw) * 100.0 / 4095;
    } else {
        ESP_LOGE(TAG, "Invalid soil moisture reading: %d", soil_raw);
    }
    
    // Read LDR values with validation
    int left_raw = adc1_get_raw(LDR_LEFT);
    int right_raw = adc1_get_raw(LDR_RIGHT);
    int ambient_raw = adc1_get_raw(LDR_LED_CONTROL);
    
    if (left_raw >= 0 && left_raw <= 4095) {
        ldrLeft = left_raw;
    }
    
    if (right_raw >= 0 && right_raw <= 4095) {
        ldrRight = right_raw;
    }
    
    if (ambient_raw >= 0 && ambient_raw <= 4095) {
        ldrAmbient = ambient_raw;
    }
    
    ESP_LOGI(TAG, "Temp: %.1f°C, Hum: %.1f%%, Moist: %.1f%%", 
             temperature, humidity, moisture);
    ESP_LOGI(TAG, "LDR Left: %d, Right: %d, Ambient: %d", 
             ldrLeft, ldrRight, ldrAmbient);
}
void update_lcd(void) {
    char buf[20];
    
    // Check for valid temperature and humidity readings
    float temp_display = (temperature <= -99) ? 0.0 : temperature;
    float humid_display = (humidity <= -99) ? 0.0 : humidity;
    
    lcd_clear();
    snprintf(buf, sizeof(buf), "T:%.1fC H:%.1f%%", temp_display, humid_display);
    lcd_print(buf);
    
    lcd_set_cursor(0, 1);
    snprintf(buf, sizeof(buf), "M:%.1f%% L:%s", moisture, ledStatus ? "ON" : "OFF");
    lcd_print(buf);
}


void check_auto_modes(void) {
  // Temperature control - Relay logic is inverted (LOW activates the relay)
if (temperature > 25 && temperature != -99) {
    gpio_set_level(FAN_PIN1, 0); // Set LOW to activate relay
    gpio_set_level(FAN_PIN2, 0);
    fanStatus = true;
    ESP_LOGI(TAG, "Fan activated: Temperature = %.1f°C", temperature);
} else {
    gpio_set_level(FAN_PIN1, 1); // Set HIGH to deactivate relay
    gpio_set_level(FAN_PIN2, 1);
    fanStatus = false;
}

    
// Moisture control - Relay logic is inverted (LOW activates the relay)
if (moisture < 20) {
    if (!pumpStatus) {
        ESP_LOGI(TAG, "Activating pump: Moisture = %.1f%%", moisture);
        gpio_set_level(PUMP_PIN, 0); // Set LOW to activate relay
        pumpStatus = true;
    }
} else {
    if (pumpStatus) {
        ESP_LOGI(TAG, "Deactivating pump: Moisture = %.1f%%", moisture);
        gpio_set_level(PUMP_PIN, 1); // Set HIGH to deactivate relay
        pumpStatus = false;
    }
}


}
void adjust_solar_panel(void) {
    int diff = ldrLeft - ldrRight;
    if (abs(diff) > 200) {
        servoAngle += (diff > 0) ? -2 : 2;
        servoAngle = (servoAngle < 0) ? 0 :
                (servoAngle > SERVO_MAX_DEGREE) ? SERVO_MAX_DEGREE : servoAngle;
        set_servo_angle(servoAngle);
    }
}

void control_led(void) {
    ledStatus = (ldrAmbient > 1500);
    gpio_set_level(LED_PIN, ledStatus);
}

void handle_motion(void) {
    motionDetected = gpio_get_level(PIR_PIN);
    if (motionDetected) {
        ESP_LOGI(TAG, "Motion detected!");
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(BUZZER_PIN, 0);
    }
}

