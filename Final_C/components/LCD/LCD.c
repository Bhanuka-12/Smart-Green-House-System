#include "lcd_i2c.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// Define LCD address (typically 0x27 for most I2C LCD modules)
#define LCD_ADDR 0x27

#define LCD_CMD 0x00
#define LCD_DATA 0x40
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY_MODE 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_FUNCTION_SET 0x20

static const char *TAG = "LCD_I2C";

static esp_err_t lcd_write_byte(uint8_t val, uint8_t mode) {
    uint8_t high = mode | (val & 0xF0) | 0x08;
    uint8_t low = mode | ((val << 4) & 0xF0) | 0x08;

    uint8_t data[4];
    data[0] = high;
    data[1] = high | 0x04;
    data[2] = low;
    data[3] = low | 0x04;

    return i2c_master_write_to_device(I2C_MASTER_NUM, LCD_I2C_ADDR, data, 4, 1000 / portTICK_PERIOD_MS);
}

esp_err_t lcd_i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    vTaskDelay(50 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(lcd_write_byte(0x03, LCD_CMD));
    vTaskDelay(5 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(lcd_write_byte(0x03, LCD_CMD));
    vTaskDelay(5 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(lcd_write_byte(0x03, LCD_CMD));
    vTaskDelay(5 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(lcd_write_byte(0x02, LCD_CMD));

    ESP_ERROR_CHECK(lcd_write_byte(LCD_FUNCTION_SET | 0x08, LCD_CMD));
    ESP_ERROR_CHECK(lcd_write_byte(LCD_DISPLAY_CONTROL | 0x04, LCD_CMD));
    ESP_ERROR_CHECK(lcd_write_byte(LCD_CLEAR, LCD_CMD));
    ESP_ERROR_CHECK(lcd_write_byte(LCD_ENTRY_MODE | 0x02, LCD_CMD));

    ESP_LOGI(TAG, "LCD initialized successfully");
    return ESP_OK;
}

esp_err_t lcd_i2c_clear(void) {
    return lcd_write_byte(LCD_CLEAR, LCD_CMD);
}

esp_err_t lcd_i2c_home(void) {
    return lcd_write_byte(LCD_HOME, LCD_CMD);
}

esp_err_t lcd_i2c_write_string(const char *str) {
    while (*str) {
        ESP_ERROR_CHECK(lcd_write_byte(*str++, LCD_DATA));
    }
    return ESP_OK;
}

esp_err_t lcd_i2c_set_cursor(uint8_t col, uint8_t row) {
    int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row >= LCD_ROWS) {
        row = LCD_ROWS - 1;
    }
    return lcd_write_byte(0x80 | (col + row_offsets[row]), LCD_CMD);
}

esp_err_t lcd_i2c_backlight_on(void) {
    uint8_t data = 0x08;
    return i2c_master_write_to_device(I2C_MASTER_NUM, LCD_I2C_ADDR, &data, 1, 1000 / portTICK_PERIOD_MS);
}

esp_err_t lcd_i2c_backlight_off(void) {
    uint8_t data = 0x00;
    return i2c_master_write_to_device(I2C_MASTER_NUM, LCD_I2C_ADDR, &data, 1, 1000 / portTICK_PERIOD_MS);
}
