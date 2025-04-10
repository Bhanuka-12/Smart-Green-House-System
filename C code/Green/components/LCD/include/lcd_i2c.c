#include "lcd_i2c.h"
#include "driver/i2c_master.h"  // Updated header
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// LCD commands
#define LCD_CMD 0x00
#define LCD_DATA 0x40
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY_MODE 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_FUNCTION_SET 0x20

// LCD backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

static const char *TAG = "LCD_I2C";

static esp_err_t lcd_write_byte(uint8_t val, uint8_t mode) {
    // Add backlight flag to all commands
    uint8_t high = mode | (val & 0xF0) | LCD_BACKLIGHT;
    uint8_t low = mode | ((val << 4) & 0xF0) | LCD_BACKLIGHT;

    uint8_t data[4];
    data[0] = high;
    data[1] = high | 0x04;  // Set EN bit
    data[2] = low;
    data[3] = low | 0x04;   // Set EN bit

    // Updated I2C write function
    uint8_t write_buf[4] = {data[0], data[1], data[2], data[3]};
    return i2c_master_write_to_device(I2C_MASTER_NUM, LCD_I2C_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
}

esp_err_t lcd_i2c_init(void) {
    // Updated I2C configuration
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    esp_err_t ret = i2c_master_bus_init(&i2c_mst_config, &I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %d", ret);
        return ret;
    }
    
    // Wait for LCD to power up
    vTaskDelay(pdMS_TO_TICKS(100));

    // 4-bit initialization sequence according to datasheet
    // First send 0x03 three times
    if ((ret = lcd_write_byte(0x03, LCD_CMD)) != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));
    if ((ret = lcd_write_byte(0x03, LCD_CMD)) != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));
    if ((ret = lcd_write_byte(0x03, LCD_CMD)) != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Now send 0x02 to enable 4-bit mode
    if ((ret = lcd_write_byte(0x02, LCD_CMD)) != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));

    // Function set: 4-bit mode, 2 lines, 5x8 font
    if ((ret = lcd_write_byte(LCD_FUNCTION_SET | 0x08, LCD_CMD)) != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Display control: Display on, cursor off, blink off
    if ((ret = lcd_write_byte(LCD_DISPLAY_CONTROL | 0x04, LCD_CMD)) != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Clear display
    if ((ret = lcd_i2c_clear()) != ESP_OK) return ret;
    
    // Entry mode set: Increment cursor, no display shift
    if ((ret = lcd_write_byte(LCD_ENTRY_MODE | 0x02, LCD_CMD)) != ESP_OK) return ret;

    ESP_LOGI(TAG, "LCD initialized successfully");
    return ESP_OK;
}

esp_err_t lcd_i2c_clear(void) {
    esp_err_t ret = lcd_write_byte(LCD_CLEAR, LCD_CMD);
    vTaskDelay(pdMS_TO_TICKS(2));  // Clear command needs 1.52ms
    return ret;
}

esp_err_t lcd_i2c_home(void) {
    esp_err_t ret = lcd_write_byte(LCD_HOME, LCD_CMD);
    vTaskDelay(pdMS_TO_TICKS(2));  // Home command needs 1.52ms
    return ret;
}

esp_err_t lcd_i2c_write_string(const char *str) {
    esp_err_t ret = ESP_OK;
    while (*str && ret == ESP_OK) {
        ret = lcd_write_byte(*str++, LCD_DATA);
    }
    return ret;
}

esp_err_t lcd_i2c_set_cursor(uint8_t col, uint8_t row) {
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row >= LCD_ROWS) {
        row = LCD_ROWS - 1;
    }
    if (col >= LCD_COLS) {
        col = LCD_COLS - 1;
    }
    return lcd_write_byte(0x80 | (col + row_offsets[row]), LCD_CMD);
}

esp_err_t lcd_i2c_backlight_on(void) {
    uint8_t data = LCD_BACKLIGHT;
    return i2c_master_write_to_device(I2C_MASTER_NUM, LCD_I2C_ADDR, &data, 1, pdMS_TO_TICKS(1000));
}

esp_err_t lcd_i2c_backlight_off(void) {
    uint8_t data = LCD_NOBACKLIGHT;
    return i2c_master_write_to_device(I2C_MASTER_NUM, LCD_I2C_ADDR, &data, 1, pdMS_TO_TICKS(1000));
}
