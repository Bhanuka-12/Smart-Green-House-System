#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "driver/i2c.h"
#include "esp_err.h"

#define LCD_I2C_ADDR 0x3F  // Corrected address from datasheet
#define LCD_COLS 16
#define LCD_ROWS 2

esp_err_t lcd_i2c_init(void);
esp_err_t lcd_i2c_clear(void);
esp_err_t lcd_i2c_home(void);
esp_err_t lcd_i2c_write_string(const char *str);
esp_err_t lcd_i2c_set_cursor(uint8_t col, uint8_t row);
esp_err_t lcd_i2c_backlight_on(void);
esp_err_t lcd_i2c_backlight_off(void);

#endif // LCD_I2C_H
