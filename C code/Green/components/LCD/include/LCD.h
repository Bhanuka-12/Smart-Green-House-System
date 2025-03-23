#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "driver/i2c.h"
#include "esp_err.h"

#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

esp_err_t lcd_init(void);
esp_err_t lcd_clear(void);
esp_err_t lcd_home(void);
esp_err_t lcd_write_string(const char *str);
esp_err_t lcd_set_cursor(uint8_t col, uint8_t row);

#endif // LCD_I2C_H
