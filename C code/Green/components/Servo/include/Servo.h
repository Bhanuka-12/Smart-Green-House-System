#ifndef SERVO_H
#define SERVO_H


#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define SERVO_GPIO 18  // Change this pin according to your setup

void servo_init(void);
void servo_set_angle(uint8_t angle);

#endif // SERVO_H
