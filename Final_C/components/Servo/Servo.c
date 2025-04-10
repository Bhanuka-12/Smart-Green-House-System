#include "Servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_MIN_PULSEWIDTH_US 500   // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MAX_ANGLE 180           // Maximum angle of servo rotation

static mcpwm_unit_t mcpwm_unit = MCPWM_UNIT_0;
static mcpwm_timer_t mcpwm_timer = MCPWM_TIMER_0;

void servo_init(void) {
    printf("Initializing servo on GPIO %d\n", SERVO_GPIO);

    mcpwm_gpio_init(mcpwm_unit, MCPWM0A, SERVO_GPIO);

    mcpwm_config_t pwm_config = {
        .frequency = 50,   // 50Hz frequency for servos
        .cmpr_a = 0,       // Initial duty cycle 0%
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER
    };

    mcpwm_init(mcpwm_unit, mcpwm_timer, &pwm_config);
}

void servo_set_angle(uint8_t angle) {
    if (angle > SERVO_MAX_ANGLE) {
        angle = SERVO_MAX_ANGLE; // Restrict angle to max
    }

    uint32_t pulse_width = SERVO_MIN_PULSEWIDTH_US + 
                           ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle / SERVO_MAX_ANGLE);

    mcpwm_set_duty_in_us(mcpwm_unit, mcpwm_timer, MCPWM_OPR_A, pulse_width);
}
