#pragma once

#include "esp_err.h"
#include "driver/gpio.h"

// Sleep duration in seconds (10 seconds for testing, increase for production)
constexpr uint64_t DEEP_SLEEP_DURATION_SEC = 300;

// Motion interrupt GPIO pin (connected to MPU6050 INT pin)
constexpr gpio_num_t MOTION_WAKEUP_GPIO = GPIO_NUM_3;

/**
 * Prepare for deep sleep by shutting down MQTT and WiFi cleanly.
 * This must be called before entering deep sleep.
 */
esp_err_t deep_sleep_prepare(void);

/**
 * Enter deep sleep with timer wakeup and motion interrupt wakeup.
 * Wakes up on either:
 *   - Timer expiration (DEEP_SLEEP_DURATION_SEC)
 *   - Motion detected (GPIO3 goes HIGH from MPU6050 INT)
 * This function does not return - the device will reset on wakeup.
 */
void deep_sleep_enter(void);
