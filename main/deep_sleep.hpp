#pragma once

#include "esp_err.h"

// Sleep duration in seconds (10 seconds for testing, increase for production)
constexpr uint64_t DEEP_SLEEP_DURATION_SEC = 10;

/**
 * Prepare for deep sleep by shutting down MQTT and WiFi cleanly.
 * This must be called before entering deep sleep.
 */
esp_err_t deep_sleep_prepare(void);

/**
 * Enter deep sleep with timer wakeup.
 * This function does not return - the device will reset on wakeup.
 */
void deep_sleep_enter(void);
