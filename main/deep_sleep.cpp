#include "deep_sleep.hpp"
#include "mqtt.hpp"
#include "wifi.hpp"
#include "mpu6050.hpp"
#include "esp_sleep.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <mutex>
#include <inttypes.h>

static const char *TAG = "deep_sleep";

esp_err_t deep_sleep_prepare(void)
{
    ESP_LOGI(TAG, "Preparing for deep sleep...");

    // 1. Stop MQTT first (it depends on WiFi)
    auto &mqtt = Mqtt::getInstance();
    {
        std::lock_guard<std::mutex> lock(mqtt.getMutex());
        mqtt.stop();
    }

    // Small delay to allow MQTT disconnect to complete
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. Disconnect WiFi
    auto &wifi = Wifi::getInstance();
    {
        std::lock_guard<std::mutex> lock(wifi.getMutex());
        wifi.wifi_disconnect();
    }

    // 3. Clear any pending motion interrupt to prevent immediate wakeup
    auto &motion_sensor = Mpu6050::getInstance();
    if (motion_sensor.is_initialized)
    {
        bool motion_pending;
        motion_sensor.check_motion_interrupt(&motion_pending);
        if (motion_pending)
        {
            ESP_LOGI(TAG, "Cleared pending motion interrupt before sleep");
        }
    }

    ESP_LOGI(TAG, "Deep sleep preparation complete");
    return ESP_OK;
}

void deep_sleep_enter(void)
{
    ESP_LOGI(TAG, "Entering deep sleep for %" PRIu64 " seconds (or until motion)...", DEEP_SLEEP_DURATION_SEC);

    // Disable all wakeup sources first
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    // Configure timer wakeup
    uint64_t sleep_time_us = DEEP_SLEEP_DURATION_SEC * 1000000ULL;
    esp_err_t ret = esp_sleep_enable_timer_wakeup(sleep_time_us);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable timer wakeup: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Timer wakeup configured for %" PRIu64 " microseconds", sleep_time_us);

    // Configure EXT0 wakeup on motion interrupt GPIO
    // Wake up when GPIO goes HIGH (MPU6050 INT pin triggers on motion)
    ret = esp_sleep_enable_ext0_wakeup(MOTION_WAKEUP_GPIO, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable EXT0 wakeup on GPIO%d: %s",
                 MOTION_WAKEUP_GPIO, esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "EXT0 wakeup configured on GPIO%d (motion interrupt)", MOTION_WAKEUP_GPIO);

    // Small delay to ensure logs are flushed
    vTaskDelay(pdMS_TO_TICKS(100));

    // Enter deep sleep (this does not return)
    esp_deep_sleep_start();
}
