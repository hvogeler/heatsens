#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <string>
#include <functional>

// Event bits for provisioning status
#define PROV_WIFI_CONNECTED_BIT BIT0
#define PROV_WIFI_FAIL_BIT BIT1
#define PROV_COMPLETE_BIT BIT2

/**
 * @brief BLE WiFi + MQTT Provisioning Manager
 *
 * Handles WiFi and MQTT credential provisioning over BLE using ESP-IDF's
 * wifi_provisioning component with custom endpoints for MQTT config.
 *
 * Provisioned data (stored in NVS "config" namespace):
 * - wifi_ssid, wifi_password (via standard WiFi provisioning)
 * - mqtt_broker, mqtt_user, mqtt_password (via custom "mqtt-config" endpoint)
 *
 * Use the ESP SoftAP Provisioning app (iOS/Android) for WiFi credentials,
 * and send MQTT config as JSON to the "mqtt-config" custom endpoint.
 */
class BleProv
{
private:
    bool is_provisioned_;
    std::string device_name_;
    std::function<void(bool)> on_prov_complete_;
    EventGroupHandle_t prov_event_group_;

    BleProv() : is_provisioned_(false), device_name_("HEATSENS"), prov_event_group_(nullptr) {}
    ~BleProv() {}

public:
    BleProv(const BleProv &) = delete;
    BleProv &operator=(const BleProv &) = delete;

    static BleProv &getInstance()
    {
        static BleProv instance;
        return instance;
    }

    /**
     * @brief Check if WiFi and MQTT credentials are already provisioned in NVS
     * @return true if all required credentials exist, false otherwise
     */
    bool check_provisioned();

    /**
     * @brief Start BLE provisioning service with custom MQTT endpoint
     * @param device_name Name to advertise over BLE (e.g., "HEATSENS_XXXX")
     * @param on_complete Callback when provisioning completes (true=success)
     * @return ESP_OK on success
     */
    esp_err_t start(const std::string &device_name, std::function<void(bool)> on_complete = nullptr);

    /**
     * @brief Wait for provisioning to complete (blocking)
     * @param timeout_ms Maximum time to wait in milliseconds (0 = forever)
     * @return ESP_OK if provisioned successfully, ESP_ERR_TIMEOUT if timed out
     */
    esp_err_t wait_for_completion(uint32_t timeout_ms = 0);

    /**
     * @brief Stop and deinitialize provisioning manager
     */
    void stop();

    /**
     * @brief Reset provisioned credentials (for testing/re-provisioning)
     * @return ESP_OK on success
     */
    esp_err_t reset_credentials();

    /**
     * @brief Get the device name used for BLE advertising
     */
    std::string get_device_name() const { return device_name_; }

    /**
     * @brief Check if device is currently provisioned
     */
    bool is_provisioned() const { return is_provisioned_; }

    /**
     * @brief Set provisioned status (called by event handler)
     */
    void set_provisioned(bool v) { is_provisioned_ = v; }

    /**
     * @brief Get the event group handle for signaling
     */
    EventGroupHandle_t get_event_group() { return prov_event_group_; }
};
