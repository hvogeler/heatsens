#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>
#include <mutex>
#include "esp_err.h"
#include "freertos/event_groups.h"
#include "esp_event.h"

class Wifi
{
private:
    EventGroupHandle_t s_wifi_event_group_;
    int s_retry_num_;
    std::string wifi_ssid;
    std::string wifi_password;

    mutable std::mutex mutex_;

    Wifi() : s_wifi_event_group_(nullptr), s_retry_num_(0), wifi_ssid("unset"), wifi_password("unset"), is_connected(false)
    {
    }

public:
    // Delete copy constructor and assignment operator
    Wifi(const Wifi &) = delete;
    Wifi &operator=(const Wifi &) = delete;

    // Static method to get the singleton instance
    static Wifi &getInstance()
    {
        static Wifi instance;
        return instance;
    }

    bool is_connected;

    esp_err_t wifi_connect(void);
    static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data);
    esp_err_t time_sync(void);
    std::string get_wifi_ssid()
    {
        return wifi_ssid;
    }

    std::mutex &getMutex() { return mutex_; }
};
