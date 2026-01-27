#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>
#include <mutex>
#include "esp_err.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_netif.h"

class Wifi
{
private:
    EventGroupHandle_t s_wifi_event_group_;
    int s_retry_num_;
    std::string wifi_ssid;
    std::string wifi_password;
    esp_netif_t *sta_netif_;
    esp_event_handler_instance_t instance_any_id_;
    esp_event_handler_instance_t instance_got_ip_;
    bool is_shutting_down_;

    mutable std::mutex mutex_;

    Wifi() : s_wifi_event_group_(nullptr), s_retry_num_(0), wifi_ssid("unset"), wifi_password("unset"),
             sta_netif_(nullptr), instance_any_id_(nullptr), instance_got_ip_(nullptr),
             is_shutting_down_(false), is_connected(false)
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
    esp_err_t wifi_disconnect(void);
    static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data);
    esp_err_t time_sync(void);
    std::string get_wifi_ssid()
    {
        return wifi_ssid;
    }
    bool is_shutting_down() const { return is_shutting_down_; }
    void set_shutting_down(bool v) { is_shutting_down_ = v; }

    std::mutex &getMutex() { return mutex_; }
};
