#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_err.h"
#include <string>
#include <functional>

class WebProv
{
private:
    EventGroupHandle_t prov_event_group_;
    httpd_handle_t server_;
    esp_netif_t *ap_netif_;
    std::string ap_ssid_;

    // Form data preserved on error
    std::string form_wifi_ssid_;
    std::string form_wifi_password_;
    std::string form_mqtt_broker_;
    std::string form_mqtt_user_;
    std::string form_mqtt_password_;
    std::string form_device_name_;
    int32_t form_heat_actuator_;
    std::string form_error_;

    WebProv()
        : prov_event_group_(nullptr),
          server_(nullptr),
          ap_netif_(nullptr),
          form_heat_actuator_(0)
    {
    }

    // Delete copy constructor and assignment operator
    WebProv(const WebProv &) = delete;
    WebProv &operator=(const WebProv &) = delete;

    esp_err_t start_wifi_ap();
    void stop_wifi_ap();
    httpd_handle_t start_webserver();
    void stop_webserver();
    void run_provisioning();
    void clear_form_data();
    std::string generate_html();
    esp_err_t save_config_to_nvs(const std::string &wifi_ssid,
                                  const std::string &wifi_password,
                                  const std::string &mqtt_broker,
                                  const std::string &mqtt_user,
                                  const std::string &mqtt_password);
    esp_err_t save_meta_to_nvs(const std::string &device_name, int32_t heat_actuator);

    static esp_err_t root_get_handler(httpd_req_t *req);
    static esp_err_t config_post_handler(httpd_req_t *req);
    static esp_err_t cancel_post_handler(httpd_req_t *req);
    static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data);

public:
    using UiCallback = std::function<void(const std::string &ap_ssid)>;

    UiCallback on_prov_start;
    UiCallback on_prov_end;

    static WebProv &getInstance()
    {
        static WebProv instance;
        return instance;
    }

    /**
     * @brief Initialize web provisioning
     *
     * Checks if the device has been provisioned. If not, automatically
     * starts the provisioning process. If already provisioned, returns
     * immediately.
     *
     * @return ESP_OK on success, or error code
     */
    esp_err_t init();

    /**
     * @brief Force start provisioning mode
     *
     * Starts the provisioning access point and HTTP server.
     * This function blocks until provisioning is complete or cancelled,
     * then reboots the device.
     *
     * Call this when the user triggers provisioning (e.g., long press BOOT button).
     */
    void start_provisioning();

    /**
     * @brief Check if the device has been provisioned
     *
     * @return true if provisioned (all required NVS values exist), false otherwise
     */
    bool is_provisioned();

    const std::string &get_ap_ssid() const { return ap_ssid_; }
};
