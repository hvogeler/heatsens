#include "ble_prov.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"
#include "protocomm.h"
#include "protocomm_security.h"
#include "nvs.hpp"
#include "cJSON.h"

static const char *TAG = "hts-ble-prov";

// Custom endpoint name for MQTT configuration
static const char *MQTT_CONFIG_ENDPOINT = "mqtt-config";

/**
 * @brief Custom endpoint handler for MQTT configuration
 *
 * Expects JSON payload:
 * {
 *   "mqtt_broker": "mqtt://host:port",
 *   "mqtt_user": "username",
 *   "mqtt_password": "password"
 * }
 */
static esp_err_t mqtt_config_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                     uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    if (inbuf == nullptr || inlen <= 0)
    {
        ESP_LOGE(TAG, "Invalid MQTT config data");
        return ESP_ERR_INVALID_ARG;
    }

    // Null-terminate the input for JSON parsing
    char *json_str = (char *)malloc(inlen + 1);
    if (!json_str)
    {
        return ESP_ERR_NO_MEM;
    }
    memcpy(json_str, inbuf, inlen);
    json_str[inlen] = '\0';

    ESP_LOGI(TAG, "Received MQTT config: %s", json_str);

    // Parse JSON
    cJSON *json = cJSON_Parse(json_str);
    free(json_str);

    if (!json)
    {
        ESP_LOGE(TAG, "Failed to parse MQTT config JSON");
        return ESP_ERR_INVALID_ARG;
    }

    // Extract fields
    cJSON *broker = cJSON_GetObjectItem(json, "mqtt_broker");
    cJSON *user = cJSON_GetObjectItem(json, "mqtt_user");
    cJSON *password = cJSON_GetObjectItem(json, "mqtt_password");

    esp_err_t ret = ESP_OK;
    Nvs nvs;
    ret = nvs.open_namespace("config");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS namespace");
        cJSON_Delete(json);
        return ret;
    }

    // Save MQTT configuration to NVS
    if (cJSON_IsString(broker) && broker->valuestring)
    {
        nvs.write("mqtt_broker", std::string(broker->valuestring));
        ESP_LOGI(TAG, "Saved MQTT broker: %s", broker->valuestring);
    }

    if (cJSON_IsString(user) && user->valuestring)
    {
        nvs.write("mqtt_user", std::string(user->valuestring));
        ESP_LOGI(TAG, "Saved MQTT user: %s", user->valuestring);
    }

    if (cJSON_IsString(password) && password->valuestring)
    {
        nvs.write("mqtt_password", std::string(password->valuestring));
        ESP_LOGI(TAG, "Saved MQTT password: ***");
    }

    cJSON_Delete(json);

    // Send success response
    const char *response = "{\"status\":\"ok\"}";
    *outlen = strlen(response);
    *outbuf = (uint8_t *)malloc(*outlen);
    if (*outbuf)
    {
        memcpy(*outbuf, response, *outlen);
    }

    return ESP_OK;
}

bool BleProv::check_provisioned()
{
    Nvs nvs;
    std::string ssid;
    std::string password;
    std::string mqtt_broker;

    esp_err_t ret = nvs.open_namespace("config");
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to open NVS namespace 'config': %s", esp_err_to_name(ret));
        is_provisioned_ = false;
        return false;
    }

    // Check WiFi credentials
    ret = nvs.read("wifi_ssid", ssid);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "WiFi SSID not found in NVS: %s", esp_err_to_name(ret));
        is_provisioned_ = false;
        return false;
    }
    if (ssid.empty() || ssid == "unset")
    {
        ESP_LOGI(TAG, "WiFi SSID is empty or unset");
        is_provisioned_ = false;
        return false;
    }

    ret = nvs.read("wifi_password", password);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "WiFi password not found in NVS: %s", esp_err_to_name(ret));
        is_provisioned_ = false;
        return false;
    }

    // Check MQTT credentials
    ret = nvs.read("mqtt_broker", mqtt_broker);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "MQTT broker not found in NVS: %s", esp_err_to_name(ret));
        is_provisioned_ = false;
        return false;
    }
    if (mqtt_broker.empty() || mqtt_broker == "unset")
    {
        ESP_LOGI(TAG, "MQTT broker is empty or unset");
        is_provisioned_ = false;
        return false;
    }

    ESP_LOGI(TAG, "Device is provisioned - SSID: %s, MQTT: %s", ssid.c_str(), mqtt_broker.c_str());
    is_provisioned_ = true;
    return true;
}

// Forward declaration
static void prov_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);

esp_err_t BleProv::start(const std::string &device_name, std::function<void(bool)> on_complete)
{
    device_name_ = device_name;
    on_prov_complete_ = on_complete;

    // Create event group for synchronization
    prov_event_group_ = xEventGroupCreate();
    if (!prov_event_group_)
    {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Starting BLE provisioning with device name: %s", device_name_.c_str());

    // Initialize the default event loop if not already done
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &prov_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &prov_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &prov_event_handler, NULL));

    // Initialize WiFi in station mode
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Initialize provisioning manager
    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM,
    };

    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    // Check if already provisioned via wifi_prov_mgr
    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (provisioned && check_provisioned())
    {
        ESP_LOGI(TAG, "Already provisioned, starting WiFi");
        is_provisioned_ = true;
        wifi_prov_mgr_deinit();
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        xEventGroupSetBits(prov_event_group_, PROV_COMPLETE_BIT);
        return ESP_OK;
    }

    // Register custom endpoint for MQTT configuration
    ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_create(MQTT_CONFIG_ENDPOINT));

    // Start provisioning
    ESP_LOGI(TAG, "Starting provisioning...");

    // Proof of possession - can be customized per device
    const char *pop = "abcd1234";

    // Service name (device name for BLE advertising)
    const char *service_name = device_name_.c_str();

    // Use security version 1 with proof of possession
    wifi_prov_security_t security = WIFI_PROV_SECURITY_1;

    ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(
        security,
        (const void *)pop,
        service_name,
        NULL));

    // Register the custom endpoint handler after starting provisioning
    ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_register(MQTT_CONFIG_ENDPOINT, mqtt_config_handler, NULL));

    ESP_LOGI(TAG, "Provisioning started. Use ESP SoftAP Prov app to connect.");
    ESP_LOGI(TAG, "Device name: %s", service_name);
    ESP_LOGI(TAG, "Proof of Possession: %s", pop);
    ESP_LOGI(TAG, "Custom endpoint '%s' available for MQTT config", MQTT_CONFIG_ENDPOINT);

    return ESP_OK;
}

esp_err_t BleProv::wait_for_completion(uint32_t timeout_ms)
{
    if (!prov_event_group_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    EventBits_t bits = xEventGroupWaitBits(
        prov_event_group_,
        PROV_WIFI_CONNECTED_BIT | PROV_WIFI_FAIL_BIT | PROV_COMPLETE_BIT,
        pdFALSE,
        pdFALSE,
        ticks);

    if (bits & (PROV_WIFI_CONNECTED_BIT | PROV_COMPLETE_BIT))
    {
        return ESP_OK;
    }
    else if (bits & PROV_WIFI_FAIL_BIT)
    {
        return ESP_FAIL;
    }

    return ESP_ERR_TIMEOUT;
}

void BleProv::stop()
{
    wifi_prov_mgr_stop_provisioning();
    wifi_prov_mgr_deinit();
    ESP_LOGI(TAG, "Provisioning stopped");
}

esp_err_t BleProv::reset_credentials()
{
    ESP_LOGI(TAG, "Resetting provisioned credentials");

    // Clear credentials from NVS
    Nvs nvs;
    esp_err_t ret = nvs.open_namespace("config");
    if (ret != ESP_OK)
    {
        return ret;
    }

    nvs.write("wifi_ssid", std::string(""));
    nvs.write("wifi_password", std::string(""));
    nvs.write("mqtt_broker", std::string(""));
    nvs.write("mqtt_user", std::string(""));
    nvs.write("mqtt_password", std::string(""));

    // Also reset the provisioning manager's internal state
    ret = wifi_prov_mgr_reset_provisioning();
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "wifi_prov_mgr_reset_provisioning failed (may not be initialized)");
    }

    is_provisioned_ = false;
    ESP_LOGI(TAG, "Credentials reset successfully");
    return ESP_OK;
}

static void prov_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    auto &prov = BleProv::getInstance();

    if (event_base == WIFI_PROV_EVENT)
    {
        switch (event_id)
        {
        case WIFI_PROV_START:
            ESP_LOGI(TAG, "Provisioning started");
            break;
        case WIFI_PROV_CRED_RECV:
        {
            wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
            ESP_LOGI(TAG, "Received WiFi credentials - SSID: %s", (const char *)wifi_sta_cfg->ssid);

            // Store WiFi credentials in NVS
            Nvs nvs;
            if (nvs.open_namespace("config") == ESP_OK)
            {
                nvs.write("wifi_ssid", std::string((char *)wifi_sta_cfg->ssid));
                nvs.write("wifi_password", std::string((char *)wifi_sta_cfg->password));
                ESP_LOGI(TAG, "WiFi credentials saved to NVS");
            }
            break;
        }
        case WIFI_PROV_CRED_FAIL:
        {
            wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
            ESP_LOGE(TAG, "Provisioning failed! Reason: %s",
                     (*reason == WIFI_PROV_STA_AUTH_ERROR) ? "Auth Error" : "AP Not Found");
            if (prov.get_event_group())
            {
                xEventGroupSetBits(prov.get_event_group(), PROV_WIFI_FAIL_BIT);
            }
            break;
        }
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "WiFi provisioning successful!");
            prov.set_provisioned(true);
            break;
        case WIFI_PROV_END:
            ESP_LOGI(TAG, "Provisioning ended");
            wifi_prov_mgr_deinit();
            if (prov.get_event_group())
            {
                xEventGroupSetBits(prov.get_event_group(), PROV_COMPLETE_BIT);
            }
            break;
        default:
            break;
        }
    }
    else if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "Disconnected, reconnecting...");
            esp_wifi_connect();
            break;
        default:
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        if (prov.get_event_group())
        {
            xEventGroupSetBits(prov.get_event_group(), PROV_WIFI_CONNECTED_BIT);
        }
    }
}
