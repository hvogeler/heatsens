#include "webprov.hpp"

#include <cstring>
#include <cinttypes>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"

static const char *TAG = "webprov";

// Embedded HTML template (from webprov.html via EMBED_FILES in CMakeLists.txt)
extern const uint8_t webprov_html_start[] asm("_binary_webprov_html_start");
extern const uint8_t webprov_html_end[] asm("_binary_webprov_html_end");

#define NVS_NAMESPACE "config"
#define NVS_NAMESPACE_META "meta"
#define HTML_BUF_SIZE 12288

// NVS keys for 'config' namespace
#define NVS_KEY_WIFI_SSID "wifi_ssid"
#define NVS_KEY_WIFI_PASSWORD "wifi_password"
#define NVS_KEY_MQTT_BROKER "mqtt_broker"
#define NVS_KEY_MQTT_USER "mqtt_user"
#define NVS_KEY_MQTT_PASSWORD "mqtt_password"

// NVS keys for 'meta' namespace
#define NVS_KEY_DEVICE_NAME "device_name"
#define NVS_KEY_HEAT_ACTUATOR "heat_actuator"

// Event bits for provisioning flow
#define PROV_DONE_BIT BIT0
#define PROV_CANCEL_BIT BIT1

void WebProv::clear_form_data()
{
    form_wifi_ssid_.clear();
    form_wifi_password_.clear();
    form_mqtt_broker_.clear();
    form_mqtt_user_.clear();
    form_mqtt_password_.clear();
    form_device_name_.clear();
    form_heat_actuator_ = 0;
    form_error_.clear();
}

std::string WebProv::generate_html()
{
    char *buf = new char[HTML_BUF_SIZE];
    snprintf(buf, HTML_BUF_SIZE, reinterpret_cast<const char *>(webprov_html_start),
             form_error_.c_str(),
             form_wifi_ssid_.c_str(),
             form_wifi_password_.c_str(),
             form_mqtt_broker_.c_str(),
             form_mqtt_user_.c_str(),
             form_mqtt_password_.c_str(),
             form_device_name_.c_str(),
             static_cast<int>(form_heat_actuator_));
    std::string result(buf);
    delete[] buf;
    return result;
}

esp_err_t WebProv::save_config_to_nvs(const std::string &wifi_ssid,
                                      const std::string &wifi_password,
                                      const std::string &mqtt_broker,
                                      const std::string &mqtt_user,
                                      const std::string &mqtt_password)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(handle, NVS_KEY_WIFI_SSID, wifi_ssid.c_str());
    if (err != ESP_OK)
        goto cleanup;

    err = nvs_set_str(handle, NVS_KEY_WIFI_PASSWORD, wifi_password.c_str());
    if (err != ESP_OK)
        goto cleanup;

    err = nvs_set_str(handle, NVS_KEY_MQTT_BROKER, mqtt_broker.c_str());
    if (err != ESP_OK)
        goto cleanup;

    err = nvs_set_str(handle, NVS_KEY_MQTT_USER, mqtt_user.c_str());
    if (err != ESP_OK)
        goto cleanup;

    err = nvs_set_str(handle, NVS_KEY_MQTT_PASSWORD, mqtt_password.c_str());
    if (err != ESP_OK)
        goto cleanup;

    err = nvs_commit(handle);

cleanup:
    nvs_close(handle);
    return err;
}

esp_err_t WebProv::save_meta_to_nvs(const std::string &device_name, int32_t heat_actuator)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE_META, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS meta namespace: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(handle, NVS_KEY_DEVICE_NAME, device_name.c_str());
    if (err != ESP_OK)
        goto cleanup;

    err = nvs_set_i32(handle, NVS_KEY_HEAT_ACTUATOR, heat_actuator);
    if (err != ESP_OK)
        goto cleanup;

    err = nvs_commit(handle);

cleanup:
    nvs_close(handle);
    return err;
}

esp_err_t WebProv::root_get_handler(httpd_req_t *req)
{
    auto &prov = WebProv::getInstance();
    std::string html = prov.generate_html();

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html.c_str(), html.length());

    return ESP_OK;
}

esp_err_t WebProv::config_post_handler(httpd_req_t *req)
{
    auto &prov = WebProv::getInstance();

    char *buf = new char[req->content_len + 1];

    int received = httpd_req_recv(req, buf, req->content_len);
    if (received <= 0)
    {
        delete[] buf;
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to receive data");
        return ESP_FAIL;
    }
    buf[received] = '\0';

    ESP_LOGI(TAG, "Received config: %s", buf);

    cJSON *json = cJSON_Parse(buf);
    delete[] buf;

    if (!json)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *wifi_ssid = cJSON_GetObjectItem(json, "wifi_ssid");
    cJSON *wifi_password = cJSON_GetObjectItem(json, "wifi_password");
    cJSON *mqtt_broker = cJSON_GetObjectItem(json, "mqtt_broker");
    cJSON *mqtt_user = cJSON_GetObjectItem(json, "mqtt_user");
    cJSON *mqtt_password = cJSON_GetObjectItem(json, "mqtt_password");
    cJSON *device_name = cJSON_GetObjectItem(json, "device_name");
    cJSON *heat_actuator = cJSON_GetObjectItem(json, "heat_actuator");

    // Validate all fields present and non-empty
    if (!cJSON_IsString(wifi_ssid) || strlen(wifi_ssid->valuestring) == 0 ||
        !cJSON_IsString(wifi_password) || strlen(wifi_password->valuestring) == 0 ||
        !cJSON_IsString(mqtt_broker) || strlen(mqtt_broker->valuestring) == 0 ||
        !cJSON_IsString(mqtt_user) || strlen(mqtt_user->valuestring) == 0 ||
        !cJSON_IsString(mqtt_password) || strlen(mqtt_password->valuestring) == 0 ||
        !cJSON_IsString(device_name) || strlen(device_name->valuestring) == 0 ||
        !cJSON_IsNumber(heat_actuator))
    {
        // Preserve form data for re-display
        if (cJSON_IsString(wifi_ssid))
            prov.form_wifi_ssid_ = wifi_ssid->valuestring;
        if (cJSON_IsString(wifi_password))
            prov.form_wifi_password_ = wifi_password->valuestring;
        if (cJSON_IsString(mqtt_broker))
            prov.form_mqtt_broker_ = mqtt_broker->valuestring;
        if (cJSON_IsString(mqtt_user))
            prov.form_mqtt_user_ = mqtt_user->valuestring;
        if (cJSON_IsString(mqtt_password))
            prov.form_mqtt_password_ = mqtt_password->valuestring;
        if (cJSON_IsString(device_name))
            prov.form_device_name_ = device_name->valuestring;
        if (cJSON_IsNumber(heat_actuator))
            prov.form_heat_actuator_ = heat_actuator->valueint;
        prov.form_error_ = "All fields are required";

        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "All fields are required");
        return ESP_FAIL;
    }

    // Save to NVS 'config' namespace
    esp_err_t err = prov.save_config_to_nvs(
        wifi_ssid->valuestring,
        wifi_password->valuestring,
        mqtt_broker->valuestring,
        mqtt_user->valuestring,
        mqtt_password->valuestring);

    if (err != ESP_OK)
    {
        cJSON_Delete(json);
        prov.form_error_ = "Failed to save configuration";
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save configuration");
        return ESP_FAIL;
    }

    // Save to NVS 'meta' namespace
    err = prov.save_meta_to_nvs(device_name->valuestring, heat_actuator->valueint);

    cJSON_Delete(json);

    if (err != ESP_OK)
    {
        prov.form_error_ = "Failed to save device metadata";
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save device metadata");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Configuration saved successfully");
    httpd_resp_sendstr(req, "Configuration saved");

    // Signal provisioning complete
    xEventGroupSetBits(prov.prov_event_group_, PROV_DONE_BIT);

    return ESP_OK;
}

esp_err_t WebProv::cancel_post_handler(httpd_req_t *req)
{
    auto &prov = WebProv::getInstance();

    ESP_LOGI(TAG, "Provisioning cancelled");
    httpd_resp_sendstr(req, "Cancelled");

    // Signal provisioning cancelled
    xEventGroupSetBits(prov.prov_event_group_, PROV_CANCEL_BIT);

    return ESP_OK;
}

httpd_handle_t WebProv::start_webserver()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);

    if (httpd_start(&server_, &config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return nullptr;
    }

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
        .user_ctx = nullptr};
    httpd_register_uri_handler(server_, &root_uri);

    httpd_uri_t config_uri = {
        .uri = "/config",
        .method = HTTP_POST,
        .handler = config_post_handler,
        .user_ctx = nullptr};
    httpd_register_uri_handler(server_, &config_uri);

    httpd_uri_t cancel_uri = {
        .uri = "/cancel",
        .method = HTTP_POST,
        .handler = cancel_post_handler,
        .user_ctx = nullptr};
    httpd_register_uri_handler(server_, &cancel_uri);

    return server_;
}

void WebProv::stop_webserver()
{
    if (server_)
    {
        httpd_stop(server_);
        server_ = nullptr;
    }
}

void WebProv::wifi_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        ESP_LOGI(TAG, "Station joined AP");
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        ESP_LOGI(TAG, "Station left AP");
    }
}

esp_err_t WebProv::start_wifi_ap()
{
    // Generate SSID: {PROJECT_NAME}_cfg_{DEVICE_ID}
    ap_ssid_ = std::string(CONFIG_WEBPROV_PROJECT_NAME) + "_cfg_" + CONFIG_HEATSENS_DEVICE_ID;

    ESP_LOGI(TAG, "Starting WiFi AP with SSID: %s", ap_ssid_.c_str());

    // Initialize networking stack if not already done
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    // Create AP netif
    ap_netif_ = esp_netif_create_default_wifi_ap();

    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    // Register event handler
    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        nullptr,
                                                        &instance_any_id));

    // Configure AP
    wifi_config_t wifi_config = {};
    wifi_config.ap.ssid_len = ap_ssid_.length();
    wifi_config.ap.channel = CONFIG_WEBPROV_AP_CHANNEL;
    wifi_config.ap.max_connection = CONFIG_WEBPROV_AP_MAX_CONNECTIONS;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    wifi_config.ap.pmf_cfg.required = false;

    strncpy((char *)wifi_config.ap.ssid, ap_ssid_.c_str(), sizeof(wifi_config.ap.ssid));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started. Connect to SSID: %s", ap_ssid_.c_str());

    return ESP_OK;
}

void WebProv::stop_wifi_ap()
{
    esp_wifi_stop();
    esp_wifi_deinit();
    if (ap_netif_)
    {
        esp_netif_destroy_default_wifi(ap_netif_);
        ap_netif_ = nullptr;
    }
}

bool WebProv::is_provisioned()
{
    ESP_LOGI(TAG, "B");

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    ESP_LOGI(TAG, "C");
    if (err != ESP_OK)
    {
        return false;
    }

    // Check if all required keys exist
    size_t required_len;
    bool provisioned = true;

    const char *keys[] = {NVS_KEY_WIFI_SSID, NVS_KEY_WIFI_PASSWORD, NVS_KEY_MQTT_BROKER,
                          NVS_KEY_MQTT_USER, NVS_KEY_MQTT_PASSWORD};

    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++)
    {
        err = nvs_get_str(handle, keys[i], nullptr, &required_len);
        if (err != ESP_OK || required_len == 0)
        {
            provisioned = false;
            break;
        }
    }

    nvs_close(handle);
    return provisioned;
}

void WebProv::run_provisioning()
{
    clear_form_data();

    // Create event group
    prov_event_group_ = xEventGroupCreate();

    // Start WiFi AP
    ESP_ERROR_CHECK(start_wifi_ap());

    // Notify UI callback
    if (on_prov_start)
    {
        on_prov_start(ap_ssid_);
    }

    // Start HTTP server
    start_webserver();

    ESP_LOGI(TAG, "Provisioning started. Waiting for configuration...");

    // Wait for provisioning to complete or cancel
    EventBits_t bits = xEventGroupWaitBits(prov_event_group_,
                                           PROV_DONE_BIT | PROV_CANCEL_BIT,
                                           pdFALSE, pdFALSE,
                                           portMAX_DELAY);

    // Small delay to allow HTTP response to be sent
    vTaskDelay(pdMS_TO_TICKS(500));

    // Cleanup
    stop_webserver();
    stop_wifi_ap();
    vEventGroupDelete(prov_event_group_);
    prov_event_group_ = nullptr;

    // Notify UI callback
    if (on_prov_end)
    {
        on_prov_end("");
    }

    if (bits & PROV_DONE_BIT)
    {
        ESP_LOGI(TAG, "Provisioning complete. Rebooting...");
    }
    else
    {
        ESP_LOGI(TAG, "Provisioning cancelled. Rebooting...");
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

esp_err_t WebProv::init()
{
    ESP_LOGI(TAG, "A");
    if (!is_provisioned())
    {
        ESP_LOGI(TAG, "Device not provisioned. Starting provisioning...");
        run_provisioning();
        // Never returns - reboots after provisioning
    }

    ESP_LOGI(TAG, "Device is provisioned");
    return ESP_OK;
}

void WebProv::start_provisioning()
{
    ESP_LOGI(TAG, "Manual provisioning requested");
    run_provisioning();
    // Never returns - reboots after provisioning
}
