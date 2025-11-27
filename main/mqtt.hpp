#pragma once
#include <mutex>
#include <format>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"

class Mqtt
{
private:
    esp_mqtt5_connection_property_config_t cnct_prop;
    esp_mqtt_client_config_t mqtt5_cfg;
    esp_mqtt5_client_handle_t mqtt_client = nullptr;
    std::string current_temp_topic;
    std::string target_temp_topic;
    std::string ctrl_info_topic;
    int msg_id;

    esp_mqtt_connect_return_code_t connect_return_code;
    bool is_mqtt_connected;

    std::string mqtt_broker_;
    std::string mqtt_user_;
    std::string mqtt_password_;

    mutable std::mutex mutex_;

public:
    // Delete copy constructor and assignment operator
    Mqtt(const Mqtt &) = delete;
    Mqtt &operator=(const Mqtt &) = delete;

    // Static method to get the singleton instance
    static Mqtt &getInstance()
    {
        static Mqtt instance;
        return instance;
    }

    std::string get_current_temp_topic()
    {
        return current_temp_topic;
    }

    std::string get_target_temp_topic()
    {
        return target_temp_topic;
    }

    std::string get_ctrl_info_topic()
    {
        return ctrl_info_topic;
    }

    bool get_is_mqtt_connected()
    {
        return is_mqtt_connected;
    }

    void set_is_mqtt_connected(bool v)
    {
        is_mqtt_connected = v;
    }

    std::string get_is_mqtt_broker_url()
    {
        return mqtt_broker_;
    }

    void set_connect_return_code(esp_mqtt_connect_return_code_t r)
    {
        connect_return_code = r;
    }

    esp_mqtt_connect_return_code_t get_connect_return_code()
    {
        return connect_return_code;
    }

    // Mutex accessor
    std::mutex &getMutex() { return mutex_; }

    // Add your public methods here
    esp_err_t connect();
    void publish(std::string);
    void publish_log(std::string, std::string);
    void subscribe();

private:
    // Private constructor
    Mqtt() : connect_return_code(MQTT_CONNECTION_ACCEPTED), is_mqtt_connected(false)
    {
        current_temp_topic = CONFIG_HEATSENS_CURRENT_TEMP_TOPIC;
        current_temp_topic += CONFIG_HEATSENS_DEVICE_ID;
        target_temp_topic = CONFIG_HEATSENS_TARGET_TEMP_TOPIC;
        target_temp_topic += CONFIG_HEATSENS_DEVICE_ID;
        ctrl_info_topic = CONFIG_HEATSENS_CTRL_INFO_TOPIC;
        ctrl_info_topic += CONFIG_HEATSENS_DEVICE_ID;

        // Initialize cnct_prop
        cnct_prop = {}; // Zero-initialize first
        cnct_prop.session_expiry_interval = 3600;
        cnct_prop.maximum_packet_size = 1024;
        cnct_prop.receive_maximum = 65535;
        cnct_prop.topic_alias_maximum = 2;
        cnct_prop.request_resp_info = true;
        cnct_prop.request_problem_info = true;
        cnct_prop.user_property = nullptr;
        cnct_prop.will_delay_interval = 10;
        cnct_prop.message_expiry_interval = 10;
        cnct_prop.payload_format_indicator = true;
        cnct_prop.content_type = nullptr;
        cnct_prop.response_topic = nullptr;
        cnct_prop.correlation_data = nullptr;
        cnct_prop.correlation_data_len = 0;
        cnct_prop.will_user_property = nullptr;

        // Initialize mqtt5_cfg
        mqtt5_cfg = {}; // Zero-initialize first
        mqtt5_cfg.broker.address.uri = "unset";
        mqtt5_cfg.session.protocol_ver = MQTT_PROTOCOL_V_5;
        mqtt5_cfg.network.disable_auto_reconnect = false;
        mqtt5_cfg.credentials.username = "unset";
        mqtt5_cfg.credentials.authentication.password = "unset";
        mqtt5_cfg.session.last_will.topic = "heatsens/will/" CONFIG_HEATSENS_DEVICE_ID;
        mqtt5_cfg.session.last_will.msg = "dirty disconnect of temp sensor";
        mqtt5_cfg.session.last_will.msg_len = 32;
        mqtt5_cfg.session.last_will.qos = 1;
        mqtt5_cfg.session.last_will.retain = true;
    }
};