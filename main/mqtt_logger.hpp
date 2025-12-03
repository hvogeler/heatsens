#pragma once
#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <esp_log.h>
#include "sdkconfig.h"
#include "cJSON.h"
#include "mqtt.hpp"

enum class Severity
{
    Error,
    Warning,
    Info,
    Debug,
    Trace,
};

inline std::string toString(Severity s)
{
    switch (s)
    {
    case Severity::Error:
        return "Error";
        break;
    case Severity::Warning:
        return "Warning";
        break;
    case Severity::Info:
        return "Info";
        break;
    case Severity::Debug:
        return "Debug";
        break;
    case Severity::Trace:
        return "Trace";
        break;
    }
    return "unknown severity";
}

enum class Environment
{
    Dev,
    Test,
    Prod,
};

class MqttLogger
{
private:
    std::string topic;
    std::string device_type;

public:
    MqttLogger() : device_type("heatsens")
    {
        std::string topic_base = CONFIG_HEATSENS_LOG_TOPIC;
        std::string device_id = CONFIG_HEATSENS_DEVICE_ID;
        topic = topic_base + device_id + "/";
    };

    std::string get_esp_localtime();

    template <typename... Args>
    void make_log(Severity severity, std::string code_loc, const char *format, Args... args)
    {
        std::string sever_str = toString(severity);
        std::string full_topic = topic + sever_str;

        char buf[256];
        snprintf(buf, sizeof(buf), format, args...);
        ESP_LOGI("MqttLogger", "%s", buf);

        cJSON *json = cJSON_CreateObject();
        cJSON_AddStringToObject(json, "ts", get_esp_localtime().c_str());
        cJSON_AddStringToObject(json, "device_type", device_type.c_str());
        cJSON_AddStringToObject(json, "device_id", CONFIG_HEATSENS_DEVICE_ID);
        cJSON_AddStringToObject(json, "severity", sever_str.c_str());
        cJSON_AddStringToObject(json, "location", code_loc.c_str());
        cJSON_AddStringToObject(json, "message", buf);
        char *json_str = cJSON_Print(json);
        auto &mqtt = Mqtt::getInstance();

        mqtt.publish_log(full_topic, std::string(json_str));
        free(json_str); // Free the memory allocated by cJSON_Print
        cJSON_Delete(json);
    }

    template <typename... Args>
    void info(std::string code_loc, const char *format, Args... args)
    {
        make_log(Severity::Info, code_loc, format, args...);
    }

    template <typename... Args>
    void error(std::string code_loc, const char *format, Args... args)
    {
        make_log(Severity::Error, code_loc, format, args...);
    }

    template <typename... Args>
    void warning(std::string code_loc, const char *format, Args... args)
    {
        make_log(Severity::Warning, code_loc, format, args...);
    }

    template <typename... Args>
    void debug(std::string code_loc, const char *format, Args... args)
    {
        make_log(Severity::Debug, code_loc, format, args...);
    }

    template <typename... Args>
    void trace(std::string code_loc, const char *format, Args... args)
    {
        make_log(Severity::Trace, code_loc, format, args...);
    }
};