#include "mqtt_logger.hpp"
#include "temp_model.hpp"
#include <chrono>
#include <mutex>

// Make sure the esp time is initialized for this function to make sense.
// Best place to do this is after wifi is initialized, call wifi.time_sync()
std::string MqttLogger::get_esp_localtime()
{
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%dT%H:%M:%S%z", &timeinfo);
    return std::string(strftime_buf);
}

// Internal implementation that handles the va_list
void MqttLogger::make_log_va(Severity severity, const char *code_loc, const char *format, va_list args)
{
    std::string sever_str = toString(severity);
    std::string full_topic = topic + sever_str;

    // Format the message
    char buf[256];
    vsnprintf(buf, sizeof(buf), format, args);
    ESP_LOGI("MqttLogger", "%s", buf);

    // Get device name from TempModel (no circular dependency issue now!)
    if (!device_name_opt.has_value())
    {
        TempModel &model = TempModel::getInstance();
        std::lock_guard<std::mutex> lock_model(model.getMutex());
        device_name_opt.emplace(model.get_device_name());
    }

    // Build JSON
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "ts", get_esp_localtime().c_str());
    cJSON_AddStringToObject(json, "device_type", device_type.c_str());
    cJSON_AddStringToObject(json, "device_id", CONFIG_HEATSENS_DEVICE_ID);
    cJSON_AddStringToObject(json, "device_name", device_name_opt.value_or("unset").c_str());
    cJSON_AddStringToObject(json, "severity", sever_str.c_str());
    cJSON_AddStringToObject(json, "location", code_loc);
    cJSON_AddStringToObject(json, "message", buf);

    char *json_str = cJSON_Print(json);
    auto &mqtt = Mqtt::getInstance();

    mqtt.publish_log(full_topic, std::string(json_str));
    free(json_str);
    cJSON_Delete(json);
}

// Public variadic functions - they just delegate to make_log_va
void MqttLogger::info(const char *code_loc, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    make_log_va(Severity::Info, code_loc, format, args);
    va_end(args);
}

void MqttLogger::error(const char *code_loc, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    make_log_va(Severity::Error, code_loc, format, args);
    va_end(args);
}

void MqttLogger::warning(const char *code_loc, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    make_log_va(Severity::Warning, code_loc, format, args);
    va_end(args);
}

void MqttLogger::debug(const char *code_loc, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    make_log_va(Severity::Debug, code_loc, format, args);
    va_end(args);
}

void MqttLogger::trace(const char *code_loc, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    make_log_va(Severity::Trace, code_loc, format, args);
    va_end(args);
}
