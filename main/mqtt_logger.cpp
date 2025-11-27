#include "mqtt_logger.hpp"
#include <chrono>

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