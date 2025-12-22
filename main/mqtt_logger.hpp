#pragma once
#include <string>
#include <optional>
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
    case Severity::Warning:
        return "Warning";
    case Severity::Info:
        return "Info";
    case Severity::Debug:
        return "Debug";
    case Severity::Trace:
        return "Trace";
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
    std::optional<std::string> device_name_opt;

    // Internal helper that does the actual work
    void make_log_va(Severity severity, const char *code_loc, const char *format, va_list args);

public:
    MqttLogger() : device_type("heatsens"), device_name_opt(std::nullopt)
    {
        std::string topic_base = CONFIG_HEATSENS_LOG_TOPIC;
        std::string device_id = CONFIG_HEATSENS_DEVICE_ID;
        topic = topic_base + device_id + "/";
    }

    std::string get_esp_localtime();

    // Variadic functions using C-style varargs (no templates!)
    void info(const char *code_loc, const char *format, ...) __attribute__((format(printf, 3, 4)));
    void error(const char *code_loc, const char *format, ...) __attribute__((format(printf, 3, 4)));
    void warning(const char *code_loc, const char *format, ...) __attribute__((format(printf, 3, 4)));
    void debug(const char *code_loc, const char *format, ...) __attribute__((format(printf, 3, 4)));
    void trace(const char *code_loc, const char *format, ...) __attribute__((format(printf, 3, 4)));
};
