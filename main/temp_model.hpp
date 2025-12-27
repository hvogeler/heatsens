#pragma once
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <esp_timer.h>
#include "cJSON.h"
#include "mqtt_logger.hpp"
#include "esp_timer_cxx.hpp"
#include <optional>

class TempModel
{
private:
    // Member variables
    std::string device_id_;
    std::string device_name_;
    int heat_actuator_;
    double cur_temp_;
    double tgt_temp_;
    bool is_heating_requested_;
    bool is_heating_;
    mutable std::mutex mutex_;
    MqttLogger logger;
    std::optional<double> night_tgt_temp;
    std::optional<int> night_start_time; // Start of night period (hour in 24h format, local time)
    std::optional<int> night_end_time;   // End of night period (hour in 24h format, local time)
    idf::esp_timer::ESPTimer update_cur_temp_timer;

public:
    // Delete copy constructor and assignment operator
    TempModel(const TempModel &) = delete;
    TempModel &operator=(const TempModel &) = delete;

    // Static method to get the singleton instance
    static TempModel &getInstance()
    {
        static TempModel instance;
        return instance;
    }

    void init();

    bool get_is_heating_requested();
    void toggle_is_heating();

    static void update_cur_temp_cb();

    // Getters
    double get_cur_temp() const { return cur_temp_; }
    double get_tgt_temp() const;
    std::string get_device_name() { return device_name_; }
    int get_heat_actuator() { return heat_actuator_; }
    bool get_is_heating()
    {
        return is_heating_;
    }

    // Setters
    void set_cur_temp(double temp) { cur_temp_ = temp; }
    void set_device_meta(std::string name, int heat_actuator, std::optional<double> night_temp = std::nullopt, std::optional<int> night_start = std::nullopt, std::optional<int> night_end = std::nullopt);
    void set_tgt_temp(double temp);
    void set_night_tgt_temp(std::optional<double> temp);
    void set_night_start_time(std::optional<int> start);
    void set_night_end_time(std::optional<int> end);
    void set_is_heating(bool v)
    {
        is_heating_ = v;
    }

    std::string to_json();
    std::string get_esp_localtime();
    void start_update_cur_temp_timer(int32_t interval_seconds)
    {
        update_cur_temp_timer.start_periodic(std::chrono::seconds(interval_seconds));
    }

    void stop_update_cur_temp_timer()
    {
        update_cur_temp_timer.stop();
    }

    void update_cur_temp_timer_interval(int32_t new_interval_seconds);

    // Mutex methods
    void lock()
    {
        mutex_.lock();
    }
    void unlock() { mutex_.unlock(); }
    std::mutex &getMutex() { return mutex_; }

private:
    // Private constructor
    TempModel() : device_id_(CONFIG_HEATSENS_DEVICE_ID), device_name_("undefined"), cur_temp_(0.0f), tgt_temp_(0.0f), is_heating_requested_(false), is_heating_(false), logger(MqttLogger()), update_cur_temp_timer(TempModel::update_cur_temp_cb)
    {
        // Don't initialize BMP280 here - I2C must be set up first
    }
};
