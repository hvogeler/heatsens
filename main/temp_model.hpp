#pragma once
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <esp_timer.h>
#include "cJSON.h"

class TempModel
{
private:
    // Member variables
    double cur_temp_;
    double tgt_temp_;
    bool is_heating_requested_;
    bool is_heating_;
    mutable std::mutex mutex_;

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
    double getCurTemp() const { return cur_temp_; }
    double getTgtTemp() const { return tgt_temp_; }
    bool get_is_heating()
    {
        return is_heating_;
    }

    // Setters
    void set_cur_temp(double temp) { cur_temp_ = temp; }
    void set_tgt_temp(double temp);
    void set_is_heating(bool v)
    {
        is_heating_ = v;
    }

    std::string toJson();
    std::string get_esp_localtime();

    // Mutex methods
    void lock()
    {
        mutex_.lock();
    }
    void unlock() { mutex_.unlock(); }
    std::mutex &getMutex() { return mutex_; }

private:
    // Private constructor
    TempModel() : cur_temp_(0.0f), tgt_temp_(0.0f), is_heating_requested_(false), is_heating_(false)
    {
        // Don't initialize BMP280 here - I2C must be set up first
    }
};
