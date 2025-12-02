#include "temp_model.hpp"
#include "esp_timer_cxx.hpp"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "bmp280.hpp"
#include "nvs.hpp"

static std::string TAG = "temp_model";

void TempModel::init()
{
    auto &bmp280 = Bmp280::getInstance();
    bmp280.init();

    Nvs nvs;
    nvs.open_namespace("heatsens");
    esp_err_t ret = nvs.read("tgt_temp", tgt_temp_);
    tgt_temp_ /= 10.0;
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG.c_str(), "No target temp found in NVS");
    }
}

void TempModel::set_tgt_temp(double temp)
{
    tgt_temp_ = temp;

    Nvs nvs;
    nvs.open_namespace("heatsens");
    esp_err_t ret = nvs.write("tgt_temp", temp * 10.0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG.c_str(), "Failed to write target temp to NVS");
    }
}

void TempModel::toggle_is_heating()
{
    double histeresis = CONFIG_HEATSENS_HISTERESIS / 100.0;

    double temp_diff = tgt_temp_ - cur_temp_;
    bool should_turn_on_heater = temp_diff > histeresis && !is_heating_requested_;
    if (should_turn_on_heater)
    {
        ESP_LOGI(TAG.c_str(), "Heating ON, tgt_temp=%.1f, cur_temp=%.1f, histeresis=%.1f", tgt_temp_, cur_temp_, histeresis);
        logger.info(TAG, "Heating ON, tgt_temp=%.1f, cur_temp=%.1f, histeresis=%.1f", tgt_temp_, cur_temp_, histeresis);
        is_heating_requested_ = true;
    }

    bool should_turn_off_heater = -temp_diff > histeresis && is_heating_requested_;
    if (should_turn_off_heater)
    {
        ESP_LOGI(TAG.c_str(), "Heating OFF, tgt_temp=%.1f, cur_temp=%.1f, histeresis=%.1f", tgt_temp_, cur_temp_, histeresis);
        logger.info(TAG, "Heating OFF, tgt_temp=%.1f, cur_temp=%.1f, histeresis=%.1f", tgt_temp_, cur_temp_, histeresis);
        is_heating_requested_ = false;
    }
}

bool TempModel::get_is_heating_requested()
{
    return is_heating_requested_;
}

// Make sure the esp time is initialized for this function to make sense.
// Best place to do this is after wifi is initialized, call wifi.time_sync()
std::string TempModel::get_esp_localtime()
{
    time_t now;
    struct tm timeinfo;

    time(&now);
    localtime_r(&now, &timeinfo);

    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%dT%H:%M:%S%z", &timeinfo);
    return std::string(strftime_buf);
}

/* Creates json string
    {
        "cur_temp":	23.205585479736328,
        "tgt_temp":	21
    }
*/
std::string TempModel::toJson()
{
    cJSON *doc = cJSON_CreateObject();
    cJSON_AddNumberToObject(doc, "cur_temp", cur_temp_);
    cJSON_AddNumberToObject(doc, "tgt_temp", tgt_temp_);
    cJSON_AddBoolToObject(doc, "is_heating", get_is_heating_requested());
    cJSON_AddStringToObject(doc, "ts", get_esp_localtime().c_str());

    char *json_str = cJSON_Print(doc);
    std::string ret(json_str);
    free(json_str); // Free the memory allocated by cJSON_Print
    cJSON_Delete(doc);
    return ret;
}

void TempModel::update_cur_temp_cb()
{
    auto &bmp280 = Bmp280::getInstance();
    std::lock_guard<std::mutex> lock_bmp280(bmp280.getMutex());

    if (bmp280.is_initialized)
    {
        int32_t raw_temp, raw_press;
        double temperature, pressure;

        if (bmp280.read_raw(&raw_temp, &raw_press) != ESP_OK)
        {
            ESP_LOGE(TAG.c_str(), "Failed to read raw sensor data");
            return;
        }

        bmp280.compensate_temp_press(raw_temp, raw_press, &temperature, &pressure);

        auto &temp_model = TempModel::getInstance();
        std::lock_guard<std::mutex> lock_temp_model(temp_model.getMutex());
        temp_model.set_cur_temp(temperature);
        temp_model.toggle_is_heating();

        // ESP_LOGI(TAG.c_str(), "Current Temp: %.2f°C, Target Temp: %.1f, Pressure: %.2f Pa", temperature, temp_model.getTgtTemp(), pressure);
        temp_model.logger.info(TAG, "Current Temp: %.2f°C, Target Temp: %.1f, Pressure: %.2f Pa", temperature, temp_model.getTgtTemp(), pressure);
    }
    else
    {
        ESP_LOGE(TAG.c_str(), "BMP280 sensor not initialized");
    }
}
