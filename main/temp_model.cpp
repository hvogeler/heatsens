#include "temp_model.hpp"
#include "esp_timer_cxx.hpp"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "bmp280.hpp"
#include "nvs.hpp"

static constexpr char *TAG = "temp_model";

void TempModel::init()
{
    auto &bmp280 = Bmp280::getInstance();
    bmp280.init();

    Nvs nvs_heatsens;
    nvs_heatsens.open_namespace("heatsens");
    esp_err_t ret = nvs_heatsens.read("tgt_temp", tgt_temp_);
    tgt_temp_ /= 10.0;
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "No target temp found in NVS");
    }

    Nvs nvs_config;
    nvs_config.open_namespace("meta");
    ret = nvs_config.read("device_name", device_name_);
    if (ret != ESP_OK)
    {
        device_name_ = "undefined";
        ESP_LOGE(TAG, "No device name found in NVS");
    }

    ret = nvs_config.read("heat_actuator", heat_actuator_);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "No heat actuator id found in NVS");
    }

    // Load night mode parameters from NVS
    double night_temp_stored;
    ret = nvs_config.read("night_tgt_temp", night_temp_stored);
    if (ret == ESP_OK)
    {
        night_tgt_temp = night_temp_stored / 10.0;
    }

    int night_start_stored;
    ret = nvs_config.read("night_start", night_start_stored);
    if (ret == ESP_OK)
    {
        night_start_time = night_start_stored;
    }

    int night_end_stored;
    ret = nvs_config.read("night_end", night_end_stored);
    if (ret == ESP_OK)
    {
        night_end_time = night_end_stored;
    }

    start_update_cur_temp_timer(CONFIG_HEATSENS_TEMP_READ_INTERVAL_SHORT);
}

void TempModel::set_tgt_temp(double temp)
{
    tgt_temp_ = temp;

    Nvs nvs;
    nvs.open_namespace("heatsens");
    esp_err_t ret = nvs.write("tgt_temp", temp * 10.0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write target temp to NVS");
    }
}

void TempModel::set_night_tgt_temp(std::optional<double> temp)
{
    night_tgt_temp = temp;

    Nvs nvs;
    esp_err_t ret = nvs.open_namespace("meta");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open meta namespace: %s", esp_err_to_name(ret));
        return;
    }

    if (temp.has_value())
    {
        ret = nvs.write("night_tgt_temp", temp.value() * 10.0);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write night_tgt_temp to NVS: %s (error code: 0x%x)", esp_err_to_name(ret), ret);
        }
        else
        {
            ESP_LOGI(TAG, "Successfully wrote night_tgt_temp=%.1f to NVS", temp.value());
        }
    }
}

void TempModel::set_night_start_time(std::optional<int> start)
{
    night_start_time = start;

    Nvs nvs;
    esp_err_t ret = nvs.open_namespace("meta");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open meta namespace: %s", esp_err_to_name(ret));
        return;
    }

    if (start.has_value())
    {
        ret = nvs.write("night_start", start.value());
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write night_start to NVS: %s (error code: 0x%x)", esp_err_to_name(ret), ret);
        }
        else
        {
            ESP_LOGI(TAG, "Successfully wrote night_start=%d to NVS", start.value());
        }
    }
}

void TempModel::set_night_end_time(std::optional<int> end)
{
    night_end_time = end;

    Nvs nvs;
    esp_err_t ret = nvs.open_namespace("meta");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open meta namespace: %s", esp_err_to_name(ret));
        return;
    }

    if (end.has_value())
    {
        ret = nvs.write("night_end", end.value());
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write night_end to NVS: %s (error code: 0x%x)", esp_err_to_name(ret), ret);
        }
        else
        {
            ESP_LOGI(TAG, "Successfully wrote night_end=%d to NVS", end.value());
        }
    }
}

void TempModel::set_device_meta(std::string name, int heat_actuator, std::optional<double> night_temp, std::optional<int> night_start, std::optional<int> night_end)
{
    device_name_ = name;
    heat_actuator_ = heat_actuator;
    night_tgt_temp = night_temp;
    night_start_time = night_start;
    night_end_time = night_end;

    Nvs nvs;
    nvs.open_namespace("meta");
    esp_err_t ret = nvs.write("device_name", name);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write device name to NVS");
    }
    ret = nvs.write("heat_actuator", heat_actuator);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write heat actuator to NVS");
    }

    // Write night mode parameters to NVS
    if (night_temp.has_value())
    {
        ret = nvs.write("night_tgt_temp", night_temp.value() * 10.0);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write night_tgt_temp to NVS");
        }
    }
    if (night_start.has_value())
    {
        ret = nvs.write("night_start", night_start.value());
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write night_start to NVS");
        }
    }
    if (night_end.has_value())
    {
        ret = nvs.write("night_end", night_end.value());
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write night_end to NVS");
        }
    }
}

void TempModel::toggle_is_heating()
{
    double histeresis = CONFIG_HEATSENS_HISTERESIS / 100.0;

    double tgt_temp = get_tgt_temp();
    double temp_diff = tgt_temp - cur_temp_;
    bool should_turn_on_heater = temp_diff > histeresis && !is_heating_requested_;
    if (should_turn_on_heater)
    {
        ESP_LOGI(TAG, "Heating ON, tgt_temp=%.1f, cur_temp=%.1f, histeresis=%.1f", tgt_temp, cur_temp_, histeresis);
        logger.info(TAG, "Heating ON, tgt_temp=%.1f, cur_temp=%.1f, histeresis=%.1f", tgt_temp, cur_temp_, histeresis);
        is_heating_requested_ = true;
    }

    bool should_turn_off_heater = -temp_diff > histeresis && is_heating_requested_;
    if (should_turn_off_heater)
    {
        ESP_LOGI(TAG, "Heating OFF, tgt_temp=%.1f, cur_temp=%.1f, histeresis=%.1f", tgt_temp, cur_temp_, histeresis);
        logger.info(TAG, "Heating OFF, tgt_temp=%.1f, cur_temp=%.1f, histeresis=%.1f", tgt_temp, cur_temp_, histeresis);
        is_heating_requested_ = false;
    }
}

bool TempModel::get_is_heating_requested()
{
    return is_heating_requested_;
}

double TempModel::get_tgt_temp() const
{
    // Check if all night variables are set
    if (!night_tgt_temp.has_value() || !night_start_time.has_value() || !night_end_time.has_value())
    {
        return tgt_temp_;
    }

    // Get current local time
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    int current_hour = timeinfo.tm_hour;

    // Check if we're in night mode
    int start = night_start_time.value();
    int end = night_end_time.value();
    bool in_night_mode = false;

    if (start == end)
    {
        // If start == end, night mode is disabled
        in_night_mode = false;
    }
    else if (start < end)
    {
        // Night period doesn't cross midnight (e.g., start=1, end=5 means 01:00-05:00)
        in_night_mode = (current_hour >= start && current_hour < end);
    }
    else
    {
        // Night period crosses midnight (e.g., start=22, end=6 means 22:00-06:00)
        in_night_mode = (current_hour >= start || current_hour < end);
    }

    return in_night_mode ? night_tgt_temp.value() : tgt_temp_;
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
std::string TempModel::to_json()
{
    cJSON *doc = cJSON_CreateObject();
    cJSON_AddStringToObject(doc, "ts", get_esp_localtime().c_str());
    cJSON_AddNumberToObject(doc, "cur_temp", cur_temp_);
    cJSON_AddNumberToObject(doc, "tgt_temp", get_tgt_temp());
    cJSON_AddBoolToObject(doc, "is_heating", get_is_heating_requested());
    cJSON_AddStringToObject(doc, "device_name", device_name_.c_str());
    cJSON_AddNumberToObject(doc, "heat_actuator", heat_actuator_);

    // Add night mode variables if they are set
    if (night_tgt_temp.has_value())
    {
        cJSON_AddNumberToObject(doc, "night_tgt_temp", night_tgt_temp.value());
    }
    if (night_start_time.has_value())
    {
        cJSON_AddNumberToObject(doc, "night_start_time", night_start_time.value());
    }
    if (night_end_time.has_value())
    {
        cJSON_AddNumberToObject(doc, "night_end_time", night_end_time.value());
    }

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
            ESP_LOGE(TAG, "Failed to read raw sensor data");
            return;
        }

        bmp280.compensate_temp_press(raw_temp, raw_press, &temperature, &pressure);

        auto &temp_model = TempModel::getInstance();
        double tgt_temp;
        {
            std::lock_guard<std::mutex> lock_temp_model(temp_model.getMutex());
            temp_model.set_cur_temp(temperature);
            temp_model.toggle_is_heating();
            tgt_temp = temp_model.get_tgt_temp();
        }

        // ESP_LOGI(TAG, "Current Temp: %.2f°C, Target Temp: %.1f, Pressure: %.2f Pa", temperature, tgt_temp, pressure);
        temp_model.logger.info(TAG, "Current Temp: %.2f°C, Target Temp: %.1f, Pressure: %.2f Pa", temperature, tgt_temp, pressure);
    }
    else
    {
        ESP_LOGE(TAG, "BMP280 sensor not initialized");
    }
}

void TempModel::update_cur_temp_timer_interval(int32_t new_interval_seconds)
{
    ESP_LOGI(TAG, "Setting update_cur_temp_timer interval to %d secs", new_interval_seconds);
    stop_update_cur_temp_timer();
    start_update_cur_temp_timer(new_interval_seconds);
}