#include "nvs.hpp"

esp_err_t Nvs::open_namespace(std::string nvs_ns)
{
    esp_err_t ret = nvs_open(nvs_ns.c_str(), NVS_READWRITE, &handle_);
    if (ret != ESP_OK)
    {
        handle_ = -1;
        is_initialized_ = false;
        return ret;
    }
    is_initialized_ = ret == ESP_OK;
    return ret;
}

esp_err_t Nvs::write(std::string key, double v)
{
    return nvs_set_i32(handle_, key.c_str(), v);
}

esp_err_t Nvs::read(std::string key, double &v)
{
    int32_t value;
    esp_err_t ret = nvs_get_i32(handle_, key.c_str(), &value);
    if (ret == ESP_OK)
    {
        v = value * 1.0;
        return ret;
    }
    else
    {
        v = 19.0;
        return ret;
    }
}

esp_err_t Nvs::read(std::string key, std::string &v)
{
    char buf[65];
    size_t len = sizeof(buf);
    esp_err_t ret = nvs_get_str(handle_, key.c_str(), buf, &len);
    if (ret == ESP_OK)
    {
        v = std::string(buf, len);
        return ret;
    }
    else
    {
        v = "";
        return ret;
    }
}