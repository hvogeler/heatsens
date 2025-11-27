#pragma once
#include <nvs.h>
#include <string>

class Nvs
{
private:
    nvs_handle_t handle_;
    bool is_initialized_;

    Nvs(const Nvs &) = delete;
    Nvs &operator=(const Nvs &) = delete;
    Nvs(Nvs &&) = delete;
    Nvs &operator=(Nvs &&) = delete;

public:
    Nvs() : handle_(-1), is_initialized_(false) {}
    ~Nvs()
    {
        if (is_initialized_ && handle_ != -1)
        {
            nvs_close(handle_);
        }
    };

    esp_err_t open_namespace(std::string);
    esp_err_t write(std::string, double tgt_temp);
    esp_err_t read(std::string, double &);
    esp_err_t read(std::string, std::string &);
};