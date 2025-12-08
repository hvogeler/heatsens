#include "button_gpio.h"
#include "iot_button.h"
#include "esp_log.h"

class Button
{
private:
    button_handle_t handle;
    button_gpio_config_t gpio_config;
    button_config_t config;
    static constexpr char *TAG = "Button";

public:
    Button(int32_t gpio, int32_t active_level, button_event_t event_type, button_cb_t callback)
    {
        gpio_config = {};
        gpio_config.active_level = active_level;
        gpio_config.gpio_num = gpio;
        gpio_config.enable_power_save = true;

        config = {};
        esp_err_t err = iot_button_new_gpio_device(&config, &gpio_config, &handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Button initialization failed");
        }
        err = iot_button_register_cb(handle, event_type, NULL, callback, this);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Button register callback failed");
        }
    }

    esp_err_t register_callback(button_event_t event_type, button_cb_t callback)
    {
        esp_err_t err = iot_button_register_cb(handle, event_type, NULL, callback, this);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Button register callback failed");
        }
        return err;
    }

    button_event_t get_event()
    {
        return iot_button_get_event(handle);
    }

    void print_event()
    {
        iot_button_print_event(handle);
    }
};