#include <stdio.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "t_display_s3.h"
#include "ui/ui.hpp"
#include "temp_model.hpp"
#include "esp_timer_cxx.hpp"
#include "wifi.hpp"
#include "mqtt.hpp"
#include <time.h>
#include <sys/time.h>
#include "mqtt_logger.hpp"

#define TAG "heatsens"

// TODO: set to lightsleep between measurments. wake on interval.
// TODO: Turn display off in light sleep. Wake on button press.


extern "C" void app_main(void)
{
    // Initialize NVS. It will be used in various parts of the firmware
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("hts_mqtt", ESP_LOG_VERBOSE);
    esp_log_level_set("heatsens", ESP_LOG_VERBOSE);
    std::string init_error = "";

    // LVGL display handle
    static lv_display_t *disp_handle;

    // initialize the LCD
    // don't turn on backlight yet - demo of gradual brightness increase is shown below
    // otherwise you can set it to true to turn on the backlight at lcd init
    lcd_init(&disp_handle, false);

    lvgl_port_lock(0);
    auto &ui = Ui::getInstance();
    ui.splash_screen();
    lvgl_port_unlock();
    // lcd_set_brightness_step(31);
    lcd_set_brightness_pct_fade(100, 1000);

    auto &wifi = Wifi::getInstance();

    {
        // std::lock_guard<mutex> lock_wifi(wifi.getMutex());
        ret = wifi.wifi_connect();
        if (ret != ESP_OK)
        {
            init_error = std::string("Wifi Error\n") + wifi.get_wifi_ssid();
        }
    }

    ret = wifi.time_sync();
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Could not sync time with time server");
    }

    auto &mqtt = Mqtt::getInstance();
    mqtt.connect();

    // Initialize BMP280 sensor (it will set up I2C internally)
    ESP_LOGI(TAG, "Initializing BMP280 sensor...");
    auto &temp_model = TempModel::getInstance();
    temp_model.init();
    ESP_LOGI(TAG, "Real Time is %s", temp_model.get_esp_localtime().c_str());

    idf::esp_timer::ESPTimer update_hw_info_timer(TempModel::update_cur_temp_cb);
    if (init_error.empty())
    {
        lvgl_port_lock(0);
        ui.main_view();
        ui.set_ssid(wifi.get_wifi_ssid());
        lvgl_port_unlock();

        update_hw_info_timer.start_periodic(std::chrono::seconds(CONFIG_HEATSENS_SENSOR_READ_INTERVAL));
    }
    else
    {
        lvgl_port_lock(0);
        ui.error_screen(init_error);
        lvgl_port_unlock();
    }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (init_error.empty())
        {
            std::lock_guard<std::mutex> lock_mqtt(mqtt.getMutex());
            std::lock_guard<std::mutex> lock_temp_model(temp_model.getMutex());
            mqtt.publish(temp_model.toJson());
        }
        // heap_trace_dump();
        ESP_LOGD(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
        // stack size dump
        BaseType_t remaining_stack = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGD(TAG, "Free stack: %lu bytes", remaining_stack * sizeof(StackType_t));
    }
}
