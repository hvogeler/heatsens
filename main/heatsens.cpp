#include <stdio.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "t_display_s3.h"
#include "ui/ui.hpp"
#include "temp_model.hpp"
#include "esp_timer_cxx.hpp"
#include "wifi.hpp"
#include "mqtt.hpp"
#include "i2c.hpp"
#include "button.hpp"
#include "mpu6050.hpp"
#include <time.h>
#include <mutex>
#include <sys/time.h>
#include "mqtt_logger.hpp"

#define TAG "heatsens"

// TODO: Configure the publish curtemp interval
// TODO: check all lock guards for possible deadlocks
static void button_click_common(uint64_t lcd_on_seconds)
{
    auto &ui = Ui::getInstance();
    auto &model = TempModel::getInstance();

    bool should_turn_off = false;
    {
        std::lock_guard<std::mutex> lock_ui(ui.getMutex());
        if (ui.get_lcd_state() == LcdState::On)
        {
            ui.dim_display(LcdState::Off);
            should_turn_off = true;
        }
        else
        {
            ui.dim_display(LcdState::On);
            ui.start_dim_on_timer(lcd_on_seconds);
        }
    }

    std::lock_guard<std::mutex> lock_model(model.getMutex());
    if (should_turn_off)
    {
        model.update_cur_temp_timer_interval(CONFIG_HEATSENS_TEMP_READ_INTERVAL_LONG);
    }
    else
    {
        model.update_cur_temp_timer_interval(CONFIG_HEATSENS_TEMP_READ_INTERVAL_SHORT);
    }
}

static void wake_up_button_cb(void *arg, void *usr_data)
{
    Button *btn = static_cast<Button *>(usr_data);
    button_event_t event = btn->get_event();
    switch (event)
    {
    case BUTTON_SINGLE_CLICK:
        button_click_common(CONFIG_HEATSENS_LCD_ON_INTERVAL_SHORT);
        break;
    case BUTTON_DOUBLE_CLICK:
    case BUTTON_LONG_PRESS_UP:
        button_click_common(CONFIG_HEATSENS_LCD_ON_INTERVAL_LONG);
        break;
    default:;
    }
    btn->print_event();
}

static void check_motion_cb()
{
    static char *local_tag = "check_motion_cb";
    auto &motion_sensor = Mpu6050::getInstance();

    // Use software-based rotation detection
    mpu6050_data data;
    esp_err_t err = motion_sensor.read_scaled(&data);
    if (err == ESP_OK)
    {
        // For gentle rotation detection, use lower threshold
        // 0.08g ≈ 5° rotation, 0.15g ≈ 10° rotation
        bool rotated = motion_sensor.is_rotated(&data, 0.08f); // Very sensitive
        if (rotated)
        {
            ESP_LOGW(local_tag, "Device rotated! Turning on LCD...");
            ESP_LOGI(local_tag, "Accel: %.3f, %.3f, %.3f g",
                     data.accel_x, data.accel_y, data.accel_z);

            // Turn on LCD when rotation detected
            // button_click_common(CONFIG_HEATSENS_LCD_ON_INTERVAL_SHORT);
            auto &ui = Ui::getInstance();
            std::lock_guard<std::mutex> lock_ui(ui.getMutex());
            ui.dim_display(LcdState::On);
            ui.start_dim_on_timer(CONFIG_HEATSENS_LCD_ON_INTERVAL_SHORT);
        }
        else
        {
            ESP_LOGD(local_tag, "Stationary. Accel: %.3f, %.3f, %.3f g",
                     data.accel_x, data.accel_y, data.accel_z);
        }
    }
}

extern "C" void app_main(void)
{
    I2c::getInstance().init();

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
    esp_log_level_set("Mpu6050", ESP_LOG_DEBUG);
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
    ret = wifi.wifi_connect();
    if (ret != ESP_OK)
    {
        init_error = std::string("Wifi Error\n") + wifi.get_wifi_ssid();
    }

    auto &mqtt = Mqtt::getInstance();
    auto &temp_model = TempModel::getInstance();
    Button wake_up_button(GPIO_NUM_14, 0, BUTTON_SINGLE_CLICK, wake_up_button_cb);

    if (init_error.empty())
    {
        ret = wifi.time_sync();
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Could not sync time with time server");
        }

        mqtt.connect();

        // Initialize BMP280 sensor (it will set up I2C internally)
        ESP_LOGI(TAG, "Initializing BMP280 sensor...");
        temp_model.init();
        ESP_LOGI(TAG, "Real Time is %s", temp_model.get_esp_localtime().c_str());

        lvgl_port_lock(0);
        ui.main_view();
        ui.set_meta(temp_model.get_device_name(), temp_model.get_heat_actuator());
        lvgl_port_unlock();
        ui.start_dim_on_timer(CONFIG_HEATSENS_LCD_ON_INTERVAL_LONG);
        wake_up_button.register_callback(BUTTON_DOUBLE_CLICK, wake_up_button_cb);
        wake_up_button.register_callback(BUTTON_LONG_PRESS_UP, wake_up_button_cb);
    }
    else
    {
        lvgl_port_lock(0);
        ui.error_screen(init_error);
        lvgl_port_unlock();
    }

    auto &motion_sensor = Mpu6050::getInstance();
    idf::esp_timer::ESPTimer check_motion_timer(check_motion_cb, "check_motion_timer");
    esp_err_t err = motion_sensor.init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Initializing the motion sensor failed");
    }
    else
    {
        ESP_LOGI(TAG, "MPU6050 initialized successfully");

        // Test: Read initial sensor data to verify it's working
        mpu6050_data test_data;
        err = motion_sensor.read_scaled(&test_data);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Initial MPU6050 reading - Accel: %.2f, %.2f, %.2f g | Temp: %.1f°C",
                     test_data.accel_x, test_data.accel_y, test_data.accel_z, test_data.temp);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read initial MPU6050 data");
        }

        // Setup motion detection with higher threshold for rotation detection
        // For 30° rotation, we expect ~0.5g change, so use 0.2g (100mg) threshold
        // threshold=100 means 200mg, duration=50ms to debounce
        err = motion_sensor.setup_motion_detection(50, 50);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to setup motion detection");
        }
        check_motion_timer.start_periodic(std::chrono::milliseconds(300));
    }

    bool is_first_iter = true;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Check every 1 second

        if (init_error.empty())
        {
            std::string json_data;
            {
                std::lock_guard<std::mutex> lock_temp_model(temp_model.getMutex());
                json_data = temp_model.toJson();
            }
            {
                std::lock_guard<std::mutex> lock_mqtt(mqtt.getMutex());
                mqtt.publish(json_data);
            }
        }

        // heap_trace_dump();
        ESP_LOGD(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
        // stack size dump
        BaseType_t remaining_stack = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGD(TAG, "Free stack: %lu bytes", remaining_stack * sizeof(StackType_t));
    }
}
