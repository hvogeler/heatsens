#include "ui.hpp"
#include "field.hpp"
#include "freertos/FreeRTOS.h"
#include "esp_lvgl_port.h"
#include "sdkconfig.h"
#include "../temp_model.hpp"
#include "../mqtt.hpp"
#include "esp_log.h"
#include "t_display_s3.h"

static const char *TAG = "ui";

LV_FONT_DECLARE(Rubik_Medium_48)
LV_FONT_DECLARE(Rubik_Regular_36)

void Ui::splash_screen()
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_clean(scr);
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr, 5, LV_PART_MAIN);

    lv_obj_t *lbl_splash = lv_label_create(scr);
    lv_label_set_text(lbl_splash, "Heat Sensor\nInitializing...");
    // lv_obj_set_width(lbl_splash, lv_pct(100));
    lv_obj_set_style_text_font(lbl_splash, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_splash, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_align(lbl_splash, LV_ALIGN_CENTER);
    // lv_obj_set_pos(lbl_splash, 0, 0);
}

void Ui::error_screen(std::string msg)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_clean(scr);
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr, 5, LV_PART_MAIN);
    lv_obj_invalidate(scr);

    // lv_obj_t *lbl_error = lv_label_create(scr);
    // lv_label_set_text(lbl_error, "Error");
    // lv_obj_set_width(lbl_error, lv_pct(100));
    // lv_obj_set_style_text_font(lbl_error, &lv_font_montserrat_24, LV_PART_MAIN);
    // lv_obj_set_style_text_color(lbl_error, lv_palette_main(LV_PALETTE_DEEP_ORANGE), LV_PART_MAIN);
    // lv_obj_set_align(lbl_error, LV_ALIGN_TOP_LEFT);

    lv_obj_t *lbl_msg = lv_label_create(scr);
    lv_label_set_text(lbl_msg, msg.c_str());
    lv_obj_set_width(lbl_msg, lv_pct(100));
    lv_obj_set_style_text_font(lbl_msg, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_msg, lv_palette_main(LV_PALETTE_YELLOW), LV_PART_MAIN);
    lv_obj_set_align(lbl_msg, LV_ALIGN_CENTER);
    // lv_obj_align_to(lbl_msg, lbl_error, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);

    // lv_obj_set_pos(lbl_splash, 0, 0);
}

void Ui::show_heating(bool is_show_heating)
{
    main_view_ = lv_screen_active();

    // Create label only once on first call
    if (!lbl_heating)
    {
        lbl_heating = lv_label_create(main_view_);
        lv_label_set_text(lbl_heating, "Heating");
        lv_obj_set_style_pad_right(lbl_heating, 2, LV_PART_MAIN);
        lv_obj_set_style_pad_top(lbl_heating, 5, LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl_heating, &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_set_align(lbl_heating, LV_ALIGN_TOP_RIGHT);
    }

    // Only update the color, not recreate the object
    lv_obj_set_style_text_color(lbl_heating, is_show_heating ? lv_palette_main(LV_PALETTE_DEEP_ORANGE) : lv_palette_darken(LV_PALETTE_GREY, 2), LV_PART_MAIN);
}

void Ui::show_heating_requested(bool is_show)
{
    main_view_ = lv_screen_active();

    // Create label only once on first call
    if (!lbl_heating_requested)
    {
        lbl_heating_requested = lv_label_create(main_view_);
        lv_label_set_text(lbl_heating_requested, "Heating Requested");
        lv_obj_set_style_pad_right(lbl_heating_requested, 2, LV_PART_MAIN);
        lv_obj_set_style_pad_top(lbl_heating_requested, 5, LV_PART_MAIN);
        lv_obj_set_style_text_font(lbl_heating_requested, &lv_font_montserrat_16, LV_PART_MAIN);
        lv_obj_align(lbl_heating_requested, LV_ALIGN_TOP_RIGHT, 0, 30);
    }

    // Only update the color, not recreate the object
    lv_obj_set_style_text_color(lbl_heating_requested, is_show ? lv_palette_lighten(LV_PALETTE_GREY, 3) : lv_palette_darken(LV_PALETTE_GREY, 2), LV_PART_MAIN);
}

void Ui::main_view()
{
    main_view_ = lv_screen_active();
    lv_obj_clean(main_view_);

    // Reset all object pointers since lv_obj_clean deleted them
    lbl_heating = nullptr;
    lbl_heating_requested = nullptr;
    cur_temp_ = nullptr;
    tgt_temp_ = nullptr;
    label_meta = nullptr;

    lv_obj_set_style_bg_color(main_view_, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_pad_all(main_view_, 5, LV_PART_MAIN);

    // Heating
    // lv_obj_t *lbl_heating = lv_label_create(main_view_);
    // lv_label_set_text(lbl_heating, "");

    // lv_obj_set_style_text_font(lbl_heating, &lv_font_montserrat_24, LV_PART_MAIN);
    // lv_obj_set_style_text_color(lbl_heating, lv_palette_main(LV_PALETTE_GREY), LV_PART_MAIN);
    // // lv_obj_set_pos(lbl_heating, 0, 0);
    // lv_obj_set_align(lbl_heating, LV_ALIGN_TOP_RIGHT);

    // Create current temp
    lv_obj_t *lbl_cur_temp = lv_label_create(main_view_);
    lv_label_set_text(lbl_cur_temp, "Current Temp °C");

    lv_obj_set_width(lbl_cur_temp, lv_pct(100));
    lv_obj_set_style_text_font(lbl_cur_temp, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_cur_temp, lv_palette_darken(LV_PALETTE_LIGHT_GREEN, 1), LV_PART_MAIN);
    lv_obj_set_pos(lbl_cur_temp, 0, 0);

    cur_temp_ = lv_label_create(main_view_);
    char temp_str[32];
    snprintf(temp_str, sizeof(temp_str), "%s", "--.-");
    lv_label_set_text_fmt(cur_temp_, "%s", temp_str);
    lv_obj_set_width(cur_temp_, lv_pct(100));
    lv_obj_set_style_text_font(cur_temp_, &Rubik_Medium_48, LV_PART_MAIN);
    lv_obj_set_style_text_color(cur_temp_, lv_palette_main(LV_PALETTE_LIGHT_GREEN), LV_PART_MAIN);

    lv_obj_align_to(cur_temp_, lbl_cur_temp, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 3);

    // Create target temp
    lv_obj_t *lbl_tgt_temp = lv_label_create(main_view_);
    lv_label_set_text(lbl_tgt_temp, "Target Temp °C");
    lv_obj_set_width(lbl_tgt_temp, lv_pct(100));
    lv_obj_set_style_text_font(lbl_tgt_temp, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_tgt_temp, lv_palette_darken(LV_PALETTE_LIGHT_GREEN, 1), LV_PART_MAIN);
    lv_obj_set_style_text_align(lbl_tgt_temp, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_pos(lbl_tgt_temp, 0, 0);

    lv_obj_align_to(lbl_tgt_temp, cur_temp_, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 2);

    tgt_temp_ = lv_label_create(main_view_);
    snprintf(temp_str, sizeof(temp_str), "%s", "--.-");
    lv_label_set_text_fmt(tgt_temp_, "%s", temp_str);
    lv_obj_set_width(tgt_temp_, lv_pct(100));
    lv_obj_set_style_text_font(tgt_temp_, &Rubik_Medium_48, LV_PART_MAIN);
    lv_obj_set_style_text_color(tgt_temp_, lv_palette_main(LV_PALETTE_LIGHT_GREEN), LV_PART_MAIN);
    lv_obj_set_style_text_align(tgt_temp_, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);

    lv_obj_align_to(tgt_temp_, lbl_tgt_temp, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 3);

    label_meta = lv_label_create(main_view_);
    lv_label_set_text_fmt(label_meta, "v%s %s-%s-%d",
                          CONFIG_APP_PROJECT_VER,
                          CONFIG_HEATSENS_DEVICE_ID,
                          "room",
                          0);
    lv_obj_set_style_text_font(label_meta, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_meta, lv_palette_lighten(LV_PALETTE_GREY, 3), LV_PART_MAIN);
    lv_obj_align(label_meta, LV_ALIGN_BOTTOM_LEFT, 0, -8);

    xTaskCreatePinnedToCore(update_task, "update_task", 4096 * 2, NULL, 0, NULL, 1);
}

void Ui::set_meta(std::string device_name, int heat_actuator)
{
    // if (!is_ssid_set && !ssid.empty())
    // {
    lv_label_set_text_fmt(label_meta, "v%s %s-%s-%d",
                          CONFIG_APP_PROJECT_VER,
                          CONFIG_HEATSENS_DEVICE_ID,
                          device_name.c_str(),
                          heat_actuator);
    is_ssid_set = true;
    // }
}

void Ui::update_ui()
{
    auto &mqtt = Mqtt::getInstance();
    std::lock_guard<std::mutex> lock_mqtt(mqtt.getMutex());
    if (mqtt.get_connect_return_code() != MQTT_CONNECTION_ACCEPTED)
    {
        std::string init_error = std::string("MQTT Error\n") + mqtt.get_is_mqtt_broker_url();
        if (lvgl_port_lock(0))
        {
            auto &ui = Ui::getInstance();
            ui.error_screen(init_error);
            lvgl_port_unlock();
            return;
        }
    }

    auto &temp_model = TempModel::getInstance();
    std::lock_guard<std::mutex> lock_model(temp_model.getMutex());
    auto cur_temp = temp_model.getCurTemp();
    auto tgt_temp = temp_model.getTgtTemp();

    if (lvgl_port_lock(0))
    {
        auto &ui = Ui::getInstance();
        ui.set_meta(temp_model.get_device_name(), temp_model.get_heat_actuator());
        ui.set_cur_temp(cur_temp);
        ui.set_tgt_temp(tgt_temp);
        ui.show_heating(temp_model.get_is_heating());
        ui.show_heating_requested(temp_model.get_is_heating_requested());
        lvgl_port_unlock();
    }
}

void Ui::set_cur_temp(double val)
{
    char temp_str[32];
    snprintf(temp_str, sizeof(temp_str), "%.1f", val);
    lv_label_set_text_fmt(cur_temp_, "%s", temp_str);
}

void Ui::set_tgt_temp(double val)
{
    char temp_str[32];
    snprintf(temp_str, sizeof(temp_str), "%.1f", val);
    lv_label_set_text_fmt(tgt_temp_, "%s", temp_str);
}

void Ui::update_task(void *pvParam)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        Ui::update_ui();
    }
}

void Ui::dim_display(LcdState to_state)
{
    switch (to_state)
    {
    case LcdState::Off:
        lcd_set_brightness_pct_fade(10, 500);
        lcd_state = LcdState::Off;
        break;
    case LcdState::On:
        lcd_set_brightness_pct_fade(100, 500);
        lcd_state = LcdState::On;
        break;
    }
}

void Ui::dim_on_timer_cb()
{
    auto &ui = Ui::getInstance();
    ui.dim_display(LcdState::Off);
    auto &model = TempModel::getInstance();
    model.update_cur_temp_timer_interval(CONFIG_HEATSENS_TEMP_READ_INTERVAL_LONG);
}

void Ui::start_dim_on_timer(int32_t seconds)
{
    try
    {
        dim_on_timer.stop();
    }
    catch (...)
    {
        // Timer wasn't running, that's fine
    }
    ESP_LOGI(TAG, "Setting Lcd On Timer to %d seconds", seconds);
    dim_on_timer.start(std::chrono::seconds(seconds));
}