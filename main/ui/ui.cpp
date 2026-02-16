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
    if (!lbl_heating)
        return;

    if (is_show_heating)
    {
        lv_obj_set_style_bg_color(lbl_heating, lv_color_hex(0xe83030), LV_PART_MAIN);
        lv_obj_set_style_shadow_color(lbl_heating, lv_color_hex(0xe83030), LV_PART_MAIN);
        lv_obj_set_style_shadow_width(lbl_heating, 10, LV_PART_MAIN);
        lv_obj_set_style_shadow_spread(lbl_heating, 3, LV_PART_MAIN);
        lv_obj_set_style_shadow_opa(lbl_heating, LV_OPA_70, LV_PART_MAIN);
    }
    else
    {
        lv_obj_set_style_bg_color(lbl_heating, lv_color_hex(0x1a1a1a), LV_PART_MAIN);
        lv_obj_set_style_shadow_width(lbl_heating, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_opa(lbl_heating, LV_OPA_TRANSP, LV_PART_MAIN);
    }
}

void Ui::show_heating_requested(bool is_show)
{
    if (!lbl_heating_requested)
        return;

    if (is_show)
    {
        lv_obj_set_style_bg_color(lbl_heating_requested, lv_color_hex(0xdaa520), LV_PART_MAIN);
        lv_obj_set_style_shadow_color(lbl_heating_requested, lv_color_hex(0xdaa520), LV_PART_MAIN);
        lv_obj_set_style_shadow_width(lbl_heating_requested, 8, LV_PART_MAIN);
        lv_obj_set_style_shadow_spread(lbl_heating_requested, 3, LV_PART_MAIN);
        lv_obj_set_style_shadow_opa(lbl_heating_requested, LV_OPA_70, LV_PART_MAIN);
    }
    else
    {
        lv_obj_set_style_bg_color(lbl_heating_requested, lv_color_hex(0x1a1a1a), LV_PART_MAIN);
        lv_obj_set_style_shadow_width(lbl_heating_requested, 0, LV_PART_MAIN);
        lv_obj_set_style_shadow_opa(lbl_heating_requested, LV_OPA_TRANSP, LV_PART_MAIN);
    }
}

void Ui::main_view()
{
    main_view_ = lv_screen_active();
    lv_obj_clean(main_view_);

    // Reset all object pointers since lv_obj_clean deleted them
    lbl_heating = nullptr;
    lbl_heating_requested = nullptr;
    lbl_tgt_arrow = nullptr;
    cur_temp_ = nullptr;
    tgt_temp_ = nullptr;
    label_meta = nullptr;

    lv_obj_set_style_bg_color(main_view_, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_pad_all(main_view_, 0, LV_PART_MAIN);

    // ── Left edge glow bars ──
    // Heating requested indicator (amber glow when active)
    lbl_heating_requested = lv_obj_create(main_view_);
    lv_obj_remove_style_all(lbl_heating_requested);
    lv_obj_set_size(lbl_heating_requested, 4, 28);
    lv_obj_set_style_radius(lbl_heating_requested, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lbl_heating_requested, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(lbl_heating_requested, lv_color_hex(0x1a1a1a), LV_PART_MAIN);
    lv_obj_set_pos(lbl_heating_requested, 6, 43);

    // Actuator open / heating indicator (red glow when active)
    lbl_heating = lv_obj_create(main_view_);
    lv_obj_remove_style_all(lbl_heating);
    lv_obj_set_size(lbl_heating, 4, 28);
    lv_obj_set_style_radius(lbl_heating, 2, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lbl_heating, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(lbl_heating, lv_color_hex(0x1a1a1a), LV_PART_MAIN);
    lv_obj_set_pos(lbl_heating, 6, 77);

    // ── Current temperature (large, white, centered) ──
    cur_temp_ = lv_label_create(main_view_);
    lv_label_set_text(cur_temp_, "--.-");
    lv_obj_set_style_text_font(cur_temp_, &Rubik_Medium_48, LV_PART_MAIN);
    lv_obj_set_style_text_color(cur_temp_, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(cur_temp_, LV_ALIGN_CENTER, 10, -28);

    // ── Target temperature (smaller, teal, with arrow prefix) ──
    tgt_temp_ = lv_label_create(main_view_);
    lv_label_set_text(tgt_temp_, "--.-");
    lv_obj_set_style_text_font(tgt_temp_, &Rubik_Regular_36, LV_PART_MAIN);
    lv_obj_set_style_text_color(tgt_temp_, lv_color_hex(0x4ecdc4), LV_PART_MAIN);
    lv_obj_align(tgt_temp_, LV_ALIGN_CENTER, 10, 22);

    // Arrow prefix for target temp
    lbl_tgt_arrow = lv_label_create(main_view_);
    lv_label_set_text(lbl_tgt_arrow, "\xEF\x81\xA8\xEF\x81\xA8\xEF\x81\x94");
    lv_obj_set_style_text_font(lbl_tgt_arrow, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_tgt_arrow, lv_color_hex(0x669f99), LV_PART_MAIN);
    lv_obj_align_to(lbl_tgt_arrow, tgt_temp_, LV_ALIGN_OUT_LEFT_MID, -4, 0);

    // ── Meta info (bottom right) ──
    label_meta = lv_label_create(main_view_);
    lv_label_set_text_fmt(label_meta, "v%s %s-%s-%d",
                          CONFIG_APP_PROJECT_VER,
                          CONFIG_HEATSENS_DEVICE_ID,
                          "room",
                          0);
    lv_obj_set_style_text_font(label_meta, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(label_meta, lv_color_hex(0xaaaaaa), LV_PART_MAIN);
    lv_obj_align(label_meta, LV_ALIGN_BOTTOM_RIGHT, -8, -4);

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
    auto &ui = Ui::getInstance();
    if (ui.is_provisioning())
    {
        return;
    }

    auto &mqtt = Mqtt::getInstance();
    std::lock_guard<std::mutex> lock_mqtt(mqtt.getMutex());
    if (mqtt.get_connect_return_code() != MQTT_CONNECTION_ACCEPTED)
    {
        std::string init_error = std::string("MQTT Error\n") + mqtt.get_is_mqtt_broker_url();
        if (lvgl_port_lock(0))
        {
            ui.error_screen(init_error);
            lvgl_port_unlock();
            return;
        }
    }

    auto &temp_model = TempModel::getInstance();
    double cur_temp, tgt_temp;
    std::string device_name;
    int heat_actuator;
    bool is_heating, is_heating_requested;
    {
        std::lock_guard<std::mutex> lock_model(temp_model.getMutex());
        cur_temp = temp_model.get_cur_temp();
        tgt_temp = temp_model.get_tgt_temp();
        device_name = temp_model.get_device_name();
        heat_actuator = temp_model.get_heat_actuator();
        is_heating = temp_model.get_is_heating();
        is_heating_requested = temp_model.get_is_heating_requested();
    }

    if (lvgl_port_lock(0))
    {
        auto &ui = Ui::getInstance();
        ui.set_meta(device_name, heat_actuator);
        ui.set_cur_temp(cur_temp);
        ui.set_tgt_temp(tgt_temp);
        ui.show_heating(is_heating);
        ui.show_heating_requested(is_heating_requested);
        lvgl_port_unlock();
    }
}

void Ui::set_cur_temp(double val)
{
    char temp_str[32];
    snprintf(temp_str, sizeof(temp_str), "%.1f°", val);
    lv_label_set_text_fmt(cur_temp_, "%s", temp_str);
}

void Ui::set_tgt_temp(double val)
{
    char temp_str[32];
    snprintf(temp_str, sizeof(temp_str), "%.1f°", val);
    lv_label_set_text_fmt(tgt_temp_, "%s", temp_str);
    // Re-align arrow to track label width changes
    if (lbl_tgt_arrow)
    {
        lv_obj_align_to(lbl_tgt_arrow, tgt_temp_, LV_ALIGN_OUT_LEFT_MID, -4, 0);
    }
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
        lcd_set_brightness_pct_fade(0, 500);
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
    {
        std::lock_guard<std::mutex> lock_ui(ui.getMutex());
        ui.dim_display(LcdState::Off);
    }

    auto &model = TempModel::getInstance();
    std::lock_guard<std::mutex> lock_ui(model.getMutex());
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

void Ui::provisioning_screen(const std::string &ap_ssid)
{
    is_provisioning_ = true;
    lv_obj_t *scr = lv_screen_active();
    lv_obj_clean(scr);
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr, 10, LV_PART_MAIN);

    // Title
    lv_obj_t *lbl_title = lv_label_create(scr);
    lv_label_set_text(lbl_title, "Provisioning Mode");
    lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_title, lv_palette_main(LV_PALETTE_AMBER), LV_PART_MAIN);
    lv_obj_align(lbl_title, LV_ALIGN_TOP_MID, 0, 10);

    // Instructions
    lv_obj_t *lbl_instructions = lv_label_create(scr);
    lv_label_set_text(lbl_instructions, "Connect to WiFi:");
    lv_obj_set_style_text_font(lbl_instructions, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_instructions, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(lbl_instructions, LV_ALIGN_CENTER, 0, -20);

    // AP SSID
    lv_obj_t *lbl_ssid = lv_label_create(scr);
    lv_label_set_text(lbl_ssid, ap_ssid.c_str());
    lv_obj_set_style_text_font(lbl_ssid, &lv_font_montserrat_24, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_ssid, lv_palette_main(LV_PALETTE_LIGHT_GREEN), LV_PART_MAIN);
    lv_obj_align(lbl_ssid, LV_ALIGN_CENTER, 0, 10);

    // URL hint
    lv_obj_t *lbl_url = lv_label_create(scr);
    lv_label_set_text(lbl_url, "Then open: http://192.168.4.1");
    lv_obj_set_style_text_font(lbl_url, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_url, lv_palette_lighten(LV_PALETTE_GREY, 2), LV_PART_MAIN);
    lv_obj_align(lbl_url, LV_ALIGN_BOTTOM_MID, 0, -10);
}