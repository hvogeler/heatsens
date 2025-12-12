#pragma once
#include "lvgl.h"
#include <string>
#include "esp_timer_cxx.hpp"

enum class LcdState
{
    Off,
    On,
};

/**
 * @brief Singleton class managing the UI using LVGL.
 *
 * This class implements the Meyers' Singleton pattern with thread-safe initialization.
 * It manages the display fields and update task for the door buzzer UI.
 *
 * @note Thread-safe singleton instance creation guaranteed by C++11.
 */
class Ui
{
private:
    // UI components
    lv_obj_t *main_view_;
    lv_obj_t *cur_temp_;
    lv_obj_t *tgt_temp_;
    lv_obj_t *lbl_heating;
    lv_obj_t *lbl_heating_requested;
    lv_obj_t *label_meta;
    std::string ssid;

    bool is_ssid_set;
    LcdState lcd_state;
    idf::esp_timer::ESPTimer dim_on_timer;

private:
    static void update_ui();
    static void update_task(void *pvParam);
    static void dim_on_timer_cb();
    Ui()
        : main_view_(nullptr), cur_temp_(nullptr), tgt_temp_(nullptr), lbl_heating(nullptr), ssid(""), is_ssid_set(false),
          lcd_state(LcdState::On), dim_on_timer(Ui::dim_on_timer_cb)
    {
    }
    ~Ui() {}

public:
    Ui(const Ui &) = delete;
    Ui &operator=(const Ui &) = delete;

    /**
     * @brief Get the singleton instance of Ui.
     * @return Reference to the singleton Ui instance.
     */
    static Ui &getInstance()
    {
        static Ui instance;
        return instance;
    }

    void splash_screen();
    void main_view();
    void error_screen(std::string);
    void show_heating(bool);
    void show_heating_requested(bool);
    void set_cur_temp(double val);
    void set_tgt_temp(double val);
    void set_meta(std::string, int);
    void dim_display(LcdState to_state);
    void start_dim_on_timer(int32_t);
    void set_lcd_state(LcdState v)
    {
        lcd_state = v;
    }

    LcdState get_lcd_state()
    {
        return lcd_state;
    }
};