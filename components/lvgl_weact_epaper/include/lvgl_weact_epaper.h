#ifndef LVGL_WEACT_EPAPER_H
#define LVGL_WEACT_EPAPER_H

#include "lvgl.h"
#include "weact_epaper_2in13.h"

/**
 * @brief Configuration for LVGL WeAct E-Paper display
 */
typedef struct {
    gpio_num_t pin_sck;      // SPI Clock
    gpio_num_t pin_mosi;     // SPI MOSI
    gpio_num_t pin_cs;       // Chip Select
    gpio_num_t pin_dc;       // Data/Command
    gpio_num_t pin_rst;      // Reset
    gpio_num_t pin_busy;     // Busy signal
    int spi_clock_speed_hz;  // SPI clock speed (default: 4MHz)
    bool landscape;          // true = landscape (250x122), false = portrait (122x250)
} lvgl_weact_epaper_config_t;

/**
 * @brief Initialize LVGL with WeAct 2.13" E-Paper display
 *
 * This function:
 * 1. Initializes the low-level display driver
 * 2. Creates LVGL display driver
 * 3. Registers flush and wait callbacks
 * 4. Returns the LVGL display object
 *
 * @param config Display pin configuration
 * @return lv_disp_t* Pointer to LVGL display object, or NULL on failure
 */
lv_disp_t *lvgl_weact_epaper_create(const lvgl_weact_epaper_config_t *config);

/**
 * @brief Get default pin configuration
 *
 * Returns the standard pin configuration for WeAct 2.13" display
 *
 * @return lvgl_weact_epaper_config_t Default configuration
 */
lvgl_weact_epaper_config_t lvgl_weact_epaper_get_default_config(void);

#endif // LVGL_WEACT_EPAPER_H
