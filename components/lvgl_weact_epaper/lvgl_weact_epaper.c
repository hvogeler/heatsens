/**
 * @file lvgl_weact_epaper.c
 * @brief LVGL 9 integration for WeAct Studio 2.13" E-Paper Display
 *
 * This component provides the LVGL 9 display driver interface for the
 * WeAct 2.13" e-paper display (122x250 pixels, SSD1680 controller).
 *
 * Compatible with LVGL 9.4.0 (ESP-IDF 5.5.1 managed component)
 *
 * Features:
 * - RGB to monochrome conversion (brightness threshold)
 * - Proper handling of e-paper refresh delays
 * - 16-byte aligned framebuffer management
 * - Full LVGL 9 display driver integration (lv_display_t)
 */

#include "lvgl_weact_epaper.h"
#include "weact_epaper_2in13.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "esp_timer.h"

static const char *TAG = "lvgl_weact_epaper";

/**
 * @brief Internal context structure
 *
 * Holds the low-level driver handle and LVGL display objects
 */
typedef struct lvgl_weact_epaper_ctx_t
{
    weact_epaper_t epaper; // Low-level driver handle
    lv_display_t *disp;    // LVGL 9 display object
    void *draw_buf1;       // LVGL draw buffer 1
    void *draw_buf2;       // LVGL draw buffer 2 (optional)
    bool landscape;        // Landscape orientation flag
} lvgl_weact_epaper_ctx_t;

// Static context (single display instance)
static lvgl_weact_epaper_ctx_t g_ctx;

/**
 * @brief Convert RGB color to monochrome
 *
 * Uses brightness threshold to determine black vs white.
 * Hardware expects: 1 = BLACK, 0 = WHITE
 * LVGL colors: (0,0,0) = black, (255,255,255) = white
 *
 * @param color LVGL RGB color
 * @return 1 for black, 0 for white (hardware format)
 */
static inline uint8_t rgb_to_mono(lv_color_t color)
{
    // Extract RGB components (LVGL 9 uses lv_color32_t internally)
    // Get brightness from color
    uint8_t r = color.red;
    uint8_t g = color.green;
    uint8_t b = color.blue;

    // Calculate brightness (weighted average for human perception)
    uint16_t brightness = (r * 30 + g * 59 + b * 11) / 100;

    // Threshold: < 128 is black (1), >= 128 is white (0)
    return (brightness < 128) ? 1 : 0;
}

/**
 * @brief LVGL 9 flush callback
 *
 * Called by LVGL when it needs to update the display.
 * Converts LVGL's RGB framebuffer to monochrome and updates the e-paper.
 *
 * In LVGL 9, the signature changed:
 * - Parameter 1: lv_display_t * (not lv_disp_drv_t *)
 * - Parameter 3: uint8_t * px_map (raw pixel data, not lv_color_t *)
 *
 * @param disp LVGL display object
 * @param area Screen area to update
 * @param px_map Raw pixel buffer
 */
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    lvgl_weact_epaper_ctx_t *ctx = (lvgl_weact_epaper_ctx_t *)lv_display_get_user_data(disp);

    ESP_LOGI(TAG, "Flush: x=%ld..%ld, y=%ld..%ld",
             (long)area->x1, (long)area->x2, (long)area->y1, (long)area->y2);

    // In LVGL 9, px_map is uint8_t* raw pixel data
    // We need to interpret it based on the color format
    lv_color_format_t cf = lv_display_get_color_format(disp);

    // Calculate pixel index
    size_t px_index = 0;
    int32_t w = lv_area_get_width(area);
    int32_t h = lv_area_get_height(area);

    for (int32_t y = 0; y < h; y++)
    {
        for (int32_t x = 0; x < w; x++)
        {
            // Get color from px_map based on color format
            lv_color_t color;

            // Most common formats
            if (cf == LV_COLOR_FORMAT_RGB565)
            {
                // RGB565: 2 bytes per pixel
                uint16_t rgb565 = ((uint16_t *)px_map)[px_index];
                color.red = ((rgb565 >> 11) & 0x1F) * 255 / 31;
                color.green = ((rgb565 >> 5) & 0x3F) * 255 / 63;
                color.blue = (rgb565 & 0x1F) * 255 / 31;
            }
            else if (cf == LV_COLOR_FORMAT_RGB888)
            {
                // RGB888: 3 bytes per pixel
                color.red = px_map[px_index * 3];
                color.green = px_map[px_index * 3 + 1];
                color.blue = px_map[px_index * 3 + 2];
            }
            else if (cf == LV_COLOR_FORMAT_XRGB8888 || cf == LV_COLOR_FORMAT_ARGB8888)
            {
                // 32-bit color: 4 bytes per pixel (A/X, R, G, B)
                color.red = px_map[px_index * 4 + 1];
                color.green = px_map[px_index * 4 + 2];
                color.blue = px_map[px_index * 4 + 3];
            }
            else
            {
                // Fallback for other formats - treat as grayscale
                color.red = color.green = color.blue = px_map[px_index];
            }

            // Convert RGB to monochrome
            uint8_t mono_color = rgb_to_mono(color);

            // Transform coordinates for landscape mode
            int32_t hw_x, hw_y;
            if (ctx->landscape)
            {
                // Landscape: Rotate 90 degrees clockwise
                // LVGL coords (0,0) is top-left of 250x122 display
                // Hardware coords (0,0) is top-left of 122x250 display
                // Transform: hw_x = y, hw_y = (WEACT_EPAPER_HEIGHT-1) - x
                hw_x = area->y1 + y;
                hw_y = (WEACT_EPAPER_HEIGHT - 1) - (area->x1 + x);
            }
            else
            {
                // Portrait: Direct mapping
                hw_x = area->x1 + x;
                hw_y = area->y1 + y;
            }

            // Draw to low-level framebuffer
            weact_epaper_draw_pixel(&ctx->epaper, hw_x, hw_y, mono_color);

            px_index++;
        }
    }

    // Tell LVGL we're done flushing first (LVGL 9 API)
    // This allows LVGL to continue while display refreshes
    lv_display_flush_ready(disp);

    // Update the physical display
    // Note: This takes ~2 seconds due to e-paper refresh time
    weact_epaper_display_frame(&ctx->epaper);
}

/**
 * @brief Get default configuration
 *
 * Returns the default pin configuration for WeAct 2.13" display.
 *
 * @return Default configuration structure
 */
lvgl_weact_epaper_config_t lvgl_weact_epaper_get_default_config(void)
{
    lvgl_weact_epaper_config_t config = {
        .pin_sck = 6,
        .pin_mosi = 7,
        .pin_cs = 10,
        .pin_dc = 9,
        .pin_rst = 4,
        .pin_busy = 18,
        .spi_clock_speed_hz = 4000000, // 4 MHz
        .landscape = false,             // Default: portrait mode
    };

    return config;
}

/**
 * @brief LVGL tick timer callback
 *
 * LVGL requires a periodic tick to handle timing, animations, etc.
 */
static void lvgl_tick_timer_cb(void *arg)
{
    lv_tick_inc(10); // 10ms tick
}

/**
 * @brief Create LVGL display
 *
 * Initializes the low-level driver and registers it with LVGL 9.
 *
 * @param config Display configuration (pin mapping, SPI speed)
 * @return Pointer to LVGL display object, or NULL on failure
 */
lv_display_t *lvgl_weact_epaper_create(const lvgl_weact_epaper_config_t *config)
{
    if (config == NULL)
    {
        ESP_LOGE(TAG, "Configuration is NULL");
        return NULL;
    }

    // Initialize low-level driver
    weact_epaper_config_t epaper_config = {
        .pin_sck = config->pin_sck,
        .pin_mosi = config->pin_mosi,
        .pin_cs = config->pin_cs,
        .pin_dc = config->pin_dc,
        .pin_rst = config->pin_rst,
        .pin_busy = config->pin_busy,
        .spi_clock_speed_hz = config->spi_clock_speed_hz,
    };

    // Store landscape orientation preference
    g_ctx.landscape = config->landscape;

    if (!weact_epaper_init(&g_ctx.epaper, &epaper_config))
    {
        ESP_LOGE(TAG, "Failed to initialize low-level driver");
        return NULL;
    }

    ESP_LOGI(TAG, "Low-level driver initialized");

    // Clear display to start with clean slate
    weact_epaper_clear_screen(&g_ctx.epaper);
    ESP_LOGI(TAG, "Display cleared");

    // ===============================================
    // LVGL 9 Display Creation
    // ===============================================

    // Determine display dimensions based on orientation
    int32_t disp_width = config->landscape ? WEACT_EPAPER_HEIGHT : WEACT_EPAPER_WIDTH;
    int32_t disp_height = config->landscape ? WEACT_EPAPER_WIDTH : WEACT_EPAPER_HEIGHT;

    // Create display object (LVGL 9 API)
    g_ctx.disp = lv_display_create(disp_width, disp_height);
    if (g_ctx.disp == NULL)
    {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        return NULL;
    }

    ESP_LOGI(TAG, "LVGL 9 display created: %dx%d (%s)",
             (int)disp_width, (int)disp_height,
             config->landscape ? "landscape" : "portrait");

    // Allocate LVGL draw buffers
    // Buffer size: For e-paper, use smaller buffer to reduce flush time
    // 1/10 screen = 3050 pixels, use ~2500 to be safe
    size_t buf_size = 122 * 250; // 122 * 250 / 8 = 3812 bytes (1 bit per pixel)

    g_ctx.draw_buf1 = heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (g_ctx.draw_buf1 == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate draw buffer 1");
        lv_display_delete(g_ctx.disp);
        return NULL;
    }

    // Optional: Use double buffering for smoother rendering
    g_ctx.draw_buf2 = heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (g_ctx.draw_buf2 == NULL)
    {
        ESP_LOGW(TAG, "Failed to allocate draw buffer 2, using single buffer");
    }

    ESP_LOGI(TAG, "Draw buffers allocated: %zu bytes each", buf_size * sizeof(lv_color_t));

    // Set display buffers (LVGL 9 API)
    // For e-paper, use FULL render mode for complete screen updates
    lv_display_set_buffers(g_ctx.disp,
                           g_ctx.draw_buf1,
                           g_ctx.draw_buf2,
                           buf_size * sizeof(lv_color_t),
                           LV_DISPLAY_RENDER_MODE_FULL);

    // Set flush callback (LVGL 9 API)
    lv_display_set_flush_cb(g_ctx.disp, lvgl_flush_cb);

    // Store context in user_data for callback access
    lv_display_set_user_data(g_ctx.disp, &g_ctx);

    // Set default display
    lv_display_set_default(g_ctx.disp);

    ESP_LOGI(TAG, "LVGL 9 display registered successfully");
    ESP_LOGI(TAG, "Display: WeAct 2.13\" E-Paper (%dx%d) %s mode",
             (int)disp_width, (int)disp_height,
             config->landscape ? "landscape" : "portrait");

    ESP_LOGI(TAG, "Setting up LVGL tick timer...");

    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = lvgl_tick_timer_cb,
        .name = "lvgl_tick"};

    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 10 * 1000)); // 10ms

    return g_ctx.disp;
}
