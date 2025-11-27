#ifndef WEACT_EPAPER_2IN13_H
#define WEACT_EPAPER_2IN13_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

/**
 * @brief SSD1680 E-Paper Display Low-Level Driver
 *
 * SSD1680 is a newer controller (successor to SSD1675) commonly found in:
 * - WeAct Studio 2.13" displays
 * - Waveshare 2.13" displays
 * - Other 250x122 e-paper modules
 *
 * Key differences from SSD1675:
 * - Different initialization sequence
 * - Updated command set
 * - Improved power management
 * - Better temperature compensation
 */

// =============================================================================
// DISPLAY SPECIFICATIONS
// =============================================================================
// WeAct 2.13" Display in PORTRAIT orientation
// Physical dimensions: 122 pixels wide × 250 pixels tall

#define WEACT_EPAPER_WIDTH       122     // Display width in pixels (portrait)
#define WEACT_EPAPER_HEIGHT      250     // Display height in pixels (portrait)
#define WEACT_EPAPER_WIDTH_BYTES 16      // Bytes per row (round up 122/8 = 15.25 to 16)
#define WEACT_EPAPER_BUFFER_SIZE (WEACT_EPAPER_WIDTH_BYTES * WEACT_EPAPER_HEIGHT)  // 16 * 250 = 4000 bytes

// =============================================================================
// SSD1680 COMMAND DEFINITIONS
// =============================================================================

#define WEACT_EPAPER_CMD_DRIVER_OUTPUT_CONTROL           0x01
#define WEACT_EPAPER_CMD_GATE_DRIVING_VOLTAGE            0x03
#define WEACT_EPAPER_CMD_SOURCE_DRIVING_VOLTAGE          0x04
#define WEACT_EPAPER_CMD_DEEP_SLEEP_MODE                 0x10
#define WEACT_EPAPER_CMD_DATA_ENTRY_MODE                 0x11
#define WEACT_EPAPER_CMD_SW_RESET                        0x12
#define WEACT_EPAPER_CMD_TEMP_SENSOR_CONTROL             0x18
#define WEACT_EPAPER_CMD_TEMP_SENSOR_WRITE               0x1A
#define WEACT_EPAPER_CMD_MASTER_ACTIVATION               0x20
#define WEACT_EPAPER_CMD_DISPLAY_UPDATE_CONTROL_1        0x21
#define WEACT_EPAPER_CMD_DISPLAY_UPDATE_CONTROL_2        0x22
#define WEACT_EPAPER_CMD_WRITE_RAM_BW                    0x24  // Write B/W RAM
#define WEACT_EPAPER_CMD_WRITE_RAM_RED                   0x26  // Write RED RAM (if supported)
#define WEACT_EPAPER_CMD_VCOM_SENSE                      0x28
#define WEACT_EPAPER_CMD_VCOM_SENSE_DURATION             0x29
#define WEACT_EPAPER_CMD_PROGRAM_VCOM_OTP                0x2A
#define WEACT_EPAPER_CMD_WRITE_VCOM_REGISTER             0x2C
#define WEACT_EPAPER_CMD_OTP_REGISTER_READ               0x2D
#define WEACT_EPAPER_CMD_WRITE_LUT_REGISTER              0x32
#define WEACT_EPAPER_CMD_DUMMY_LINE_PERIOD               0x3A
#define WEACT_EPAPER_CMD_GATE_LINE_WIDTH                 0x3B
#define WEACT_EPAPER_CMD_BORDER_WAVEFORM_CONTROL         0x3C
#define WEACT_EPAPER_CMD_SET_RAM_X_ADDRESS_START_END     0x44
#define WEACT_EPAPER_CMD_SET_RAM_Y_ADDRESS_START_END     0x45
#define WEACT_EPAPER_CMD_AUTO_WRITE_RED_PATTERN          0x46
#define WEACT_EPAPER_CMD_AUTO_WRITE_BW_PATTERN           0x47
#define WEACT_EPAPER_CMD_SET_RAM_X_ADDRESS_COUNTER       0x4E
#define WEACT_EPAPER_CMD_SET_RAM_Y_ADDRESS_COUNTER       0x4F
#define WEACT_EPAPER_CMD_NOP                             0x7F

// =============================================================================
// CONFIGURATION STRUCTURE
// =============================================================================

/**
 * @brief Pin configuration for SSD1680
 */
typedef struct {
    gpio_num_t pin_sck;     // SPI Clock
    gpio_num_t pin_mosi;    // SPI MOSI (Master Out Slave In)
    gpio_num_t pin_cs;      // Chip Select (active LOW)
    gpio_num_t pin_dc;      // Data/Command select (LOW=cmd, HIGH=data)
    gpio_num_t pin_rst;     // Reset (active LOW)
    gpio_num_t pin_busy;    // Busy signal (HIGH=busy)
    int spi_clock_speed_hz; // SPI clock speed (typically 4-20 MHz)
} weact_epaper_config_t;

/**
 * @brief SSD1680 device handle
 */
typedef struct {
    spi_device_handle_t spi;
    weact_epaper_config_t config;
    uint8_t *framebuffer;   // Pointer to framebuffer in memory
} weact_epaper_t;

// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================

/**
 * @brief Initialize the SSD1680 display
 *
 * @param dev Pointer to device handle
 * @param config Pointer to pin configuration
 * @return true on success, false on failure
 */
bool weact_epaper_init(weact_epaper_t *dev, const weact_epaper_config_t *config);

/**
 * @brief Send a command to SSD1680
 *
 * @param dev Device handle
 * @param cmd Command byte
 */
void weact_epaper_send_command(weact_epaper_t *dev, uint8_t cmd);

/**
 * @brief Send data to SSD1680
 *
 * @param dev Device handle
 * @param data Pointer to data buffer
 * @param len Number of bytes to send
 */
void weact_epaper_send_data(weact_epaper_t *dev, const uint8_t *data, size_t len);

/**
 * @brief Send a single data byte
 *
 * @param dev Device handle
 * @param data Data byte
 */
void weact_epaper_send_data_byte(weact_epaper_t *dev, uint8_t data);

/**
 * @brief Wait until display is not busy
 *
 * @param dev Device handle
 */
void weact_epaper_wait_until_idle(weact_epaper_t *dev);

/**
 * @brief Hardware reset the display
 *
 * @param dev Device handle
 */
void weact_epaper_reset(weact_epaper_t *dev);

/**
 * @brief Clear the entire display (all white)
 *
 * @param dev Device handle
 */
void weact_epaper_clear_screen(weact_epaper_t *dev);

/**
 * @brief Draw a pixel in the framebuffer
 *
 * @param dev Device handle
 * @param x X coordinate (0 to WEACT_EPAPER_WIDTH-1)
 * @param y Y coordinate (0 to WEACT_EPAPER_HEIGHT-1)
 * @param color 1 = BLACK, 0 = WHITE
 */
void weact_epaper_draw_pixel(weact_epaper_t *dev, int x, int y, uint8_t color);

/**
 * @brief Draw a rectangle in the framebuffer
 *
 * @param dev Device handle
 * @param x0 Top-left X coordinate
 * @param y0 Top-left Y coordinate
 * @param x1 Bottom-right X coordinate
 * @param y1 Bottom-right Y coordinate
 * @param filled true = filled rectangle, false = outline only
 */
void weact_epaper_draw_rectangle(weact_epaper_t *dev, int x0, int y0, int x1, int y1, bool filled);

/**
 * @brief Send the framebuffer to the display and refresh
 *
 * @param dev Device handle
 */
void weact_epaper_display_frame(weact_epaper_t *dev);

/**
 * @brief Enter deep sleep mode (low power)
 *
 * @param dev Device handle
 */
void weact_epaper_sleep(weact_epaper_t *dev);

#endif // WEACT_EPAPER_2IN13_H
