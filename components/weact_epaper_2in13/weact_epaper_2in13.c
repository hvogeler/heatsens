#include "weact_epaper_2in13.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "WEACT_EPAPER";

// =============================================================================
// LUT (Look-Up Table) for SSD1680
// =============================================================================
/**
 * Full refresh LUT for SSD1680 (currently unused - using internal LUT)
 * This LUT is optimized for the WeAct 2.13" display with SSD1680 controller
 *
 * The SSD1680 uses a 70-byte LUT format (different from SSD1675's 30 bytes!)
 * Uncomment to use custom LUT instead of internal one
 */
#if 0 // Set to 1 to use custom LUT
static const uint8_t weact_epaper_lut_full_update[] = {
    // LUT for VCOM
    0x00, 0x16, 0x16, 0x0D, 0x00, 0x01,

    // LUT for White to White
    0x00, 0x16, 0x16, 0x0D, 0x00, 0x01,

    // LUT for Black to White
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // LUT for White to Black
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // LUT for Black to Black
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
#endif

/**
 * Partial refresh LUT (faster but may have ghosting)
 * Uncomment if you want to experiment with partial updates
 */
// static const uint8_t weact_epaper_lut_partial_update[] = {
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x0A, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00,
// };

// =============================================================================
// LOW-LEVEL SPI COMMUNICATION FUNCTIONS
// =============================================================================

void weact_epaper_send_command(weact_epaper_t *dev, uint8_t cmd)
{
    spi_transaction_t trans = {
        .length = 8,
        .tx_buffer = &cmd,
        .rx_buffer = NULL,
    };

    gpio_set_level(dev->config.pin_dc, 0); // Command mode
    ESP_ERROR_CHECK(spi_device_polling_transmit(dev->spi, &trans));
}

void weact_epaper_send_data(weact_epaper_t *dev, const uint8_t *data, size_t len)
{
    if (len == 0)
        return;

    spi_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = data,
        .rx_buffer = NULL,
    };

    gpio_set_level(dev->config.pin_dc, 1); // Data mode
    ESP_ERROR_CHECK(spi_device_polling_transmit(dev->spi, &trans));
}

void weact_epaper_send_data_byte(weact_epaper_t *dev, uint8_t data)
{
    weact_epaper_send_data(dev, &data, 1);
}

// =============================================================================
// CONTROL FUNCTIONS
// =============================================================================

void weact_epaper_wait_until_idle(weact_epaper_t *dev)
{
    ESP_LOGI(TAG, "Waiting for display...");

    uint32_t timeout = 0;
    const uint32_t MAX_TIMEOUT_MS = 5000; // 5 second timeout

    // Wait while BUSY is HIGH (display is busy)
    // SSD1680 BUSY logic: HIGH = busy, LOW = ready
    while (gpio_get_level(dev->config.pin_busy) == 1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout += 10;

        // ESP_LOGI(TAG, "...still busy (%lu ms elapsed)", timeout);

        // Feed the watchdog every 100ms to prevent timeout during long refresh
        // if (timeout % 100 == 0)
        // {
        //     esp_task_wdt_reset();
        // }

        if (timeout > MAX_TIMEOUT_MS)
        {
            ESP_LOGW(TAG, "Display busy timeout! Continuing anyway...");
            break;
        }
    }

    ESP_LOGI(TAG, "Display ready (waited %lu ms)", timeout);
}

void weact_epaper_reset(weact_epaper_t *dev)
{
    ESP_LOGI(TAG, "Hardware reset");

    gpio_set_level(dev->config.pin_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    gpio_set_level(dev->config.pin_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(2));

    gpio_set_level(dev->config.pin_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
}

// =============================================================================
// INITIALIZATION
// =============================================================================

bool weact_epaper_init(weact_epaper_t *dev, const weact_epaper_config_t *config)
{
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "Initializing SSD1680 (250x122 e-paper)");
    ESP_LOGI(TAG, "=================================================");

    memcpy(&dev->config, config, sizeof(weact_epaper_config_t));

    // -------------------------------------------------------------------------
    // GPIO Configuration
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Configuring GPIO pins");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->pin_dc) | (1ULL << config->pin_rst),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    io_conf.pin_bit_mask = (1ULL << config->pin_busy);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // -------------------------------------------------------------------------
    // SPI Bus Configuration
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Configuring SPI bus");

    spi_bus_config_t buscfg = {
        .mosi_io_num = config->pin_mosi,
        .miso_io_num = -1,
        .sclk_io_num = config->pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = WEACT_EPAPER_BUFFER_SIZE,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = config->spi_clock_speed_hz,
        .mode = 0,
        .spics_io_num = config->pin_cs,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &dev->spi));

    // -------------------------------------------------------------------------
    // Framebuffer Allocation
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Allocating framebuffer (%d bytes)", WEACT_EPAPER_BUFFER_SIZE);

    dev->framebuffer = heap_caps_malloc(WEACT_EPAPER_BUFFER_SIZE, MALLOC_CAP_DMA);
    if (dev->framebuffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate framebuffer!");
        return false;
    }

    // Initialize to white (0xFF in e-paper RAM = white)
    memset(dev->framebuffer, 0xFF, WEACT_EPAPER_BUFFER_SIZE);

    // -------------------------------------------------------------------------
    // Hardware Reset
    // -------------------------------------------------------------------------
    weact_epaper_reset(dev);
    weact_epaper_wait_until_idle(dev);

    // -------------------------------------------------------------------------
    // SSD1680 Initialization Sequence
    // -------------------------------------------------------------------------
    ESP_LOGI(TAG, "Sending SSD1680 initialization sequence");

    // Software Reset
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_SW_RESET);
    weact_epaper_wait_until_idle(dev);

    // Driver Output Control
    // A[7:0]: MUX Gate lines = 250-1 = 249 = 0xF9
    // A[8] and B[2:0]: Gate scanning sequence
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_DRIVER_OUTPUT_CONTROL);
    weact_epaper_send_data_byte(dev, 0xF9); // 250-1 (height - 1) LOW byte
    weact_epaper_send_data_byte(dev, 0x00); // HIGH byte
    weact_epaper_send_data_byte(dev, 0x00); // GD=0, SM=0, TB=0

    // Data Entry Mode
    // Sets how data is written to RAM
    // Bit 0-1: Address counter direction (00=Y-, 01=Y+, 10=X-, 11=X+)
    // Bit 2: I/D mode (0=X direction, 1=Y direction)
    // 0x03 = X direction, X increment, Y increment
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_DATA_ENTRY_MODE);
    weact_epaper_send_data_byte(dev, 0x03);

    // Set RAM X address start/end
    // For portrait with aligned rows: 16 bytes per row (128 pixels, using 122)
    // X is in bytes: 0-15 (0x00 to 0x0F)
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_SET_RAM_X_ADDRESS_START_END);
    weact_epaper_send_data_byte(dev, 0x00); // X start (0)
    weact_epaper_send_data_byte(dev, 0x0F); // X end (15)

    // Set RAM Y address start/end
    // For portrait: treat as rows (tall dimension)
    // Y is in pixels: 0 to 249 (0xF9)
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_SET_RAM_Y_ADDRESS_START_END);
    weact_epaper_send_data_byte(dev, 0x00); // Y start LOW
    weact_epaper_send_data_byte(dev, 0x00); // Y start HIGH
    weact_epaper_send_data_byte(dev, 0xF9); // Y end LOW (249)
    weact_epaper_send_data_byte(dev, 0x00); // Y end HIGH

    // Border Waveform Control
    // This controls the border color during refresh
    // 0x05 = Follow LUT (normal behavior)
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_BORDER_WAVEFORM_CONTROL);
    weact_epaper_send_data_byte(dev, 0x05);

    // Display Update Control 1
    // This sets the display update sequence options
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_DISPLAY_UPDATE_CONTROL_1);
    weact_epaper_send_data_byte(dev, 0x00);
    weact_epaper_send_data_byte(dev, 0x80);

    // Temperature Sensor Control
    // 0x80 = Internal temperature sensor
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_TEMP_SENSOR_CONTROL);
    weact_epaper_send_data_byte(dev, 0x80);

    // Load LUT (optional - comment out to use internal LUT)
    // ESP_LOGI(TAG, "Loading custom LUT");
    // weact_epaper_send_command(dev, WEACT_EPAPER_CMD_WRITE_LUT_REGISTER);
    // weact_epaper_send_data(dev, weact_epaper_lut_full_update, sizeof(weact_epaper_lut_full_update));

    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "SSD1680 initialization complete!");
    ESP_LOGI(TAG, "=================================================");

    return true;
}

// =============================================================================
// DRAWING FUNCTIONS
// =============================================================================

void weact_epaper_draw_pixel(weact_epaper_t *dev, int x, int y, uint8_t color)
{
    if (x < 0 || x >= WEACT_EPAPER_WIDTH || y < 0 || y >= WEACT_EPAPER_HEIGHT)
    {
        return;
    }

    // Calculate byte position with aligned row width
    // Each row is WEACT_EPAPER_WIDTH_BYTES (16) bytes, even though we only use 15.25
    int byte_index = y * WEACT_EPAPER_WIDTH_BYTES + (x / 8);
    int bit_index = x % 8;

    if (color == 0)
    {
        // WHITE: Set bit to 1
        dev->framebuffer[byte_index] |= (1 << (7 - bit_index));
    }
    else
    {
        // BLACK: Clear bit to 0
        dev->framebuffer[byte_index] &= ~(1 << (7 - bit_index));
    }
}

void weact_epaper_draw_rectangle(weact_epaper_t *dev, int x0, int y0, int x1, int y1, bool filled)
{
    if (x0 > x1)
    {
        int temp = x0;
        x0 = x1;
        x1 = temp;
    }
    if (y0 > y1)
    {
        int temp = y0;
        y0 = y1;
        y1 = temp;
    }

    if (filled)
    {
        for (int y = y0; y <= y1; y++)
        {
            for (int x = x0; x <= x1; x++)
            {
                weact_epaper_draw_pixel(dev, x, y, 1);
            }
        }
    }
    else
    {
        for (int x = x0; x <= x1; x++)
        {
            weact_epaper_draw_pixel(dev, x, y0, 1);
            weact_epaper_draw_pixel(dev, x, y1, 1);
        }
        for (int y = y0; y <= y1; y++)
        {
            weact_epaper_draw_pixel(dev, x0, y, 1);
            weact_epaper_draw_pixel(dev, x1, y, 1);
        }
    }
}

void weact_epaper_clear_screen(weact_epaper_t *dev)
{
    ESP_LOGI(TAG, "Clearing screen to white");

    // Fill framebuffer with 0xFF (all white)
    memset(dev->framebuffer, 0xFF, WEACT_EPAPER_BUFFER_SIZE);

    // Set RAM address to start position
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_SET_RAM_X_ADDRESS_COUNTER);
    weact_epaper_send_data_byte(dev, 0x00);

    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_SET_RAM_Y_ADDRESS_COUNTER);
    weact_epaper_send_data_byte(dev, 0x00);
    weact_epaper_send_data_byte(dev, 0x00);

    // Write white data to both BW and RED RAM (if applicable)
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_WRITE_RAM_BW);
    weact_epaper_send_data(dev, dev->framebuffer, WEACT_EPAPER_BUFFER_SIZE);

    // Also clear RED RAM (even if display doesn't support red)
    // This prevents any residual data
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_SET_RAM_X_ADDRESS_COUNTER);
    weact_epaper_send_data_byte(dev, 0x00);
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_SET_RAM_Y_ADDRESS_COUNTER);
    weact_epaper_send_data_byte(dev, 0x00);
    weact_epaper_send_data_byte(dev, 0x00);

    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_WRITE_RAM_RED);
    weact_epaper_send_data(dev, dev->framebuffer, WEACT_EPAPER_BUFFER_SIZE);

    // Trigger display update
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_DISPLAY_UPDATE_CONTROL_2);
    weact_epaper_send_data_byte(dev, 0xF7);

    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_MASTER_ACTIVATION);

    weact_epaper_wait_until_idle(dev);

    ESP_LOGI(TAG, "Screen cleared successfully");
}

void weact_epaper_display_frame(weact_epaper_t *dev)
{
    ESP_LOGI(TAG, "Uploading framebuffer to display");

    // Set RAM X address counter
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_SET_RAM_X_ADDRESS_COUNTER);
    weact_epaper_send_data_byte(dev, 0x00);

    // Set RAM Y address counter
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_SET_RAM_Y_ADDRESS_COUNTER);
    weact_epaper_send_data_byte(dev, 0x00); // Y LOW
    weact_epaper_send_data_byte(dev, 0x00); // Y HIGH

    // Write to Black/White RAM
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_WRITE_RAM_BW);
    weact_epaper_send_data(dev, dev->framebuffer, WEACT_EPAPER_BUFFER_SIZE);

    // Display Update Control 2
    // 0xF7 = Full refresh with display mode 1
    // 0xC7 = Partial refresh (faster but may ghost)
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_DISPLAY_UPDATE_CONTROL_2);
    weact_epaper_send_data_byte(dev, 0xF7); // Use 0xC7 for partial refresh

    // Master Activation (start the refresh)
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_MASTER_ACTIVATION);

    // Wait for refresh to complete
    weact_epaper_wait_until_idle(dev);

    ESP_LOGI(TAG, "Display update complete!");
}

void weact_epaper_sleep(weact_epaper_t *dev)
{
    ESP_LOGI(TAG, "Entering deep sleep mode");

    // Deep sleep mode
    // 0x01 = Deep sleep mode 1 (RAM preserved)
    // 0x03 = Deep sleep mode 2 (RAM not preserved, lower power)
    weact_epaper_send_command(dev, WEACT_EPAPER_CMD_DEEP_SLEEP_MODE);
    weact_epaper_send_data_byte(dev, 0x01);

    vTaskDelay(pdMS_TO_TICKS(100));
}
