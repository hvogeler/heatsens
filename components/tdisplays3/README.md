# T-Display S3 Component

ESP-IDF component for the LilyGO T-Display S3 board with ST7789 LCD display and LVGL integration.

## Features

- ST7789 LCD driver (320x170, RGB565)
- LVGL 9.x integration via esp_lvgl_port
- AW9364 backlight brightness control with fade support
- Battery voltage monitoring (optional)
- Configurable via Kconfig

## Dependencies

This component requires:
- `esp_lvgl_port`
- `lvgl`
- `esp_lcd`
- `hiruna/esp-idf-aw9364` (backlight controller)

## Installation

### Option 1: Using idf_component.yml (Recommended)

Add to your project's `main/idf_component.yml`:

```yaml
dependencies:
  tdisplays3:
    git: https://github.com/hvogeler/esp-components.git
    path: tdisplays3
    version: main
```

Then run:
```bash
idf.py reconfigure
```

The component and its dependencies will be automatically fetched.

### Option 2: As a local component

Copy the `tdisplays3` directory to your project's `components/` folder.

### Customizing the AW9364 dependency

If you want to use a specific fork of the AW9364 driver, you can override it in your project's `main/idf_component.yml`:

```yaml
dependencies:
  tdisplays3:
    git: https://github.com/hvogeler/esp-components.git
    path: tdisplays3
    version: main
  hiruna/esp-idf-aw9364:
    git: https://github.com/hvogeler/esp-idf-aw9364.git
    version: main
```

## Configuration

Run `idf.py menuconfig` and navigate to **T-Display S3 Configuration**.

### LVGL Task Settings

| Option | Default | Range | Description |
|--------|---------|-------|-------------|
| `LVGL task priority` | 2 | 1-24 | Priority of the LVGL task |
| `LVGL task stack size` | 8192 | 4096-16384 | Stack size in bytes |
| `LVGL tick period (ms)` | 5 | 1-20 | Lower = more responsive, higher CPU |
| `LVGL max sleep time (ms)` | 10 | 2-100 | Affects screen refresh rate |

### LCD Settings

| Option | Default | Range | Description |
|--------|---------|-------|-------------|
| `Backlight LEDC channel` | 0 | 0-7 | Change if conflicts with other LEDC usage |
| `LCD pixel clock (MHz)` | 17 | 2-17 | Higher = better performance |
| `I80 transaction queue size` | 20 | 10-50 | Higher = more memory, better throughput |

## API

### Initialization

```c
#include "t_display_s3.h"

lv_disp_t *display;
lcd_init(&display, true);  // Initialize LCD with backlight on
```

### Brightness Control

```c
// Set brightness by step (0-16)
lcd_set_brightness_step(8);
lcd_set_brightness_step_fade(16, 500);  // Fade to max over 500ms

// Set brightness by percentage (0-100)
lcd_set_brightness_pct(50);
lcd_set_brightness_pct_fade(100, 1000);  // Fade to 100% over 1s

// Increment/decrement
lcd_increment_brightness_step();
lcd_decrement_brightness_step();

// Get current brightness
uint8_t step = lcd_get_brightness_step();
uint8_t pct = lcd_get_brightness_pct();
```

### Battery Monitoring (optional)

Battery monitoring requires calling `init_battery_monitor()` first (currently commented out in `lcd_init`).

```c
int voltage_mv = get_battery_voltage();      // Returns millivolts
int percentage = get_battery_percentage();   // Returns 0-100%
bool usb = usb_power_connected();            // True if USB power detected
```

## Pin Definitions

| Pin | GPIO | Function |
|-----|------|----------|
| LCD_PWR | 15 | LCD power enable |
| LCD_BL | 38 | Backlight control |
| LCD_D0-D7 | 39-48 | 8-bit parallel data |
| LCD_WR | 8 | Write clock |
| LCD_RD | 9 | Read (active high) |
| LCD_DC | 7 | Data/Command |
| LCD_CS | 6 | Chip select |
| LCD_RST | 5 | Reset |
| BAT_VOLT | 4 | Battery ADC (ADC1_CH3) |
| BTN1 | 0 | Boot button |
| BTN2 | 14 | User button |

## Example

```c
#include "t_display_s3.h"

void app_main(void)
{
    lv_disp_t *display;
    lcd_init(&display, true);

    // Create LVGL UI
    lvgl_port_lock(0);

    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello T-Display S3!");
    lv_obj_center(label);

    lvgl_port_unlock();

    // Adjust brightness
    lcd_set_brightness_pct(75);
}
```

## License

MIT License - Based on work by Hiruna Wijesinghe
