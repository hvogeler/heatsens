# WeAct E-Paper 2.13" Display Driver

Low-level hardware driver for the **WeAct Studio 2.13" E-Paper Display**.

## Specifications

- **Controller**: SSD1680
- **Resolution**: 122 × 250 pixels (portrait orientation)
- **Colors**: Black and White
- **Interface**: 4-wire SPI
- **Refresh Time**: ~2 seconds

## Features

- Hardware SPI communication with DMA support
- Byte-aligned framebuffer (16 bytes per row)
- Direct pixel and rectangle drawing
- Full screen refresh
- Power management (deep sleep mode)

## Usage

```c
#include "weact_epaper_2in13.h"

weact_epaper_config_t config = {
    .pin_sck = 6,
    .pin_mosi = 7,
    .pin_cs = 10,
    .pin_dc = 9,
    .pin_rst = 4,
    .pin_busy = 18,
    .spi_clock_speed_hz = 4000000,
};

weact_epaper_t display;
weact_epaper_init(&display, &config);

weact_epaper_draw_pixel(&display, 50, 100, 1);  // Black pixel
weact_epaper_display_frame(&display);           // Update display
```

## Note

This driver is specifically tuned for the WeAct Studio 2.13" display.
It includes the correct initialization sequence, memory alignment, 
and display configuration for this specific hardware.
