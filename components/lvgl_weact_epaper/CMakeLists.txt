idf_component_register(
    SRCS "lvgl_weact_epaper.c"
    INCLUDE_DIRS "include"
    REQUIRES weact_epaper_2in13 lvgl__lvgl esp_timer
)
