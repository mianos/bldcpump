idf_component_register(SRCS "display.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_lcd esp_driver_spi esp_driver_gpio esp_timer
                                esc_telem lvgl__lvgl espressif__esp_lvgl_port)
