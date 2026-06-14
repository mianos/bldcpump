idf_component_register(SRCS "esc_telem.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_driver_uart esp_timer)
