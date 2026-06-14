cmake_minimum_required(VERSION 3.16)

# Exclude mianesp's C++ Button component (a transitive dep of wifimanager that
# we don't actually use). Required because espressif/esp_lvgl_port detects any
# component named "button" and tries to compile its iot_button shim against it —
# but mianesp's Button has a different (incompatible) API. Cutting button out
# avoids that collision; wifimanager doesn't include Button.h.
set(EXCLUDE_COMPONENTS button)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESC)
