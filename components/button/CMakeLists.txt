# Empty stub. mianesp's webserver and wifimanager components list "button"
# (a C++ Button class) as a transitive dependency, but neither actually uses
# it for this app's flow. We need to prevent it from being added to the build
# anyway: espressif/esp_lvgl_port's CMakeLists detects any component named
# "button" in BUILD_COMPONENTS and tries to compile its iot_button shim
# against it — which fails because mianesp's Button has a different API.
#
# This local component shadows the managed one (local takes priority). It
# registers no sources and no headers, so neither LVGL's button shim nor any
# downstream consumer sees a usable Button API — fine, because nothing in
# this app needs one.
idf_component_register()
