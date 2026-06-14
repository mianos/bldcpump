#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "EscWebServer.h"
#include "Settings.h"
#include "WebServer.h"
#include "WifiConnection.h"

#include "display.h"
#include "drive.h"
#include "esc_telem.h"

static const char *TAG = "main";

namespace {
constexpr int DSHOT_GPIO_NUM     = 13;   // TTGO: DShot signal to ESC
constexpr int ESC_TELEM_GPIO_NUM = 12;   // ESC TX (telemetry) -> ESP RX
}  // namespace

extern "C" void app_main(void) {
    // NVS init — required by WiFiManager + Settings.
    {
        esp_err_t r = nvs_flash_init();
        if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            r = nvs_flash_init();
        }
        ESP_ERROR_CHECK(r);
    }

    if (display_init() != ESP_OK) {
        ESP_LOGE(TAG, "Display init failed; continuing without display");
    }
    if (esc_telem_init(ESC_TELEM_GPIO_NUM) != ESP_OK) {
        ESP_LOGE(TAG, "ESC telemetry init failed");
    }

    static Settings settings;
    if (settings.load_hostname("").empty()) {
        ESP_LOGI(TAG, "seeding default hostname 'esc-pump' into NVS");
        settings.save_hostname("esc-pump");
    }

    // Apply persisted voltage calibration to the live telemetry parser before
    // any packets are decoded into the median ring.
    float v_scale = settings.load_voltage_scale();
    esc_telem_set_voltage_scale(v_scale);
    ESP_LOGI(TAG, "voltage_scale = %.4f (from NVS)", v_scale);

    // Push pole_pairs into the display so eRPM -> mech RPM uses the configured
    // value instead of the display's hardcoded default.
    int pole_pairs = settings.load_pole_pairs();
    display_set_pole_pairs(pole_pairs);
    ESP_LOGI(TAG, "pole_pairs = %d (from NVS)", pole_pairs);

    // Start the drive task before WiFi: ESC needs the arm hold + beep sequence
    // to come up before anything will move, and we want that out of the way
    // whether or not WiFi associates.
    ESP_ERROR_CHECK(drive_init(DSHOT_GPIO_NUM));

    static WifiConnection wifi(settings);
    wifi.wait_for_ip();

    static WebContext   web_ctx(&wifi.manager());
    static EscWebServer web(&web_ctx, settings);
    ESP_ERROR_CHECK(web.start());
    ESP_LOGI(TAG, "webserver up: POST /motor {\"throttle\":0-2047,\"enabled\":bool,\"pole_pairs\":N}, "
                  "GET /motor for status, POST /pump {\"duty\":0-100}, "
                  "POST/GET /pump_range {\"min\":N,\"max\":N}, "
                  "POST /firmware (raw .bin), GET /firmware, "
                  "GET/POST /config {\"voltage_scale\":N,\"pole_pairs\":N,\"pump_min\":N,\"pump_max\":N,\"hostname\":\"...\"}, "
                  "POST /config/reset {\"hostname\":\"...\"?,\"wifi\":bool?}, "
                  "POST /calibrate/pump_min {\"ascending\":bool?,\"save\":bool?,...}, "
                  "POST /esc/reset (replay AM32 arm+beep)");

    // Cancel OTA rollback once WiFi + webserver are up.
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        ESP_LOGI(TAG, "OTA: image marked valid (rollback cancelled)");
    }

    // drive task + webserver workers handle everything; app_main blocks.
    while (true) {
        vTaskDelay(portMAX_DELAY);
    }
}
