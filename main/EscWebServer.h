#pragma once

#include <cstdint>

#include "esp_err.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "JsonWrapper.h"
#include "WebServer.h"

class Settings;

// HTTP control surface for the ESC: /motor, /pump, /pump_range, /firmware
// (plus /healthz from the WebServer base). The handlers recover this instance
// from req->user_ctx. Owns the runtime pump-range and pole-pair values, loaded
// from Settings at construction.
class EscWebServer : public WebServer {
public:
    EscWebServer(WebContext *ctx, Settings &settings);

    esp_err_t start() override;

private:
    static esp_err_t motor_post_handler(httpd_req_t *req);
    static esp_err_t motor_get_handler(httpd_req_t *req);
    static esp_err_t pump_post_handler(httpd_req_t *req);
    static esp_err_t pump_range_post_handler(httpd_req_t *req);
    static esp_err_t pump_range_get_handler(httpd_req_t *req);
    static esp_err_t firmware_post_handler(httpd_req_t *req);
    static esp_err_t firmware_get_handler(httpd_req_t *req);
    static esp_err_t config_get_handler(httpd_req_t *req);
    static esp_err_t config_post_handler(httpd_req_t *req);
    static esp_err_t config_reset_post_handler(httpd_req_t *req);
    static esp_err_t calibrate_pump_min_post_handler(httpd_req_t *req);
    static esp_err_t esc_reset_post_handler(httpd_req_t *req);

    void status_to_json(JsonWrapper &json);
    void config_to_json(JsonWrapper &json);
    uint16_t pump_duty_to_throttle(float duty) const;

    // Duty below this percent disables the drive (closed-loop PID hovering
    // near 0 shouldn't keep re-arming the bridge).
    static constexpr float kPumpOffDutyPct = 0.5f;

    Settings &settings_;

    // Protects the four scalars below so paired reads (e.g. pump_min + pump_max
    // in pump_duty_to_throttle) see a coherent snapshot. portMUX is a CAS
    // spinlock — fine for these microsecond-scale critical sections.
    mutable portMUX_TYPE config_mu_ = portMUX_INITIALIZER_UNLOCKED;
    uint16_t pump_min_throttle_;
    uint16_t pump_max_throttle_;
    int      pole_pairs_;
    float    voltage_scale_;
};
