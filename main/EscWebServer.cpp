#include "EscWebServer.h"

#include <algorithm>
#include <array>
#include <cinttypes>
#include <string>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Settings.h"
#include "display.h"
#include "drive.h"
#include "esc_telem.h"

namespace {

constexpr const char *TAG = "escweb";

// JSON endpoints are tiny — keep the input bounded so a hostile
// Content-Length can't trigger a 4 GiB std::string::reserve and OOM the
// worker. Firmware upload has its own bound (the OTA partition size).
constexpr size_t kMaxJsonBodyBytes = 4 * 1024;

std::string read_request_body(httpd_req_t *req) {
    std::string body;
    body.reserve(req->content_len);
    char buf[256];
    int remaining = req->content_len;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, std::min<int>(remaining, static_cast<int>(sizeof(buf))));
        if (got <= 0) break;
        body.append(buf, got);
        remaining -= got;
    }
    return body;
}

// RAII wrapper around drive_try_acquire_exclusive / drive_release_exclusive.
// Construct at the top of any handler that takes long-running ownership of the
// drive; check held() and return 409 if false. Release happens automatically
// on every exit path.
class DriveExclusive {
public:
    DriveExclusive() : held_(drive_try_acquire_exclusive()) {}
    ~DriveExclusive() { if (held_) drive_release_exclusive(); }
    bool held() const { return held_; }
    DriveExclusive(const DriveExclusive &)            = delete;
    DriveExclusive &operator=(const DriveExclusive &) = delete;
private:
    bool held_;
};

esp_err_t send_json(httpd_req_t *req, const JsonWrapper &json) {
    httpd_resp_set_type(req, "application/json");
    std::string out = json.ToString();
    return httpd_resp_sendstr(req, out.c_str());
}

struct SweepResult {
    bool ok;
    uint16_t floor_throttle;   // lowest observed throttle that kept the rotor spinning
    uint16_t recommended;      // floor + safety margin (caller-tunable)
    uint16_t stalled_at;       // throttle where eRPM dropped below stall_erpm (0 if never)
    uint16_t last_erpm_raw;    // most recent raw eRPM (AM32 BLHeli32 ×100 convention)
    const char *error;         // static string; only valid when ok == false
};

// Step down from `start` in `step` units every `step_ms` until the median
// eRPM falls below `stall_erpm` (AM32 raw, i.e. actual/100). Stops at min_stop
// if the motor keeps spinning all the way down. Leaves the drive disabled on
// every exit path so a failed sweep can't strand a powered motor.
static SweepResult sweep_descending(uint16_t start, uint16_t step, uint32_t step_ms,
                                    uint16_t stall_erpm, float margin_pct, uint16_t min_stop) {
    SweepResult r = {};

    drive_set_throttle(start);
    drive_set_enabled(true);
    // Give AM32 time to spin up and the median ring to fill at the new throttle.
    vTaskDelay(pdMS_TO_TICKS(step_ms * 3 < 600 ? 600 : step_ms * 3));

    esc_telem_packet_t pkt;
    if (!esc_telem_is_fresh(1000) || !esc_telem_get_latest(&pkt)) {
        drive_set_enabled(false);
        r.error = "no fresh telemetry at start_throttle";
        return r;
    }
    if (pkt.erpm < stall_erpm) {
        drive_set_enabled(false);
        r.last_erpm_raw = pkt.erpm;
        r.error = "motor not spinning at start_throttle";
        return r;
    }

    uint16_t prev_good = start;
    uint16_t throttle  = start;
    while (throttle > min_stop) {
        throttle = (throttle > step + min_stop) ? (throttle - step) : min_stop;
        drive_set_throttle(throttle);
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        if (!esc_telem_is_fresh(1000)) {
            drive_set_enabled(false);
            r.error = "telemetry stalled mid-sweep";
            return r;
        }
        esc_telem_get_latest(&pkt);
        r.last_erpm_raw = pkt.erpm;
        if (pkt.erpm < stall_erpm) {
            drive_set_enabled(false);
            r.ok = true;
            r.floor_throttle = prev_good;
            r.stalled_at     = throttle;
            r.recommended    = static_cast<uint16_t>(prev_good * (1.0f + margin_pct / 100.0f) + 0.5f);
            if (r.recommended > 2047) r.recommended = 2047;
            return r;
        }
        prev_good = throttle;
    }

    drive_set_enabled(false);
    r.ok = true;
    r.floor_throttle = prev_good;
    r.stalled_at     = 0;
    r.recommended    = static_cast<uint16_t>(prev_good * (1.0f + margin_pct / 100.0f) + 0.5f);
    return r;
}

// Step up from `start` until the motor first reports raw eRPM >= spin_erpm.
// Returns the first throttle that produced motion (the cold-start floor).
// Gives up at `cap` if the rotor never breaks free.
static SweepResult sweep_ascending(uint16_t start, uint16_t step, uint16_t cap,
                                   uint32_t step_ms, uint16_t spin_erpm, float margin_pct) {
    SweepResult r = {};
    if (start > cap) { r.error = "start_throttle > cap"; return r; }

    drive_set_throttle(start);
    drive_set_enabled(true);

    uint16_t throttle = start;
    while (throttle <= cap) {
        drive_set_throttle(throttle);
        vTaskDelay(pdMS_TO_TICKS(step_ms));
        if (!esc_telem_is_fresh(1000)) {
            drive_set_enabled(false);
            r.error = "telemetry stalled mid-sweep";
            return r;
        }
        esc_telem_packet_t pkt;
        esc_telem_get_latest(&pkt);
        r.last_erpm_raw = pkt.erpm;
        if (pkt.erpm >= spin_erpm) {
            drive_set_enabled(false);
            r.ok = true;
            r.floor_throttle = throttle;
            r.recommended    = static_cast<uint16_t>(throttle * (1.0f + margin_pct / 100.0f) + 0.5f);
            if (r.recommended > 2047) r.recommended = 2047;
            return r;
        }
        throttle = (throttle + step > cap) ? (cap + 1) : (throttle + step);
    }

    drive_set_enabled(false);
    r.error = "reached cap without rotor spinning up";
    return r;
}

}  // namespace

EscWebServer::EscWebServer(WebContext *ctx, Settings &settings)
    : WebServer(ctx), settings_(settings) {
    settings_.load_pump_range(pump_min_throttle_, pump_max_throttle_);
    pole_pairs_    = settings_.load_pole_pairs();
    voltage_scale_ = settings_.load_voltage_scale();
    ESP_LOGI(TAG, "config loaded: pump duty 0-100%% -> throttle %u..%u, pole_pairs=%d, v_scale=%.4f",
        pump_min_throttle_, pump_max_throttle_, pole_pairs_, voltage_scale_);
}

uint16_t EscWebServer::pump_duty_to_throttle(float duty) const {
    duty = std::clamp(duty, 0.0f, 100.0f);
    uint16_t lo, hi;
    portENTER_CRITICAL(&config_mu_);
    lo = pump_min_throttle_;
    hi = pump_max_throttle_;
    portEXIT_CRITICAL(&config_mu_);
    float span = static_cast<float>(hi) - static_cast<float>(lo);
    float v = static_cast<float>(lo) + (duty / 100.0f) * span;
    if (v < 0) v = 0;
    if (v > 2047) v = 2047;
    return static_cast<uint16_t>(v + 0.5f);
}

void EscWebServer::status_to_json(JsonWrapper &json) {
    DriveStatus ds = drive_get_status();
    int   pp;
    float vs;
    portENTER_CRITICAL(&config_mu_);
    pp = pole_pairs_;
    vs = voltage_scale_;
    portEXIT_CRITICAL(&config_mu_);

    json.AddItem("target_throttle",  static_cast<int>(ds.target_throttle));
    json.AddItem("current_throttle", static_cast<int>(ds.current_throttle));
    // duty_pct is -1 if no /pump command has been issued (or a direct
    // /motor throttle was used since). Surface that as null-ish: report
    // duty_known + duty_pct so clients can distinguish.
    json.AddItem("duty_known",       ds.duty_pct >= 0.0f);
    json.AddItem("duty_pct",         ds.duty_pct >= 0.0f ? ds.duty_pct : 0.0f);
    json.AddItem("enabled",          ds.enabled);
    json.AddItem("armed",            ds.armed);
    json.AddItem("pole_pairs",       pp);
    json.AddItem("voltage_scale",    vs);

    esc_telem_packet_t pkt;
    bool have = esc_telem_get_latest(&pkt);
    bool fresh = esc_telem_is_fresh(500);
    json.AddItem("telem_have",  have);
    json.AddItem("telem_fresh", fresh);
    if (have) {
        json.AddItem("voltage_v",     pkt.voltage_v);
        json.AddItem("temperature_c", static_cast<int>(pkt.temperature_c));
        // AM32 reports eRPM/100 (BLHeli32 convention); mech RPM = raw*100/pole_pairs.
        int erpm_actual = static_cast<int>(pkt.erpm) * 100;
        int mech_rpm = pp > 0 ? erpm_actual / pp : erpm_actual;
        json.AddItem("erpm_raw",      static_cast<int>(pkt.erpm));
        json.AddItem("erpm",          erpm_actual);
        json.AddItem("rpm_mech",      mech_rpm);
        // current_a is reported but the hardware has no current-sense, so the
        // value is the ADC noise floor (~31.9 A). Returned for completeness;
        // don't use it.
        json.AddItem("current_a",     pkt.current_a);
    }

    uint32_t valid = 0, crc_bad = 0, bytes_seen = 0;
    esc_telem_stats(&valid, &crc_bad, &bytes_seen);
    json.AddItem("telem_pkts_valid", static_cast<int>(valid));
    json.AddItem("telem_pkts_bad",   static_cast<int>(crc_bad));
}

esp_err_t EscWebServer::start() {
    esp_err_t r = WebServer::start();
    if (r != ESP_OK) return r;

    struct Route {
        const char    *uri;
        httpd_method_t method;
        esp_err_t (*handler)(httpd_req_t *);
    };
    const std::array<Route, 12> routes = {{
        {"/motor",              HTTP_POST, motor_post_handler},
        {"/motor",              HTTP_GET,  motor_get_handler},
        {"/pump",               HTTP_POST, pump_post_handler},
        {"/pump_range",         HTTP_POST, pump_range_post_handler},
        {"/pump_range",         HTTP_GET,  pump_range_get_handler},
        {"/firmware",           HTTP_POST, firmware_post_handler},
        {"/firmware",           HTTP_GET,  firmware_get_handler},
        {"/config",             HTTP_GET,  config_get_handler},
        {"/config",             HTTP_POST, config_post_handler},
        {"/config/reset",       HTTP_POST, config_reset_post_handler},
        {"/calibrate/pump_min", HTTP_POST, calibrate_pump_min_post_handler},
        {"/esc/reset",          HTTP_POST, esc_reset_post_handler},
    }};

    for (const Route &route : routes) {
        httpd_uri_t uri = {
            .uri      = route.uri,
            .method   = route.method,
            .handler  = route.handler,
            .user_ctx = this,
        };
        esp_err_t err = httpd_register_uri_handler(server, &uri);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "register %s %s: %s",
                route.method == HTTP_POST ? "POST" : "GET",
                route.uri, esp_err_to_name(err));
            return err;
        }
    }
    return ESP_OK;
}

// POST /motor {"throttle":0-2047,"enabled":bool,"pole_pairs":N}
esp_err_t EscWebServer::motor_post_handler(httpd_req_t *req) {
    EscWebServer *self = static_cast<EscWebServer *>(req->user_ctx);
    if (req->content_len > kMaxJsonBodyBytes) return sendJsonError(req, 413, "request body too large");
    std::string body = read_request_body(req);
    if (body.empty()) return sendJsonError(req, 400, "empty body");
    auto json = JsonWrapper::Parse(body);
    if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");

    int throttle;
    if (json.GetField("throttle", throttle)) {
        // 0 disarms; 48..2047 is the real throttle range. 1..47 are DShot
        // command codes (beep/reverse/etc.) and must not arrive at the ESC
        // as throttle — reject up here for a clear error.
        if (throttle < 0 || throttle > 2047 || (throttle > 0 && throttle < 48)) {
            return sendJsonError(req, 400, "throttle out of range; 0 or [48,2047]");
        }
        drive_set_throttle(static_cast<uint16_t>(throttle));
        // Direct throttle command — clear any previously-recorded duty so the
        // display doesn't keep advertising a stale pump %.
        drive_set_duty_pct(-1.0f);
    }
    bool enabled;
    if (json.GetField("enabled", enabled)) {
        drive_set_enabled(enabled);
    }
    int pp;
    if (json.GetField("pole_pairs", pp)) {
        if (pp < 1) pp = 1;
        portENTER_CRITICAL(&self->config_mu_);
        self->pole_pairs_ = pp;
        portEXIT_CRITICAL(&self->config_mu_);
        self->settings_.save_pole_pairs(pp);
        display_set_pole_pairs(pp);
    }

    JsonWrapper resp;
    self->status_to_json(resp);
    return send_json(req, resp);
}

esp_err_t EscWebServer::motor_get_handler(httpd_req_t *req) {
    EscWebServer *self = static_cast<EscWebServer *>(req->user_ctx);
    JsonWrapper resp;
    self->status_to_json(resp);
    return send_json(req, resp);
}

// POST /pump {"duty":0-100} — stillerate-compatible speed control.
// duty<=0.5% disables the drive (treated as off); otherwise maps onto
// [pump_min,pump_max] throttle range and enables.
esp_err_t EscWebServer::pump_post_handler(httpd_req_t *req) {
    EscWebServer *self = static_cast<EscWebServer *>(req->user_ctx);
    if (req->content_len > kMaxJsonBodyBytes) return sendJsonError(req, 413, "request body too large");
    std::string body = read_request_body(req);
    if (body.empty()) return sendJsonError(req, 400, "empty body");
    auto json = JsonWrapper::Parse(body);
    if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");

    float duty;
    if (!json.GetField("duty", duty)) return sendJsonError(req, 400, "missing 'duty'");
    if (duty < 0.0f || duty > 100.0f) return sendJsonError(req, 400, "duty out of range [0,100]");

    std::string name;
    json.GetField("name", name);  // optional, informational

    uint16_t throttle = 0;
    bool enabled;
    if (duty <= kPumpOffDutyPct) {
        drive_set_enabled(false);
        drive_set_duty_pct(0.0f);
        enabled = false;
    } else {
        throttle = self->pump_duty_to_throttle(duty);
        drive_set_throttle(throttle);
        drive_set_duty_pct(duty);
        drive_set_enabled(true);
        enabled = true;
    }

    JsonWrapper resp;
    resp.AddItem("status",        std::string("success"));
    resp.AddItem("received_duty", static_cast<int>(duty + 0.5f));
    resp.AddItem("throttle",      static_cast<int>(throttle));
    resp.AddItem("enabled",       enabled);
    return send_json(req, resp);
}

// POST /pump_range {"min":N,"max":N} — set duty->throttle mapping. Either
// field may be omitted. Persisted to NVS.
esp_err_t EscWebServer::pump_range_post_handler(httpd_req_t *req) {
    EscWebServer *self = static_cast<EscWebServer *>(req->user_ctx);
    if (req->content_len > kMaxJsonBodyBytes) return sendJsonError(req, 413, "request body too large");
    std::string body = read_request_body(req);
    if (body.empty()) return sendJsonError(req, 400, "empty body");
    auto json = JsonWrapper::Parse(body);
    if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");

    portENTER_CRITICAL(&self->config_mu_);
    int new_min = self->pump_min_throttle_;
    int new_max = self->pump_max_throttle_;
    portEXIT_CRITICAL(&self->config_mu_);
    json.GetField("min", new_min);
    json.GetField("max", new_max);

    if (new_min < 0 || new_min > 2047) return sendJsonError(req, 400, "min out of range [0,2047]");
    if (new_max <= new_min || new_max > 2047) return sendJsonError(req, 400, "max must be > min and <=2047");

    portENTER_CRITICAL(&self->config_mu_);
    self->pump_min_throttle_ = static_cast<uint16_t>(new_min);
    self->pump_max_throttle_ = static_cast<uint16_t>(new_max);
    uint16_t lo = self->pump_min_throttle_;
    uint16_t hi = self->pump_max_throttle_;
    portEXIT_CRITICAL(&self->config_mu_);
    if (!self->settings_.save_pump_range(lo, hi)) {
        ESP_LOGW(TAG, "pump range applied to runtime but NVS save failed");
    }

    JsonWrapper resp;
    resp.AddItem("min", static_cast<int>(lo));
    resp.AddItem("max", static_cast<int>(hi));
    return send_json(req, resp);
}

esp_err_t EscWebServer::pump_range_get_handler(httpd_req_t *req) {
    EscWebServer *self = static_cast<EscWebServer *>(req->user_ctx);
    uint16_t lo, hi;
    portENTER_CRITICAL(&self->config_mu_);
    lo = self->pump_min_throttle_;
    hi = self->pump_max_throttle_;
    portEXIT_CRITICAL(&self->config_mu_);
    JsonWrapper resp;
    resp.AddItem("min", static_cast<int>(lo));
    resp.AddItem("max", static_cast<int>(hi));
    return send_json(req, resp);
}

// RFC 1123-ish: 1-32 chars, [a-zA-Z0-9-], no leading/trailing hyphen.
static bool valid_hostname(const std::string &h) {
    if (h.empty() || h.size() > 32) return false;
    if (h.front() == '-' || h.back() == '-') return false;
    for (char c : h) {
        bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
               || (c >= '0' && c <= '9') || c == '-';
        if (!ok) return false;
    }
    return true;
}

void EscWebServer::config_to_json(JsonWrapper &json) {
    uint16_t lo, hi;
    int      pp;
    float    vs;
    portENTER_CRITICAL(&config_mu_);
    lo = pump_min_throttle_;
    hi = pump_max_throttle_;
    pp = pole_pairs_;
    vs = voltage_scale_;
    portEXIT_CRITICAL(&config_mu_);

    json.AddItem("hostname",      settings_.load_hostname("esc-pump"));
    json.AddItem("voltage_scale", vs);
    json.AddItem("pole_pairs",    pp);
    json.AddItem("pump_min",      static_cast<int>(lo));
    json.AddItem("pump_max",      static_cast<int>(hi));
    json.AddItem("voltage_scale_min", Settings::kMinVoltageScale);
    json.AddItem("voltage_scale_max", Settings::kMaxVoltageScale);
}

esp_err_t EscWebServer::config_get_handler(httpd_req_t *req) {
    EscWebServer *self = static_cast<EscWebServer *>(req->user_ctx);
    JsonWrapper resp;
    self->config_to_json(resp);
    return send_json(req, resp);
}

// POST /config {"voltage_scale":1.05,"pole_pairs":6,"pump_min":400,"pump_max":1000}
// All fields optional; only present ones are updated + persisted.
esp_err_t EscWebServer::config_post_handler(httpd_req_t *req) {
    EscWebServer *self = static_cast<EscWebServer *>(req->user_ctx);
    if (req->content_len > kMaxJsonBodyBytes) return sendJsonError(req, 413, "request body too large");
    std::string body = read_request_body(req);
    if (body.empty()) return sendJsonError(req, 400, "empty body");
    auto json = JsonWrapper::Parse(body);
    if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");

    std::string hostname;
    if (json.GetField("hostname", hostname)) {
        if (!valid_hostname(hostname)) {
            return sendJsonError(req, 400, "hostname must be 1-32 chars [a-zA-Z0-9-], no leading/trailing '-'");
        }
        // Apply live so the STA netif advertises it on the next DHCP renew,
        // then persist for next boot. Some DHCP servers only re-read Option 12
        // at full DISCOVER, so a reboot may be needed to refresh the name on
        // the router's lease table.
        esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (sta) {
            esp_err_t e = esp_netif_set_hostname(sta, hostname.c_str());
            if (e != ESP_OK) {
                ESP_LOGW(TAG, "esp_netif_set_hostname: %s", esp_err_to_name(e));
            }
        }
        if (!self->settings_.save_hostname(hostname)) {
            ESP_LOGW(TAG, "hostname applied to runtime but NVS save failed");
        }
    }

    float v_scale;
    if (json.GetField("voltage_scale", v_scale)) {
        if (v_scale < Settings::kMinVoltageScale || v_scale > Settings::kMaxVoltageScale) {
            return sendJsonError(req, 400, "voltage_scale out of range");
        }
        portENTER_CRITICAL(&self->config_mu_);
        self->voltage_scale_ = v_scale;
        portEXIT_CRITICAL(&self->config_mu_);
        esc_telem_set_voltage_scale(v_scale);
        if (!self->settings_.save_voltage_scale(v_scale)) {
            ESP_LOGW(TAG, "voltage_scale applied to runtime but NVS save failed");
        }
    }

    int pp;
    if (json.GetField("pole_pairs", pp)) {
        if (pp < 1) return sendJsonError(req, 400, "pole_pairs must be >= 1");
        portENTER_CRITICAL(&self->config_mu_);
        self->pole_pairs_ = pp;
        portEXIT_CRITICAL(&self->config_mu_);
        self->settings_.save_pole_pairs(pp);
        display_set_pole_pairs(pp);
    }

    portENTER_CRITICAL(&self->config_mu_);
    int new_min = self->pump_min_throttle_;
    int new_max = self->pump_max_throttle_;
    portEXIT_CRITICAL(&self->config_mu_);
    bool range_changed = false;
    if (json.GetField("pump_min", new_min)) range_changed = true;
    if (json.GetField("pump_max", new_max)) range_changed = true;
    if (range_changed) {
        if (new_min < 0 || new_min > 2047) return sendJsonError(req, 400, "pump_min out of range [0,2047]");
        if (new_max <= new_min || new_max > 2047) return sendJsonError(req, 400, "pump_max must be > pump_min and <=2047");
        portENTER_CRITICAL(&self->config_mu_);
        self->pump_min_throttle_ = static_cast<uint16_t>(new_min);
        self->pump_max_throttle_ = static_cast<uint16_t>(new_max);
        uint16_t lo = self->pump_min_throttle_;
        uint16_t hi = self->pump_max_throttle_;
        portEXIT_CRITICAL(&self->config_mu_);
        self->settings_.save_pump_range(lo, hi);
    }

    JsonWrapper resp;
    self->config_to_json(resp);
    return send_json(req, resp);
}

// POST /config/reset {"hostname":"..."}? — wipe every persisted parameter back
// to its default. Body is optional; if present and `hostname` is set, that
// hostname is used in place of the seeded default. Refuses while the motor is
// enabled (resetting pump_min/max under a running drive would jolt the load).
esp_err_t EscWebServer::config_reset_post_handler(httpd_req_t *req) {
    EscWebServer *self = static_cast<EscWebServer *>(req->user_ctx);
    DriveExclusive ex;
    if (!ex.held()) return sendJsonError(req, 409, "another exclusive operation in progress");
    if (drive_get_status().enabled) {
        return sendJsonError(req, 409, "motor must be disabled before /config/reset");
    }

    std::string new_hostname;
    bool wipe_wifi = false;
    if (req->content_len > 0) {
        if (req->content_len > kMaxJsonBodyBytes) return sendJsonError(req, 413, "request body too large");
        std::string body = read_request_body(req);
        if (!body.empty()) {
            auto json = JsonWrapper::Parse(body);
            if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");
            if (json.GetField("hostname", new_hostname)) {
                if (!valid_hostname(new_hostname)) {
                    return sendJsonError(req, 400, "hostname must be 1-32 chars [a-zA-Z0-9-], no leading/trailing '-'");
                }
            }
            json.GetField("wifi", wipe_wifi);
        }
    }

    if (!self->settings_.reset_to_defaults(new_hostname)) {
        ESP_LOGW(TAG, "reset_to_defaults: not all NVS writes succeeded");
    }

    // Mirror persisted defaults into the live state.
    portENTER_CRITICAL(&self->config_mu_);
    self->pump_min_throttle_ = Settings::kDefaultPumpMinThr;
    self->pump_max_throttle_ = Settings::kDefaultPumpMaxThr;
    self->pole_pairs_        = Settings::kDefaultPolePairs;
    self->voltage_scale_     = Settings::kDefaultVoltageScale;
    portEXIT_CRITICAL(&self->config_mu_);
    esc_telem_set_voltage_scale(Settings::kDefaultVoltageScale);
    display_set_pole_pairs(Settings::kDefaultPolePairs);

    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta) {
        const std::string &applied = new_hostname.empty() ? std::string("esc-pump") : new_hostname;
        esp_err_t e = esp_netif_set_hostname(sta, applied.c_str());
        if (e != ESP_OK) ESP_LOGW(TAG, "esp_netif_set_hostname: %s", esp_err_to_name(e));
    }

    ESP_LOGW(TAG, "config reset to defaults (hostname=%s, wipe_wifi=%d)",
        new_hostname.empty() ? "esc-pump" : new_hostname.c_str(), wipe_wifi);

    JsonWrapper resp;
    resp.AddItem("status", std::string(wipe_wifi ? "reset+wifi_clear+reboot" : "reset"));
    resp.AddItem("wifi_cleared", wipe_wifi);
    self->config_to_json(resp);
    esp_err_t r = send_json(req, resp);

    if (wipe_wifi) {
        // esp_wifi_restore() blanks the saved STA config in NVS. The device
        // comes back up unprovisioned, so ESP-Touch v2 / espnow provisioning
        // will pair from scratch. Restart so the running STA session ends
        // and the next boot enters the provisioning path.
        ESP_LOGW(TAG, "wiping wifi credentials and rebooting in 500 ms");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_err_t e = esp_wifi_restore();
        if (e != ESP_OK) ESP_LOGE(TAG, "esp_wifi_restore: %s", esp_err_to_name(e));
        esp_restart();
    }
    return r;
}

// POST /calibrate/pump_min — sweep throttle and watch eRPM telemetry to find
// the lowest reliable drive command for the attached pump+motor under its
// current load.
//
// Default mode is descending: start at pump_max, step down until the rotor
// stalls. Pass {"ascending": true} for a cold-start search that ramps up
// from rest. Both modes return:
//   floor_throttle        — the empirically-determined edge value
//   recommended_pump_min  — floor + margin_pct% (defaults to +5%)
// Pass {"save": true} to persist the recommendation as pump_min in NVS.
//
// Body (all optional):
//   start_throttle  default = current pump_max (descending) or 100 (ascending)
//   step            default 10        — DShot units per step
//   step_ms         default 200       — dwell per step (median needs ~50 ms+)
//   stall_erpm      default 30        — raw AM32 eRPM; pkt.erpm < this = stalled
//   margin_pct      default 5.0       — added to floor for the recommendation
//   min_stop        default 100       — descending sweep gives up below this
//   ascending       default false
//   save            default false
//
// Refuses (409) if the motor is already enabled — calibration owns the drive
// for the duration of the sweep. The drive is force-disabled on every exit.
esp_err_t EscWebServer::calibrate_pump_min_post_handler(httpd_req_t *req) {
    EscWebServer *self = static_cast<EscWebServer *>(req->user_ctx);
    DriveExclusive ex;
    if (!ex.held()) return sendJsonError(req, 409, "another exclusive operation in progress");
    if (drive_get_status().enabled) {
        return sendJsonError(req, 409, "motor must be disabled before /calibrate/pump_min");
    }

    bool ascending = false;
    int start      = -1;
    int step       = 10;
    int step_ms    = 200;
    int stall_erpm = 30;
    float margin_pct = 5.0f;
    int min_stop   = 100;
    bool save      = false;

    if (req->content_len > 0) {
        if (req->content_len > kMaxJsonBodyBytes) return sendJsonError(req, 413, "request body too large");
        std::string body = read_request_body(req);
        if (!body.empty()) {
            auto json = JsonWrapper::Parse(body);
            if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");
            json.GetField("ascending",      ascending);
            json.GetField("start_throttle", start);
            json.GetField("step",           step);
            json.GetField("step_ms",        step_ms);
            json.GetField("stall_erpm",     stall_erpm);
            json.GetField("margin_pct",     margin_pct);
            json.GetField("min_stop",       min_stop);
            json.GetField("save",           save);
        }
    }

    portENTER_CRITICAL(&self->config_mu_);
    uint16_t pump_max_snapshot = self->pump_max_throttle_;
    portEXIT_CRITICAL(&self->config_mu_);
    if (start < 0) start = ascending ? 100 : static_cast<int>(pump_max_snapshot);

    if (start < 48 || start > 2047)         return sendJsonError(req, 400, "start_throttle out of range [48,2047]");
    if (step < 1 || step > 200)             return sendJsonError(req, 400, "step out of range [1,200]");
    if (step_ms < 50 || step_ms > 5000)     return sendJsonError(req, 400, "step_ms out of range [50,5000]");
    if (stall_erpm < 1 || stall_erpm > 10000) return sendJsonError(req, 400, "stall_erpm out of range [1,10000]");
    if (margin_pct < 0.0f || margin_pct > 50.0f) return sendJsonError(req, 400, "margin_pct out of range [0,50]");
    if (min_stop < 48 || min_stop >= start) return sendJsonError(req, 400, "min_stop must be in [48, start_throttle)");

    ESP_LOGW(TAG, "calibrate pump_min: mode=%s start=%d step=%d step_ms=%d stall_erpm=%d margin=%.1f%% save=%d",
        ascending ? "ascending" : "descending", start, step, step_ms, stall_erpm, margin_pct, save);

    SweepResult r;
    if (ascending) {
        r = sweep_ascending(static_cast<uint16_t>(start), static_cast<uint16_t>(step),
                            pump_max_snapshot, static_cast<uint32_t>(step_ms),
                            static_cast<uint16_t>(stall_erpm), margin_pct);
    } else {
        r = sweep_descending(static_cast<uint16_t>(start), static_cast<uint16_t>(step),
                             static_cast<uint32_t>(step_ms), static_cast<uint16_t>(stall_erpm),
                             margin_pct, static_cast<uint16_t>(min_stop));
    }

    if (!r.ok) {
        ESP_LOGW(TAG, "calibrate failed: %s (last_erpm_raw=%u)", r.error, r.last_erpm_raw);
        JsonWrapper err;
        err.AddItem("error",         std::string(r.error ? r.error : "unknown"));
        err.AddItem("last_erpm_raw", static_cast<int>(r.last_erpm_raw));
        httpd_resp_set_status(req, "422 Unprocessable Entity");
        return send_json(req, err);
    }

    bool saved = false;
    const char *save_note = nullptr;
    if (save) {
        // We hold the exclusive lock, so pump_max can't have been overwritten
        // by /config or /pump_range while we were sweeping. Re-snapshot anyway
        // in case /motor or non-exclusive endpoints touched it.
        portENTER_CRITICAL(&self->config_mu_);
        pump_max_snapshot = self->pump_max_throttle_;
        portEXIT_CRITICAL(&self->config_mu_);
        if (r.recommended >= pump_max_snapshot) {
            save_note = "recommendation >= pump_max; not saved (raise pump_max first)";
            ESP_LOGW(TAG, "%s", save_note);
        } else {
            portENTER_CRITICAL(&self->config_mu_);
            self->pump_min_throttle_ = r.recommended;
            uint16_t lo = self->pump_min_throttle_;
            uint16_t hi = self->pump_max_throttle_;
            portEXIT_CRITICAL(&self->config_mu_);
            saved = self->settings_.save_pump_range(lo, hi);
            if (!saved) save_note = "NVS save failed";
        }
    }

    portENTER_CRITICAL(&self->config_mu_);
    uint16_t final_lo = self->pump_min_throttle_;
    uint16_t final_hi = self->pump_max_throttle_;
    portEXIT_CRITICAL(&self->config_mu_);

    ESP_LOGI(TAG, "calibrate done: floor=%u recommended=%u stalled_at=%u last_erpm_raw=%u saved=%d",
        r.floor_throttle, r.recommended, r.stalled_at, r.last_erpm_raw, saved);

    JsonWrapper resp;
    resp.AddItem("mode",                 std::string(ascending ? "ascending" : "descending"));
    resp.AddItem("floor_throttle",       static_cast<int>(r.floor_throttle));
    resp.AddItem("recommended_pump_min", static_cast<int>(r.recommended));
    resp.AddItem("stalled_at",           static_cast<int>(r.stalled_at));
    resp.AddItem("last_erpm_raw",        static_cast<int>(r.last_erpm_raw));
    resp.AddItem("margin_pct",           margin_pct);
    resp.AddItem("saved",                saved);
    if (save_note) resp.AddItem("save_note", std::string(save_note));
    resp.AddItem("pump_min",             static_cast<int>(final_lo));
    resp.AddItem("pump_max",             static_cast<int>(final_hi));
    return send_json(req, resp);
}

// POST /esc/reset — force-disable the drive and replay AM32's boot-time arm
// sequence (1 s disarm hold + 5-beep test). Recovers an ESC that has stopped
// commutating after a too-fast throttle change or fault. If AM32 beeps it's
// listening; if not, power-cycle is the only option left.
//
// Body (optional):
//   timeout_ms  default 12000 — how long to wait for drive_task to report armed
esp_err_t EscWebServer::esc_reset_post_handler(httpd_req_t *req) {
    DriveExclusive ex;
    if (!ex.held()) return sendJsonError(req, 409, "another exclusive operation in progress");

    int timeout_ms = 12000;
    if (req->content_len > 0) {
        if (req->content_len > kMaxJsonBodyBytes) return sendJsonError(req, 413, "request body too large");
        std::string body = read_request_body(req);
        if (!body.empty()) {
            auto json = JsonWrapper::Parse(body);
            if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");
            json.GetField("timeout_ms", timeout_ms);
        }
    }
    if (timeout_ms < 3000 || timeout_ms > 60000) {
        return sendJsonError(req, 400, "timeout_ms out of range [3000,60000]");
    }

    ESP_LOGW(TAG, "ESC reset: forcing disarm and requesting rearm");
    drive_set_enabled(false);
    drive_set_throttle(0);
    drive_set_duty_pct(-1.0f);
    drive_request_rearm();

    TickType_t start   = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    bool armed = false;
    // Poll drive_task's armed flag. It goes false at the start of the arm
    // sequence (via run_arm_sequence) and true again once the beeps finish.
    // First wait for it to drop, then for it to come back up — otherwise a
    // request that arrives before the drive task picks up the atomic would
    // see the still-armed=true from the previous run and return immediately.
    bool saw_disarm = false;
    while ((xTaskGetTickCount() - start) < timeout) {
        DriveStatus ds = drive_get_status();
        if (!ds.armed) saw_disarm = true;
        if (saw_disarm && ds.armed) { armed = true; break; }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    JsonWrapper resp;
    resp.AddItem("status",  std::string(armed ? "rearmed" : "rearm_timeout"));
    resp.AddItem("armed",   armed);
    resp.AddItem("enabled", drive_get_status().enabled);
    if (!armed) {
        resp.AddItem("note", std::string("AM32 did not complete arm sequence; "
                                         "power-cycle the ESC and try again"));
    }
    return send_json(req, resp);
}

// POST /firmware — raw .bin body. Streams into the inactive OTA slot, sets it
// as next boot, reboots. Refuses while the motor is enabled — disable first.
// Deploy: curl --data-binary @build/<project>.bin http://<host>/firmware
esp_err_t EscWebServer::firmware_post_handler(httpd_req_t *req) {
    DriveExclusive ex;
    if (!ex.held()) return sendJsonError(req, 409, "another exclusive operation in progress");
    if (drive_get_status().enabled) {
        return sendJsonError(req, 409, "motor must be disabled before /firmware");
    }
    if (req->content_len <= 0) return sendJsonError(req, 400, "Content-Length required");

    const esp_partition_t *target = esp_ota_get_next_update_partition(nullptr);
    if (target == nullptr) return sendJsonError(req, 500, "no OTA partition available");
    if (req->content_len > static_cast<int>(target->size)) {
        return sendJsonError(req, 413, "image larger than OTA partition");
    }
    ESP_LOGI(TAG, "OTA: writing %d bytes to %s @ 0x%" PRIx32,
        req->content_len, target->label, target->address);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) return sendJsonError(req, 500, esp_err_to_name(err));

    char buf[1024];
    int remaining = req->content_len;
    int written = 0;
    while (remaining > 0) {
        int got = httpd_req_recv(req, buf, std::min<int>(remaining, static_cast<int>(sizeof(buf))));
        if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (got <= 0) {
            esp_ota_abort(handle);
            return sendJsonError(req, 400, "request body truncated");
        }
        err = esp_ota_write(handle, buf, got);
        if (err != ESP_OK) {
            esp_ota_abort(handle);
            return sendJsonError(req, 500, esp_err_to_name(err));
        }
        written   += got;
        remaining -= got;
    }

    err = esp_ota_end(handle);
    if (err != ESP_OK) return sendJsonError(req, 400, esp_err_to_name(err));
    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) return sendJsonError(req, 500, esp_err_to_name(err));

    ESP_LOGW(TAG, "OTA: %d bytes written to %s; rebooting", written, target->label);
    JsonWrapper resp;
    resp.AddItem("status",    std::string("ok"));
    resp.AddItem("written",   written);
    resp.AddItem("partition", std::string(target->label));
    send_json(req, resp);

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

esp_err_t EscWebServer::firmware_get_handler(httpd_req_t *req) {
    const esp_app_desc_t  *desc    = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    JsonWrapper resp;
    resp.AddItem("version",   std::string(desc->version));
    resp.AddItem("idf_ver",   std::string(desc->idf_ver));
    resp.AddItem("date",      std::string(desc->date));
    resp.AddItem("time",      std::string(desc->time));
    resp.AddItem("partition", std::string(running->label));
    return send_json(req, resp);
}
