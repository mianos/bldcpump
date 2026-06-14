#include "Settings.h"

#include <cstdio>
#include <cstdlib>

namespace {

bool parse_int(const std::string &s, int &out) {
    char *end = nullptr;
    long v = std::strtol(s.c_str(), &end, 10);
    if (end == s.c_str()) return false;
    out = static_cast<int>(v);
    return true;
}

}  // namespace

void Settings::load_pump_range(uint16_t &min_throttle, uint16_t &max_throttle) const {
    min_throttle = kDefaultPumpMinThr;
    max_throttle = kDefaultPumpMaxThr;
    std::string s;
    int v;
    if (retrieve("pump_min", s) && !s.empty() && parse_int(s, v) && v >= 0 && v <= 2047) {
        min_throttle = static_cast<uint16_t>(v);
    }
    if (retrieve("pump_max", s) && !s.empty() && parse_int(s, v) && v >= 0 && v <= 2047) {
        max_throttle = static_cast<uint16_t>(v);
    }
}

bool Settings::save_pump_range(uint16_t min_throttle, uint16_t max_throttle) {
    bool ok = true;
    ok &= store("pump_min", std::to_string(min_throttle));
    ok &= store("pump_max", std::to_string(max_throttle));
    return ok;
}

int Settings::load_pole_pairs() const {
    int pole_pairs = kDefaultPolePairs;
    std::string s;
    if (retrieve("pole_pairs", s) && !s.empty()) parse_int(s, pole_pairs);
    return pole_pairs;
}

bool Settings::save_pole_pairs(int pole_pairs) {
    return store("pole_pairs", std::to_string(pole_pairs));
}

float Settings::load_voltage_scale() const {
    std::string s;
    if (!retrieve("v_scale", s) || s.empty()) return kDefaultVoltageScale;
    char *end = nullptr;
    float v = std::strtof(s.c_str(), &end);
    if (end == s.c_str()) return kDefaultVoltageScale;
    if (v < kMinVoltageScale || v > kMaxVoltageScale) return kDefaultVoltageScale;
    return v;
}

bool Settings::save_voltage_scale(float scale) {
    if (scale < kMinVoltageScale) scale = kMinVoltageScale;
    if (scale > kMaxVoltageScale) scale = kMaxVoltageScale;
    char buf[16];
    snprintf(buf, sizeof(buf), "%.4f", scale);
    return store("v_scale", std::string(buf));
}

std::string Settings::load_hostname(const std::string &def) const {
    std::string hn;
    if (retrieve("hostname", hn) && !hn.empty()) return hn;
    return def;
}

bool Settings::save_hostname(const std::string &hostname) {
    return store("hostname", hostname);
}

bool Settings::reset_to_defaults(const std::string &hostname_override) {
    bool ok = true;
    ok &= save_pump_range(kDefaultPumpMinThr, kDefaultPumpMaxThr);
    ok &= save_pole_pairs(kDefaultPolePairs);
    ok &= save_voltage_scale(kDefaultVoltageScale);
    ok &= save_hostname(hostname_override.empty() ? std::string("esc-pump") : hostname_override);
    return ok;
}
