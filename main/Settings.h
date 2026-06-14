#pragma once

#include <cstdint>
#include <string>

#include "NvsStorageManager.h"

// ESC-local persisted settings, layered on the shared NvsStorageManager
// (string key/value). Typed accessors + NVS keys + defaults all in one place.
class Settings : public NvsStorageManager {
public:
    explicit Settings(const std::string &ns = "storage") : NvsStorageManager(ns) {}

    // Pump duty->throttle mapping range (DShot 11-bit throttle, 48..2047). The
    // ramp loop wrote 400..1000 — those become the defaults. Missing keys
    // fall back to defaults.
    void load_pump_range(uint16_t &min_throttle, uint16_t &max_throttle) const;
    bool save_pump_range(uint16_t min_throttle, uint16_t max_throttle);

    // Motor pole-pair count (RPM display + JSON status). Returns default if unset.
    int  load_pole_pairs() const;
    bool save_pole_pairs(int pole_pairs);

    // Voltage scale factor applied to AM32's reported voltage. AM32's onboard
    // divider is uncalibrated; this trims it. true_voltage = reported * scale.
    // Clamped to [kMinVoltageScale, kMaxVoltageScale] on save.
    float load_voltage_scale() const;
    bool  save_voltage_scale(float scale);

    // Device hostname. Returns `def` if unset/empty.
    std::string load_hostname(const std::string &def) const;
    bool save_hostname(const std::string &hostname);

    // Overwrite every persisted parameter with its default. Hostname is set
    // to `hostname_override` if non-empty, otherwise to "esc-pump". Returns
    // true if all writes succeeded.
    bool reset_to_defaults(const std::string &hostname_override = "");

    static constexpr uint16_t kDefaultPumpMinThr   = 400;
    static constexpr uint16_t kDefaultPumpMaxThr   = 1000;
    static constexpr int      kDefaultPolePairs    = 6;     // 12-pole motor
    static constexpr float    kDefaultVoltageScale = 1.0f;
    static constexpr float    kMinVoltageScale     = 0.5f;
    static constexpr float    kMaxVoltageScale     = 2.0f;
};
