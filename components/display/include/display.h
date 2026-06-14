#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the LCD and starts the internal refresh task. The task computes uptime from
// esp_timer and renders it; throttle is pushed in via display_set_throttle().
esp_err_t display_init(void);

void display_set_throttle(uint16_t throttle);

// Push the last commanded /pump duty (0-100). Pass a negative value to mark
// "unknown" (direct throttle command, no duty mapping) — the screen will
// show "--%" in that case.
void display_set_duty(float duty_pct);

// Push the hostname + IP shown at the bottom of the screen on two centered
// lines. Long hostnames are ellipsized to fit the panel width. Pass NULL or
// empty strings to fall back to "no wifi" / blank. The strings are copied
// internally; the caller doesn't need to keep them alive.
void display_set_network(const char *host, const char *ip);

// Override the pole-pair count used to convert eRPM to mechanical RPM on the
// display. Default is 6 (12-pole motor). Caller loads from NVS at boot and
// pushes again when /config or /motor changes pole_pairs. Values < 1 are
// clamped to 1 (avoid divide-by-zero).
void display_set_pole_pairs(int pole_pairs);

#ifdef __cplusplus
}
#endif
