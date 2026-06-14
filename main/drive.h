#pragma once

#include <cstdint>

#include "esp_err.h"

// Owns the RMT TX channel, the DShot encoder, and the FreeRTOS task that streams
// DShot frames to the ESC. Sets up BDshot push-pull (inverted line + inverted CRC),
// runs the arm hold + beep test once at boot, then enters the live drive loop.
//
// Once running, the loop reads two atomics on every frame:
//   - enabled: false  -> send throttle 0 (disarm)
//   - enabled: true   -> send target_throttle (clamped to [0, 2047])
// telemetry_req is always set so AM32 streams the UART telemetry packets.

struct DriveStatus {
    uint16_t target_throttle;   // requested by caller (web, ramp, etc.)
    uint16_t current_throttle;  // value actually being transmitted right now
    float    duty_pct;          // last /pump duty (0-100); -1 if direct throttle
    bool     enabled;
    bool     armed;             // arm+beep sequence complete, loop is live
};

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t drive_init(int dshot_gpio);
// Throttle is clamped to [0, 2047]; values in [1, 47] are DShot command codes
// (beep/reverse/etc.) and get coerced to 0 so they can never reach the ESC as
// throttle. Use drive_request_rearm() to trigger the beep sequence.
void      drive_set_throttle(uint16_t throttle);
void      drive_set_enabled(bool enabled);
// Force the drive task to replay its boot-time arm hold + 5-beep test.
// Use to recover an AM32 that has gone unresponsive (over-fast throttle
// step, sync loss, etc.). Sets enabled=false and armed=false while the
// sequence runs; armed becomes true once AM32 finishes the beep test.
void      drive_request_rearm(void);
// Record the last /pump duty (0-100). Caller (EscWebServer) sets this so the
// display + status JSON can show a human-friendly figure alongside the raw
// throttle. Pass a negative value to indicate "direct throttle, no duty known".
void      drive_set_duty_pct(float duty_pct);
// Coherent snapshot of every status field — all five fields are sampled under
// one critical section, so a caller never sees enabled=true mid-flip with a
// stale throttle.
DriveStatus drive_get_status(void);

// Cooperative exclusive-access lock for long-running drive operations (OTA,
// calibration, /config/reset, /esc/reset). Returns true if the caller obtained
// the lock; false means another exclusive op is in progress and the caller
// should bail with 409. drive_release_exclusive() MUST be called once the
// operation finishes (success or failure). Plain /motor and /pump commands
// don't take this lock — they're momentary; the lock is just to keep two
// long ops from clobbering each other.
bool      drive_try_acquire_exclusive(void);
void      drive_release_exclusive(void);

#ifdef __cplusplus
}
#endif
