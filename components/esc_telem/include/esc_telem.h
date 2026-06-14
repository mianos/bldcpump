#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// One-wire UART telemetry from the AM32 ESC (115200 baud, 8N1, AM32 TX -> ESP RX).
// AM32 sends one 10-byte packet for each DShot frame that has telemetry_req=1.
// Packet layout (BLHeli32 / KISS standard):
//   [0]   temperature (degC, uint8)
//   [1-2] voltage (centivolts, big-endian uint16)        -> volts = value / 100
//   [3-4] current (centiamps, big-endian uint16)         -> amps  = value / 100
//   [5-6] consumption (mAh used, big-endian uint16)
//   [7-8] eRPM (big-endian uint16)                       -> RPM = eRPM * 100 / pole_pairs
//   [9]   CRC8 (polynomial 0xD5)

typedef struct {
    uint8_t  temperature_c;
    float    voltage_v;
    float    current_a;
    uint16_t consumption_mah;
    uint16_t erpm;
    uint64_t timestamp_us;
} esc_telem_packet_t;

// Set up UART listener on the given RX GPIO. Spawns an internal task that parses
// 10-byte packets, validates CRC, and stores the latest valid packet for readout.
esp_err_t esc_telem_init(int rx_gpio);

// Returns true and fills `out` if a valid packet has been received. Returns false
// if no packet has ever arrived (or only invalid CRCs so far).
bool esc_telem_get_latest(esc_telem_packet_t *out);

// Cumulative stats for diagnostics.
void esc_telem_stats(uint32_t *valid, uint32_t *crc_bad, uint32_t *bytes_seen);

// True if a valid telemetry packet was received within the last `max_age_ms`.
// More useful than "is the parser locked?" because AM32 mixes 10-byte telemetry
// packets with filler bytes — alignment-based locking misfires, but
// "saw-a-valid-packet-recently" is a clean health signal.
bool esc_telem_is_fresh(uint32_t max_age_ms);

// Scale applied to the raw voltage decoded from AM32 packets, before the
// plausibility filter and median ring. Default is 1.0 (no correction). AM32's
// onboard voltage divider is uncalibrated, so this lets a higher-level layer
// trim it against a known reference. Existing ring values are NOT rescaled
// retroactively — the median converges to the new factor over the next N packets.
void esc_telem_set_voltage_scale(float scale);
float esc_telem_get_voltage_scale(void);

#ifdef __cplusplus
}
#endif
