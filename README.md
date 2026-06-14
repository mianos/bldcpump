# ESC Pump Controller

ESP32 firmware that drives an AM32 brushless ESC over DShot600 and exposes a
small HTTP API for control, telemetry, calibration, and OTA. The motor is a
2400 KV BLDC repurposed as a pump; a TTGO T-Display panel shows live status,
and the device joins WiFi via ESP-Touch v2 provisioning.

The default hostname is `esc-pump`; examples here use `reflux` (the deployed
device's hostname).

## Hardware

| Component        | Notes                                                  |
|------------------|--------------------------------------------------------|
| MCU              | ESP32 (TTGO T-Display v1.1, dual-core)                 |
| ESC              | AM32-firmware DShot600, locked into BDshot mode        |
| Motor            | 2400 KV BLDC (12-pole), driving a pump                 |
| Display          | ST7789 240×135 over SPI (TTGO onboard)                 |
| DShot signal     | GPIO 13 (RMT TX, push-pull, inverted line)             |
| Telemetry RX     | GPIO 12 (UART2, 115200 8N1)                            |
| Display backlight| GPIO 4                                                 |

Drive task is pinned to APP_CPU (core 1) so WiFi/lwIP storms on PRO_CPU
can't preempt DShot frame queueing (AM32 disarms after ~250 ms of frame
loss). All status fields are bundled under a portMUX spinlock so callers
get coherent snapshots.

## Build & flash

```sh
. $IDF_PATH/export.sh
idf.py build
idf.py -p /dev/cu.usbserial-XXXX flash monitor    # first time, over USB
curl -X POST --data-binary @build/ESC.bin http://reflux/firmware  # OTA after
```

Partition layout is dual-OTA on 4 MB flash. NVS keeps its standard offset so
WiFi creds, hostname, and pump range survive partition swaps.

## HTTP API

All endpoints return JSON. Errors return `{"error": "...", "statusCode": N}`.

### Realtime control

#### `POST /motor` — raw DShot throttle

```json
{"throttle": 400, "enabled": true, "pole_pairs": 6}
```

| Field        | Type | Range          | Notes                                        |
|--------------|------|----------------|----------------------------------------------|
| `throttle`   | int  | 0 or [48,2047] | 0 disarms; 1–47 are DShot command codes and are rejected |
| `enabled`    | bool | —              | drive output gate; false sends throttle 0    |
| `pole_pairs` | int  | ≥1             | optional; persisted to NVS, also pushed to display |

Returns the same JSON as `GET /motor`. Clears any active `/pump` duty.

#### `GET /motor` — status snapshot

Returns the full status object — drive state, telemetry, packet counters.

```json
{
  "target_throttle": 400, "current_throttle": 400,
  "duty_known": false, "duty_pct": 0,
  "enabled": true, "armed": true,
  "pole_pairs": 6, "voltage_scale": 1.0,
  "telem_have": true, "telem_fresh": true,
  "voltage_v": 13.62, "temperature_c": 38,
  "erpm_raw": 240, "erpm": 24000, "rpm_mech": 4000,
  "current_a": 31.9,
  "telem_pkts_valid": 18432, "telem_pkts_bad": 4
}
```

`current_a` is the AM32 ADC noise floor (~31.9 A) — this ESC has no current
shunt; the field is reported for completeness, **do not trust it**.

#### `POST /pump` — duty-cycle control

```json
{"duty": 35.0, "name": "main_pump"}
```

| Field  | Type   | Range    | Notes                                            |
|--------|--------|----------|--------------------------------------------------|
| `duty` | float  | 0–100    | maps to `[pump_min, pump_max]` throttle          |
| `name` | string | optional | informational, echoed nowhere meaningful         |

`duty ≤ 0.5` disables the drive. Records the duty so the display shows
"35%" instead of just the raw throttle.

### Pump range

#### `GET /pump_range` — current `{min, max}` throttle endpoints.

#### `POST /pump_range`

```json
{"min": 400, "max": 1000}
```

Both fields optional; either can be omitted to keep the existing value.
`max` must be `> min` and `≤ 2047`. Persisted to NVS.

### Config (everything persistent in one place)

#### `GET /config`

```json
{
  "hostname": "reflux",
  "voltage_scale": 1.0,
  "pole_pairs": 6,
  "pump_min": 400, "pump_max": 1000,
  "voltage_scale_min": 0.5, "voltage_scale_max": 2.0
}
```

#### `POST /config` — update any subset

All fields are optional; only the present ones are written.

```json
{
  "hostname": "reflux",
  "voltage_scale": 1.046,
  "pole_pairs": 6,
  "pump_min": 410,
  "pump_max": 1000
}
```

Hostname is applied to the live STA netif and persisted; full DHCP
re-advertise may need a router lease refresh. `voltage_scale` is also
pushed into the live telemetry parser. `pole_pairs` is pushed to the
display so eRPM→mech RPM uses the new value immediately.

#### `POST /config/reset` — wipe to defaults

```json
{"hostname": "reflux", "wifi": true}
```

Both fields optional. `hostname` overrides the default `"esc-pump"`.
`wifi: true` also calls `esp_wifi_restore()` (wipes saved STA creds) and
reboots — the device comes back unprovisioned, ready for ESP-Touch.

Refuses (409) while the motor is enabled, or if another exclusive
operation is in progress.

### Calibration

#### `POST /calibrate/pump_min` — empirically find the lowest reliable throttle

Default mode is **descending**: starts at `pump_max`, steps down until the
rotor stalls (eRPM falls below `stall_erpm`). The last throttle that kept
the rotor spinning is the floor; the recommendation is `floor × (1 + margin_pct/100)`.

```json
{
  "ascending": false,
  "start_throttle": 1000,
  "step": 10,
  "step_ms": 200,
  "stall_erpm": 30,
  "margin_pct": 5.0,
  "min_stop": 100,
  "save": false
}
```

| Field            | Default                  | Range       |
|------------------|--------------------------|-------------|
| `ascending`      | `false`                  | bool        |
| `start_throttle` | `pump_max` / 100         | [48, 2047]  |
| `step`           | 10                       | [1, 200]    |
| `step_ms`        | 200                      | [50, 5000]  |
| `stall_erpm`     | 30 (AM32 raw, ÷100)      | [1, 10000]  |
| `margin_pct`     | 5.0                      | [0, 50]     |
| `min_stop`       | 100                      | [48, start) |
| `save`           | `false`                  | bool        |

`ascending: true` ramps **up** from rest until the rotor first starts spinning
— this is the cold-start floor, often higher than the descending floor.

Response includes `floor_throttle`, `recommended_pump_min`, `stalled_at`,
`last_erpm_raw`. With `"save": true` the recommendation is written into
`pump_min`. Refuses (409) while the motor is enabled or another exclusive
operation is in progress.

### Recovery

#### `POST /esc/reset` — re-arm the AM32

Force-disables the drive and replays the boot-time arm hold + 5-beep test.
Recovers an ESC that has stopped commutating after a too-fast throttle step
or transient fault. If AM32 beeps, it's listening again.

```json
{"timeout_ms": 12000}
```

`timeout_ms` (default 12000, range [3000, 60000]) is how long the handler
waits for the drive task to report `armed: true` again. Response:

```json
{"status": "rearmed", "armed": true, "enabled": false}
```

Returns `"rearm_timeout"` and `armed: false` if the sequence didn't
complete — in that case, power-cycle the ESC.

### OTA

#### `GET /firmware` — running image info

```json
{
  "version": "...", "idf_ver": "...",
  "date": "...", "time": "...",
  "partition": "ota_0"
}
```

#### `POST /firmware` — upload `.bin`, reboot

```sh
curl -X POST --data-binary @build/ESC.bin http://reflux/firmware
```

The `@` prefix is required — without it curl sends the literal filename
as the body and `esp_ota_end` returns `ESP_ERR_OTA_VALIDATE_FAILED`.

Refuses (409) while the motor is enabled. Rejects (413) images larger
than the OTA partition. On success: writes to the inactive slot, sets it
as next boot, restarts. Rollback is armed for ~10 s after boot; once
WiFi + webserver come up, `esp_ota_mark_app_valid_cancel_rollback()`
commits the image.

## Settings reference

All settings live in NVS namespace `storage`. Defaults are baked into
`Settings.h`.

| Key            | Type   | Default     | Range         | Meaning                                                           |
|----------------|--------|-------------|---------------|-------------------------------------------------------------------|
| `hostname`     | string | `esc-pump`  | 1–32 chars    | DHCP/mDNS hostname. `[a-zA-Z0-9-]`, no leading/trailing `-`       |
| `pump_min`     | uint16 | 400         | [0, 2047]     | DShot throttle at `duty=0%`                                       |
| `pump_max`     | uint16 | 1000        | (min, 2047]   | DShot throttle at `duty=100%`                                     |
| `pole_pairs`   | int    | 6           | ≥1            | Magnet pole pairs (12-pole motor = 6). Used everywhere eRPM → mech RPM |
| `v_scale`      | float  | 1.0         | [0.5, 2.0]    | Multiplier applied to AM32's reported voltage; trims its uncalibrated divider |

### Notes on individual settings

**`pump_min` / `pump_max`** — the `/pump duty:N` endpoint maps `0..100` linearly
onto `[pump_min, pump_max]`. Below ~0.5% duty the drive is disabled
entirely. Calibrate via `/calibrate/pump_min` rather than guessing; the
floor depends on the pump load.

**`pole_pairs`** — wrong value doesn't break anything, but RPM readings on
the display and in `/motor` JSON are off by a constant factor. For a 12-pole
motor (typical for racing BLDCs in this size class) the value is 6:
`mech_rpm = erpm_raw × 100 / pole_pairs`.

**`v_scale`** — AM32's onboard voltage divider is uncalibrated. Pick a
correction factor by comparing the reported voltage to a known reference
(bench PSU display, multimeter): `v_scale = true_voltage / reported_voltage`.
Applied at packet parse time, before the plausibility filter and the median
ring, so the median converges to the new factor over ~N packets.

**`hostname`** — applied live to the STA netif on update, and persisted
for next boot. Some DHCP servers only re-read Option 12 on a full DISCOVER,
so a router-side refresh may need a reboot.

## AM32 quirks worth knowing

- **BDshot only** — this ESC is locked into BDshot. Standard DShot won't arm
  it. The firmware sends inverted line + inverted CRC; the BDshot return
  frame is *not* decoded (push-pull line makes that impossible), but UART
  telemetry on GPIO 12 still works.
- **Non-standard TLM pad** — the dedicated TLM pad on the ESC outputs something
  other than BLHeli32 telemetry. Use the UART RX path documented above.
- **3D mode** — if the AM32 is configured for 3D mode, throttle 0 = disarm and
  1048 is neutral; `[48, 1047]` is reverse and `[1049, 2047]` is forward.
  Pick `pump_min`/`pump_max` on the correct side of 1048 for the direction
  you want.
- **Arm sequence** — boot runs a 1-second disarm hold then the 5-beep test
  (DShot commands 1–5). AM32 needs this to leave config/idle mode and
  start commutating. `/esc/reset` replays it on demand.
