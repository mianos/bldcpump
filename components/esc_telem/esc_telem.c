#include <string.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esc_telem.h"

#define ESC_UART_NUM    UART_NUM_2
#define ESC_UART_BAUD   115200
#define UART_BUF_BYTES  256
#define PACKET_SIZE     10

static const char *TAG = "esc_telem";

#define MEDIAN_N 11 // median over last N valid packets — needs >5 in a row to move
static SemaphoreHandle_t s_mutex = NULL;
static esc_telem_packet_t s_latest = {0};
static esc_telem_packet_t s_history[MEDIAN_N];
static int  s_history_count = 0;  // how many history slots are populated
static int  s_history_idx   = 0;  // next slot to write
static bool s_have_packet = false;

static _Atomic uint32_t s_valid_count = 0;
static _Atomic uint32_t s_crc_bad_count = 0;
static _Atomic uint32_t s_bytes_seen = 0;
static _Atomic float    s_voltage_scale;  // initialised to 1.0f in esc_telem_init

// AM32's serial_telemetry CRC8 (polynomial 0x07, XOR-then-shift). Matches the source
// at Mcu/f051/Src/serial_telemetry.c in AlkaMotors/AM32-MultiRotor-ESC-firmware.
static uint8_t crc8_byte(uint8_t crc, uint8_t b)
{
    crc ^= b;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    }
    return crc;
}

static uint8_t crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) crc = crc8_byte(crc, data[i]);
    return crc;
}

// Insertion sort a small array — O(n²) but n is tiny and we want zero deps.
static void sort_f(float *a, int n) {
    for (int i = 1; i < n; i++) {
        float v = a[i]; int j = i;
        while (j > 0 && a[j-1] > v) { a[j] = a[j-1]; j--; }
        a[j] = v;
    }
}
static void sort_u16(uint16_t *a, int n) {
    for (int i = 1; i < n; i++) {
        uint16_t v = a[i]; int j = i;
        while (j > 0 && a[j-1] > v) { a[j] = a[j-1]; j--; }
        a[j] = v;
    }
}
static void sort_u8(uint8_t *a, int n) {
    for (int i = 1; i < n; i++) {
        uint8_t v = a[i]; int j = i;
        while (j > 0 && a[j-1] > v) { a[j] = a[j-1]; j--; }
        a[j] = v;
    }
}

// Compute per-field median across the populated history slots. Updates `s_latest`.
// Mutex must be held by caller.
static void update_median_locked(void)
{
    int n = s_history_count;
    if (n == 0) { s_have_packet = false; return; }

    float vs[MEDIAN_N], is[MEDIAN_N];
    uint16_t mahs[MEDIAN_N], erpms[MEDIAN_N];
    uint8_t  temps[MEDIAN_N];
    for (int i = 0; i < n; i++) {
        vs[i]    = s_history[i].voltage_v;
        is[i]    = s_history[i].current_a;
        mahs[i]  = s_history[i].consumption_mah;
        erpms[i] = s_history[i].erpm;
        temps[i] = s_history[i].temperature_c;
    }
    sort_f(vs, n); sort_f(is, n);
    sort_u16(mahs, n); sort_u16(erpms, n);
    sort_u8(temps, n);

    s_latest.voltage_v       = vs[n/2];
    s_latest.current_a       = is[n/2];
    s_latest.consumption_mah = mahs[n/2];
    s_latest.erpm            = erpms[n/2];
    s_latest.temperature_c   = temps[n/2];
    // timestamp_us reflects the most recent packet that contributed
    s_latest.timestamp_us    = esp_timer_get_time();
    s_have_packet = true;
}

static void try_parse_packet(const uint8_t *p)
{
    uint8_t expected = crc8(p, 9);
    if (expected != p[9]) {
        atomic_fetch_add(&s_crc_bad_count, 1);
        return;
    }
    float scale = atomic_load(&s_voltage_scale);
    esc_telem_packet_t pkt = {
        .temperature_c   = p[0],
        .voltage_v       = (((p[1] << 8) | p[2]) / 100.0f) * scale,
        .current_a       = ((p[3] << 8) | p[4]) / 100.0f,
        .consumption_mah = (p[5] << 8) | p[6],
        .erpm            = (p[7] << 8) | p[8],
        .timestamp_us    = esp_timer_get_time(),
    };
    // Plausibility filter — reject false-positive CRC matches that landed in
    // physically impossible ranges. With an 8-bit CRC, ~0.4% of misaligned
    // sliding-window positions match by chance; without this filter they
    // splash into the display.
    if (pkt.voltage_v < 5.0f  || pkt.voltage_v > 30.0f) goto bogus;
    if (pkt.current_a < 0.0f  || pkt.current_a > 100.0f) goto bogus;
    if (pkt.temperature_c < 10 || pkt.temperature_c > 120) goto bogus;
    // AM32 reports eRPM/100 — raw values above ~3000 mean >300k actual eRPM, impossible.
    if (pkt.erpm > 3000) goto bogus;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    // Push into ring buffer, then recompute median across populated slots.
    s_history[s_history_idx] = pkt;
    s_history_idx = (s_history_idx + 1) % MEDIAN_N;
    if (s_history_count < MEDIAN_N) s_history_count++;
    update_median_locked();
    xSemaphoreGive(s_mutex);
    atomic_fetch_add(&s_valid_count, 1);
    return;

bogus:
    atomic_fetch_add(&s_crc_bad_count, 1);
}

static void rx_task(void *arg)
{
    ESP_LOGI(TAG, "rx_task running (1-byte locking reader)");

    // Locking reader:
    // - Maintain a 10-byte window. Read 1 byte at a time into it.
    // - When the window is full, try CRC.
    //   * Valid  → next packet starts fresh: clear window to empty.
    //   * Invalid → drop the oldest byte (shift left), so we try the next alignment.
    // Once aligned, every 10 bytes is a clean packet. No bytes are ever lost on
    // partial UART reads (unlike block-mode read).

    uint8_t window[PACKET_SIZE];
    int filled = 0;
    int64_t last_stats_us = esp_timer_get_time();

    while (1) {
        uint8_t b;
        int n = uart_read_bytes(ESC_UART_NUM, &b, 1, pdMS_TO_TICKS(100));
        if (n == 1) {
            atomic_fetch_add(&s_bytes_seen, 1);
            window[filled++] = b;
            if (filled == PACKET_SIZE) {
                uint32_t before_good = atomic_load(&s_valid_count);
                try_parse_packet(window);
                if (atomic_load(&s_valid_count) > before_good) {
                    filled = 0; // locked: next read starts a fresh packet
                } else {
                    memmove(window, window + 1, PACKET_SIZE - 1);
                    filled = PACKET_SIZE - 1;
                }
            }
        }

        int64_t now = esp_timer_get_time();
        if (now - last_stats_us > 2000000) {
            uint32_t valid = atomic_load(&s_valid_count);
            static uint32_t prev_valid = 0;
            uint32_t delta = valid - prev_valid;
            prev_valid = valid;
            esc_telem_packet_t latest;
            if (esc_telem_get_latest(&latest)) {
                int64_t age_ms = (esp_timer_get_time() - latest.timestamp_us) / 1000;
                // Last-known-good values stick in the median ring forever; once
                // they stop being current, print STALE so the log doesn't read
                // like the ESC is still running on cached numbers.
                if (age_ms < 1000) {
                    ESP_LOGI(TAG, "pkts: %lu/s total=%lu | V=%.2f I=%.2f T=%dC eRPM=%u (age %lldms)",
                             (unsigned long)(delta / 2), (unsigned long)valid,
                             latest.voltage_v, latest.current_a, latest.temperature_c,
                             latest.erpm, (long long)age_ms);
                } else {
                    ESP_LOGI(TAG, "pkts: 0/s total=%lu STALE (age %lldms)",
                             (unsigned long)valid, (long long)age_ms);
                }
            } else {
                ESP_LOGI(TAG, "pkts: 0/s no telemetry yet");
            }
            last_stats_us = now;
        }
    }
}

esp_err_t esc_telem_init(int rx_gpio)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;
    atomic_store(&s_voltage_scale, 1.0f);

    uart_config_t cfg = {
        .baud_rate  = ESC_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(ESC_UART_NUM, UART_BUF_BYTES, 0, 0, NULL, 0),
                        TAG, "driver install");
    ESP_RETURN_ON_ERROR(uart_param_config(ESC_UART_NUM, &cfg), TAG, "param config");
    ESP_RETURN_ON_ERROR(uart_set_pin(ESC_UART_NUM, UART_PIN_NO_CHANGE, rx_gpio,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "set pin");

    xTaskCreate(rx_task, "esc_telem", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    ESP_LOGI(TAG, "ESC telemetry RX listening on GPIO %d (UART2, 115200 8N1)", rx_gpio);
    return ESP_OK;
}

bool esc_telem_get_latest(esc_telem_packet_t *out)
{
    if (!s_mutex) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool have = s_have_packet;
    if (have && out) *out = s_latest;
    xSemaphoreGive(s_mutex);
    return have;
}

void esc_telem_stats(uint32_t *valid, uint32_t *crc_bad, uint32_t *bytes_seen)
{
    if (valid)      *valid      = atomic_load(&s_valid_count);
    if (crc_bad)    *crc_bad    = atomic_load(&s_crc_bad_count);
    if (bytes_seen) *bytes_seen = atomic_load(&s_bytes_seen);
}

bool esc_telem_is_fresh(uint32_t max_age_ms)
{
    if (!s_have_packet) return false;
    int64_t now = esp_timer_get_time();
    int64_t age_us = now - (int64_t)s_latest.timestamp_us;
    return age_us >= 0 && age_us < (int64_t)max_age_ms * 1000;
}

void esc_telem_set_voltage_scale(float scale)
{
    atomic_store(&s_voltage_scale, scale);
}

float esc_telem_get_voltage_scale(void)
{
    return atomic_load(&s_voltage_scale);
}
