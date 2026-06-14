#include "drive.h"

#include <atomic>

#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dshot_esc_encoder.h"
#include "esc_telem.h"
#include "display.h"

namespace {

constexpr const char *TAG = "drive";

constexpr uint32_t DSHOT_BAUD_RATE      = 600000;
#if CONFIG_IDF_TARGET_ESP32H2
constexpr uint32_t DSHOT_RESOLUTION_HZ  = 32000000;
#else
constexpr uint32_t DSHOT_RESOLUTION_HZ  = 40000000;
#endif

// All status fields live in one struct guarded by a portMUX spinlock so callers
// see a coherent snapshot. portMUX is a CAS spinlock with no context switch —
// the per-frame overhead at 1 kHz is negligible.
portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
DriveStatus  s_status     = {
    .target_throttle  = 0,
    .current_throttle = 0,
    .duty_pct         = -1.0f,
    .enabled          = false,
    .armed            = false,
};

std::atomic<bool> s_rearm_request{false};

// Single-slot mutex for long-running drive operations. compare_exchange so a
// second caller fails fast instead of waiting.
std::atomic<bool> s_exclusive{false};

rmt_channel_handle_t  s_chan = nullptr;
rmt_encoder_handle_t  s_enc  = nullptr;

void send_one(uint16_t throttle, const rmt_transmit_config_t &tx) {
    dshot_esc_throttle_t sig = { .throttle = throttle, .telemetry_req = true };
    ESP_ERROR_CHECK(rmt_transmit(s_chan, s_enc, &sig, sizeof(sig), &tx));
    rmt_tx_wait_all_done(s_chan, -1);
    portENTER_CRITICAL(&s_status_mux);
    s_status.current_throttle = throttle;
    portEXIT_CRITICAL(&s_status_mux);
    display_set_throttle(throttle);
}

void hold_for(uint16_t throttle, TickType_t ticks, const rmt_transmit_config_t &tx) {
    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < ticks) {
        send_one(throttle, tx);
        vTaskDelay(1);
    }
}

void run_arm_sequence(const rmt_transmit_config_t &tx_config) {
    portENTER_CRITICAL(&s_status_mux);
    s_status.armed           = false;
    s_status.enabled         = false;
    s_status.target_throttle = 0;
    portEXIT_CRITICAL(&s_status_mux);

    ESP_LOGI(TAG, "Arming hold at throttle 0 for 1s");
    hold_for(0, pdMS_TO_TICKS(1000), tx_config);
    ESP_LOGI(TAG, "Armed");

    // DShot command 1..5 beep test. AM32 needs this to leave config/idle mode
    // and start commutating. ~10 repeated frames per command, then a disarm
    // pause so each beep is audible distinctly.
    const uint16_t beep_cmds[] = { 1, 2, 3, 4, 5 };
    for (uint16_t cmd : beep_cmds) {
        ESP_LOGI(TAG, "Beep cmd %u", cmd);
        for (int n = 0; n < 10; n++) {
            send_one(cmd, tx_config);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        hold_for(0, pdMS_TO_TICKS(800), tx_config);
    }
    ESP_LOGI(TAG, "Beep sequence done; drive loop live");
    portENTER_CRITICAL(&s_status_mux);
    s_status.armed = true;
    portEXIT_CRITICAL(&s_status_mux);
}

void drive_task(void *arg) {
    rmt_transmit_config_t tx_config = {};
    tx_config.loop_count = 0;

    bool first = true;
    while (true) {
        // Run the arm+beep sequence at boot, and any time the web layer
        // requests a re-arm. exchange() atomically clears the request so a
        // request that lands mid-sequence isn't lost — it triggers a second
        // arm pass on the next loop.
        bool rearm = s_rearm_request.exchange(false);
        if (first || rearm) {
            if (rearm) ESP_LOGW(TAG, "rearm requested — replaying arm sequence");
            run_arm_sequence(tx_config);
            first = false;
        }

        // Live drive loop. enabled=false holds disarm; enabled=true sends the
        // current target. enabled+target are read together so a concurrent
        // enable doesn't combine with an old throttle.
        uint16_t t;
        portENTER_CRITICAL(&s_status_mux);
        t = s_status.enabled ? s_status.target_throttle : 0;
        portEXIT_CRITICAL(&s_status_mux);
        send_one(t, tx_config);
        vTaskDelay(1);
    }
}

}  // namespace

esp_err_t drive_init(int dshot_gpio) {
    ESP_LOGI(TAG, "RMT TX (BDshot push-pull) on GPIO %d", dshot_gpio);
    rmt_tx_channel_config_t tx_chan_config = {};
    tx_chan_config.gpio_num          = static_cast<gpio_num_t>(dshot_gpio);
    tx_chan_config.clk_src           = RMT_CLK_SRC_DEFAULT;
    tx_chan_config.resolution_hz     = DSHOT_RESOLUTION_HZ;
    tx_chan_config.mem_block_symbols = 64;
    tx_chan_config.trans_queue_depth = 10;
    // AM32 on this ESC is locked into BDshot — needs inverted line + inverted CRC.
    // Push-pull (not open-drain) means the BDshot return frame can't be decoded
    // on this line, but UART telemetry on GPIO 12 still works.
    tx_chan_config.flags.invert_out  = true;
    esp_err_t err = rmt_new_tx_channel(&tx_chan_config, &s_chan);
    if (err != ESP_OK) return err;

    dshot_esc_encoder_config_t encoder_config = {
        .resolution    = DSHOT_RESOLUTION_HZ,
        .baud_rate     = DSHOT_BAUD_RATE,
        .post_delay_us = 50,
        .bidirectional = true,
    };
    err = rmt_new_dshot_esc_encoder(&encoder_config, &s_enc);
    if (err != ESP_OK) return err;

    err = rmt_enable(s_chan);
    if (err != ESP_OK) return err;

    // Pin the drive task to APP_CPU (core 1). WiFi/lwIP run on PRO_CPU (core 0)
    // by default, so this keeps WiFi storms from preempting DShot frame
    // queueing. The RMT peripheral handles bit-level timing in hardware; what
    // we're protecting here is the inter-frame cadence (AM32 disarms after
    // ~250 ms of frame loss).
    BaseType_t ok = xTaskCreatePinnedToCore(drive_task, "drive", 4096, nullptr,
                                            tskIDLE_PRIORITY + 2, nullptr,
                                            APP_CPU_NUM);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void drive_set_throttle(uint16_t throttle) {
    if (throttle > 2047) throttle = 2047;
    // DShot 0 = disarm; 1..47 are command codes (beep, reverse, set-direction,
    // etc.) and would make the ESC do something other than commutate. Coerce
    // to disarm so a stray value can't sneak past handler validation. Real
    // throttle starts at 48.
    if (throttle > 0 && throttle < 48) throttle = 0;
    portENTER_CRITICAL(&s_status_mux);
    s_status.target_throttle = throttle;
    portEXIT_CRITICAL(&s_status_mux);
}

void drive_set_enabled(bool enabled) {
    portENTER_CRITICAL(&s_status_mux);
    s_status.enabled = enabled;
    portEXIT_CRITICAL(&s_status_mux);
}

void drive_request_rearm(void) {
    s_rearm_request.store(true);
}

void drive_set_duty_pct(float duty_pct) {
    portENTER_CRITICAL(&s_status_mux);
    s_status.duty_pct = duty_pct;
    portEXIT_CRITICAL(&s_status_mux);
    display_set_duty(duty_pct);
}

DriveStatus drive_get_status(void) {
    DriveStatus st;
    portENTER_CRITICAL(&s_status_mux);
    st = s_status;
    portEXIT_CRITICAL(&s_status_mux);
    return st;
}

bool drive_try_acquire_exclusive(void) {
    bool expected = false;
    return s_exclusive.compare_exchange_strong(expected, true);
}

void drive_release_exclusive(void) {
    s_exclusive.store(false);
}
