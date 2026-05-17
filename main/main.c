#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"

#define DSHOT_GPIO_NUM           18
#define DSHOT_BAUD_RATE          600000    // DSHOT600

#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_RESOLUTION_HZ      32000000
#else
#define DSHOT_RESOLUTION_HZ      40000000
#endif

static const char *TAG = "dshot_app";

void app_main(void)
{
    ESP_LOGI(TAG, "Configuring RMT TX Channel...");
    rmt_channel_handle_t esc_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = DSHOT_GPIO_NUM,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &esc_chan));

    ESP_LOGI(TAG, "Allocating DShot ESC Encoder...");
    rmt_encoder_handle_t dshot_encoder = NULL;
    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_RESOLUTION_HZ,
        .baud_rate = DSHOT_BAUD_RATE,
        .post_delay_us = 50,
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));

    ESP_LOGI(TAG, "Enabling RMT Channel...");
    ESP_ERROR_CHECK(rmt_enable(esc_chan));

    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    dshot_esc_throttle_t motor_signal = { .throttle = 0, .telemetry_req = false };

    // Hold throttle 48 for 3s so the ESC can finish its boot tones and arm.
    // This ESC arms on throttle 48 (its lowest valid motor command), not on
    // throttle 0 - confirmed non-conformant with the DShot spec but workable.
    // Motor spins at minimum speed during this hold.
    ESP_LOGI(TAG, "Arming hold at throttle 48 for 3s");
    motor_signal.throttle = 48;
    TickType_t arm_start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - arm_start) < pdMS_TO_TICKS(3000)) {
        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &motor_signal, sizeof(motor_signal), &tx_config));
        rmt_tx_wait_all_done(esc_chan, -1);
    }
    ESP_LOGI(TAG, "Armed.");

    // 20-step ramp from 100 to 150 and back.
    const uint16_t RAMP_LOW  = 100;
    const uint16_t RAMP_HIGH = 150;
    const int RAMP_STEPS = 20;
    const TickType_t hold_ticks = pdMS_TO_TICKS(1500);

    while (1) {
        for (int i = 0; i <= RAMP_STEPS; i++) {
            uint16_t t = RAMP_LOW + ((RAMP_HIGH - RAMP_LOW) * i) / RAMP_STEPS;
            motor_signal.throttle = t;
            ESP_LOGI(TAG, "Throttle = %u (up %d/%d)", t, i, RAMP_STEPS);
            TickType_t start = xTaskGetTickCount();
            while ((xTaskGetTickCount() - start) < hold_ticks) {
                ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &motor_signal, sizeof(motor_signal), &tx_config));
                rmt_tx_wait_all_done(esc_chan, -1);
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        for (int i = RAMP_STEPS - 1; i > 0; i--) {
            uint16_t t = RAMP_LOW + ((RAMP_HIGH - RAMP_LOW) * i) / RAMP_STEPS;
            motor_signal.throttle = t;
            ESP_LOGI(TAG, "Throttle = %u (down %d/%d)", t, RAMP_STEPS - i, RAMP_STEPS);
            TickType_t start = xTaskGetTickCount();
            while ((xTaskGetTickCount() - start) < hold_ticks) {
                ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &motor_signal, sizeof(motor_signal), &tx_config));
                rmt_tx_wait_all_done(esc_chan, -1);
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
    }
}
