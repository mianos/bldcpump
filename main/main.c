#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"

#define DSHOT_GPIO_NUM           18
#define DSHOT_BAUD_RATE          600000    // DSHOT600 (Use 300000 for DSHOT300)

#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_RESOLUTION_HZ      32000000  // 32MHz
#else
#define DSHOT_RESOLUTION_HZ      40000000  // 40MHz
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

    ESP_LOGI(TAG, "Allocating Official DShot ESC Encoder...");
    rmt_encoder_handle_t dshot_encoder = NULL;
    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_RESOLUTION_HZ,
        .baud_rate = DSHOT_BAUD_RATE,
        .post_delay_us = 50, // Gap between successive frames
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));

    ESP_LOGI(TAG, "Enabling RMT Channel...");
    ESP_ERROR_CHECK(rmt_enable(esc_chan));

    // Configure transmission behavior (Set loop count to 0 for single explicit bursts)
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    // Prepare data payloads
    dshot_esc_throttle_t motor_signal = {
        .throttle = 0,
        .telemetry_req = false,
    };

    // 1. ESC ARMING SEQUENCE
    // ESCs require a steady stream of zero-throttle frames on boot to safely unlock
    ESP_LOGI(TAG, "Sending zero-throttle arming sequence...");
    for (int i = 0; i < 300; i++) { 
        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &motor_signal, sizeof(motor_signal), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(esc_chan, -1));
        vTaskDelay(pdMS_TO_TICKS(5)); // Send every 5ms
    }
    ESP_LOGI(TAG, "ESC Successfully Armed.");

    // 2. RUNTIME PRODUCTION MOTOR LOOP
    uint16_t active_throttle = 48; // DShot minimum standard spin value

    while (1) {
        // Update data payload
        motor_signal.throttle = active_throttle;
        motor_signal.telemetry_req = false;

        // Push frame asynchronously directly to hardware registers
        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &motor_signal, sizeof(motor_signal), &tx_config));
        
        // Wait for hardware to completely finish shifting out bits before next iteration
        rmt_tx_wait_all_done(esc_chan, -1);

        // Deterministic delay pacing (e.g., 1kHz execution rate)
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
