#include <stdint.h>
#include <stdio.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"
#include "esp_check.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esc_telem.h"
#include "display.h"

// TTGO T-Display v1.1 pinout (ESP32 + ST7789, 135x240 panel, used in landscape).
#define PIN_MOSI 19
#define PIN_SCLK 18
#define PIN_CS    5
#define PIN_DC   16
#define PIN_RST  23
#define PIN_BL    4

#define LCD_HOST       SPI2_HOST
#define LCD_PCLK_HZ    (40 * 1000 * 1000)
#define LCD_CMD_BITS   8
#define LCD_PARAM_BITS 8

#define LCD_W       240
#define LCD_H       135
#define LCD_X_GAP   40
#define LCD_Y_GAP   52

static const char *TAG = "display";

static lv_display_t *s_lv_disp = NULL;
static lv_obj_t *s_throttle_label = NULL;
static lv_obj_t *s_voltage_label = NULL;
static lv_obj_t *s_rpm_label     = NULL;
static lv_obj_t *s_duty_label    = NULL;
static lv_obj_t *s_temp_label    = NULL;
static lv_obj_t *s_host_label    = NULL;
static lv_obj_t *s_ip_label      = NULL;
static _Atomic uint16_t s_throttle   = 0;
static _Atomic float    s_duty_pct   = -1.0f;
static _Atomic int      s_pole_pairs = 6;   // 12-pole default; overridden via display_set_pole_pairs()

static void throttle_updater(void *arg)
{
    // Use stdio snprintf into buffers, not lv_label_set_text_fmt, because LVGL's
    // internal vsnprintf often ships without %f support — values come out blank.
    char buf[24];

    while (1) {
        uint16_t t = atomic_load(&s_throttle);
        esc_telem_packet_t pkt;
        bool have_telem = esc_telem_get_latest(&pkt);
        // Treat anything older than 1s as no data. AM32 sends telemetry on
        // every DShot frame (~3 kHz), so a 1s gap means the ESC is gone
        // (power off, wire pull, hardware fault) — don't leave the last-known
        // values on screen pretending the motor is still running.
        bool fresh = esc_telem_is_fresh(1000);

        float duty = atomic_load(&s_duty_pct);

        if (lvgl_port_lock(100)) {
            snprintf(buf, sizeof(buf), "%u", t);
            lv_label_set_text(s_throttle_label, buf);
            if (duty >= 0.0f) {
                snprintf(buf, sizeof(buf), "%.0f%%", duty);
            } else {
                snprintf(buf, sizeof(buf), "--%%");
            }
            lv_label_set_text(s_duty_label, buf);
            if (have_telem && fresh) {
                snprintf(buf, sizeof(buf), "%.2f V", pkt.voltage_v);
                lv_label_set_text(s_voltage_label, buf);
                // AM32 reports eRPM as actual/100 (BLHeli32 convention).
                // Mechanical RPM = (raw × 100) / pole_pairs.
                int pp = atomic_load(&s_pole_pairs);
                snprintf(buf, sizeof(buf), "%u rpm",
                         (unsigned)((uint32_t)pkt.erpm * 100 / pp));
                lv_label_set_text(s_rpm_label, buf);
                snprintf(buf, sizeof(buf), "%u\xC2\xB0""C", (unsigned)pkt.temperature_c);
                lv_label_set_text(s_temp_label, buf);
            } else {
                lv_label_set_text(s_voltage_label, "--.-- V");
                lv_label_set_text(s_rpm_label,     "--- rpm");
                lv_label_set_text(s_temp_label,    "--\xC2\xB0""C");
            }
            lvgl_port_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t display_init(void)
{
    gpio_config_t bl_cfg = {
        .pin_bit_mask = 1ULL << PIN_BL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&bl_cfg), TAG, "bl gpio");
    gpio_set_level(PIN_BL, 0);

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_W * 40 * sizeof(uint16_t) + 16,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi bus");

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = PIN_DC,
        .cs_gpio_num = PIN_CS,
        .pclk_hz = LCD_PCLK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io),
                        TAG, "panel io");

    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(io, &panel_cfg, &panel), TAG, "panel st7789");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), TAG, "init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(panel, true), TAG, "invert");
    // Gap (column/row offsets into the ST7789's 240x320 RAM for the 135x240 visible panel).
    // lvgl_port will apply swap_xy/mirror below; gap stays as-is.
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(panel, LCD_X_GAP, LCD_Y_GAP), TAG, "gap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true), TAG, "disp on");

    // Hand the panel off to LVGL via esp_lvgl_port. The port creates its own internal
    // task that drives lv_timer_handler(); we just need to grab the lock when touching
    // LVGL objects from outside.
    const lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_RETURN_ON_ERROR(lvgl_port_init(&port_cfg), TAG, "lvgl port init");

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io,
        .panel_handle = panel,
        .buffer_size = LCD_W * 40,
        .double_buffer = true,
        .hres = LCD_W,
        .vres = LCD_H,
        .monochrome = false,
        .rotation = {
            .swap_xy = true,    // landscape: long edge (240) is horizontal
            .mirror_x = false,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = true,
            .swap_bytes = true,  // ESP32 is little-endian; ST7789 expects MSB first per pixel
        },
    };
    s_lv_disp = lvgl_port_add_disp(&disp_cfg);
    if (!s_lv_disp) {
        ESP_LOGE(TAG, "lvgl_port_add_disp failed");
        return ESP_FAIL;
    }

    if (lvgl_port_lock(0)) {
        lv_obj_t *screen = lv_display_get_screen_active(s_lv_disp);
        lv_obj_set_style_bg_color(screen, lv_color_black(), LV_PART_MAIN);
        // 240x135 landscape. Throttle is the headline at top (M48); 4 telemetry
        // values fit in a 2x2 grid below using fixed-width cells so they never
        // get clipped, and never auto-shift when text length changes.
        const int W = 240;
        const int CELL_W = 115;   // half of 240 minus margin
        const int CELL_H = 22;
        const int THR_Y  = 0;
        // Telemetry rows are nudged up vs the original layout to make room for
        // two M14 network lines at the bottom (host + ip on their own lines).
        const int ROW1_Y = 58;
        const int ROW2_Y = 82;
        const int HOST_Y = 105;
        const int IP_Y   = 119;

        const lv_color_t c_green  = lv_color_make(0x90, 0xEE, 0x90);
        const lv_color_t c_yellow = lv_color_make(0xFF, 0xE8, 0x80);
        const lv_color_t c_orange = lv_color_make(0xFF, 0xB0, 0x60);
        const lv_color_t c_red    = lv_color_make(0xFF, 0x80, 0x80);

        s_throttle_label = lv_label_create(screen);
        lv_obj_set_style_text_color(s_throttle_label, lv_color_white(), 0);
        lv_obj_set_style_text_font(s_throttle_label, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_align(s_throttle_label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_size(s_throttle_label, W, 56);
        lv_obj_set_pos(s_throttle_label, 0, THR_Y);
        lv_label_set_text(s_throttle_label, "----");

        // Voltage (left, row 1)
        s_voltage_label = lv_label_create(screen);
        lv_obj_set_style_text_color(s_voltage_label, c_green, 0);
        lv_obj_set_style_text_font(s_voltage_label, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_align(s_voltage_label, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_set_size(s_voltage_label, CELL_W, CELL_H);
        lv_obj_set_pos(s_voltage_label, 5, ROW1_Y);
        lv_label_set_text(s_voltage_label, "--.-- V");

        // RPM (right, row 1)
        s_rpm_label = lv_label_create(screen);
        lv_obj_set_style_text_color(s_rpm_label, c_orange, 0);
        lv_obj_set_style_text_font(s_rpm_label, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_align(s_rpm_label, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_size(s_rpm_label, CELL_W, CELL_H);
        lv_obj_set_pos(s_rpm_label, W - CELL_W - 5, ROW1_Y);
        lv_label_set_text(s_rpm_label, "--- rpm");

        // Duty% (left, row 2) — the last /pump command. "--%" when unset
        // (direct /motor throttle command, no duty mapping in play).
        s_duty_label = lv_label_create(screen);
        lv_obj_set_style_text_color(s_duty_label, c_yellow, 0);
        lv_obj_set_style_text_font(s_duty_label, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_align(s_duty_label, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_set_size(s_duty_label, CELL_W, CELL_H);
        lv_obj_set_pos(s_duty_label, 5, ROW2_Y);
        lv_label_set_text(s_duty_label, "--%");

        // Temperature (right, row 2). Current hidden — AM32 reports it but
        // the hardware on this ESC has no current-sense, so it's meaningless.
        s_temp_label = lv_label_create(screen);
        lv_obj_set_style_text_color(s_temp_label, c_red, 0);
        lv_obj_set_style_text_font(s_temp_label, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_align(s_temp_label, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_size(s_temp_label, CELL_W, CELL_H);
        lv_obj_set_pos(s_temp_label, W - CELL_W - 5, ROW2_Y);
        lv_label_set_text(s_temp_label, "--\xC2\xB0""C");

        // Network footer (host + ip on two M14 lines). Light gray —
        // informational, not a primary readout. Populated by WifiConnection
        // on got-IP. Hostname line uses LONG_DOT (ellipsize) so a 32-char
        // hostname can't overflow the panel.
        const lv_color_t c_gray = lv_color_make(0xB0, 0xB0, 0xB0);
        s_host_label = lv_label_create(screen);
        lv_obj_set_style_text_color(s_host_label, c_gray, 0);
        lv_obj_set_style_text_font(s_host_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(s_host_label, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_long_mode(s_host_label, LV_LABEL_LONG_DOT);
        lv_obj_set_size(s_host_label, W, 14);
        lv_obj_set_pos(s_host_label, 0, HOST_Y);
        lv_label_set_text(s_host_label, "no wifi");

        s_ip_label = lv_label_create(screen);
        lv_obj_set_style_text_color(s_ip_label, c_gray, 0);
        lv_obj_set_style_text_font(s_ip_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(s_ip_label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_size(s_ip_label, W, 14);
        lv_obj_set_pos(s_ip_label, 0, IP_Y);
        lv_label_set_text(s_ip_label, "");

        lvgl_port_unlock();
    }

    gpio_set_level(PIN_BL, 1);
    xTaskCreate(throttle_updater, "thr_upd", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    ESP_LOGI(TAG, "Display ready (LVGL + Montserrat 48)");
    return ESP_OK;
}

void display_set_throttle(uint16_t throttle)
{
    atomic_store(&s_throttle, throttle);
}

void display_set_duty(float duty_pct)
{
    atomic_store(&s_duty_pct, duty_pct);
}

void display_set_network(const char *host, const char *ip)
{
    if (!s_host_label || !s_ip_label) return;
    const char *host_text = (host && host[0]) ? host : "no wifi";
    const char *ip_text   = (ip   && ip[0])   ? ip   : "";
    if (lvgl_port_lock(100)) {
        lv_label_set_text(s_host_label, host_text);
        lv_label_set_text(s_ip_label,   ip_text);
        lvgl_port_unlock();
    }
}

void display_set_pole_pairs(int pole_pairs)
{
    if (pole_pairs < 1) pole_pairs = 1;
    atomic_store(&s_pole_pairs, pole_pairs);
}
