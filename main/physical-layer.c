#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/* Infrared transmitter / receiver */
#define IR_TX_GPIO          GPIO_NUM_18
#define IR_RX_GPIO          GPIO_NUM_26

/* Demodulating receivers (TSOP/VS1838/etc.) expect a 38 kHz carrier. */
#define IR_CARRIER_HZ       38000
#define IR_CARRIER_RES      LEDC_TIMER_10_BIT
#define IR_CARRIER_DUTY     ((1 << 10) / 2)     /* 50 % of the 10-bit range */

/* One cycle: carrier on for IR_ON_MS, then off for IR_OFF_MS. */
#define IR_ON_MS            1000
#define IR_OFF_MS           1000

/* Receiver sampling period while the transmitter is active. */
#define IR_RX_POLL_US       2000

/* ESP-NOW */
#define ESPNOW_CHANNEL      1                   /* both boards must match */

/* One bit plus a counter, so dropped frames are visible on the other side. */
typedef struct __attribute__((packed)) {
    uint8_t  bit;
    uint32_t seq;
} bit_frame_t;

static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};

static void ir_tx_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = IR_CARRIER_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = IR_CARRIER_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t chan_conf = {
        .gpio_num = IR_TX_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,                                  /* start silent */
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan_conf));
}

static void ir_tx_set(bool on)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                                  on ? IR_CARRIER_DUTY : 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void ir_rx_init(void)
{
    gpio_config_t rx_conf = {
        .pin_bit_mask = 1ULL << IR_RX_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rx_conf));
}

/*
 * Sample the receiver for duration_us and report whether its level ever
 * changed. Checking for an edge instead of a fixed level makes this work
 * with both active-low demodulating modules and raw phototransistors.
 */
static bool ir_rx_detect(uint32_t duration_us)
{
    const int64_t start = esp_timer_get_time();
    int last_level = gpio_get_level(IR_RX_GPIO);
    bool detected = false;

    while ((esp_timer_get_time() - start) < (int64_t)duration_us) {
        int level = gpio_get_level(IR_RX_GPIO);
        if (level != last_level) {
            detected = true;
            last_level = level;
        }
        esp_rom_delay_us(IR_RX_POLL_US);
    }

    return detected;
}

static void wifi_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Fix the channel so every board sits on the same one without an AP. */
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

static void on_send_done(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    if (status != ESP_NOW_SEND_SUCCESS) {
        printf("esp-now: transmission failed\n");
    }
}

static void on_frame(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len < (int)sizeof(bit_frame_t)) {
        printf("esp-now: short frame (%d bytes)\n", len);
        return;
    }

    bit_frame_t frame;
    memcpy(&frame, data, sizeof(frame));

    const uint8_t *src = info->src_addr;
    printf("esp-now: rx bit %u seq %lu from %02x:%02x:%02x:%02x:%02x:%02x (rssi %d)\n",
           frame.bit, (unsigned long)frame.seq,
           src[0], src[1], src[2], src[3], src[4], src[5],
           info->rx_ctrl->rssi);
}

static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_send_done));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_frame));

    /* Broadcast peer: no MAC address needs to be configured on either board. */
    esp_now_peer_info_t peer = {
        .channel = ESPNOW_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void app_main(void)
{
    ir_tx_init();
    ir_rx_init();
    wifi_init();
    espnow_init();

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    printf("Physical Layer Project\n");
    printf("IR transmitter on GPIO%d (%d Hz carrier), receiver on GPIO%d\n",
           IR_TX_GPIO, IR_CARRIER_HZ, IR_RX_GPIO);
    printf("ESP-NOW on channel %d, station MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
           ESPNOW_CHANNEL, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    uint32_t cycle = 0;
    while (1) {
        ir_tx_set(true);
        bool detected = ir_rx_detect(IR_ON_MS * 1000);
        ir_tx_set(false);

        bit_frame_t frame = {
            .bit = detected ? 1 : 0,
            .seq = ++cycle,
        };
        esp_err_t err = esp_now_send(broadcast_mac, (const uint8_t *)&frame,
                                     sizeof(frame));

        printf("cycle %lu: receiver %s -> esp-now bit %u %s\n",
               (unsigned long)cycle,
               detected ? "DETECTED a signal" : "saw nothing",
               frame.bit,
               err == ESP_OK ? "queued" : esp_err_to_name(err));

        vTaskDelay(pdMS_TO_TICKS(IR_OFF_MS));
    }
}
