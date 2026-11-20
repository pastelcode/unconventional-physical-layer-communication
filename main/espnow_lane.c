#include "espnow_lane.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/semphr.h"

#include "diagnostics.h"
#include "frame.h"
#include "protocol_config.h"

static QueueHandle_t s_rx_queue;
static SemaphoreHandle_t s_send_done;
static volatile bool s_send_ok;

static uint8_t s_send_mac[6];      /* ESP-NOW destination */
static bool s_send_is_broadcast;

/* Sender validation: a configured peer MAC is authoritative; with no
 * configured MAC (lab fallback) the first foreign sender is latched. */
static uint8_t s_filter_mac[6];
static bool s_filter_locked;

static uint8_t s_local_mac[6];

static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static char s_peer_string[18];
static char s_local_string[18];

static void mac_to_string(const uint8_t mac[6], char out[18])
{
    snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static bool parse_mac(const char *text, uint8_t out[6])
{
    unsigned int v[6];
    if (text == NULL || text[0] == '\0') {
        return false;
    }
    if (sscanf(text, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        if (v[i] > 0xFFu) {
            return false;
        }
        out[i] = (uint8_t)v[i];
    }
    return true;
}

/* ------------------------------------------------------------------ callbacks */

static void lane_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    s_send_ok = (status == ESP_NOW_SEND_SUCCESS);
    if (s_send_done != NULL) {
        xSemaphoreGive(s_send_done);
    }
}

static void lane_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (info == NULL || data == NULL) {
        return;
    }
    if (len != LANE_ENVELOPE_LEN) {
        g_phy_stats.lane_rx_bad_len++;
        return;
    }

    lane_envelope_t env;
    memcpy(&env, data, sizeof(env));

    if (env.version != LANE_VERSION) {
        g_phy_stats.lane_rx_bad_version++;
        return;
    }
    if (env.cycle_index >= FRAME_CYCLES || env.lane_bit > 1u) {
        g_phy_stats.lane_rx_bad_field++;
        return;
    }
    const uint16_t session = (uint16_t)(((uint16_t)env.session_id_hi << 8) | env.session_id_lo);
    if (session != SESSION_ID) {
        g_phy_stats.lane_rx_bad_session++;
        return;
    }

    const uint8_t *src = info->src_addr;
    if (memcmp(src, s_local_mac, 6) == 0) {
        /* ESP-NOW does not loop packets back, but be defensive. */
        g_phy_stats.lane_rx_wrong_peer++;
        return;
    }
    if (s_filter_locked) {
        if (memcmp(src, s_filter_mac, 6) != 0) {
            g_phy_stats.lane_rx_wrong_peer++;
            return;
        }
    } else {
        memcpy(s_filter_mac, src, 6);
        s_filter_locked = true;
        mac_to_string(s_filter_mac, s_peer_string);
    }

    lane_rx_event_t ev = {
        .ts_us = esp_timer_get_time(),
        .session_id = session,
        .frame_seq = (uint16_t)(((uint16_t)env.frame_seq_hi << 8) | env.frame_seq_lo),
        .cycle_index = env.cycle_index,
        .lane_bit = env.lane_bit,
        .rssi = (int8_t)(info->rx_ctrl ? info->rx_ctrl->rssi : 0),
    };
    memcpy(ev.src_mac, src, 6);

    if (s_rx_queue == NULL || xQueueSend(s_rx_queue, &ev, 0) != pdTRUE) {
        /* Dropped: the transaction layer will see the missing cycle. */
        return;
    }
    g_phy_stats.lane_rx++;
}

/* --------------------------------------------------------------------- setup */

static esp_err_t wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());

    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* No power save: modem sleep adds hundreds of milliseconds of latency to
     * one-bit ESP-NOW packets and would break the cycle pacing. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /* Fix the channel; both boards must match (spec section 6). */
    return esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
}

esp_err_t espnow_lane_init(QueueHandle_t rx_queue)
{
    s_rx_queue = rx_queue;
    s_send_ok = false;
    s_filter_locked = false;

    ESP_ERROR_CHECK(esp_read_mac(s_local_mac, ESP_MAC_WIFI_STA));
    mac_to_string(s_local_mac, s_local_string);

    if (parse_mac(CONFIG_PHY_PEER_MAC, s_send_mac)) {
        memcpy(s_filter_mac, s_send_mac, 6);
        s_filter_locked = true;
        s_send_is_broadcast = false;
    } else {
        memcpy(s_send_mac, s_broadcast_mac, 6);
        s_send_is_broadcast = true;
    }
    mac_to_string(s_filter_locked ? s_filter_mac : s_send_mac, s_peer_string);

    esp_err_t err = wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    s_send_done = xSemaphoreCreateBinary();
    if (s_send_done == NULL) {
        return ESP_ERR_NO_MEM;
    }

    err = esp_now_init();
    if (err != ESP_OK) {
        return err;
    }
    ESP_ERROR_CHECK(esp_now_register_send_cb(lane_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(lane_recv_cb));

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, s_send_mac, 6);
    peer.channel = ESPNOW_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        return err;
    }
    return ESP_OK;
}

/* ---------------------------------------------------------------------- send */

esp_err_t espnow_lane_send_bit(uint16_t session_id, uint16_t frame_seq,
                               uint8_t cycle_index, uint8_t lane_bit,
                               uint32_t timeout_ms)
{
    if (cycle_index >= FRAME_CYCLES || lane_bit > 1u) {
        return ESP_ERR_INVALID_ARG;
    }

    lane_envelope_t env = {
        .version = LANE_VERSION,
        .session_id_hi = (uint8_t)(session_id >> 8),
        .session_id_lo = (uint8_t)(session_id & 0xFF),
        .frame_seq_hi = (uint8_t)(frame_seq >> 8),
        .frame_seq_lo = (uint8_t)(frame_seq & 0xFF),
        .cycle_index = cycle_index,
        .lane_bit = lane_bit,
    };

    /* Drop any completion notification left over from an earlier send. */
    xSemaphoreTake(s_send_done, 0);
    s_send_ok = false;

    const int64_t start_us = esp_timer_get_time();
    const int64_t deadline_us = start_us + (int64_t)timeout_ms * 1000;

    for (;;) {
        esp_err_t err = esp_now_send(s_send_mac, (const uint8_t *)&env, sizeof(env));
        if (err == ESP_OK) {
            break;
        }
        if (err == ESP_ERR_ESPNOW_NO_MEM && esp_timer_get_time() < deadline_us) {
            vTaskDelay(1); /* Wi-Fi buffer busy: back off one tick */
            continue;
        }
        g_phy_stats.lane_tx_fail++;
        return err;
    }

    const int64_t now_us = esp_timer_get_time();
    TickType_t wait = pdMS_TO_TICKS(timeout_ms);
    if (now_us < deadline_us) {
        wait = pdMS_TO_TICKS((uint32_t)((deadline_us - now_us) / 1000));
    }
    if (xSemaphoreTake(s_send_done, wait) != pdTRUE) {
        g_phy_stats.lane_tx_fail++;
        return ESP_ERR_TIMEOUT;
    }
    if (!s_send_ok) {
        g_phy_stats.lane_tx_fail++;
        return ESP_FAIL;
    }

    g_phy_stats.lane_tx++;
    return ESP_OK;
}

/* -------------------------------------------------------------------- status */

const char *espnow_lane_peer_string(void)
{
    return s_peer_string;
}

bool espnow_lane_peer_is_broadcast(void)
{
    return s_send_is_broadcast;
}

void espnow_lane_local_mac_string(char out[18])
{
    memcpy(out, s_local_string, 18);
}
