/*
 * ESP-NOW data lane (spec section 6).
 *
 * Exactly one ESP-NOW application message per clock cycle, carrying one
 * logical lane bit plus transport-only correlation metadata.  The 7-byte
 * envelope is NOT part of the 36-bit logical frame:
 *
 *   [0]     version          0x01
 *   [1..2]  session_id       uint16 big-endian
 *   [3..4]  frame_seq        uint16 big-endian
 *   [5]     cycle_index      0..17
 *   [6]     lane_bit         0 or 1
 *
 * Sending blocks until the local send-completion callback fires (or the
 * timeout expires).  That callback is not an application acknowledgement.
 */
#ifndef PHY_ESPNOW_LANE_H
#define PHY_ESPNOW_LANE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define LANE_ENVELOPE_LEN 7
#define LANE_VERSION      0x01

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t session_id_hi;
    uint8_t session_id_lo;
    uint8_t frame_seq_hi;
    uint8_t frame_seq_lo;
    uint8_t cycle_index;
    uint8_t lane_bit;
} lane_envelope_t;

typedef struct {
    int64_t ts_us;
    uint16_t session_id;
    uint16_t frame_seq;
    uint8_t cycle_index;
    uint8_t lane_bit;
    int8_t rssi;
    uint8_t src_mac[6];
} lane_rx_event_t;

/* Brings up Wi-Fi station mode on the configured channel and ESP-NOW with
 * the configured peer.  Packets that pass validation are queued on rx_queue
 * as lane_rx_event_t; everything else is dropped and counted. */
esp_err_t espnow_lane_init(QueueHandle_t rx_queue);

/* Blocking one-bit send.  Returns ESP_OK only if the local send-completion
 * callback reported success within timeout_ms. */
esp_err_t espnow_lane_send_bit(uint16_t session_id, uint16_t frame_seq,
                               uint8_t cycle_index, uint8_t lane_bit,
                               uint32_t timeout_ms);

/* "aa:bb:cc:dd:ee:ff" for the destination currently configured/learnt. */
const char *espnow_lane_peer_string(void);
bool espnow_lane_peer_is_broadcast(void);
void espnow_lane_local_mac_string(char out[18]);

#endif /* PHY_ESPNOW_LANE_H */
