/*
 * Serial diagnostics: counters and formatted log lines.
 *
 * Nothing here is safe to call from an ISR or from a Wi-Fi callback; the
 * callbacks and the GPIO ISR only enqueue events (spec section 11).
 */
#ifndef PHY_DIAGNOSTICS_H
#define PHY_DIAGNOSTICS_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "protocol_config.h"

#define PHY_LOGI(fmt, ...) printf("[phy]  " fmt "\n", ##__VA_ARGS__)
#define PHY_LOGW(fmt, ...) printf("[phy!] " fmt "\n", ##__VA_ARGS__)
#define PHY_LOGE(fmt, ...) printf("[phyX] " fmt "\n", ##__VA_ARGS__)

#if PHY_LOG_VERBOSE
#define PHY_LOGV(fmt, ...) printf("[phy.] " fmt "\n", ##__VA_ARGS__)
#else
#define PHY_LOGV(fmt, ...) do { } while (0)
#endif

typedef struct {
    /* transport */
    uint32_t lane_tx;                 /* one-bit packets submitted successfully */
    uint32_t lane_tx_fail;            /* submission or completion failure */
    uint32_t lane_rx;                 /* accepted one-bit packets */
    uint32_t lane_rx_bad_len;
    uint32_t lane_rx_bad_version;
    uint32_t lane_rx_bad_session;
    uint32_t lane_rx_bad_field;       /* cycle_index or lane_bit out of range */
    uint32_t lane_rx_wrong_peer;

    /* clock */
    uint32_t clk_rx_edges;
    uint32_t clk_rx_overflow;

    /* frames */
    uint32_t frames_tx;
    uint32_t frames_rx;
    uint32_t frames_rx_dup;
    uint32_t frames_rx_bad_framing;
    uint32_t frames_rx_bad_crc;
    uint32_t frames_rx_missing_lane;
    uint32_t frames_rx_unsupported;
    uint32_t frames_rx_unexpected;
    uint32_t rx_timeouts;
    uint32_t ack_tx;
    uint32_t nack_tx;
    uint32_t ack_rx;
    uint32_t nack_rx;
    uint32_t retries;
    uint32_t link_errors;
    uint32_t aborted_frames;          /* attempt ended before cycle 17 */

    /* application */
    uint32_t data_delivered;
} phy_stats_t;

extern phy_stats_t g_phy_stats;

void diag_print_stats(void);
void diag_reset_stats(void);

/* "<36 bits>" in a rotating static buffer, for debug lines only. */
const char *diag_bits_str(const uint8_t *bits, int count);

#endif /* PHY_DIAGNOSTICS_H */
