#include "diagnostics.h"

#include "frame.h"

#include <stdio.h>
#include <string.h>

phy_stats_t g_phy_stats;

void diag_reset_stats(void)
{
    memset(&g_phy_stats, 0, sizeof(g_phy_stats));
}

void diag_print_stats(void)
{
    const phy_stats_t *s = &g_phy_stats;

    printf("--- counters ---------------------------------------------\n");
    printf(" lane   tx %lu (fail %lu)  rx %lu\n",
           (unsigned long)s->lane_tx, (unsigned long)s->lane_tx_fail,
           (unsigned long)s->lane_rx);
    printf(" lane drops: len %lu version %lu session %lu field %lu peer %lu\n",
           (unsigned long)s->lane_rx_bad_len, (unsigned long)s->lane_rx_bad_version,
           (unsigned long)s->lane_rx_bad_session, (unsigned long)s->lane_rx_bad_field,
           (unsigned long)s->lane_rx_wrong_peer);
    printf(" clock  edges %lu  overflow %lu\n",
           (unsigned long)s->clk_rx_edges, (unsigned long)s->clk_rx_overflow);
    printf(" frames tx %lu  rx %lu  dup %lu  data %lu\n",
           (unsigned long)s->frames_tx, (unsigned long)s->frames_rx,
           (unsigned long)s->frames_rx_dup, (unsigned long)s->data_delivered);
    printf(" errors framing %lu crc %lu missing-lane %lu unsupported %lu\n",
           (unsigned long)s->frames_rx_bad_framing, (unsigned long)s->frames_rx_bad_crc,
           (unsigned long)s->frames_rx_missing_lane, (unsigned long)s->frames_rx_unsupported);
    printf(" replies ack tx %lu nack tx %lu | ack rx %lu nack rx %lu\n",
           (unsigned long)s->ack_tx, (unsigned long)s->nack_tx,
           (unsigned long)s->ack_rx, (unsigned long)s->nack_rx);
    printf(" retries %lu  aborted %lu  rx-timeouts %lu  link-errors %lu\n",
           (unsigned long)s->retries, (unsigned long)s->aborted_frames,
           (unsigned long)s->rx_timeouts, (unsigned long)s->link_errors);
    printf("----------------------------------------------------------\n");
}

const char *diag_bits_str(const uint8_t *bits, int count)
{
    static char buffers[2][FRAME_BITS + 1];
    static int which;

    if (bits == NULL || count <= 0) {
        return "";
    }
    if (count > FRAME_BITS) {
        count = FRAME_BITS;
    }
    which ^= 1;
    char *buf = buffers[which];
    for (int i = 0; i < count; i++) {
        buf[i] = (bits[i] & 1u) ? '1' : '0';
    }
    buf[count] = '\0';
    return buf;
}
