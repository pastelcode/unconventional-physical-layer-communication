/*
 * One in-flight receive transaction: cycle alignment of the wired clock
 * edges and the one-bit ESP-NOW lane packets (spec sections 7 and 10).
 *
 * The assembly rules implemented here are deliberately index based:
 *
 *  - a lane packet that arrives before its clock edge is buffered in its own
 *    cycle slot, it never advances the cycle counter;
 *  - a missing lane packet leaves its slot empty, so the following bits do
 *    not shift;
 *  - duplicates for the same cycle are ignored, and duplicates that disagree
 *    invalidate the whole transaction;
 *  - lane packets carrying another (session, frame_seq) are rejected once
 *    real data has been collected.
 *
 * The module is pure C with caller-supplied timestamps so the same code runs
 * on target and in the host unit tests.
 */
#ifndef PHY_TRANSACTION_H
#define PHY_TRANSACTION_H

#include <stdbool.h>
#include <stdint.h>

#include "frame.h"

typedef struct {
    bool active;
    bool have_identity;                  /* session/seq learnt from a lane packet */
    uint16_t session_id;
    uint16_t frame_seq;
    bool ir_received[FRAME_CYCLES];
    bool wifi_received[FRAME_CYCLES];
    uint8_t ir_bit[FRAME_CYCLES];
    uint8_t wifi_bit[FRAME_CYCLES];
    uint8_t edge_count;
    bool invalid;
    int64_t started_us;
    int64_t last_event_us;
} rx_transaction_t;

typedef enum {
    RX_TXN_STORED = 0,
    RX_TXN_DUPLICATE,
    RX_TXN_CONFLICT,     /* duplicate bit disagrees -> transaction invalidated */
    RX_TXN_OVERFLOW,     /* more than 18 rising edges -> transaction invalidated */
    RX_TXN_REJECTED,     /* foreign sequence, or transaction already invalid */
    RX_TXN_BAD_ARG,
} rx_txn_result_t;

void rx_txn_clear(rx_transaction_t *txn);

/* Record one rising clock edge and the IR bit sampled at that instant. */
rx_txn_result_t rx_txn_on_edge(rx_transaction_t *txn, uint8_t ir_bit, int64_t now_us);

/* Record one ESP-NOW lane bit; may arrive before or after its clock edge. */
rx_txn_result_t rx_txn_on_lane_bit(rx_transaction_t *txn, uint16_t session_id,
                                   uint16_t frame_seq, uint8_t cycle_index,
                                   uint8_t lane_bit, int64_t now_us);

bool rx_txn_is_complete(const rx_transaction_t *txn);
bool rx_txn_missing_lane(const rx_transaction_t *txn);
bool rx_txn_expired(const rx_transaction_t *txn, int64_t now_us, int64_t timeout_us);

/* Rebuild the 36-bit stream; only meaningful for complete transactions. */
bool rx_txn_assemble(const rx_transaction_t *txn, uint8_t bits[FRAME_BITS]);

/* -------------------------------------------------------------------------
 * Responder-side duplicate and sequence tracking (spec sections 6 and 9).
 *
 * Kept separate from the transaction buffer so the exactly-once rules can be
 * unit tested without FreeRTOS.
 * ---------------------------------------------------------------------- */

typedef struct {
    bool synced;
    uint16_t last_seq;
} rx_seq_t;

void rx_seq_reset(rx_seq_t *tracker);
/* True when `seq` may start a new request: first request after a reset, or
 * exactly last_seq + 1 (modulo 65536). */
bool rx_seq_accepts(const rx_seq_t *tracker, uint16_t seq_value);
void rx_seq_commit(rx_seq_t *tracker, uint16_t seq_value);

typedef struct {
    bool valid;
    uint16_t session_id;
    uint16_t seq;
    uint8_t req_control;
    uint8_t req_data;
    uint8_t resp_control;
    uint8_t resp_data;
} resp_cache_t;

void resp_cache_clear(resp_cache_t *cache);
void resp_cache_store(resp_cache_t *cache, uint16_t session_id, uint16_t seq,
                      uint8_t req_control, uint8_t req_data,
                      uint8_t resp_control, uint8_t resp_data);
/* A retransmission is the same session, sequence, type and DATA. */
bool resp_cache_matches(const resp_cache_t *cache, uint16_t session_id, uint16_t seq,
                        uint8_t req_control, uint8_t req_data);


#endif /* PHY_TRANSACTION_H */
