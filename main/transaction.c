#include "transaction.h"

#include <string.h>

void rx_txn_clear(rx_transaction_t *txn)
{
    if (txn == NULL) {
        return;
    }
    memset(txn, 0, sizeof(*txn));
    txn->active = false;
    txn->have_identity = false;
    txn->edge_count = 0;
    txn->invalid = false;
}

rx_txn_result_t rx_txn_on_edge(rx_transaction_t *txn, uint8_t ir_bit, int64_t now_us)
{
    if (txn == NULL || ir_bit > 1u) {
        return RX_TXN_BAD_ARG;
    }
    if (txn->invalid) {
        return RX_TXN_REJECTED;
    }
    if (!txn->active) {
        rx_txn_clear(txn);
        txn->active = true;
        txn->started_us = now_us;
    }
    if (txn->edge_count >= FRAME_CYCLES) {
        txn->invalid = true;
        txn->last_event_us = now_us;
        return RX_TXN_OVERFLOW;
    }

    txn->ir_received[txn->edge_count] = true;
    txn->ir_bit[txn->edge_count] = (uint8_t)(ir_bit & 1u);
    txn->edge_count++;
    txn->last_event_us = now_us;
    return RX_TXN_STORED;
}

rx_txn_result_t rx_txn_on_lane_bit(rx_transaction_t *txn, uint16_t session_id,
                                   uint16_t frame_seq, uint8_t cycle_index,
                                   uint8_t lane_bit, int64_t now_us)
{
    if (txn == NULL || cycle_index >= FRAME_CYCLES || lane_bit > 1u) {
        return RX_TXN_BAD_ARG;
    }
    if (txn->invalid) {
        return RX_TXN_REJECTED;
    }
    if (!txn->active) {
        rx_txn_clear(txn);
        txn->active = true;
        txn->started_us = now_us;
    }

    if (!txn->have_identity) {
        txn->have_identity = true;
        txn->session_id = session_id;
        txn->frame_seq = frame_seq;
    } else if (txn->session_id != session_id || txn->frame_seq != frame_seq) {
        if (txn->edge_count == 0 && !rx_txn_missing_lane(txn)) {
            /* Nothing collected yet: this packet simply identifies the frame
             * that is about to start (it arrived before the first edge). */
            rx_txn_clear(txn);
            txn->active = true;
            txn->have_identity = true;
            txn->session_id = session_id;
            txn->frame_seq = frame_seq;
            txn->started_us = now_us;
        } else {
            txn->invalid = true;
            txn->last_event_us = now_us;
            return RX_TXN_REJECTED;
        }
    }

    if (txn->wifi_received[cycle_index]) {
        if (txn->wifi_bit[cycle_index] != (uint8_t)(lane_bit & 1u)) {
            txn->invalid = true;
            txn->last_event_us = now_us;
            return RX_TXN_CONFLICT;
        }
        return RX_TXN_DUPLICATE;
    }

    txn->wifi_received[cycle_index] = true;
    txn->wifi_bit[cycle_index] = (uint8_t)(lane_bit & 1u);
    txn->last_event_us = now_us;
    return RX_TXN_STORED;
}

bool rx_txn_is_complete(const rx_transaction_t *txn)
{
    return txn != NULL && txn->active && !txn->invalid && txn->edge_count == FRAME_CYCLES;
}

bool rx_txn_missing_lane(const rx_transaction_t *txn)
{
    if (txn == NULL) {
        return true;
    }
    for (int k = 0; k < FRAME_CYCLES; k++) {
        if (!txn->ir_received[k] || !txn->wifi_received[k]) {
            return true;
        }
    }
    return false;
}

bool rx_txn_expired(const rx_transaction_t *txn, int64_t now_us, int64_t timeout_us)
{
    if (txn == NULL || !txn->active) {
        return false;
    }
    return (now_us - txn->started_us) >= timeout_us;
}

bool rx_txn_assemble(const rx_transaction_t *txn, uint8_t bits[FRAME_BITS])
{
    if (txn == NULL || bits == NULL) {
        return false;
    }
    for (int k = 0; k < FRAME_CYCLES; k++) {
        if (!txn->ir_received[k] || !txn->wifi_received[k]) {
            return false;
        }
        bits[2 * k] = (uint8_t)(txn->ir_bit[k] & 1u);
        bits[2 * k + 1] = (uint8_t)(txn->wifi_bit[k] & 1u);
    }
    return true;
}

/* ------------------------------------------------- sequence / duplicate cache */

void rx_seq_reset(rx_seq_t *tracker)
{
    if (tracker == NULL) {
        return;
    }
    tracker->synced = false;
    tracker->last_seq = 0;
}

bool rx_seq_accepts(const rx_seq_t *tracker, uint16_t seq_value)
{
    if (tracker == NULL) {
        return false;
    }
    if (!tracker->synced) {
        return true; /* first request of a session, or after a TURN/END/reset */
    }
    return seq_value == (uint16_t)(tracker->last_seq + 1);
}

void rx_seq_commit(rx_seq_t *tracker, uint16_t seq_value)
{
    if (tracker == NULL) {
        return;
    }
    tracker->synced = true;
    tracker->last_seq = seq_value;
}

void resp_cache_clear(resp_cache_t *cache)
{
    if (cache != NULL) {
        memset(cache, 0, sizeof(*cache));
        cache->valid = false;
    }
}

void resp_cache_store(resp_cache_t *cache, uint16_t session_id, uint16_t seq,
                      uint8_t req_control, uint8_t req_data,
                      uint8_t resp_control, uint8_t resp_data)
{
    if (cache == NULL) {
        return;
    }
    cache->valid = true;
    cache->session_id = session_id;
    cache->seq = seq;
    cache->req_control = req_control;
    cache->req_data = req_data;
    cache->resp_control = resp_control;
    cache->resp_data = resp_data;
}

bool resp_cache_matches(const resp_cache_t *cache, uint16_t session_id, uint16_t seq,
                        uint8_t req_control, uint8_t req_data)
{
    if (cache == NULL || !cache->valid) {
        return false;
    }
    return cache->session_id == session_id && cache->seq == seq &&
           cache->req_control == req_control && cache->req_data == req_data;
}
