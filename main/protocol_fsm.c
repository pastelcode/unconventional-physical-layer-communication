#include "protocol_fsm.h"

#include <stdio.h>
#include <string.h>

#include "clock_gpio.h"
#include "diagnostics.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "espnow_lane.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "infrared.h"
#include "protocol_config.h"
#include "transaction.h"

/* Queue item used to carry one message: one DATA byte each, terminated by
 * TX_ITEM_MESSAGE_END so an empty message still sends END. */
#define TX_ITEM_MESSAGE_END 0x100u

typedef enum {
    EV_CLK_EDGE = 1,
    EV_LANE_RX,
} event_type_t;

typedef struct {
    event_type_t type;
    clk_edge_t edge;
    lane_rx_event_t lane;
} event_t;

typedef enum {
    LST_LINK_IDLE = 0,
    LST_RX_FRAME,
    LST_TX_FRAME,
    LST_WAIT_RESPONSE,
    LST_LINK_ERROR,
} link_state_t;

static struct {
    link_state_t state;

    uint16_t session_id;
    uint16_t next_seq;              /* sequence for the next originated request */
    bool local_is_originator;
    bool boot_originator;

    /* responder view of the peer's request sequence + last answer */
    rx_seq_t seq;
    resp_cache_t cache;

    /* outstanding request */
    uint8_t out_control;
    uint8_t out_data;
    uint16_t out_seq;
    uint8_t attempts;               /* transmissions already made */
    bool needs_quiet_gap;
    int64_t out_started_us;

    /* queued outbound message */
    bool msg_active;
    uint16_t msg_first_seq;
    uint16_t msg_bytes;

    /* receive side */
    rx_transaction_t rx;
    bool rx_complete_armed;
    int64_t rx_arm_us;              /* events older than this are stale */
    int64_t settle_deadline_us;
    int64_t response_deadline_us;

    protocol_app_t app;
} s_fsm;

static QueueHandle_t s_events;
static QueueHandle_t s_requests;
static QueueHandle_t s_tx_items;    /* uint16_t: DATA byte or MESSAGE_END */

/* Bench fault injection: flip one IR bit of the next request frame. */
static bool s_inject_ir_error;

/* DATA occupies logical bits 20..27, i.e. cycle 10 carries bit 20 (IR). */
#define INJECT_CYCLE 10

/* --------------------------------------------------------------- utilities */

static const char *state_name(link_state_t state)
{
    switch (state) {
    case LST_LINK_IDLE:     return "LINK_IDLE";
    case LST_RX_FRAME:      return "RX_FRAME";
    case LST_TX_FRAME:      return "TX_FRAME";
    case LST_WAIT_RESPONSE: return "WAIT_RESPONSE";
    case LST_LINK_ERROR:    return "LINK_ERROR";
    default:                return "UNKNOWN";
    }
}

static void set_state(link_state_t state)
{
    if (s_fsm.state != state) {
        /* One line per protocol step, always visible: this is the trace the
         * host tool shows while a frame is being exchanged. */
        PHY_LOGI("step: %s -> %s", state_name(s_fsm.state), state_name(state));
    }
    s_fsm.state = state;
}

static int64_t event_ts(const event_t *ev)
{
    return (ev->type == EV_CLK_EDGE) ? ev->edge.ts_us : ev->lane.ts_us;
}

static void notify_error(const char *message)
{
    PHY_LOGE("%s", message);
    if (s_fsm.app.on_error != NULL) {
        s_fsm.app.on_error(message);
    }
}

static void notify_request_done(proto_tx_status_t status, uint8_t reason)
{
    if (s_fsm.app.on_request_done != NULL) {
        s_fsm.app.on_request_done(s_fsm.out_control, s_fsm.out_data, s_fsm.out_seq, status,
                                  s_fsm.attempts, reason);
    }
}

/* Finish the current receive transaction and go back to idle. */
static void finish_transaction(void)
{
    rx_txn_clear(&s_fsm.rx);
    s_fsm.rx_complete_armed = false;
    s_fsm.rx_arm_us = esp_timer_get_time();
    set_state(LST_LINK_IDLE);
}

/* Open a receive window.  `arm_us` is the timestamp of the event that starts
 * it, so events that were already queued for an earlier transaction are
 * recognised as stale while the rest of the new burst is kept. */
static void start_rx_window(int64_t arm_us)
{
    rx_txn_clear(&s_fsm.rx);
    s_fsm.rx_arm_us = arm_us;
    s_fsm.rx_complete_armed = false;
    set_state(LST_RX_FRAME);
}

/* Take the next link event, discarding anything from before the current
 * window.  Returns false when the queue holds nothing usable. */
static bool take_pending_event(event_t *ev)
{
    for (;;) {
        if (xQueueReceive(s_events, ev, 0) != pdTRUE) {
            return false;
        }
        if (event_ts(ev) >= s_fsm.rx_arm_us) {
            return true;
        }
        PHY_LOGV("dropped stale event from before the last transaction");
    }
}

/* ------------------------------------------------------------ clock TX core */

/*
 * Emit the whole 36-bit frame: 18 cycles, each with one IR symbol and one
 * one-bit ESP-NOW packet, separated by exactly one rising clock edge.
 *
 * The ESP-NOW packet is submitted first and its local send-completion
 * callback is awaited; only then does the clock edge follow.  A failure
 * before the first edge is not a partial frame; a failure later leaves a
 * partial burst behind, which the caller must follow with the quiet gap.
 */
static bool transmit_frame(const phy_frame_t *frame, uint16_t seq, bool *aborted_partial)
{
    uint8_t ir_bits[FRAME_CYCLES];
    uint8_t wifi_bits[FRAME_CYCLES];

    frame_to_lanes(frame, ir_bits, wifi_bits);
    *aborted_partial = false;
    clock_tx_low();
    ir_tx_idle();

    if (s_inject_ir_error && frame_type_is_request(frame->control)) {
        s_inject_ir_error = false;
        ir_bits[INJECT_CYCLE] ^= 1u;
        PHY_LOGW("fault injection: IR bit of cycle %d flipped (expect NACK 0x01)",
                 INJECT_CYCLE);
    }

    PHY_LOGV("tx %s seq=%u bits=%s", frame_type_name(frame->control), seq,
             diag_bits_str(frame->bits, FRAME_BITS));

    for (int k = 0; k < FRAME_CYCLES; k++) {
        const int64_t cycle_start_us = esp_timer_get_time();

        /* step 2: IR symbol, held for the whole sampling window */
        ir_tx_set_logical_bit(ir_bits[k] != 0);

        /* steps 3-4: companion ESP-NOW bit, wait for local completion */
        esp_err_t err = espnow_lane_send_bit(s_fsm.session_id, seq, (uint8_t)k,
                                             wifi_bits[k], ESP_SEND_TIMEOUT_MS);
        if (err != ESP_OK) {
            ir_tx_idle();
            clock_tx_low();
            *aborted_partial = (k > 0);
            PHY_LOGW("cycle %d: lane send failed (%s); attempt aborted after %d edge(s)",
                     k, esp_err_to_name(err), k);
            return false;
        }

        /* step 5: guard time plus the minimum IR setup window */
        vTaskDelay(pdMS_TO_TICKS(AFTER_SEND_GUARD_MS));
        const int64_t elapsed_us = esp_timer_get_time() - cycle_start_us;
        if (elapsed_us < IR_SETUP_US) {
            esp_rom_delay_us((uint32_t)(IR_SETUP_US - elapsed_us));
        }

        /* step 6: exactly one rising edge for this cycle_index */
        clock_tx_pulse();
        ir_tx_idle();

        PHY_LOGV("  cycle %2d ir=%u wifi=%u (t+%lld us)", k, ir_bits[k], wifi_bits[k],
                 (long long)(esp_timer_get_time() - cycle_start_us));
    }

    g_phy_stats.frames_tx++;
    return true;
}

/* ------------------------------------------------------- response emission */

static void cache_store(const phy_frame_t *req, uint8_t resp_control, uint8_t resp_data)
{
    resp_cache_store(&s_fsm.cache, s_fsm.session_id, s_fsm.rx.frame_seq, req->control,
                     req->data, resp_control, resp_data);
}

/* Send a complete ACK/NACK frame over this board's own IR + ESP-NOW + clock. */
static void respond(uint8_t control, uint8_t data, uint16_t seq)
{
    if (!s_fsm.rx.have_identity) {
        PHY_LOGW("cannot answer: transaction identity unknown (peer will time out)");
        finish_transaction();
        return;
    }

    if (control == FRAME_TYPE_NACK) {
        g_phy_stats.nack_tx++;
        PHY_LOGI("TX NACK seq=%u reason=0x%02x (%s)", seq, data, frame_nack_reason_name(data));
    } else {
        g_phy_stats.ack_tx++;
        PHY_LOGI("TX ACK  seq=%u", seq);
    }
    if (s_fsm.app.on_reply_sent != NULL) {
        s_fsm.app.on_reply_sent(control, data, seq);
    }

    /* RESPONSE_START_GUARD_MS of silence between reception and answer. */
    vTaskDelay(pdMS_TO_TICKS(RESPONSE_START_GUARD_MS));

    phy_frame_t frame;
    frame_build(&frame, control, data);

    bool partial = false;
    if (!transmit_frame(&frame, seq, &partial)) {
        g_phy_stats.aborted_frames++;
        PHY_LOGW("response aborted%s; peer will retry and hit the cached response",
                 partial ? " after a partial frame" : "");
    }
    finish_transaction();
}

static void respond_nack(uint8_t reason, uint16_t seq)
{
    respond(FRAME_TYPE_NACK, reason, seq);
}

/* Semantic rejection of a well-framed request: remember the answer so a
 * retransmission of the same request gets exactly the same response. */
static void reject_request(const phy_frame_t *req, uint8_t reason)
{
    cache_store(req, FRAME_TYPE_NACK, reason);
    respond_nack(reason, s_fsm.rx.frame_seq);
}

/* --------------------------------------------------- inbound request handling */

static void handle_request(const phy_frame_t *frame)
{
    const uint16_t seq = s_fsm.rx.frame_seq;

    /* 1. Retransmission of an already handled request: same answer, and no
     *    second delivery / second TURN / second END. */
    if (resp_cache_matches(&s_fsm.cache, s_fsm.session_id, seq, frame->control, frame->data)) {
        g_phy_stats.frames_rx_dup++;
        PHY_LOGI("%s seq=%u already handled -> resending cached %s%s",
                 frame_type_name(frame->control), seq,
                 frame_type_name(s_fsm.cache.resp_control),
                 s_fsm.cache.resp_control == FRAME_TYPE_NACK ? " (same reason)" : "");
        respond(s_fsm.cache.resp_control, s_fsm.cache.resp_data, seq);
        return;
    }

    /* 2. Sequence continuity. */
    if (!rx_seq_accepts(&s_fsm.seq, seq)) {
        g_phy_stats.frames_rx_unexpected++;
        PHY_LOGW("unexpected request seq=%u (expected %u) -> NACK 0x05", seq,
                 (unsigned)(uint16_t)(s_fsm.seq.last_seq + 1));
        reject_request(frame, NACK_REASON_STATE);
        return;
    }

    /* 3. Only the current originator may originate DATA/TURN/END. */
    if (s_fsm.local_is_originator) {
        g_phy_stats.frames_rx_unexpected++;
        PHY_LOGW("%s from peer while this node is the originator -> NACK 0x05",
                 frame_type_name(frame->control));
        reject_request(frame, NACK_REASON_STATE);
        return;
    }

    /* 4. Apply, cache, then acknowledge. */
    if (s_fsm.app.on_remote_frame != NULL) {
        s_fsm.app.on_remote_frame(frame, seq);
    }

    switch (frame->control) {
    case FRAME_TYPE_DATA:
        rx_seq_commit(&s_fsm.seq, seq);
        cache_store(frame, FRAME_TYPE_ACK, 0x00);
        PHY_LOGV("frame %s crc ok", diag_bits_str(frame->bits, FRAME_BITS));
        g_phy_stats.data_delivered++;
        respond(FRAME_TYPE_ACK, 0x00, seq);
        break;

    case FRAME_TYPE_TURN:
        if (frame->data != 0x00) {
            PHY_LOGW("TURN with reserved DATA=0x%02x -> NACK 0x05", frame->data);
            reject_request(frame, NACK_REASON_STATE);
            break;
        }
        /* Ownership is applied BEFORE the ACK (spec section 5). */
        s_fsm.local_is_originator = true;
        s_fsm.next_seq = SEQ_FIRST;
        rx_seq_reset(&s_fsm.seq);
        cache_store(frame, FRAME_TYPE_ACK, 0x00);
        PHY_LOGI("TURN received: this node is now the DATA originator");
        respond(FRAME_TYPE_ACK, 0x00, seq);
        break;

    case FRAME_TYPE_END:
        if (frame->data != 0x00) {
            PHY_LOGW("END with reserved DATA=0x%02x -> NACK 0x05", frame->data);
            reject_request(frame, NACK_REASON_STATE);
            break;
        }
        s_fsm.next_seq = SEQ_FIRST;
        rx_seq_reset(&s_fsm.seq);
        cache_store(frame, FRAME_TYPE_ACK, 0x00);
        PHY_LOGI("END received: logical session closed, sequence state reset");
        respond(FRAME_TYPE_ACK, 0x00, seq);
        break;

    default:
        /* frame_parse_bits() already rejected unknown types. */
        respond_nack(NACK_REASON_UNSUPPORTED, seq);
        break;
    }
}

static void finish_request_frame(void)
{
    phy_frame_t frame;
    uint8_t bits[FRAME_BITS];

    /* 1. exactly 18 cycles, one bit from each lane per cycle */
    if (rx_txn_missing_lane(&s_fsm.rx)) {
        g_phy_stats.frames_rx_missing_lane++;
        PHY_LOGW("frame incomplete after %u/18 edges: lane bit missing -> NACK 0x03",
                 s_fsm.rx.edge_count);
        respond_nack(NACK_REASON_MISSING_LANE, s_fsm.rx.frame_seq);
        return;
    }
    if (!rx_txn_assemble(&s_fsm.rx, bits)) {
        finish_transaction();
        return;
    }

    PHY_LOGV("rx bits %s", diag_bits_str(bits, FRAME_BITS));

    /* 2-4. SYN, SFD, CONTROL, DATA, CRC */
    switch (frame_parse_bits(bits, &frame)) {
    case FRAME_PARSE_BAD_FRAMING:
        g_phy_stats.frames_rx_bad_framing++;
        PHY_LOGW("SYN/SFD mismatch -> NACK 0x02");
        respond_nack(NACK_REASON_FRAMING, s_fsm.rx.frame_seq);
        return;
    case FRAME_PARSE_UNSUPPORTED:
        g_phy_stats.frames_rx_unsupported++;
        PHY_LOGW("unsupported CONTROL 0x%x -> NACK 0x06", frame.control);
        respond_nack(NACK_REASON_UNSUPPORTED, s_fsm.rx.frame_seq);
        return;
    case FRAME_PARSE_BAD_CRC:
        g_phy_stats.frames_rx_bad_crc++;
        PHY_LOGW("CRC mismatch (got 0x%02x) -> NACK 0x01", frame.crc);
        respond_nack(NACK_REASON_CRC, s_fsm.rx.frame_seq);
        return;
    case FRAME_PARSE_OK:
        break;
    }

    g_phy_stats.frames_rx++;

    /* 5. An ACK/NACK is only meaningful as a response to our own request. */
    if (frame_type_is_response(frame.control)) {
        g_phy_stats.frames_rx_unexpected++;
        PHY_LOGW("unexpected %s frame with no outstanding request; not answering",
                 frame_type_name(frame.control));
        finish_transaction();
        return;
    }

    handle_request(&frame);
}

/* ---------------------------------------------------- outbound request paths */

static void finish_message(proto_tx_status_t status)
{
    if (!s_fsm.msg_active) {
        return;
    }
    s_fsm.msg_active = false;
    if (s_fsm.app.on_message_done != NULL) {
        s_fsm.app.on_message_done(status, s_fsm.msg_first_seq, s_fsm.out_seq,
                                  s_fsm.msg_bytes);
    }
}

static void start_request(uint8_t control, uint8_t data);

static void start_next_message_frame(void)
{
    uint16_t item;
    if (xQueuePeek(s_tx_items, &item, 0) != pdTRUE) {
        finish_message(PROTO_TX_ACK);
        return;
    }
    xQueueReceive(s_tx_items, &item, 0);
    if (item == TX_ITEM_MESSAGE_END) {
        start_request(FRAME_TYPE_END, 0x00);
    } else {
        s_fsm.msg_bytes++;
        start_request(FRAME_TYPE_DATA, (uint8_t)(item & 0xFF));
    }
}

static void maybe_start_message(void)
{
    uint16_t item;

    if (s_fsm.msg_active || s_fsm.state != LST_LINK_IDLE) {
        return;
    }
    if (xQueuePeek(s_tx_items, &item, 0) != pdTRUE) {
        return;
    }
    if (!s_fsm.local_is_originator) {
        return; /* queued until this node becomes the originator */
    }

    s_fsm.msg_active = true;
    s_fsm.msg_first_seq = s_fsm.next_seq;
    s_fsm.msg_bytes = 0;
    start_next_message_frame();
}

static void flush_tx_queue(void)
{
    uint16_t item;
    while (xQueueReceive(s_tx_items, &item, 0) == pdTRUE) {
    }
}

static void fail_link(const char *reason)
{
    g_phy_stats.link_errors++;
    notify_request_done(PROTO_TX_ERROR, 0x00);
    if (s_fsm.msg_active) {
        finish_message(PROTO_TX_ERROR);
        flush_tx_queue();
    }
    PHY_LOGE("giving up on seq=%u after %u attempt(s): %s", s_fsm.out_seq,
             (unsigned)s_fsm.attempts, reason);
    notify_error("retries exhausted: type RESET on BOTH boards to restart the session");
    finish_transaction();
    set_state(LST_LINK_ERROR);

    /* The host contract reports STATE on state changes as well as on STATUS. */
    if (s_fsm.app.on_status != NULL) {
        s_fsm.app.on_status(state_name(s_fsm.state), s_fsm.local_is_originator,
                            s_fsm.next_seq);
    }
}

static void retry_request(const char *reason, proto_tx_status_t status, uint8_t nack_reason)
{
    rx_txn_clear(&s_fsm.rx);
    s_fsm.rx_complete_armed = false;

    if (s_fsm.attempts > MAX_RETRANSMISSIONS) {
        fail_link(reason);
        return;
    }
    notify_request_done(status, nack_reason);

    g_phy_stats.retries++;
    PHY_LOGW("%s: retransmitting %s seq=%u (attempt %u of %u)", reason,
             frame_type_name(s_fsm.out_control), s_fsm.out_seq,
             (unsigned)(s_fsm.attempts + 1), (unsigned)(MAX_RETRANSMISSIONS + 1));
    set_state(LST_TX_FRAME);
}

static void commit_local_sequence(uint8_t control)
{
    if (control == FRAME_TYPE_END) {
        s_fsm.next_seq = SEQ_FIRST;
        rx_seq_reset(&s_fsm.seq);
    } else {
        s_fsm.next_seq = (uint16_t)(s_fsm.next_seq + 1);
        if (s_fsm.next_seq == 0) {
            PHY_LOGW("frame_seq wrapped; restart both boards to clear session history (spec 6)");
            s_fsm.next_seq = SEQ_FIRST;
            rx_seq_reset(&s_fsm.seq);
        }
    }
}

static void commit_request(void)
{
    const uint8_t control = s_fsm.out_control;

    PHY_LOGI("ACK seq=%u for %s after %lld ms", s_fsm.out_seq, frame_type_name(control),
             (long long)((esp_timer_get_time() - s_fsm.out_started_us) / 1000));

    notify_request_done(PROTO_TX_ACK, 0x00);

    if (control == FRAME_TYPE_TURN) {
        s_fsm.local_is_originator = false;
        rx_seq_reset(&s_fsm.seq); /* the new originator starts a fresh era */
        flush_tx_queue();         /* we no longer hold DATA authority */
        PHY_LOGI("TURN acknowledged: this node is now the responder");
        finish_transaction();
        return;
    }

    commit_local_sequence(control);

    const bool message_frame = s_fsm.msg_active;
    const bool message_end = message_frame && control == FRAME_TYPE_END;

    finish_transaction();

    if (message_end) {
        finish_message(PROTO_TX_ACK);
    } else if (message_frame) {
        start_next_message_frame();
    }
}

static void handle_response_frame(void)
{
    phy_frame_t frame;
    uint8_t bits[FRAME_BITS];

    if (rx_txn_missing_lane(&s_fsm.rx)) {
        g_phy_stats.frames_rx_missing_lane++;
        retry_request("incomplete response (missing lane bit)", PROTO_TX_TIMEOUT, 0x00);
        return;
    }
    if (!rx_txn_assemble(&s_fsm.rx, bits)) {
        retry_request("response assembly failed", PROTO_TX_TIMEOUT, 0x00);
        return;
    }

    PHY_LOGV("response bits %s", diag_bits_str(bits, FRAME_BITS));

    frame_parse_result_t parsed = frame_parse_bits(bits, &frame);
    if (parsed == FRAME_PARSE_BAD_CRC) {
        g_phy_stats.frames_rx_bad_crc++;
        retry_request("response CRC mismatch", PROTO_TX_TIMEOUT, NACK_REASON_CRC);
        return;
    }
    if (parsed == FRAME_PARSE_BAD_FRAMING) {
        g_phy_stats.frames_rx_bad_framing++;
        retry_request("response framing mismatch", PROTO_TX_TIMEOUT, NACK_REASON_FRAMING);
        return;
    }
    if (parsed == FRAME_PARSE_UNSUPPORTED) {
        g_phy_stats.frames_rx_unsupported++;
        retry_request("response type unsupported", PROTO_TX_TIMEOUT, 0x00);
        return;
    }

    if (s_fsm.rx.session_id != s_fsm.session_id || s_fsm.rx.frame_seq != s_fsm.out_seq) {
        g_phy_stats.frames_rx_unexpected++;
        retry_request("response for another transaction", PROTO_TX_TIMEOUT, 0x00);
        return;
    }

    g_phy_stats.frames_rx++;

    if (frame.control == FRAME_TYPE_ACK) {
        g_phy_stats.ack_rx++;
        commit_request();
        return;
    }
    if (frame.control == FRAME_TYPE_NACK) {
        g_phy_stats.nack_rx++;
        PHY_LOGW("NACK seq=%u reason=0x%02x (%s)", s_fsm.out_seq, frame.data,
                 frame_nack_reason_name(frame.data));
        retry_request("peer NACK", PROTO_TX_NACK, frame.data);
        return;
    }

    /* A request frame arriving while we await our own response: a protocol
     * violation on the peer side.  Do not answer (an ACK of an ACK or a
     * NACK of a NACK would loop); the outstanding request keeps its window. */
    g_phy_stats.frames_rx_unexpected++;
    PHY_LOGW("request %s arrived while waiting for a response; ignoring",
             frame_type_name(frame.control));
    rx_txn_clear(&s_fsm.rx);
    s_fsm.rx_complete_armed = false;
    s_fsm.rx_arm_us = esp_timer_get_time();
}

static void start_request(uint8_t control, uint8_t data)
{
    s_fsm.out_control = control;
    s_fsm.out_data = data;
    s_fsm.out_seq = s_fsm.next_seq;
    s_fsm.attempts = 0;
    s_fsm.needs_quiet_gap = false;
    s_fsm.out_started_us = esp_timer_get_time();

    PHY_LOGI("originator: queued %s data=0x%02x seq=%u", frame_type_name(control), data,
             s_fsm.out_seq);
    set_state(LST_TX_FRAME);
}

/* ------------------------------------------------------------- app requests */

static void emit_status(void)
{
    if (s_fsm.app.on_status != NULL) {
        s_fsm.app.on_status(state_name(s_fsm.state), s_fsm.local_is_originator,
                            s_fsm.next_seq);
    }

    printf("--- status -----------------------------------------------\n");
    printf(" role        : %s (%s)\n",
           s_fsm.local_is_originator ? "DATA originator" : "responder",
           NODE_ROLE_A ? "node A" : "node B");
    printf(" state       : %s\n", state_name(s_fsm.state));
    printf(" session     : 0x%04x  next request seq: %u\n", s_fsm.session_id, s_fsm.next_seq);
    if (s_fsm.seq.synced) {
        printf(" peer tx seq : expecting %u (last accepted %u)\n",
               (unsigned)(uint16_t)(s_fsm.seq.last_seq + 1), (unsigned)s_fsm.seq.last_seq);
    } else {
        printf(" peer tx seq : not synced (any first sequence is accepted)\n");
    }
    if (s_fsm.state == LST_WAIT_RESPONSE || s_fsm.state == LST_TX_FRAME) {
        printf(" outstanding : %s seq=%u attempt %u/%u\n",
               frame_type_name(s_fsm.out_control), s_fsm.out_seq,
               (unsigned)s_fsm.attempts, (unsigned)(MAX_RETRANSMISSIONS + 1));
    }
    printf(" timings     : clock %u Hz nominal (%u us high, %u us low), IR carrier %u Hz\n",
           (unsigned)clock_rate_hz(), (unsigned)clock_high_us(), (unsigned)clock_low_us(),
           (unsigned)ir_carrier_hz());
    printf(" wifi        : channel %u, peer %s%s\n", ESPNOW_WIFI_CHANNEL,
           espnow_lane_peer_string(), espnow_lane_peer_is_broadcast() ? " (broadcast)" : "");
    printf("----------------------------------------------------------\n");
    diag_print_stats();
}

static void reset_local_session(const char *why)
{
    rx_txn_clear(&s_fsm.rx);
    s_fsm.rx_complete_armed = false;
    s_fsm.rx_arm_us = esp_timer_get_time();

    s_fsm.next_seq = SEQ_FIRST;
    s_fsm.out_seq = SEQ_FIRST;
    s_fsm.out_control = 0;
    s_fsm.out_data = 0;
    s_fsm.attempts = 0;
    s_fsm.needs_quiet_gap = false;
    s_fsm.msg_active = false;
    flush_tx_queue();
    rx_seq_reset(&s_fsm.seq);
    resp_cache_clear(&s_fsm.cache);
    s_fsm.local_is_originator = s_fsm.boot_originator;

    set_state(LST_LINK_IDLE);
    PHY_LOGI("local session reset (%s): seq=%u, role=%s -- reset the peer as well", why,
             s_fsm.next_seq, s_fsm.local_is_originator ? "originator" : "responder");
    if (s_fsm.app.on_ready != NULL) {
        s_fsm.app.on_ready();
    }
}

static void handle_app_request(proto_req_type_t type)
{
    switch (type) {
    case PROTO_REQ_STATUS:
        emit_status();
        break;

    case PROTO_REQ_RESET:
        reset_local_session("console");
        emit_status();
        break;

    case PROTO_REQ_TURN:
        if (s_fsm.msg_active) {
            notify_error("cannot TURN while a message is in flight");
            break;
        }
        if (!s_fsm.local_is_originator) {
            notify_error("TURN must come from the current originator; this node is the responder");
            break;
        }
        start_request(FRAME_TYPE_TURN, 0x00);
        break;

    case PROTO_REQ_END:
        if (s_fsm.msg_active) {
            notify_error("cannot END while a message is in flight");
            break;
        }
        if (!s_fsm.local_is_originator) {
            notify_error("END must come from the current originator; this node is the responder");
            break;
        }
        start_request(FRAME_TYPE_END, 0x00);
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------- state bodies */

static void ingest_event(const event_t *ev)
{
    const int64_t now = esp_timer_get_time();

    if (ev->type == EV_CLK_EDGE) {
        rx_txn_result_t r = rx_txn_on_edge(&s_fsm.rx, ev->edge.ir_bit, now);
        g_phy_stats.clk_rx_edges++;
        if (r == RX_TXN_OVERFLOW) {
            PHY_LOGW("19th clock edge in one transaction -> invalid");
        }
    } else {
        rx_txn_result_t r = rx_txn_on_lane_bit(&s_fsm.rx, ev->lane.session_id, ev->lane.frame_seq,
                                               ev->lane.cycle_index, ev->lane.lane_bit, now);
        switch (r) {
        case RX_TXN_DUPLICATE:
            PHY_LOGV("duplicate lane packet cycle %u (identical)", ev->lane.cycle_index);
            break;
        case RX_TXN_CONFLICT:
            PHY_LOGW("duplicate lane packet cycle %u disagrees -> invalid",
                     ev->lane.cycle_index);
            break;
        case RX_TXN_REJECTED:
            PHY_LOGW("lane packet for another sequence while a frame is in progress");
            break;
        default:
            break;
        }
    }
}

static void st_idle(void)
{
    event_t ev;
    proto_req_type_t req;

    /* Inbound first: never start a transmission into a frame that is already
     * arriving, or the two boards would clock at each other.  A fresh clock
     * burst, or an early lane packet, opens a transaction; an early packet is
     * buffered and never advances the cycle counter. */
    if (take_pending_event(&ev)) {
        start_rx_window(event_ts(&ev));
        ingest_event(&ev);
        return;
    }

    if (xQueueReceive(s_requests, &req, 0) == pdTRUE) {
        handle_app_request(req);
        return;
    }

    maybe_start_message();
    if (s_fsm.state != LST_LINK_IDLE) {
        return;
    }

    if (xQueueReceive(s_events, &ev, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }
    if (event_ts(&ev) < s_fsm.rx_arm_us) {
        PHY_LOGV("dropped stale event from before the last transaction");
        return;
    }

    start_rx_window(event_ts(&ev));
    ingest_event(&ev);
}

static void st_rx_frame(void)
{
    const int64_t now = esp_timer_get_time();

    if (s_fsm.rx.invalid) {
        const bool identity = s_fsm.rx.have_identity;
        const uint16_t seq = s_fsm.rx.frame_seq;
        const uint8_t edges = s_fsm.rx.edge_count;
        PHY_LOGW("transaction invalidated after %u edge(s)", edges);
        if (identity && edges > 0) {
            respond_nack(NACK_REASON_FRAMING, seq);
        } else {
            finish_transaction();
        }
        return;
    }

    if (rx_txn_is_complete(&s_fsm.rx)) {
        if (!s_fsm.rx_complete_armed) {
            s_fsm.settle_deadline_us = now + (int64_t)RX_LANE_SETTLE_MS * 1000;
            s_fsm.rx_complete_armed = true;
            PHY_LOGV("18 edges seen; settling for %d ms of late lane packets",
                     RX_LANE_SETTLE_MS);
        }
        if (now >= s_fsm.settle_deadline_us) {
            finish_request_frame();
            return;
        }
    }

    if (rx_txn_expired(&s_fsm.rx, now, (int64_t)RX_FRAME_TIMEOUT_MS * 1000)) {
        g_phy_stats.rx_timeouts++;
        PHY_LOGW("frame timeout after %u/18 edges; dropping the partial transaction",
                 s_fsm.rx.edge_count);
        finish_transaction();
        return;
    }

    event_t ev;
    if (xQueueReceive(s_events, &ev, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }
    if (event_ts(&ev) < s_fsm.rx_arm_us) {
        PHY_LOGV("dropped stale event");
        return;
    }
    ingest_event(&ev);
}

static void st_tx_frame(void)
{
    if (s_fsm.needs_quiet_gap) {
        PHY_LOGW("partial attempt: waiting the %d ms quiet gap before reusing seq=%u",
                 QUIET_GAP_MS, s_fsm.out_seq);
        vTaskDelay(pdMS_TO_TICKS(QUIET_GAP_MS));
        s_fsm.needs_quiet_gap = false;
    }

    phy_frame_t frame;
    frame_build(&frame, s_fsm.out_control, s_fsm.out_data);

    PHY_LOGI("TX %s seq=%u data=0x%02x (attempt %u of %u)", frame_type_name(s_fsm.out_control),
             s_fsm.out_seq, s_fsm.out_data, (unsigned)(s_fsm.attempts + 1),
             (unsigned)(MAX_RETRANSMISSIONS + 1));

    bool partial = false;
    const bool ok = transmit_frame(&frame, s_fsm.out_seq, &partial);
    s_fsm.attempts++;

    if (!ok) {
        if (partial) {
            g_phy_stats.aborted_frames++;
        }
        s_fsm.needs_quiet_gap = partial;
        if (s_fsm.attempts > MAX_RETRANSMISSIONS) {
            fail_link("lane transmission failure");
            return;
        }
        if (!partial) {
            vTaskDelay(pdMS_TO_TICKS(LANE_FAIL_BACKOFF_MS));
        }
        return;
    }

    rx_txn_clear(&s_fsm.rx);
    s_fsm.rx_complete_armed = false;
    s_fsm.rx_arm_us = esp_timer_get_time();
    s_fsm.response_deadline_us = s_fsm.rx_arm_us + (int64_t)RESPONSE_FRAME_TIMEOUT_MS * 1000;
    set_state(LST_WAIT_RESPONSE);
}

static void st_wait_response(void)
{
    const int64_t now = esp_timer_get_time();

    if (rx_txn_is_complete(&s_fsm.rx)) {
        if (!s_fsm.rx_complete_armed) {
            s_fsm.settle_deadline_us = now + (int64_t)RX_LANE_SETTLE_MS * 1000;
            s_fsm.rx_complete_armed = true;
        }
        if (now >= s_fsm.settle_deadline_us) {
            handle_response_frame();
            return;
        }
    }

    if (now >= s_fsm.response_deadline_us) {
        g_phy_stats.rx_timeouts++;
        retry_request("no response", PROTO_TX_TIMEOUT, 0x00);
        return;
    }

    event_t ev;
    if (xQueueReceive(s_events, &ev, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }
    if (event_ts(&ev) < s_fsm.rx_arm_us) {
        PHY_LOGV("dropped stale event while waiting for a response");
        return;
    }
    ingest_event(&ev);
}

static void st_link_error(void)
{
    proto_req_type_t req;
    if (xQueueReceive(s_requests, &req, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (req == PROTO_REQ_RESET) {
            reset_local_session("console");
            emit_status();
        } else if (req == PROTO_REQ_STATUS) {
            emit_status();
        } else {
            notify_error("link is in error; send RESET");
        }
        return;
    }

    /* Drain link events so the queue cannot fill up while in error. */
    event_t ev;
    while (xQueueReceive(s_events, &ev, 0) == pdTRUE) {
    }
}

/* --------------------------------------------------------------- task setup */

static void protocol_task(void *arg)
{
    (void)arg;

    for (;;) {
        switch (s_fsm.state) {
        case LST_LINK_IDLE:
            st_idle();
            break;
        case LST_RX_FRAME:
            st_rx_frame();
            break;
        case LST_TX_FRAME:
            st_tx_frame();
            break;
        case LST_WAIT_RESPONSE:
            st_wait_response();
            break;
        case LST_LINK_ERROR:
            st_link_error();
            break;
        default:
            set_state(LST_LINK_IDLE);
            break;
        }
    }
}

esp_err_t protocol_init(const protocol_app_t *app)
{
    memset(&s_fsm, 0, sizeof(s_fsm));
    if (app != NULL) {
        s_fsm.app = *app;
    }

    s_fsm.session_id = SESSION_ID;
    s_fsm.next_seq = SEQ_FIRST;
    s_fsm.out_seq = SEQ_FIRST;

    s_fsm.boot_originator = (NODE_ROLE_A && INITIAL_ORIGINATOR_A) ||
                            (!NODE_ROLE_A && !INITIAL_ORIGINATOR_A);
    s_fsm.local_is_originator = s_fsm.boot_originator;

    s_events = xQueueCreate(PHY_EVENT_QUEUE_LEN, sizeof(event_t));
    s_requests = xQueueCreate(PHY_REQUEST_QUEUE_LEN, sizeof(proto_req_type_t));
    s_tx_items = xQueueCreate(PHY_TX_QUEUE_LEN, sizeof(uint16_t));
    if (s_events == NULL || s_requests == NULL || s_tx_items == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = ir_init();
    if (err != ESP_OK) {
        return err;
    }
    err = clock_gpio_init(s_events);
    if (err != ESP_OK) {
        return err;
    }
    return espnow_lane_init(s_events);
}

void protocol_start(void)
{
    PHY_LOGI("step: %s (clock %u Hz nominal, IR carrier %u Hz)",
             state_name(s_fsm.state), (unsigned)clock_rate_hz(),
             (unsigned)ir_carrier_hz());
    xTaskCreate(protocol_task, "phy_proto", PHY_PROTOCOL_TASK_STACK, NULL,
                PHY_PROTOCOL_TASK_PRIO, NULL);
}

void protocol_inject_ir_bit_error_once(void)
{
    s_inject_ir_error = true;
}

bool protocol_is_originator(void)
{
    return s_fsm.local_is_originator;
}

bool protocol_send_message(const uint8_t *bytes, size_t len)
{
    if (s_tx_items == NULL || (bytes == NULL && len > 0) || len > PHY_MESSAGE_MAX) {
        return false;
    }
    if (!s_fsm.local_is_originator) {
        return false;
    }
    if (uxQueueSpacesAvailable(s_tx_items) < len + 1) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        uint16_t item = bytes[i];
        if (xQueueSend(s_tx_items, &item, 0) != pdTRUE) {
            return false;
        }
    }
    uint16_t end_marker = TX_ITEM_MESSAGE_END;
    return xQueueSend(s_tx_items, &end_marker, 0) == pdTRUE;
}

bool protocol_post(proto_req_type_t type)
{
    if (s_requests == NULL) {
        return false;
    }
    return xQueueSend(s_requests, &type, 0) == pdTRUE;
}
