/*
 * Protocol finite state machine (spec sections 5, 7, 9 and 10).
 *
 * One serialized task owns every mutable protocol structure.  The GPIO ISR
 * and the ESP-NOW callbacks only enqueue events; application requests from
 * the console arrive on a separate queue.  The task arbitrates between
 * originating a request, waiting for its response, and answering an inbound
 * request, so TX_FRAME and TX_RESPONSE can never overlap.
 */
#ifndef PHY_PROTOCOL_FSM_H
#define PHY_PROTOCOL_FSM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "frame.h"

typedef enum {
    PROTO_TX_ACK = 1,   /* complete ACK frame received */
    PROTO_TX_NACK,      /* peer answered NACK (retried, may recover) */
    PROTO_TX_TIMEOUT,   /* no valid response within the window */
    PROTO_TX_ERROR,     /* lane failure, retries exhausted, link error */
} proto_tx_status_t;

typedef struct {
    /* Every accepted inbound request (DATA, TURN or END), after validation and
     * exactly once per (session, frame_seq), before the answer is sent. */
    void (*on_remote_frame)(const phy_frame_t *frame, uint16_t seq);
    /* One line per originated frame: completed, NACKed or timed out. */
    void (*on_request_done)(uint8_t control, uint8_t data, uint16_t seq,
                            proto_tx_status_t status, uint8_t attempts, uint8_t reason);
    /* Every ACK/NACK this node transmits in answer to a request. */
    void (*on_reply_sent)(uint8_t control, uint8_t data, uint16_t seq);
    /* End of a queued message (all DATA bytes plus END). */
    void (*on_message_done)(proto_tx_status_t status, uint16_t first_seq,
                            uint16_t last_seq, uint16_t bytes);
    /* Requested status snapshot. */
    void (*on_status)(const char *state, bool is_originator, uint16_t next_seq);
    /* Emitted after a local session reset, so the host can resynchronise. */
    void (*on_ready)(void);
    void (*on_error)(const char *message);
} protocol_app_t;

typedef enum {
    PROTO_REQ_TURN = 1,   /* request authority to originate DATA */
    PROTO_REQ_END,        /* close the current logical message */
    PROTO_REQ_STATUS,     /* emit a status snapshot */
    PROTO_REQ_RESET,      /* reset local session/sequence state */
} proto_req_type_t;

esp_err_t protocol_init(const protocol_app_t *app);
void protocol_start(void);

/* Queue a message: one DATA frame per byte, then one END frame.  Returns
 * false if the transmit queue is full, or immediately if this node is not
 * the current originator. */
bool protocol_send_message(const uint8_t *bytes, size_t len);

/* Corrupt one IR lane bit of the next originated request frame, to exercise
 * the CRC/NACK/retry path on the bench (spec section 12).  The retry is clean,
 * so end-to-end delivery must still be exactly once. */
void protocol_inject_ir_bit_error_once(void);

/* True while this node holds DATA authority. */
bool protocol_is_originator(void);

/* Non-blocking control request; returns false if the request queue is full. */
bool protocol_post(proto_req_type_t type);

#endif /* PHY_PROTOCOL_FSM_H */
