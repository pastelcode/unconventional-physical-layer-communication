/*
 * 36-bit logical PHY frame (spec sections 3 and 4).
 *
 *   SYN[7..0] | SFD[7..0] | CONTROL[3..0] | DATA[7..0] | CRC[7..0]
 *
 * MSB first inside every field.  Consecutive serialized bits are paired onto
 * the two data lanes: IR carries logical position 2*k, ESP-NOW carries
 * 2*k + 1, for clock cycle k = 0..17.
 */
#ifndef PHY_FRAME_H
#define PHY_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#define FRAME_BITS   36
#define FRAME_CYCLES 18

#define FRAME_SYN 0x66u
#define FRAME_SFD 0xD5u

typedef enum {
    FRAME_TYPE_DATA = 1,
    FRAME_TYPE_ACK  = 2,
    FRAME_TYPE_NACK = 3,
    FRAME_TYPE_TURN = 4,
    FRAME_TYPE_END  = 5,
} frame_type_t;

typedef enum {
    NACK_REASON_CRC          = 0x01,
    NACK_REASON_FRAMING      = 0x02,
    NACK_REASON_MISSING_LANE = 0x03,
    NACK_REASON_TIMEOUT      = 0x04,
    NACK_REASON_STATE        = 0x05,
    NACK_REASON_UNSUPPORTED  = 0x06,
} nack_reason_t;

typedef struct {
    uint8_t control;                 /* low 4 bits used, high nibble zero */
    uint8_t data;
    uint8_t crc;
    uint8_t bits[FRAME_BITS];        /* one decoded logical bit each */
} phy_frame_t;

typedef enum {
    FRAME_PARSE_OK = 0,
    FRAME_PARSE_BAD_FRAMING,         /* SYN or SFD mismatch */
    FRAME_PARSE_BAD_CRC,             /* CRC covers CONTROL + DATA */
    FRAME_PARSE_UNSUPPORTED,         /* well framed, unknown CONTROL value */
} frame_parse_result_t;

/* Fill bits[] (and crc) from control/data.  control is masked to 4 bits. */
void frame_build(phy_frame_t *frame, uint8_t control, uint8_t data);

/* Build a frame straight from the decoded bits, e.g. for unit tests. */
void frame_build_from_bits(phy_frame_t *frame, const uint8_t bits[FRAME_BITS]);

/* Split into per-cycle lane bits (IR = even logical position). */
void frame_to_lanes(const phy_frame_t *frame,
                    uint8_t ir_bits[FRAME_CYCLES],
                    uint8_t wifi_bits[FRAME_CYCLES]);

/* Rebuild the 36-bit stream from the two lane arrays (IR first per cycle). */
void frame_from_lanes(const uint8_t ir_bits[FRAME_CYCLES],
                      const uint8_t wifi_bits[FRAME_CYCLES],
                      uint8_t bits[FRAME_BITS]);

/* Structural validation: SYN, SFD, type, CRC.  No protocol-semantics checks. */
frame_parse_result_t frame_parse_bits(const uint8_t bits[FRAME_BITS], phy_frame_t *out);

bool frame_type_is_supported(uint8_t control);
bool frame_type_is_request(uint8_t control);   /* DATA / TURN / END */
bool frame_type_is_response(uint8_t control);  /* ACK / NACK */

const char *frame_type_name(uint8_t control);
const char *frame_nack_reason_name(uint8_t reason);

#endif /* PHY_FRAME_H */
