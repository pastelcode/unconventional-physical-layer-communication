#include "frame.h"

#include <string.h>

#include "crc8.h"

static void bits_from_byte(uint8_t *dst, uint8_t value)
{
    for (int i = 0; i < 8; i++) {
        dst[i] = (uint8_t)((value >> (7 - i)) & 1u);
    }
}

static uint8_t byte_from_bits(const uint8_t *src)
{
    uint8_t value = 0;
    for (int i = 0; i < 8; i++) {
        value = (uint8_t)((value << 1) | (src[i] & 1u));
    }
    return value;
}

void frame_build(phy_frame_t *frame, uint8_t control, uint8_t data)
{
    memset(frame, 0, sizeof(*frame));
    frame->control = (uint8_t)(control & 0x0Fu);
    frame->data = data;
    frame->crc = crc8_frame_checksum(frame->control, data);

    uint8_t *b = frame->bits;
    bits_from_byte(&b[0], FRAME_SYN);
    bits_from_byte(&b[8], FRAME_SFD);
    b[16] = (uint8_t)((frame->control >> 3) & 1u);
    b[17] = (uint8_t)((frame->control >> 2) & 1u);
    b[18] = (uint8_t)((frame->control >> 1) & 1u);
    b[19] = (uint8_t)(frame->control & 1u);
    bits_from_byte(&b[20], frame->data);
    bits_from_byte(&b[28], frame->crc);
}

void frame_build_from_bits(phy_frame_t *frame, const uint8_t bits[FRAME_BITS])
{
    memset(frame, 0, sizeof(*frame));
    memcpy(frame->bits, bits, FRAME_BITS);

    frame->control = (uint8_t)(((bits[16] & 1u) << 3) | ((bits[17] & 1u) << 2) |
                               ((bits[18] & 1u) << 1) | (bits[19] & 1u));
    frame->data = byte_from_bits(&bits[20]);
    frame->crc = byte_from_bits(&bits[28]);
}

void frame_to_lanes(const phy_frame_t *frame,
                    uint8_t ir_bits[FRAME_CYCLES],
                    uint8_t wifi_bits[FRAME_CYCLES])
{
    for (int k = 0; k < FRAME_CYCLES; k++) {
        ir_bits[k] = (uint8_t)(frame->bits[2 * k] & 1u);
        wifi_bits[k] = (uint8_t)(frame->bits[2 * k + 1] & 1u);
    }
}

void frame_from_lanes(const uint8_t ir_bits[FRAME_CYCLES],
                      const uint8_t wifi_bits[FRAME_CYCLES],
                      uint8_t bits[FRAME_BITS])
{
    for (int k = 0; k < FRAME_CYCLES; k++) {
        bits[2 * k] = (uint8_t)(ir_bits[k] & 1u);
        bits[2 * k + 1] = (uint8_t)(wifi_bits[k] & 1u);
    }
}

frame_parse_result_t frame_parse_bits(const uint8_t bits[FRAME_BITS], phy_frame_t *out)
{
    phy_frame_t local;
    phy_frame_t *frame = out ? out : &local;

    frame_build_from_bits(frame, bits);

    if (byte_from_bits(&bits[0]) != FRAME_SYN || byte_from_bits(&bits[8]) != FRAME_SFD) {
        return FRAME_PARSE_BAD_FRAMING;
    }
    if (!frame_type_is_supported(frame->control)) {
        return FRAME_PARSE_UNSUPPORTED;
    }
    if (crc8_frame_checksum(frame->control, frame->data) != frame->crc) {
        return FRAME_PARSE_BAD_CRC;
    }
    return FRAME_PARSE_OK;
}

bool frame_type_is_supported(uint8_t control)
{
    switch (control & 0x0Fu) {
    case FRAME_TYPE_DATA:
    case FRAME_TYPE_ACK:
    case FRAME_TYPE_NACK:
    case FRAME_TYPE_TURN:
    case FRAME_TYPE_END:
        return true;
    default:
        return false;
    }
}

bool frame_type_is_request(uint8_t control)
{
    switch (control & 0x0Fu) {
    case FRAME_TYPE_DATA:
    case FRAME_TYPE_TURN:
    case FRAME_TYPE_END:
        return true;
    default:
        return false;
    }
}

bool frame_type_is_response(uint8_t control)
{
    switch (control & 0x0Fu) {
    case FRAME_TYPE_ACK:
    case FRAME_TYPE_NACK:
        return true;
    default:
        return false;
    }
}

const char *frame_type_name(uint8_t control)
{
    switch (control & 0x0Fu) {
    case FRAME_TYPE_DATA: return "DATA";
    case FRAME_TYPE_ACK:  return "ACK";
    case FRAME_TYPE_NACK: return "NACK";
    case FRAME_TYPE_TURN: return "TURN";
    case FRAME_TYPE_END:  return "END";
    default:              return "INVALID";
    }
}

const char *frame_nack_reason_name(uint8_t reason)
{
    switch (reason) {
    case NACK_REASON_CRC:          return "crc";
    case NACK_REASON_FRAMING:      return "framing";
    case NACK_REASON_MISSING_LANE: return "missing-lane-bit";
    case NACK_REASON_TIMEOUT:      return "timeout";
    case NACK_REASON_STATE:        return "unexpected-state";
    case NACK_REASON_UNSUPPORTED:  return "unsupported-type";
    default:                       return "unknown";
    }
}
