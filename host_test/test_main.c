/*
 * Host unit tests for the parts of the PHY protocol that do not need an
 * ESP32 (spec section 12, "Pure software unit tests").
 *
 * Build and run:
 *     make -C host_test
 *
 * The same crc8.c / frame.c / transaction.c sources are compiled into the
 * firmware, so a passing test here exercises the shipping code.
 */
#include <stdio.h>
#include <string.h>

#include "crc8.h"
#include "frame.h"
#include "transaction.h"

static int g_failures;
static int g_checks;

#define CHECK(cond)                                                              \
    do {                                                                         \
        g_checks++;                                                              \
        if (!(cond)) {                                                           \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);              \
            g_failures++;                                                        \
        }                                                                        \
    } while (0)

#define CHECK_EQ_INT(actual, expected)                                           \
    do {                                                                         \
        g_checks++;                                                              \
        long long a_ = (long long)(actual);                                      \
        long long e_ = (long long)(expected);                                    \
        if (a_ != e_) {                                                          \
            printf("  FAIL %s:%d: %s == %lld, expected %lld\n", __FILE__,         \
                   __LINE__, #actual, a_, e_);                                   \
            g_failures++;                                                        \
        }                                                                        \
    } while (0)

static const uint8_t kCheckString[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

static const struct {
    crc8_profile_t profile;
    uint8_t check;
    uint8_t data_0x41;
} kCrcVectors[] = {
    {CRC8_PROFILE_SMBUS, 0xF4, 0xD5},
    {CRC8_PROFILE_AUTOSAR, 0xDF, 0x67},
    {CRC8_PROFILE_SAE_J1850, 0x4B, 0xFC},
    {CRC8_PROFILE_MAXIM_DOW, 0xA1, 0xDC},
};

/* ---------------------------------------------------------------- CRC tests */

static void test_crc_profiles(void)
{
    printf("crc: catalogue check values\n");
    for (size_t i = 0; i < sizeof(kCrcVectors) / sizeof(kCrcVectors[0]); i++) {
        const crc8_profile_t p = kCrcVectors[i].profile;
        CHECK_EQ_INT(crc8_compute(p, kCheckString, sizeof(kCheckString)), kCrcVectors[i].check);
        CHECK_EQ_INT(crc8_catalogue_check(p), kCrcVectors[i].check);
    }

    /* Spec section 4: CRC-8/SMBUS of ASCII "123456789" is 0xF4. */
    CHECK_EQ_INT(crc8_compute(CRC8_PROFILE_SMBUS, kCheckString, sizeof(kCheckString)), 0xF4);

    printf("crc: frame checksum covers CONTROL then DATA\n");
    CHECK_EQ_INT(crc8_frame_checksum(FRAME_TYPE_DATA, 0x41), kCrcVectors[0].data_0x41);
    CHECK_EQ_INT(crc8_compute(CRC8_PROFILE_SMBUS, (const uint8_t[]){0x01, 0x41}, 2), 0xD5);
    CHECK_EQ_INT(crc8_frame_checksum(FRAME_TYPE_ACK, 0x00), 0x2A);
    CHECK_EQ_INT(crc8_frame_checksum(FRAME_TYPE_NACK, NACK_REASON_MISSING_LANE), 0x36);
    CHECK_EQ_INT(crc8_frame_checksum(FRAME_TYPE_END, 0x00), 0x41);

    /* The high nibble of CONTROL must not leak into the CRC input. */
    CHECK_EQ_INT(crc8_frame_checksum(0xF1, 0x41), crc8_frame_checksum(0x01, 0x41));
}

/* -------------------------------------------------------------- frame tests */

static const char *bits_string(const phy_frame_t *frame)
{
    static char buf[FRAME_BITS + 1];
    for (int i = 0; i < FRAME_BITS; i++) {
        buf[i] = frame->bits[i] ? '1' : '0';
    }
    buf[FRAME_BITS] = '\0';
    return buf;
}

static void test_frame_layout(void)
{
    printf("frame: 36-bit layout, SYN/SFD/CONTROL/DATA/CRC\n");
    phy_frame_t frame;
    frame_build(&frame, FRAME_TYPE_DATA, 0x41);

    CHECK_EQ_INT(frame.control, 0x01);
    CHECK_EQ_INT(frame.data, 0x41);
    CHECK_EQ_INT(frame.crc, 0xD5);

    /* SYN = 0x66 = 01100110 */
    static const uint8_t syn[8] = {0, 1, 1, 0, 0, 1, 1, 0};
    static const uint8_t sfd[8] = {1, 1, 0, 1, 0, 1, 0, 1}; /* 0xD5 */
    CHECK(memcmp(&frame.bits[0], syn, 8) == 0);
    CHECK(memcmp(&frame.bits[8], sfd, 8) == 0);

    /* CONTROL 0x1 -> 0001, MSB first */
    CHECK_EQ_INT(frame.bits[16], 0);
    CHECK_EQ_INT(frame.bits[17], 0);
    CHECK_EQ_INT(frame.bits[18], 0);
    CHECK_EQ_INT(frame.bits[19], 1);

    /* DATA 0x41 = 01000001 */
    static const uint8_t data_bits[8] = {0, 1, 0, 0, 0, 0, 0, 1};
    CHECK(memcmp(&frame.bits[20], data_bits, 8) == 0);

    /* CRC 0xD5 = 11010101 */
    static const uint8_t crc_bits[8] = {1, 1, 0, 1, 0, 1, 0, 1};
    CHECK(memcmp(&frame.bits[28], crc_bits, 8) == 0);

    /* Full bit string from an independent calculation. */
    CHECK(strcmp(bits_string(&frame), "011001101101010100010100000111010101") == 0);
}

static void test_frame_roundtrip(void)
{
    printf("frame: serialize/deserialize every type and edge payload\n");

    static const uint8_t controls[] = {FRAME_TYPE_DATA, FRAME_TYPE_ACK, FRAME_TYPE_NACK,
                                       FRAME_TYPE_TURN, FRAME_TYPE_END};
    static const uint8_t payloads[] = {0x00, 0xFF, 0x41, 0x55, 0xAA, 0x01, 0x7F, 0x80};

    for (size_t c = 0; c < sizeof(controls); c++) {
        for (size_t d = 0; d < sizeof(payloads); d++) {
            phy_frame_t built;
            frame_build(&built, controls[c], payloads[d]);

            phy_frame_t parsed;
            CHECK_EQ_INT(frame_parse_bits(built.bits, &parsed), FRAME_PARSE_OK);
            CHECK_EQ_INT(parsed.control, controls[c]);
            CHECK_EQ_INT(parsed.data, payloads[d]);
            CHECK_EQ_INT(parsed.crc, built.crc);
            CHECK(memcmp(parsed.bits, built.bits, FRAME_BITS) == 0);

            /* Through the two lanes and back. */
            uint8_t ir[FRAME_CYCLES];
            uint8_t wifi[FRAME_CYCLES];
            uint8_t bits[FRAME_BITS];
            frame_to_lanes(&built, ir, wifi);
            frame_from_lanes(ir, wifi, bits);
            CHECK(memcmp(bits, built.bits, FRAME_BITS) == 0);

            phy_frame_t from_lanes;
            CHECK_EQ_INT(frame_parse_bits(bits, &from_lanes), FRAME_PARSE_OK);
            CHECK_EQ_INT(from_lanes.data, payloads[d]);
        }
    }
}

static void test_lane_pairing(void)
{
    printf("frame: IR carries 2*k, ESP-NOW carries 2*k+1\n");
    phy_frame_t frame;
    frame_build(&frame, FRAME_TYPE_DATA, 0x41);

    uint8_t ir[FRAME_CYCLES];
    uint8_t wifi[FRAME_CYCLES];
    frame_to_lanes(&frame, ir, wifi);

    for (int k = 0; k < FRAME_CYCLES; k++) {
        CHECK_EQ_INT(ir[k], frame.bits[2 * k]);
        CHECK_EQ_INT(wifi[k], frame.bits[2 * k + 1]);
        /* The two lanes must not be assembled as 18 + 18 bits. */
        CHECK_EQ_INT(frame.bits[0], ir[0]);
        CHECK_EQ_INT(frame.bits[1], wifi[0]);
    }

    /* Spec section 3 example: DATA 0x41 sits in cycles 10..13. */
    CHECK_EQ_INT(ir[10] * 2 + wifi[10], 1); /* pair 01 */
    CHECK_EQ_INT(ir[11] * 2 + wifi[11], 0); /* pair 00 */
    CHECK_EQ_INT(ir[12] * 2 + wifi[12], 0); /* pair 00 */
    CHECK_EQ_INT(ir[13] * 2 + wifi[13], 1); /* pair 01 */
}

static void test_bit_flip_detection(void)
{
    printf("frame: single-bit corruption is rejected\n");
    for (uint8_t control = 1; control <= 5; control++) {
        phy_frame_t frame;
        frame_build(&frame, control, 0x5A);

        /* The protected region is CONTROL + DATA = bits 16..27. */
        for (int bit = 16; bit < 28; bit++) {
            uint8_t corrupted[FRAME_BITS];
            memcpy(corrupted, frame.bits, FRAME_BITS);
            corrupted[bit] ^= 1u;

            phy_frame_t parsed;
            const frame_parse_result_t result = frame_parse_bits(corrupted, &parsed);
            CHECK(result != FRAME_PARSE_OK);
            if (bit >= 20) {
                /* DATA damage leaves CONTROL intact but must still be caught. */
                CHECK_EQ_INT(parsed.control, control);
                CHECK_EQ_INT(result, FRAME_PARSE_BAD_CRC);
            }
        }
    }

    /* SYN and SFD damage must be reported as framing, not as a CRC error. */
    phy_frame_t frame;
    frame_build(&frame, FRAME_TYPE_DATA, 0x00);
    uint8_t bits[FRAME_BITS];
    memcpy(bits, frame.bits, FRAME_BITS);
    bits[0] ^= 1u;
    CHECK_EQ_INT(frame_parse_bits(bits, NULL), FRAME_PARSE_BAD_FRAMING);

    memcpy(bits, frame.bits, FRAME_BITS);
    bits[15] ^= 1u;
    CHECK_EQ_INT(frame_parse_bits(bits, NULL), FRAME_PARSE_BAD_FRAMING);

    /* A well-framed frame with a reserved CONTROL value. */
    phy_frame_t reserved;
    frame_build(&reserved, FRAME_TYPE_DATA, 0x10);
    memcpy(bits, reserved.bits, FRAME_BITS);
    bits[16] = 0;
    bits[17] = 1;
    bits[18] = 1;
    bits[19] = 0; /* 0x6 is reserved */
    CHECK_EQ_INT(frame_parse_bits(bits, NULL), FRAME_PARSE_UNSUPPORTED);
}

/* ------------------------------------------------------- transaction tests */

static void fill_transaction(rx_transaction_t *txn, const phy_frame_t *frame, uint16_t seq,
                             bool in_order)
{
    uint8_t ir[FRAME_CYCLES];
    uint8_t wifi[FRAME_CYCLES];
    frame_to_lanes(frame, ir, wifi);

    int64_t now = 1000;
    if (in_order) {
        for (int k = 0; k < FRAME_CYCLES; k++) {
            rx_txn_on_lane_bit(txn, 1, seq, (uint8_t)k, wifi[k], now++);
            rx_txn_on_edge(txn, ir[k], now++);
        }
    } else {
        /* Lane packets all before the clock burst, and out of order. */
        for (int k = FRAME_CYCLES - 1; k >= 0; k--) {
            rx_txn_on_lane_bit(txn, 1, seq, (uint8_t)k, wifi[k], now++);
        }
        for (int k = 0; k < FRAME_CYCLES; k++) {
            rx_txn_on_edge(txn, ir[k], now++);
        }
    }
}

static void test_transaction_assembly(void)
{
    printf("rx: 18 cycles assemble back to the frame, in order and out of order\n");

    phy_frame_t frame;
    frame_build(&frame, FRAME_TYPE_NACK, NACK_REASON_CRC);

    for (int pass = 0; pass < 2; pass++) {
        rx_transaction_t txn;
        rx_txn_clear(&txn);
        fill_transaction(&txn, &frame, 7, pass == 0);

        CHECK_EQ_INT(txn.edge_count, FRAME_CYCLES);
        CHECK(rx_txn_is_complete(&txn));
        CHECK(!rx_txn_missing_lane(&txn));

        uint8_t bits[FRAME_BITS];
        CHECK(rx_txn_assemble(&txn, bits));

        phy_frame_t parsed;
        CHECK_EQ_INT(frame_parse_bits(bits, &parsed), FRAME_PARSE_OK);
        CHECK_EQ_INT(parsed.control, FRAME_TYPE_NACK);
        CHECK_EQ_INT(parsed.data, NACK_REASON_CRC);
    }
}

static void test_transaction_delayed_and_missing(void)
{
    printf("rx: late packets fill their own slot; a missing bit does not shift\n");

    phy_frame_t frame;
    frame_build(&frame, FRAME_TYPE_DATA, 0xAA);
    uint8_t ir[FRAME_CYCLES];
    uint8_t wifi[FRAME_CYCLES];
    frame_to_lanes(&frame, ir, wifi);

    rx_transaction_t txn;
    rx_txn_clear(&txn);

    int64_t now = 5000;
    /* All 18 edges first (no lane data yet): a complete frame is not claimed. */
    for (int k = 0; k < FRAME_CYCLES; k++) {
        rx_txn_on_edge(&txn, ir[k], now++);
    }
    CHECK(rx_txn_is_complete(&txn));
    CHECK(rx_txn_missing_lane(&txn));

    /* Cycle 5 is lost; all late packets arrive out of order afterwards. */
    for (int k = FRAME_CYCLES - 1; k >= 0; k--) {
        if (k == 5) {
            continue;
        }
        rx_txn_on_lane_bit(&txn, 1, 9, (uint8_t)k, wifi[k], now++);
    }
    CHECK(rx_txn_missing_lane(&txn));

    uint8_t bits[FRAME_BITS];
    CHECK(!rx_txn_assemble(&txn, bits));

    /* Fill the gap: the assembled frame equals the transmitted one. */
    rx_txn_on_lane_bit(&txn, 1, 9, 5, wifi[5], now++);
    CHECK(!rx_txn_missing_lane(&txn));
    CHECK(rx_txn_assemble(&txn, bits));

    phy_frame_t parsed;
    CHECK_EQ_INT(frame_parse_bits(bits, &parsed), FRAME_PARSE_OK);
    CHECK_EQ_INT(parsed.data, 0xAA);
}

static void test_transaction_duplicates_and_rejects(void)
{
    printf("rx: duplicates, conflicting duplicates, foreign sequences, bad args\n");

    phy_frame_t frame;
    frame_build(&frame, FRAME_TYPE_DATA, 0x55);
    uint8_t ir[FRAME_CYCLES];
    uint8_t wifi[FRAME_CYCLES];
    frame_to_lanes(&frame, ir, wifi);

    rx_transaction_t txn;
    rx_txn_clear(&txn);

    /* Identical duplicate: ignored, transaction stays valid. */
    CHECK_EQ_INT(rx_txn_on_lane_bit(&txn, 1, 3, 0, wifi[0], 10), RX_TXN_STORED);
    CHECK_EQ_INT(rx_txn_on_lane_bit(&txn, 1, 3, 0, wifi[0], 11), RX_TXN_DUPLICATE);
    CHECK(!txn.invalid);

    /* Conflicting duplicate: invalidates. */
    rx_txn_clear(&txn);
    rx_txn_on_lane_bit(&txn, 1, 3, 0, wifi[0], 10);
    CHECK_EQ_INT(rx_txn_on_lane_bit(&txn, 1, 3, 0, !wifi[0], 11), RX_TXN_CONFLICT);
    CHECK(txn.invalid);

    /* Foreign sequence while data is present: rejected. */
    rx_txn_clear(&txn);
    rx_txn_on_lane_bit(&txn, 1, 3, 0, wifi[0], 10);
    rx_txn_on_edge(&txn, ir[0], 11);
    CHECK_EQ_INT(rx_txn_on_lane_bit(&txn, 1, 4, 1, wifi[1], 12), RX_TXN_REJECTED);
    CHECK(txn.invalid);

    /* A packet that arrives before the first edge identifies the frame. */
    rx_txn_clear(&txn);
    CHECK_EQ_INT(rx_txn_on_lane_bit(&txn, 1, 5, 2, wifi[2], 10), RX_TXN_STORED);
    CHECK_EQ_INT(txn.frame_seq, 5);
    CHECK_EQ_INT(txn.edge_count, 0);

    /* Out of range cycle, invalid bit value. */
    CHECK_EQ_INT(rx_txn_on_lane_bit(&txn, 1, 5, FRAME_CYCLES, 0, 12), RX_TXN_BAD_ARG);
    CHECK_EQ_INT(rx_txn_on_lane_bit(&txn, 1, 5, 0, 2, 12), RX_TXN_BAD_ARG);
    CHECK_EQ_INT(rx_txn_on_lane_bit(&txn, 1, 5, 0, 0, 12), RX_TXN_STORED);
    CHECK_EQ_INT(rx_txn_on_edge(&txn, 2, 13), RX_TXN_BAD_ARG);

    /* A 19th rising edge invalidates the transaction. */
    rx_txn_clear(&txn);
    for (int k = 0; k < FRAME_CYCLES; k++) {
        rx_txn_on_edge(&txn, ir[k], 100 + k);
    }
    CHECK_EQ_INT(rx_txn_on_edge(&txn, ir[0], 200), RX_TXN_OVERFLOW);
    CHECK(txn.invalid);

    /* Timeout is measured from the first event of the transaction. */
    rx_txn_clear(&txn);
    rx_txn_on_edge(&txn, 0, 1000);
    CHECK(!rx_txn_expired(&txn, 1000 + 999, 1000));
    CHECK(rx_txn_expired(&txn, 1000 + 1000, 1000));
}

static void test_sequence_and_duplicate_cache(void)
{
    printf("proto: sequence expectations and exactly-once response cache\n");

    rx_seq_t tracker;
    rx_seq_reset(&tracker);
    CHECK(rx_seq_accepts(&tracker, 42));      /* first request of a session */
    rx_seq_commit(&tracker, 42);
    CHECK(!rx_seq_accepts(&tracker, 42));     /* retransmission, handled by cache */
    CHECK(rx_seq_accepts(&tracker, 43));
    CHECK(!rx_seq_accepts(&tracker, 45));
    rx_seq_commit(&tracker, 43);

    /* Wraparound is modulo 65536. */
    rx_seq_reset(&tracker);
    rx_seq_commit(&tracker, 0xFFFF);
    CHECK(rx_seq_accepts(&tracker, 0x0000));

    /* A TURN handoff starts a new era: any first sequence is accepted. */
    rx_seq_reset(&tracker);
    CHECK(rx_seq_accepts(&tracker, 900));

    resp_cache_t cache;
    resp_cache_clear(&cache);
    CHECK(!resp_cache_matches(&cache, 1, 10, FRAME_TYPE_DATA, 0x41));

    resp_cache_store(&cache, 1, 10, FRAME_TYPE_DATA, 0x41, FRAME_TYPE_ACK, 0x00);
    CHECK(resp_cache_matches(&cache, 1, 10, FRAME_TYPE_DATA, 0x41));
    CHECK_EQ_INT(cache.resp_control, FRAME_TYPE_ACK);

    /* Same sequence but different content is a new request, not a duplicate. */
    CHECK(!resp_cache_matches(&cache, 1, 10, FRAME_TYPE_DATA, 0x42));
    CHECK(!resp_cache_matches(&cache, 1, 10, FRAME_TYPE_END, 0x00));
    CHECK(!resp_cache_matches(&cache, 2, 10, FRAME_TYPE_DATA, 0x41));

    /* After END the next request may reuse a sequence number. */
    resp_cache_store(&cache, 1, 1, FRAME_TYPE_END, 0x00, FRAME_TYPE_ACK, 0x00);
    CHECK(!resp_cache_matches(&cache, 1, 1, FRAME_TYPE_DATA, 0x41));

    resp_cache_clear(&cache);
    CHECK(!resp_cache_matches(&cache, 1, 1, FRAME_TYPE_END, 0x00));
}

static void test_type_helpers(void)
{
    printf("frame: type classification matches the specification table\n");
    CHECK(frame_type_is_supported(1) && frame_type_is_supported(2) && frame_type_is_supported(3));
    CHECK(frame_type_is_supported(4) && frame_type_is_supported(5));
    CHECK(!frame_type_is_supported(0));
    for (uint8_t t = 6; t <= 0x0F; t++) {
        CHECK(!frame_type_is_supported(t));
    }
    CHECK(frame_type_is_request(FRAME_TYPE_DATA));
    CHECK(frame_type_is_request(FRAME_TYPE_TURN));
    CHECK(frame_type_is_request(FRAME_TYPE_END));
    CHECK(!frame_type_is_request(FRAME_TYPE_ACK));
    CHECK(frame_type_is_response(FRAME_TYPE_ACK));
    CHECK(frame_type_is_response(FRAME_TYPE_NACK));
    CHECK(!frame_type_is_response(FRAME_TYPE_DATA));
    CHECK(strcmp(frame_type_name(FRAME_TYPE_TURN), "TURN") == 0);
    CHECK(strcmp(frame_nack_reason_name(NACK_REASON_MISSING_LANE), "missing-lane-bit") == 0);
}

int main(void)
{
    printf("dual-medium ESP32 PHY protocol -- host unit tests\n\n");

    test_crc_profiles();
    test_frame_layout();
    test_frame_roundtrip();
    test_lane_pairing();
    test_bit_flip_detection();
    test_type_helpers();
    test_transaction_assembly();
    test_transaction_delayed_and_missing();
    test_transaction_duplicates_and_rejects();
    test_sequence_and_duplicate_cache();

    printf("\n%d checks, %d failure(s)\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
