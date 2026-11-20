/*
 * Infrared data lane (spec section 8).
 *
 * The electrical layer is hardware dependent.  Two compile-time
 * representations are provided (protocol_config.h):
 *
 *  PHY_IR_DEMODULATING = 1  -- logical 1 is a 38 kHz carrier burst, logical 0
 *      is silence, for a demodulating module (TSOP/VS1838 class) whose output
 *      is active low.  The burst must stay up long enough for the module's
 *      AGC/latency (IR_SETUP_US before the clock edge, and CLK_HIGH_US while
 *      the edge is high).
 *
 *  PHY_IR_DEMODULATING = 0  -- a static GPIO level carries the bit
 *      (HIGH = 1) for a raw emitter/phototransistor front end.
 *
 * ir_tx_set_logical_bit() asserts the symbol and leaves it asserted; the
 * caller releases it with ir_tx_idle() once the clock edge has passed.
 */
#ifndef PHY_INFRARED_H
#define PHY_INFRARED_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t ir_init(void);

/* 38 kHz class carrier frequency, runtime selectable (30..56 kHz). */
esp_err_t ir_set_carrier_hz(uint32_t hz);
uint32_t ir_carrier_hz(void);

void ir_tx_set_logical_bit(bool bit);
void ir_tx_idle(void);

/* Both are safe to call from the GPIO interrupt handler. */
bool ir_rx_sample_logical_bit(void);
bool ir_rx_raw_level(void);

#endif /* PHY_INFRARED_H */
