/*
 * Wired clock channel (spec sections 2 and 7).
 *
 * GPIO23 (CLK_TX) is a push-pull output that idles LOW and only produces
 * pulses while this board is clocking out a frame.  GPIO22 (CLK_RX) is an
 * input; the rising-edge handler in this module latches the clock level and
 * the infrared bit sampled at that instant, then hands the record to the
 * protocol task.  No protocol logic runs in the ISR.
 */
#ifndef PHY_CLOCK_GPIO_H
#define PHY_CLOCK_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    int64_t ts_us;     /* esp_timer_get_time() at the edge */
    uint8_t clk_level; /* level after the edge (1 = rising edge) */
    uint8_t ir_bit;    /* infrared logical bit sampled at the edge */
} clk_edge_t;

/* edge_queue receives clk_edge_t records; it must already exist. */
esp_err_t clock_gpio_init(QueueHandle_t edge_queue);

/* Nominal cycle rate.  Pulse widths become half a period high, half a period
 * low; the real rate is lower because every cycle waits for ESP-NOW. */
esp_err_t clock_set_rate_hz(uint32_t hz);
uint32_t clock_rate_hz(void);
uint32_t clock_high_us(void);
uint32_t clock_low_us(void);

/* Clock output control.  clock_tx_pulse() is HIGH for CLK_HIGH_US and then
 * LOW for at least CLK_LOW_MIN_US, both busy-waited so the pulse width does
 * not depend on the scheduler tick. */
void clock_tx_low(void);
void clock_tx_pulse(void);

/* Rising edges seen since boot, maintained by the ISR. */
uint32_t clock_rx_edge_count(void);

#endif /* PHY_CLOCK_GPIO_H */
