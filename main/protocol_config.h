/*
 * Dual-medium ESP32 PHY protocol -- build configuration.
 *
 * Everything a lab build may need to change lives here.  Values marked
 * "lab default, must be validated" are the starter constants from the
 * specification and are *not* measured guarantees.
 *
 * Kconfig-backed options (menu "Physical layer protocol") are prefixed
 * CONFIG_PHY_*.  When this header is used by the host unit-test build
 * (PHY_HOST_BUILD) the Kconfig values are absent and the fallbacks below
 * apply.
 */
#ifndef PHY_PROTOCOL_CONFIG_H
#define PHY_PROTOCOL_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef PHY_HOST_BUILD
#include "sdkconfig.h"
#endif

/* -------------------------------------------------------------------------
 * Node identity / role
 * ---------------------------------------------------------------------- */
#if defined(CONFIG_PHY_NODE_B) && CONFIG_PHY_NODE_B
#define NODE_ROLE_A 0
#else
#define NODE_ROLE_A 1
#endif

/* Originator at boot: A originates DATA, B answers.  See spec section 5. */
#if defined(CONFIG_PHY_INITIAL_ORIGINATOR_A) && !CONFIG_PHY_INITIAL_ORIGINATOR_A
#define INITIAL_ORIGINATOR_A 0
#else
#define INITIAL_ORIGINATOR_A 1
#endif

#ifndef CONFIG_PHY_SESSION_ID
#define CONFIG_PHY_SESSION_ID 0x0001
#endif
#define SESSION_ID ((uint16_t)CONFIG_PHY_SESSION_ID)

#ifndef CONFIG_PHY_WIFI_CHANNEL
#define CONFIG_PHY_WIFI_CHANNEL 6
#endif
#define ESPNOW_WIFI_CHANNEL ((uint8_t)CONFIG_PHY_WIFI_CHANNEL)

#ifndef CONFIG_PHY_PEER_MAC
#define CONFIG_PHY_PEER_MAC ""
#endif

/* First frame_seq allocated by a fresh session. */
#define SEQ_FIRST 1u

/* -------------------------------------------------------------------------
 * CRC profile (spec section 4).  Both boards must be built identically.
 * ---------------------------------------------------------------------- */
#define CRC_PROFILE_SMBUS       0
#define CRC_PROFILE_AUTOSAR     1
#define CRC_PROFILE_SAE_J1850   2
#define CRC_PROFILE_MAXIM_DOW   3

#if defined(CONFIG_PHY_CRC_AUTOSAR) && CONFIG_PHY_CRC_AUTOSAR
#define PHY_CRC_PROFILE CRC_PROFILE_AUTOSAR
#elif defined(CONFIG_PHY_CRC_SAE_J1850) && CONFIG_PHY_CRC_SAE_J1850
#define PHY_CRC_PROFILE CRC_PROFILE_SAE_J1850
#elif defined(CONFIG_PHY_CRC_MAXIM_DOW) && CONFIG_PHY_CRC_MAXIM_DOW
#define PHY_CRC_PROFILE CRC_PROFILE_MAXIM_DOW
#else
#define PHY_CRC_PROFILE CRC_PROFILE_SMBUS
#endif

/* -------------------------------------------------------------------------
 * GPIO assignment (spec section 2).  Identical on both boards.
 * ---------------------------------------------------------------------- */
#define GPIO_IR_TX   18
#define GPIO_IR_RX   26
#define GPIO_CLK_TX  23
#define GPIO_CLK_RX  22

#define CLK_IDLE_LEVEL 0

/* -------------------------------------------------------------------------
 * Infrared representation (spec section 8)
 *
 * PHY_IR_DEMODULATING=1 : the receiver is a 38 kHz demodulating module
 *   (TSOP/VS1838 class).  Logical 1 is a carrier burst ("mark"), logical 0
 *   is silence.  The demodulator output is assumed active low
 *   (PHY_IR_RX_ACTIVE_LOW=1).
 * PHY_IR_DEMODULATING=0 : a raw emitter/detector pair where the GPIO level
 *   itself carries the logical bit (HIGH = 1).
 *
 * Both are hardware assumptions that must be confirmed on the bench.
 * ---------------------------------------------------------------------- */
#if defined(CONFIG_PHY_IR_DEMODULATING) && !CONFIG_PHY_IR_DEMODULATING
#define PHY_IR_DEMODULATING 0
#else
#define PHY_IR_DEMODULATING 1
#endif

#if defined(CONFIG_PHY_IR_RX_ACTIVE_LOW) && !CONFIG_PHY_IR_RX_ACTIVE_LOW
#define PHY_IR_RX_ACTIVE_LOW 0
#else
#define PHY_IR_RX_ACTIVE_LOW 1
#endif

#ifndef CONFIG_PHY_IR_CARRIER_HZ
#define CONFIG_PHY_IR_CARRIER_HZ 38000
#endif
#define IR_CARRIER_HZ ((uint32_t)CONFIG_PHY_IR_CARRIER_HZ)

/* -------------------------------------------------------------------------
 * Timing (spec section 7).  Starter constants -- measure and replace.
 * ---------------------------------------------------------------------- */
#ifndef IR_SETUP_US
#define IR_SETUP_US              2000   /* IR symbol asserted before clock edge */
#endif
#ifndef ESP_SEND_TIMEOUT_MS
#define ESP_SEND_TIMEOUT_MS      250    /* local send-completion notification */
#endif
#ifndef AFTER_SEND_GUARD_MS
#define AFTER_SEND_GUARD_MS      10     /* airtime margin after send callback */
#endif
#ifndef CLK_HIGH_US
#define CLK_HIGH_US              2000
#endif
#ifndef CLK_LOW_MIN_US
#define CLK_LOW_MIN_US           2000
#endif
#ifndef RX_FRAME_TIMEOUT_MS
#define RX_FRAME_TIMEOUT_MS      10000  /* hard bound per 18-cycle frame */
#endif
#ifndef RX_LANE_SETTLE_MS
#define RX_LANE_SETTLE_MS        100    /* wait for a late ESP-NOW bit */
#endif
#ifndef RESPONSE_START_GUARD_MS
#define RESPONSE_START_GUARD_MS  20     /* silence before the answer starts */
#endif
#ifndef RESPONSE_FRAME_TIMEOUT_MS
#define RESPONSE_FRAME_TIMEOUT_MS 10000
#endif
#ifndef MAX_RETRANSMISSIONS
#define MAX_RETRANSMISSIONS      3      /* 1 initial + up to 3 retries */
#endif

/* Runtime-selectable rates (console: CLK <hz>, IRFREQ <hz>).
 *
 * CLK_HZ is a *nominal* cycle rate: the transmitter still waits for each
 * ESP-NOW completion, so the effective rate is always at or below it.  The
 * pulse widths are derived from it (half period high, half period low). */
#define CLK_RATE_MIN_HZ   5
#define CLK_RATE_MAX_HZ   500

#define IR_CARRIER_MIN_HZ 30000
#define IR_CARRIER_MAX_HZ 56000

/* Quiet gap required after a frame attempt that emitted fewer than 18
 * clock pulses (spec section 7, "required framing/attempt boundary rule"). */
#define QUIET_GAP_MS (RX_FRAME_TIMEOUT_MS + RESPONSE_START_GUARD_MS)

/* Backoff before retrying after an immediate (cycle 0) lane-send failure. */
#ifndef LANE_FAIL_BACKOFF_MS
#define LANE_FAIL_BACKOFF_MS     50
#endif

/* -------------------------------------------------------------------------
 * Firmware plumbing
 * ---------------------------------------------------------------------- */
#define PHY_PROTOCOL_TASK_STACK   6144
#define PHY_PROTOCOL_TASK_PRIO    10
#define PHY_EVENT_QUEUE_LEN       64
#define PHY_REQUEST_QUEUE_LEN     32
#define PHY_TX_QUEUE_LEN          192   /* message bytes + END markers */
#define PHY_MESSAGE_MAX           64    /* longest single message in bytes */
#define PHY_CONSOLE_LINE_LEN      96

#if defined(CONFIG_PHY_LOG_VERBOSE) && CONFIG_PHY_LOG_VERBOSE
#define PHY_LOG_VERBOSE 1
#else
#define PHY_LOG_VERBOSE 0
#endif

#endif /* PHY_PROTOCOL_CONFIG_H */
