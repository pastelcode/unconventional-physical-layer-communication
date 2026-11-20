/*
 * Serial control and event interface.
 *
 * Two audiences share one UART:
 *
 *  - a human watching plain log lines ("[phy] ...", "[phy!] ...");
 *  - the host tool tools/board-cli, which speaks a line protocol of
 *    "@PHY <EVENT> key=value ..." lines and writes commands as plain lines.
 *
 * Commands accepted (case-insensitive, one per line):
 *
 *   SEND <text>        queue a message; one DATA frame per byte, then END
 *   SENDHEX <hex>      same for arbitrary bytes, e.g. "68 65 6c"
 *   STATUS             emit STATE + STATS
 *   PING               emit PONG
 *   TURN               request DATA authority handoff
 *   END                close the current logical message
 *   RESET              reset local session state (reset the peer too)
 *   HELP               human help
 *
 * A line whose first token is all capitals and at least two characters long
 * is treated as a command and rejected with @PHY ERR and @PHY CMD ok=0 when
 * it is unknown.  Any other line is a SEND of the text, which keeps a plain
 * serial terminal useful without the host tool.
 */
#ifndef PHY_SERIAL_CLI_H
#define PHY_SERIAL_CLI_H

#include "esp_err.h"
#include "protocol_fsm.h"

esp_err_t serial_cli_start(void);

/* Protocol callbacks that turn FSM events into @PHY lines. */
extern const protocol_app_t serial_cli_app;

/* Events for the host tool; safe to call from any task (single printf). */
void serial_event_ready(void);
void serial_event_state(const char *state, bool is_originator, uint16_t next_seq);
void serial_event_tx(uint8_t control, uint8_t data, uint16_t seq, proto_tx_status_t status,
                     uint8_t attempts, uint8_t reason);
void serial_event_txdone(proto_tx_status_t status, uint16_t first_seq, uint16_t last_seq,
                         uint16_t bytes);
void serial_event_rx(const phy_frame_t *frame, uint16_t seq);
void serial_event_reply(uint8_t control, uint8_t data, uint16_t seq);
void serial_event_nack(uint8_t reason, uint16_t seq);
void serial_event_error(const char *message);
void serial_event_pong(void);
void serial_event_cmd(bool ok, const char *cmd, const char *detail);

#endif /* PHY_SERIAL_CLI_H */
