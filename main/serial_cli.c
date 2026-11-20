#include "serial_cli.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "clock_gpio.h"
#include "crc8.h"
#include "diagnostics.h"
#include "esp_timer.h"
#include "espnow_lane.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "infrared.h"
#include "protocol_config.h"

#define RX_MSG_MAX 256

static char s_rx_text[RX_MSG_MAX + 1];
static size_t s_rx_len;

/* ----------------------------------------------------------------- helpers */

static const char *tx_status_name(proto_tx_status_t status)
{
    switch (status) {
    case PROTO_TX_ACK:     return "ACK";
    case PROTO_TX_NACK:    return "NACK";
    case PROTO_TX_TIMEOUT: return "TIMEOUT";
    case PROTO_TX_ERROR:   return "FAIL";
    default:               return "UNKNOWN";
    }
}

static const char *txdone_reason_name(proto_tx_status_t status)
{
    switch (status) {
    case PROTO_TX_NACK:    return "NACK";
    case PROTO_TX_TIMEOUT: return "TIMEOUT";
    case PROTO_TX_ERROR:   return "RETRIES";
    default:               return NULL;
    }
}

/* Print a value as a quoted, escaped token so the host parser can read it. */
static void print_quoted(const char *text)
{
    putchar('"');
    for (const char *p = text; p != NULL && *p != '\0'; p++) {
        unsigned char c = (unsigned char)*p;
        if (c == '"' || c == '\\') {
            putchar('\\');
            putchar(c);
        } else if (c == '\n') {
            fputs("\\n", stdout);
        } else if (c == '\r') {
            fputs("\\r", stdout);
        } else if (c < 0x20 || c == 0x7F) {
            printf("\\x%02x", c);
        } else {
            putchar(c);
        }
    }
    putchar('"');
}

/* --------------------------------------------------------------- RX message
 * Every accepted DATA frame is appended; an END frame closes the message.  The
 * firmware prints RXMSG after RXEND, and the host tool drops the duplicate it
 * assembled from the per-frame RX lines, so a message is shown exactly once. */

static void rx_message_reset(void)
{
    s_rx_len = 0;
    s_rx_text[0] = '\0';
}

static void rx_message_append(uint8_t byte)
{
    if (s_rx_len < RX_MSG_MAX) {
        s_rx_text[s_rx_len++] = (char)byte;
        s_rx_text[s_rx_len] = '\0';
    }
}

static void rx_message_flush(void)
{
    bool printable = true;

    for (size_t i = 0; i < s_rx_len; i++) {
        const unsigned char c = (unsigned char)s_rx_text[i];
        if (c < 0x20 || c > 0x7E) {
            printable = false;
            break;
        }
    }

    /* Binary payloads are left to the host, which assembles them from the
     * per-frame RX lines and adds a hex dump; quoting them here could not
     * round-trip through UTF-8 and would print the message twice. */
    if (s_rx_len == 0 || !printable) {
        rx_message_reset();
        return;
    }

    printf("@PHY RXMSG bytes=%u text=", (unsigned)s_rx_len);
    print_quoted(s_rx_text);
    printf("\n");
    rx_message_reset();
}

/* ----------------------------------------------------------------- events */

void serial_event_ready(void)
{
    printf("@PHY READY role=%c session=0x%04x crc=%s channel=%u\n", NODE_ROLE_A ? 'A' : 'B',
           SESSION_ID, crc8_profile_name(crc8_active_profile()), ESPNOW_WIFI_CHANNEL);
}

/* The host contract names the states without the internal LST_ prefix. */
static const char *host_state_name(const char *state)
{
    return (strcmp(state, "LINK_IDLE") == 0) ? "IDLE" : state;
}

void serial_event_state(const char *state, bool is_originator, uint16_t next_seq)
{
    printf("@PHY STATE link=%s owner=%c seq_next=%u role=%c session=0x%04x\n",
           host_state_name(state), is_originator ? (NODE_ROLE_A ? 'A' : 'B')
                                                : (NODE_ROLE_A ? 'B' : 'A'),
           next_seq, NODE_ROLE_A ? 'A' : 'B', SESSION_ID);
}

void serial_event_tx(uint8_t control, uint8_t data, uint16_t seq, proto_tx_status_t status,
                     uint8_t attempts, uint8_t reason)
{
    printf("@PHY TX type=%s seq=%u data=0x%02x status=%s retries=%u", frame_type_name(control),
           seq, data, tx_status_name(status), (unsigned)(attempts > 0 ? attempts - 1 : 0));
    if (status == PROTO_TX_NACK && reason != 0) {
        printf(" reason=0x%02x", reason);
    }
    printf("\n");
}

void serial_event_txdone(proto_tx_status_t status, uint16_t first_seq, uint16_t last_seq,
                         uint16_t bytes)
{
    const char *reason = txdone_reason_name(status);
    printf("@PHY TXDONE first_seq=%u last_seq=%u bytes=%u status=%s", first_seq, last_seq,
           bytes, reason == NULL ? "OK" : "FAIL");
    if (reason != NULL) {
        printf(" reason=%s", reason);
    }
    printf("\n");
}

void serial_event_rx(const phy_frame_t *frame, uint16_t seq)
{
    printf("@PHY RX type=%s seq=%u data=0x%02x\n", frame_type_name(frame->control), seq,
           frame->data);
}

void serial_event_reply(uint8_t control, uint8_t data, uint16_t seq)
{
    printf("@PHY REPLY type=%s seq=%u data=0x%02x\n", frame_type_name(control), seq, data);
}

void serial_event_nack(uint8_t reason, uint16_t seq)
{
    printf("@PHY NACK reason=0x%02x seq=%u\n", reason, seq);
}

void serial_event_error(const char *message)
{
    printf("@PHY ERR msg=");
    print_quoted(message != NULL ? message : "error");
    printf("\n");
}

void serial_event_pong(void)
{
    printf("@PHY PONG uptime_ms=%lld role=%c\n",
           (long long)(esp_timer_get_time() / 1000), NODE_ROLE_A ? 'A' : 'B');
}

void serial_event_cmd(bool ok, const char *cmd, const char *detail)
{
    printf("@PHY CMD ok=%d cmd=%s", ok ? 1 : 0, cmd != NULL ? cmd : "?");
    if (detail != NULL && detail[0] != '\0') {
        printf(" detail=");
        print_quoted(detail);
    }
    printf("\n");
}

/* ------------------------------------------------------------- application
 * Callbacks handed to the protocol task.  All of them only print. */

static void app_remote_frame(const phy_frame_t *frame, uint16_t seq)
{
    if (frame->control == FRAME_TYPE_END) {
        printf("@PHY RXEND seq=%u\n", seq);
        rx_message_flush();
        return;
    }
    serial_event_rx(frame, seq);
    if (frame->control == FRAME_TYPE_DATA) {
        rx_message_append(frame->data);
    }
}

static void app_request_done(uint8_t control, uint8_t data, uint16_t seq,
                             proto_tx_status_t status, uint8_t attempts, uint8_t reason)
{
    serial_event_tx(control, data, seq, status, attempts, reason);
    if (status == PROTO_TX_NACK) {
        serial_event_nack(reason, seq);
    }
}

static void app_reply_sent(uint8_t control, uint8_t data, uint16_t seq)
{
    serial_event_reply(control, data, seq);
}

static void app_message_done(proto_tx_status_t status, uint16_t first_seq, uint16_t last_seq,
                             uint16_t bytes)
{
    serial_event_txdone(status, first_seq, last_seq, bytes);
}

static void app_status(const char *state, bool is_originator, uint16_t next_seq)
{
    serial_event_state(state, is_originator, next_seq);

    printf("@PHY STATS lane_tx=%lu lane_rx=%lu frames_tx=%lu frames_rx=%lu data=%lu "
           "crc_err=%lu missing=%lu timeouts=%lu retries=%lu clk_hz=%lu ir_hz=%lu\n",
           (unsigned long)g_phy_stats.lane_tx, (unsigned long)g_phy_stats.lane_rx,
           (unsigned long)g_phy_stats.frames_tx, (unsigned long)g_phy_stats.frames_rx,
           (unsigned long)g_phy_stats.data_delivered, (unsigned long)g_phy_stats.frames_rx_bad_crc,
           (unsigned long)g_phy_stats.frames_rx_missing_lane,
           (unsigned long)g_phy_stats.rx_timeouts, (unsigned long)g_phy_stats.retries,
           (unsigned long)clock_rate_hz(), (unsigned long)ir_carrier_hz());
}

static void app_error(const char *message)
{
    serial_event_error(message);
}

static void app_ready(void)
{
    serial_event_ready();
}

const protocol_app_t serial_cli_app = {
    .on_remote_frame = app_remote_frame,
    .on_request_done = app_request_done,
    .on_reply_sent = app_reply_sent,
    .on_message_done = app_message_done,
    .on_status = app_status,
    .on_ready = app_ready,
    .on_error = app_error,
};

/* -------------------------------------------------------------- commands */

static void print_help(void)
{
    printf("--- console ----------------------------------------------\n");
    printf(" SEND <text>      send text as DATA frames, then END\n");
    printf(" SENDHEX <hex>    send arbitrary bytes, e.g. 68 65 6c\n");
    printf(" CLK <hz>         nominal clock rate, %d..%d Hz (RATE is an alias)\n",
           CLK_RATE_MIN_HZ, CLK_RATE_MAX_HZ);
    printf(" IRFREQ <hz>      IR carrier, %d..%d Hz\n", IR_CARRIER_MIN_HZ,
           IR_CARRIER_MAX_HZ);
    printf(" TIMING           print the active timings\n");
    printf(" CORRUPT          corrupt one IR bit of the next frame\n");
    printf(" STATUS  PING  TURN  END  RESET  HELP\n");
    printf(" any other line is sent as a message\n");
    printf("----------------------------------------------------------\n");
}

static bool parse_u32(const char *text, uint32_t *out)
{
    if (text == NULL || *text == '\0') {
        return false;
    }
    char *end = NULL;
    const unsigned long value = strtoul(text, &end, 10);
    if (end == text) {
        return false;
    }
    while (end != NULL && *end != '\0' && isspace((unsigned char)*end)) {
        end++;
    }
    if (end != NULL && *end != '\0') {
        return false;
    }
    *out = (uint32_t)value;
    return true;
}

static void print_timing(void)
{
    printf("--- timings ----------------------------------------------\n");
    printf(" clock       : %u Hz nominal (%u us high, %u us low)\n",
           (unsigned)clock_rate_hz(), (unsigned)clock_high_us(), (unsigned)clock_low_us());
    printf(" IR carrier  : %u Hz\n", (unsigned)ir_carrier_hz());
    printf(" IR setup    : %d us before each clock edge\n", IR_SETUP_US);
    printf(" send guard  : %d ms after the ESP-NOW callback\n", AFTER_SEND_GUARD_MS);
    printf(" timeouts    : frame %d ms, lane settle %d ms, response %d ms\n",
           RX_FRAME_TIMEOUT_MS, RX_LANE_SETTLE_MS, RESPONSE_FRAME_TIMEOUT_MS);
    printf(" retries     : %d\n", MAX_RETRANSMISSIONS);
    printf("----------------------------------------------------------\n");
}

static bool parse_hex(const char *text, uint8_t *out, size_t max, size_t *out_len)
{
    size_t len = 0;
    const char *p = text;

    while (*p != '\0') {
        while (*p != '\0' && (isspace((unsigned char)*p) || *p == ',' || *p == ';')) {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
            p += 2;
        }
        if (!isxdigit((unsigned char)p[0])) {
            return false;
        }
        char byte_text[3] = {p[0], p[1], '\0'};
        if (!isxdigit((unsigned char)p[1])) {
            return false; /* odd number of digits */
        }
        if (len >= max) {
            return false;
        }
        out[len++] = (uint8_t)strtoul(byte_text, NULL, 16);
        p += 2;
    }
    *out_len = len;
    return len > 0;
}

/* A rejected SEND still completes the host's pending send: the contract's
 * TXDONE reason codes exist for exactly this, and without one the host would
 * wait for its timeout instead of reporting the real problem. */
static void reject_message(const char *command, const char *detail, const char *reason)
{
    serial_event_cmd(false, command, detail);
    printf("@PHY TXDONE bytes=0 status=FAIL reason=%s\n", reason);
}

static void submit_message(const uint8_t *data, size_t len, const char *command)
{
    if (len == 0) {
        reject_message(command, "empty message", "REJECTED");
        return;
    }
    if (len > PHY_MESSAGE_MAX) {
        reject_message(command, "message longer than PHY_MESSAGE_MAX", "REJECTED");
        return;
    }
    if (!protocol_is_originator()) {
        reject_message(command, "not the DATA originator; send TURN first", "NOT_OWNER");
        return;
    }
    if (!protocol_send_message(data, len)) {
        reject_message(command, "transmit queue full", "REJECTED");
        return;
    }
    printf("[app]  queued %u byte(s) for transmission\n", (unsigned)len);
    printf("@PHY CMD ok=1 cmd=%s bytes=%u\n", command, (unsigned)len);
}

static void handle_line(char *line)
{
    char *cmd = line;
    while (*cmd != '\0' && isspace((unsigned char)*cmd)) {
        cmd++;
    }
    size_t cmd_len = strlen(cmd);
    while (cmd_len > 0 && isspace((unsigned char)cmd[cmd_len - 1])) {
        cmd[--cmd_len] = '\0';
    }
    if (cmd_len == 0) {
        return;
    }

    char *arg = cmd;
    while (*arg != '\0' && !isspace((unsigned char)*arg)) {
        arg++;
    }
    if (*arg != '\0') {
        *arg++ = '\0'; /* exactly one separator, the rest is the payload */
    }

    if (strcasecmp(cmd, "SEND") == 0) {
        submit_message((const uint8_t *)arg, strlen(arg), "SEND");
        return;
    }
    if (strcasecmp(cmd, "SENDHEX") == 0) {
        uint8_t bytes[PHY_MESSAGE_MAX];
        size_t len = 0;
        if (!parse_hex(arg, bytes, sizeof(bytes), &len)) {
            serial_event_cmd(false, "SENDHEX", "expected hex bytes");
            return;
        }
        submit_message(bytes, len, "SENDHEX");
        return;
    }
    if (strcasecmp(cmd, "STATUS") == 0) {
        protocol_post(PROTO_REQ_STATUS);
        serial_event_cmd(true, "STATUS", NULL);
        return;
    }
    if (strcasecmp(cmd, "PING") == 0) {
        serial_event_pong();
        serial_event_cmd(true, "PING", NULL);
        return;
    }
    if (strcasecmp(cmd, "TURN") == 0 || strcasecmp(cmd, "!turn") == 0) {
        protocol_post(PROTO_REQ_TURN);
        serial_event_cmd(true, "TURN", NULL);
        return;
    }
    if (strcasecmp(cmd, "END") == 0 || strcasecmp(cmd, "!end") == 0) {
        protocol_post(PROTO_REQ_END);
        serial_event_cmd(true, "END", NULL);
        return;
    }
    if (strcasecmp(cmd, "RESET") == 0 || strcasecmp(cmd, "!reset") == 0) {
        protocol_post(PROTO_REQ_RESET);
        serial_event_cmd(true, "RESET", NULL);
        return;
    }
    if (strcasecmp(cmd, "CLK") == 0 || strcasecmp(cmd, "RATE") == 0) {
        uint32_t hz = 0;
        if (!parse_u32(arg, &hz)) {
            serial_event_cmd(false, "CLK", "usage: CLK <hz>");
            return;
        }
        if (clock_set_rate_hz(hz) != ESP_OK) {
            serial_event_cmd(false, "CLK", "rate must be 5..500 Hz");
            return;
        }
        char detail[72];
        snprintf(detail, sizeof(detail), "%u Hz nominal, %u us high, %u us low",
                 (unsigned)clock_rate_hz(), (unsigned)clock_high_us(),
                 (unsigned)clock_low_us());
        PHY_LOGI("clock rate set: %s", detail);
        serial_event_cmd(true, "CLK", detail);
        return;
    }
    if (strcasecmp(cmd, "IRFREQ") == 0) {
        uint32_t hz = 0;
        if (!parse_u32(arg, &hz)) {
            serial_event_cmd(false, "IRFREQ", "usage: IRFREQ <hz>");
            return;
        }
        if (ir_set_carrier_hz(hz) != ESP_OK) {
            serial_event_cmd(false, "IRFREQ", "carrier must be 30000..56000 Hz");
            return;
        }
        char detail[48];
        snprintf(detail, sizeof(detail), "%u Hz carrier", (unsigned)ir_carrier_hz());
        PHY_LOGI("IR carrier set: %s", detail);
        serial_event_cmd(true, "IRFREQ", detail);
        return;
    }
    if (strcasecmp(cmd, "TIMING") == 0) {
        print_timing();
        serial_event_cmd(true, "TIMING", NULL);
        return;
    }
    if (strcasecmp(cmd, "CORRUPT") == 0 || strcasecmp(cmd, "!corrupt") == 0) {
        protocol_inject_ir_bit_error_once();
        printf("[app]  next request frame will carry one corrupted IR bit\n");
        serial_event_cmd(true, "CORRUPT", NULL);
        return;
    }
    if (strcasecmp(cmd, "HELP") == 0 || strcasecmp(cmd, "?") == 0 ||
        strcasecmp(cmd, "!help") == 0) {
        print_help();
        serial_event_cmd(true, "HELP", NULL);
        return;
    }

    /* A command-shaped token (all capitals, at least two characters) that is
     * not recognised is an error; anything else is a human typing a message
     * into a serial terminal, which would be useless if it were rejected. */
    bool command_shaped = true;
    size_t token_len = 0;
    for (const char *p = cmd; *p != '\0' && !isspace((unsigned char)*p); p++) {
        if (!isupper((unsigned char)*p) && !isdigit((unsigned char)*p) && *p != '_') {
            command_shaped = false;
            break;
        }
        token_len++;
    }
    if (command_shaped && token_len >= 2) {
        char message[64];
        snprintf(message, sizeof(message), "unknown command: %s", cmd);
        serial_event_error(message);
        serial_event_cmd(false, cmd, "unknown command");
        return;
    }

    submit_message((const uint8_t *)cmd, strlen(cmd), "SEND");
}

static void cli_task(void *arg)
{
    (void)arg;

    char line[PHY_CONSOLE_LINE_LEN];
    size_t len = 0;
    bool overflow = false;

    print_help();

    for (;;) {
        const int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            line[len] = '\0';
            if (overflow) {
                /* Never act on a silently truncated command or message. */
                const bool send_like = strncasecmp(line, "SEND", 4) == 0;
                serial_event_error("console line too long; command not executed");
                serial_event_cmd(false, send_like ? "SEND" : "?", "console line too long");
                if (send_like) {
                    printf("@PHY TXDONE bytes=0 status=FAIL reason=REJECTED\n");
                }
            } else {
                handle_line(line);
            }
            len = 0;
            overflow = false;
            continue;
        }
        if (c == 0x7F || c == '\b') {
            if (len > 0) {
                len--;
            }
            overflow = false;
            continue;
        }
        if (len + 1 < sizeof(line)) {
            line[len++] = (char)c;
        } else {
            overflow = true;
        }
    }
}

esp_err_t serial_cli_start(void)
{
    rx_message_reset();
    if (xTaskCreate(cli_task, "console", 4096, NULL, 5, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
