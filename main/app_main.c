/*
 * Dual-medium ESP32 PHY protocol -- application entry point.
 *
 * Startup only: bring up NVS, verify the wire CRC profile, initialise the
 * protocol (IR, wired clock, ESP-NOW lane), then start the protocol task and
 * the serial console.
 *
 * Serial demo (spec section 12, "Initial success demonstration"):
 *   - SEND <text> (or just typing a line) turns each byte into one DATA frame;
 *   - the peer prints a byte only after the complete frame passed framing and
 *     CRC checks, and answers every DATA frame with a full ACK frame;
 *   - the message ends with an END frame, which is acknowledged as well;
 *   - TURN hands DATA authority to the peer, RESET restarts the session.
 * See serial_cli.h for the command set and the @PHY event lines.
 */
#include <stdio.h>

#include "esp_err.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "clock_gpio.h"
#include "crc8.h"
#include "diagnostics.h"
#include "espnow_lane.h"
#include "frame.h"
#include "infrared.h"
#include "protocol_config.h"
#include "protocol_fsm.h"
#include "serial_cli.h"

static void print_banner(void)
{
    uint8_t mac[6];
    char mac_str[18];
    char local[18] = "";

    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        snprintf(mac_str, sizeof(mac_str), "??");
    }
    espnow_lane_local_mac_string(local);

    printf("\n=== Dual-medium ESP32 PHY protocol =======================\n");
    printf(" role        : node %c, %s at boot\n", NODE_ROLE_A ? 'A' : 'B',
           ((NODE_ROLE_A && INITIAL_ORIGINATOR_A) || (!NODE_ROLE_A && !INITIAL_ORIGINATOR_A))
               ? "DATA originator"
               : "responder");
    printf(" station MAC : %s\n", mac_str);
    printf(" session     : 0x%04x (both boards must match)\n", SESSION_ID);
    printf(" wifi channel: %u (both boards must match)\n", ESPNOW_WIFI_CHANNEL);
    printf(" peer        : %s%s\n", espnow_lane_peer_string(),
           espnow_lane_peer_is_broadcast() ? " (broadcast, latches first sender)" : "");
    printf(" gpio        : IR tx %d, IR rx %d, CLK tx %d, CLK rx %d\n",
           GPIO_IR_TX, GPIO_IR_RX, GPIO_CLK_TX, GPIO_CLK_RX);
    printf(" CRC         : %s (both boards must be built identically)\n",
           crc8_profile_name(crc8_active_profile()));
    printf(" IR layer    : %s, receiver %s\n",
           PHY_IR_DEMODULATING ? "38 kHz carrier burst = 1" : "static level = 1 (no carrier)",
           PHY_IR_RX_ACTIVE_LOW ? "active low" : "active high");
    printf(" timings     : ir_setup %d us, after_send_guard %d ms, clk_high %d us,\n"
           "               clk_low_min %d us, rx_frame_timeout %d ms, lane_settle %d ms,\n"
           "               response_guard %d ms, response_timeout %d ms, retries %d\n",
           IR_SETUP_US, AFTER_SEND_GUARD_MS, CLK_HIGH_US, CLK_LOW_MIN_US,
           RX_FRAME_TIMEOUT_MS, RX_LANE_SETTLE_MS, RESPONSE_START_GUARD_MS,
           RESPONSE_FRAME_TIMEOUT_MS, MAX_RETRANSMISSIONS);
    if (local[0] != '\0') {
        printf(" local MAC   : %s\n", local);
    }
    printf(" these timings are lab defaults, not measured guarantees\n");
    printf("==========================================================\n\n");
}

void app_main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Verify the wire CRC profile against its catalogue check value before
     * any frame is sent (spec section 4, unit test 3). */
    static const uint8_t check_string[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    const uint8_t check = crc8_compute(crc8_active_profile(), check_string, sizeof(check_string));
    const uint8_t expected = crc8_catalogue_check(crc8_active_profile());
    printf("[phy]  CRC-8 profile %s, check(\"123456789\")=0x%02x (expected 0x%02x) %s\n",
           crc8_profile_name(crc8_active_profile()), check, expected,
           check == expected ? "OK" : "MISMATCH");
    if (check != expected) {
        printf("[phyX] CRC self-test failed; refusing to start a link that cannot interoperate\n");
        return;
    }

    ESP_ERROR_CHECK(protocol_init(&serial_cli_app));
    print_banner();

    protocol_start();
    ESP_ERROR_CHECK(serial_cli_start());

    serial_event_ready();
}
