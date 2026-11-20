# physical-layer

Firmware for the dual-medium ESP32 PHY link described in
`PHY_PROTOCOL_IMPLEMENTATION.md` (spec v1.0): two ESP32-WROOM-32 boards talk
over three channels — a cross-wired clock, an infrared data lane and an
ESP-NOW data lane that carries one bit per packet.

```
 board A  GPIO23 (CLK_TX) --------> GPIO22 (CLK_RX)  board B
 board A  GPIO22 (CLK_RX) <-------- GPIO23 (CLK_TX)  board B
 board A  GND --------------------- GND              board B
 board A  GPIO18 -> IR LED  ~~~~~>  IR detector -> GPIO26  board B
 board B  GPIO18 -> IR LED  ~~~~~>  IR detector -> GPIO26  board A
 board A  ESP-NOW  <~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~>  ESP-NOW  board B
```

### Wiring cautions

- Do **not** tie the two `CLK_TX` pins together; each drives only the other
  board's `CLK_RX`.
- Common ground is required. Do not join the 3.3 V rails just because the
  clock wires are shared.
- Drive the IR LEDs from a transistor/MOSFET with a series resistor; never
  from a GPIO directly. Keep the wires short and consider 220–1000 Ω in series
  with the clock lines while bringing the link up.
- GPIO26 expects a 3.3 V-compatible receiver output. Shield the emitter from
  its own board's detector if the reverse direction turns out noisy.

A 36-bit frame (`SYN 0x66 | SFD 0xD5 | CONTROL | DATA | CRC-8`) is split over
18 clock cycles: the IR lane carries logical bit `2k`, the ESP-NOW lane carries
`2k+1`. Every frame is answered with a complete ACK or NACK frame clocked from
the responder's own `CLK_TX`.

## Layout

| Path | Contents |
|---|---|
| `main/protocol_config.h` | pins, IDs, timings, compile-time switches |
| `main/frame.[ch]` | 36-bit serialisation, lane pairing, structural validation |
| `main/crc8.[ch]` | CRC-8/SMBUS (default) plus AUTOSAR, SAE-J1850, MAXIM-DOW |
| `main/clock_gpio.[ch]` | `CLK_TX` pulses, `CLK_RX` rising-edge ISR |
| `main/infrared.[ch]` | IR symbol TX/RX, modulated or static |
| `main/espnow_lane.[ch]` | 7-byte one-bit envelope, Wi-Fi/ESP-NOW bring-up |
| `main/transaction.[ch]` | cycle slots, duplicates, timeouts, sequence/dedup rules |
| `main/protocol_fsm.[ch]` | stop-and-wait FSM, retries, TURN/END, message queue |
| `main/serial_cli.[ch]` | console commands and the `@PHY` event lines |
| `main/diagnostics.[ch]` | counters and log macros |
| `main/app_main.c` | startup and banner |
| `host_test/` | host unit tests for the pure-C modules |
| `tools/board-cli/` | USB-serial host tool (see its own README) |

The firmware is C on ESP-IDF (built and verified with IDF v6.1.1, target
`esp32`). Both boards run the same binary; only `CONFIG_PHY_NODE_ROLE` and
optionally the peer MAC differ.

## Build, configure, flash

```sh
. ~/.espressif/tools/activate_idf_v6.1.1.sh   # or your IDF export script
idf.py set-target esp32                       # first time only
idf.py menuconfig                             # "Physical layer protocol"
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

`menuconfig` → *Physical layer protocol*:

| Option | Default | Notes |
|---|---|---|
| Node role | A | A originates DATA at boot, B answers |
| Peer Wi-Fi MAC | empty | empty = broadcast + latch the first sender |
| Session ID | 1 | identical on both boards |
| Wi-Fi channel | 6 | identical on both boards |
| CRC-8 profile | SMBUS | both boards must be built identically |
| IR demodulating receiver | y | 38 kHz carrier burst = logical 1 |
| IR receiver active low | y | TSOP/VS1838 class modules |
| Verbose per-cycle logging | n | for timing measurements |

Board B: set the role to *Node B* and rebuild/flash. If the peer MAC is left
empty, the firmware sends to the broadcast address and locks onto the first
foreign sender for validation; that is convenient in a two-board lab but does
not reject third-party traffic that arrives first.

## Using the link

Either drive it with `tools/board-cli` (recommended, see
`tools/board-cli/README.md`) or type into a serial terminal at 115200 baud:

```
SEND hello          one DATA frame per byte, then END
SENDHEX 68 65 6c    arbitrary bytes
CLK 20              nominal wired-clock rate in Hz (5..500, RATE is an alias)
IRFREQ 40000        IR carrier frequency in Hz (30..56 kHz)
TIMING              print the active timings
CORRUPT             corrupt one IR bit of the next frame (NACK/retry demo)
STATUS  PING  TURN  END  RESET  HELP
```

Both rates are selectable at runtime, without rebuilding: `CLK` derives the
pulse widths (half a period high, half a period low) and `IRFREQ` retunes the
LEDC carrier. The clock rate is *nominal* — each cycle still waits for its
ESP-NOW completion, so the effective rate is at or below the setting; that is
what makes it safe to slow the link down for observation with a logic analyzer.

Every protocol step is logged, so the host tool can show the exchange as it
happens:

```
[phy]  step: LINK_IDLE -> TX_FRAME
[phy]  TX DATA seq=1 data=0x68 (attempt 1 of 4)
[phy]  step: TX_FRAME -> WAIT_RESPONSE
[phy]  ACK seq=1 for DATA after 312 ms
[phy]  step: WAIT_RESPONSE -> LINK_IDLE
```

The current originator prints every received request, and answers with a full
ACK frame. `TURN` hands DATA authority to the peer (acknowledged like DATA);
`END` closes the message on both sides. A line whose first token is not an
all-capitals command is sent as a message, so plain typing works too.

Machine-readable events share the UART with the log:

```
@PHY READY role=A session=0x0001 crc=CRC-8/SMBUS channel=6
@PHY TX type=DATA seq=1 data=0x68 status=ACK retries=0
@PHY RX type=DATA seq=1 data=0x68
@PHY REPLY type=ACK seq=1 data=0x00
@PHY TXDONE first_seq=1 last_seq=6 bytes=5 status=OK
@PHY RXMSG bytes=5 text="hello"
@PHY STATE link=IDLE owner=A seq_next=7 role=A session=0x0001
@PHY STATS lane_tx=... lane_rx=... frames_tx=... frames_rx=...
@PHY CMD ok=1 cmd=SEND bytes=5
@PHY TXDONE bytes=0 status=FAIL reason=NOT_OWNER     (a rejected SEND)
@PHY ERR msg="retries exhausted: type RESET on BOTH boards ..."
```

`RXMSG` follows `RXEND`; the host tool keeps the one message it assembled from
the per-frame `RX` lines and drops the duplicate, so a message is shown once.
A rejected `SEND` always ends with a `TXDONE ... status=FAIL reason=...`
(`NOT_OWNER` or `REJECTED`) so the host never waits out a timeout. An unknown
all-capitals command is answered with `@PHY ERR` and `@PHY CMD ok=0`.

Log lines keep their plain `[phy]`, `[phy!]`, `[phyX]`, `[phy.]` prefixes.

## Timing behaviour

One cycle is paced by ESP-NOW completion, never by a fixed clock: the
transmitter asserts the IR symbol, submits the one-bit ESP-NOW packet, waits
for the *local* send-completion callback, waits `AFTER_SEND_GUARD_MS`, then
emits one rising edge (`CLK_HIGH_US`) and holds `CLK_LOW_MIN_US` low. So a
frame costs roughly 18 × (send latency + 12 ms) ≈ 0.3–0.5 s, not 18 × 2 ms.

The constants in `main/protocol_config.h` are the spec's starter values and are
**not** measured guarantees:

```
IR_SETUP_US 2000            ESP_SEND_TIMEOUT_MS 250     AFTER_SEND_GUARD_MS 10
CLK_HIGH_US 2000            CLK_LOW_MIN_US 2000         RX_FRAME_TIMEOUT_MS 10000
RX_LANE_SETTLE_MS 100       RESPONSE_START_GUARD_MS 20  RESPONSE_FRAME_TIMEOUT_MS 10000
MAX_RETRANSMISSIONS 3
```

Every ACK prints the round trip in milliseconds; enable verbose logging to get
the per-cycle timings needed to replace these values.

Because an aborted attempt can leave fewer than 18 clock pulses behind, a
restart of that sequence waits `RX_FRAME_TIMEOUT_MS + RESPONSE_START_GUARD_MS`
(the spec's quiet-gap rule) so the receiver can drop the partial frame.

## Duplicate and retry rules

- Retransmissions reuse the same `frame_seq`; the responder caches
  `(session, seq, type, DATA)` and its answer, so DATA is delivered once and a
  repeated TURN/END is not applied twice.
- A new request must be `last_seq + 1` (mod 65536) unless the responder has
  just started an era (boot, TURN, END, RESET). Anything else is NACKed with
  reason `0x05` and is *not* adopted.
- ACK/NACK frames are never answered: an ACK for an ACK or a NACK for a NACK
  is not sent, so there is no response loop.
- After `MAX_RETRANSMISSIONS` the link enters `LINK_ERROR` and no new DATA is
  originated until `RESET` (send it on both boards).

## Assumptions that must be verified on hardware

1. **IR front end.** Compiled for a 38 kHz demodulating module with an
   active-low output (`PHY_IR_DEMODULATING`, `PHY_IR_RX_ACTIVE_LOW`). With a
   raw emitter/phototransistor pair, disable `PHY_IR_DEMODULATING` so the GPIO
   level carries the bit. Confirm the idle polarity and that a static
   level/burst is detected at the chosen symbol lengths.
2. **Timings.** `IR_SETUP_US`, `AFTER_SEND_GUARD_MS` and the timeouts must
   cover the measured ESP-NOW jitter plus margin.
3. **Channel and MACs.** Both boards on the same channel, and either the peer
   MAC configured or the broadcast fallback accepted.
4. **Optical crosstalk.** Both IR emitters can be seen by both detectors;
   shield them if the reverse direction is disturbed.
5. **Independent reboot.** A static session ID and a shared sequence history
   mean both boards must be restarted together (spec §6 and §13.5). `RESET`
   restarts the local state only; a sender that reboots mid-session will
   NACK-loop until the peer is reset too.
6. **Sequence wraparound.** After 65535 frames the firmware restarts at
   sequence 1 and logs a warning; the spec asks for an explicit session reset
   at that point.

## Tests

Pure software unit tests (no board needed):

```sh
make -C host_test
```

They compile the shipping `crc8.c`, `frame.c` and `transaction.c` and check the
CRC catalogue vectors, the 36-bit layout from the spec example, all five frame
types with edge payloads, lane pairing, single-bit corruption detection,
delayed/reordered/missing lane bits, duplicate handling, and the
sequence/response-cache rules — 642 assertions.

Hardware acceptance follows spec §12: confirm 18 rising edges per direction,
`CLK_TX` low between frames, 18 one-bit ESP-NOW messages per frame, IR `0`/`1`
in both directions, corrupt-bit NACK plus retry, dropped-lane NACK, lost-ACK
retransmission with exactly-once delivery, TURN handoff, END, and a measured
latency budget for the timeouts.
