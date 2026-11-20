# Board CLI — USB serial control for the dual-medium PHY link

A host-side command-line tool that talks to one ESP32 board over its USB cable.
It sends a short text message (the firmware turns each byte into a DATA frame),
shows every frame result, and assembles the message received from the other
board — so you can test the link without wiring anything except USB.

```
host ── USB ──> board A ──IR + ESP-NOW + clock──> board B
                    └──────── ACK/NACK ───────────┘
```

The tool lives entirely in this folder. It does not change the firmware; it
speaks a small line protocol that the firmware's console task must print and
accept. That protocol is specified below.

## Requirements

- macOS (also works on Linux) and [uv](https://docs.astral.sh/uv/), or a
  Python 3.9+ environment with `pyserial`.
- One ESP32 dev board connected by USB. Both boards must be powered and running
  the same firmware build, with the A/B role and peer MAC configured.

`uv` reads the dependency block at the top of `board_cli.py`; no virtualenv or
global install is needed. The CLI has no build step.

## Quick start

```sh
# 1. Find the board's port. With one board attached it is the only usb* entry.
uv run tools/board-cli/board_cli.py ports

# 2. Watch boot logs and anything the firmware prints.
uv run tools/board-cli/board_cli.py --port /dev/cu.usbmodem1101 monitor

# 3. Send a message and wait for the last byte to be acknowledged.
uv run tools/board-cli/board_cli.py --port /dev/cu.usbmodem1101 send "hello"

# 4. Or work interactively: plain text sends, /help lists commands.
uv run tools/board-cli/board_cli.py --port /dev/cu.usbmodem1101
```

`--port` is mandatory on purpose; ports are not auto-detected. Typical names
are `/dev/cu.usbmodem*` (native USB) and `/dev/cu.usbserial-*`,
`/dev/cu.SLAB_USBtoUART`, `/dev/cu.wchusbserial*` (USB-UART bridges). Baud
defaults to 115200, which matches `CONFIG_ESP_CONSOLE_UART_BAUDRATE`.

Opening the port deasserts DTR/RTS so the board is normally not reset. Some
USB-UART bridges reset on open anyway; add `--reset` to request a reset
explicitly.

## Commands

Global options come before the subcommand:

| Option | Meaning |
|---|---|
| `--port DEVICE` | serial device (required for serial commands) |
| `--baud N` | baud rate, default 115200 |
| `--reset` | pulse DTR/RTS to reset the board on connect |
| `--no-color` | plain output |
| `--no-timestamps` | omit the `[hh:mm:ss.mmm]` prefix |
| `--decode-bits` | decode any 36-bit bit string in a log line |
| `--quiet` | hide verbose `[phy.]` lines |
| `--steps` | tag per-step `[phy] step: ...` lines and keep them under `--quiet` |

| Command | What it does |
|---|---|
| `monitor [--seconds N] [--no-stdin]` | stream board output; stdin is an interactive console |
| `send TEXT...` | send a message; waits for `@PHY TXDONE ... status=OK` |
| `send --hex "68 65 6c"` | send raw bytes |
| `send "hi" --dry-run` | print the frames the firmware will build, without a board |
| `send "hi" --no-wait` | send, show the first frames, do not wait for completion |
| `status` | request link state and counters |
| `ping` | check that the board console is alive |
| `turn` | request originator handoff (spec section 5) |
| `end` | end the current message/session |
| `reset` | reset protocol session and sequence state |
| `clk HZ` | set the nominal wired-clock rate (5..500 Hz) |
| `irfreq HZ` | set the IR carrier frequency (30..56 kHz) |
| `cmd ANY LINE` | send one arbitrary command line |
| `frame --type DATA --data 0x41` | offline: build a frame, show fields, bits and lane pairs |
| `decode 0110...` | offline: decode 36 bits (argument or stdin) |
| `ports` | list serial ports |

Interactive commands: plain text sends a message; `/send`, `/hex`, `/turn`,
`/end`, `/status`, `/reset`, `/ping`, `/clk`, `/irfreq`, `/timing`, `/raw`,
`/help`, `/quit` also work.

Every protocol step is visible. The firmware prints one `[phy] step: A -> B`
line per state transition (spec section 10) plus frame-level progress, and the
CLI shows them as they arrive; `--steps` tags them and keeps them when `--quiet`
is used, so a run reads as `LINK_IDLE -> TX_FRAME -> WAIT_RESPONSE -> LINK_IDLE`
around each frame.

Exit codes: `0` success, `1` serial/board-reported failure, `2` usage error,
`3` no confirmation from the firmware (for example, the firmware does not
implement the protocol yet). Offline commands do not need `--port` and, apart
from `ports`, do not even need pyserial: plain `python3 board_cli.py frame …`
works.

## Serial line protocol (firmware contract)

This is the interface between the CLI and the firmware console task. It is a
debug/control channel over the same UART as the ESP-IDF log; it is **not** part
of the 36-bit PHY frame and does not change the radio protocol.

### Host → board

One ASCII/UTF-8 line per command, terminated by `\n`.

| Line | Meaning |
|---|---|
| `SEND <text>` | Originator request: frame each byte of `<text>` (UTF-8) as a DATA frame, in order, as one logical message |
| `SENDHEX <hh> [hh ...]` | Same, with raw bytes in hex |
| `TURN` | Send a TURN frame to hand originator authority to the peer |
| `END` | Send an END frame to close the current logical message |
| `STATUS` | Print link state and counters |
| `RESET` | Reset protocol session/sequence state and cached duplicate handling |
| `PING` | Print `@PHY PONG` |
| `CLK <hz>` | Set the nominal wired-clock rate, 5..500 Hz |
| `IRFREQ <hz>` | Set the IR carrier frequency, 30..56 kHz |
| `TIMING` | Print the active timings |

Command parsing must tolerate trailing spaces and a `\r` (the console may run
in cooked mode). Reject unknown commands with `@PHY ERR msg="..."`.

### Board → host

Events are single lines starting with `@PHY ` (one space), followed by an
uppercase event name and `key=value` fields. Values with spaces are quoted with
double quotes; all other values are single tokens. The CLI ignores unknown
keys and unknown events, so fields may be added later.

| Event | Fields | When |
|---|---|---|
| `READY` | `role=A\|B session=0x0001 crc=CRC-8/SMBUS [channel=6]` | once after boot and after a console-visible reset |
| `CMD` | `ok=1 cmd=SEND [bytes=5 first_seq=41]` or `ok=0 cmd=... detail="..."` | response to every host command |
| `TX` | `seq=N type=DATA\|TURN\|END data=0xNN status=ACK\|NACK\|TIMEOUT\|FAIL retries=N [reason=0xNN]` | one line per originated request frame, after its response is validated |
| `TXDONE` | `first_seq=N last_seq=N bytes=N status=OK\|FAIL [reason=NACK\|TIMEOUT\|RETRIES\|NOT_OWNER\|REJECTED]` | once when a `SEND` completes, fails, exhausts retries, or is rejected (empty, too long, not the owner, queue full) |
| `RX` | `seq=N type=DATA data=0xNN` | once per **newly delivered** DATA request (not duplicates) |
| `RX` | `seq=N type=END` / `type=TURN` | once per newly delivered non-DATA request |
| `RXEND` | `seq=N` | once when a delivered END closes the message (firmware may use this instead of `RX type=END`) |
| `RXMSG` | `bytes=N text="hello"` | optional: a whole message already assembled by the firmware. The reference firmware prints it after `RXEND`; the CLI shows one message either way |
| `REPLY` | `seq=N type=ACK\|NACK [data=0xNN] [status=...]` | optional: response frames transmitted or received |
| `STATE` | `link=IDLE\|TX_FRAME\|WAIT_RESPONSE\|RX_FRAME\|VALIDATE_RX\|TX_RESPONSE\|LINK_ERROR owner=A\|B seq_next=N [session=0x0001]` | reply to `STATUS` and on state changes |
| `STATS` | arbitrary `key=value` pairs | reply to `STATUS`, after `STATE` |
| `PONG` | `[uptime_ms=N]` | reply to `PING` |
| `NACK` | `seq=N reason=0xNN` | optional: report an inbound NACK in addition to the `TX` line |
| `ERR` | `msg="..."` | protocol/console error |

`CLK` sets a *nominal* cycle rate: the transmitter still waits for each ESP-NOW
completion before the next clock edge, so the effective rate is always at or
below it (spec section 7). Both rates are runtime-adjustable, need no rebuild,
and appear in `@PHY STATS` as `clk_hz=` and `ir_hz=`; `IRFREQ` is rejected when
the firmware runs the non-demodulating IR mode.

`DATA` is reported as a numeric byte; the CLI derives the printable character.
For NACK frames the DATA byte is the reason code from the spec
(`0x01` CRC … `0x06` unsupported type).

The CLI tracks a `send` by matching `TX` lines (for progress) and completes on
`TXDONE`. A sender that only prints `TX` lines without `TXDONE` still shows
progress; the command then reports "no completion event" and exits `3`.

`RX` is about **delivery**, not reception: emit it once per accepted request,
even when retransmissions arrive. That lets the CLI print a message exactly
once, matching the duplicate-delivery protection in spec section 9. A good
place to emit it is the point where the frame has been validated, delivered to
the application, and its ACK cached — before or after sending the ACK, not from
the Wi-Fi callback.

Do **not** print these lines from an ISR or ESP-NOW callback. Format them in
the single protocol/console task (spec section 11). Keep each line under about
100 characters; `PHY_CONSOLE_LINE_LEN` is 96 and a `SEND` line spends 5 of it.
The message payload limit is `PHY_MESSAGE_MAX` (64 bytes).

The firmware implementation of this contract is `main/serial_cli.c`
(`serial_cli_app` callbacks plus the console task); its header documents the
command set from the board side.

### Example transcript

```
[12:04:03.118] board ready role=A session=0x0001 crc=CRC-8/SMBUS
[12:04:03.118] info  role=A session=0x0001 crc=CRC-8/SMBUS
[12:04:07.402] info  command accepted cmd=SEND
[12:04:07.681] tx    DATA 0x68 'h'  seq=41  ACK  retries=0
[12:04:08.012] tx    DATA 0x65 'e'  seq=42  ACK  retries=0
[12:04:08.344] tx    DATA 0x6c 'l'  seq=43  ACK  retries=0
[12:04:08.676] tx    DATA 0x6c 'l'  seq=44  ACK  retries=0
[12:04:09.008] tx    DATA 0x6f 'o'  seq=45  ACK  retries=0
[12:04:09.010] info  send status=OK bytes=5 seq 41..45
[12:04:10.220] rx    DATA 0x68 'h'  seq=41
[12:04:10.551] rx    DATA 0x65 'e'  seq=42
[12:04:10.883] rx    DATA 0x6c 'l'  seq=43
[12:04:11.214] rx    DATA 0x6c 'l'  seq=44
[12:04:11.546] rx    DATA 0x6f 'o'  seq=45
[12:04:11.548] msg   message (5 bytes): 'hello'
```

On board B the same `RX` lines appear; on board A they should not, because A is
the originator. After `TURN` and an ACK, B originates and A receives.

### Minimal firmware side

The firmware already logs with `printf`; adding the protocol is mostly a
formatter plus a line reader. A sketch (ESP-IDF, inside the console task, not a
callback):

```c
printf("@PHY READY role=%c session=0x%04X crc=%s\n",
       NODE_ROLE_A ? 'A' : 'B', SESSION_ID, crc8_profile_name(crc8_active_profile()));
printf("@PHY CMD ok cmd=SEND bytes=%d first_seq=%u\n", len, first_seq);
printf("@PHY TX seq=%u type=%s data=0x%02X status=%s retries=%u reason=0x%02X\n",
       seq, frame_type_name(control), data, "ACK", retries, nack_reason);
printf("@PHY TXDONE first_seq=%u last_seq=%u bytes=%d status=%s\n",
       first, last, count, "OK");
printf("@PHY RX seq=%u type=DATA data=0x%02X\n", seq, data);
```

For commands, read one line at a time (the console task may use `fgets` on
`stdin` once the UART driver is configured, or a small ring buffer in the UART
event task), strip `\r\n`, and dispatch on the first token. `SEND` text may
contain spaces; the payload is everything after the single separating space.

## Behaviour details and limits

- `SEND` is a *request*: the firmware must honor the current owner. If the
  board is not the originator, answer `CMD ok=0 cmd=SEND detail="..."` and do
  not transmit; use `turn` or reset the session as needed. The reference
  firmware also emits `TXDONE ... status=FAIL reason=NOT_OWNER`, and
  `reason=REJECTED` for an empty, too-long or unqueueable message, so a
  `send` always terminates with a reason instead of a timeout.
- A frame takes about 0.3-0.5 s at the default pacing and proportionally longer
  at low `clk` rates; raise `send --timeout` when running slowly.
- `send` warns above 64 bytes (`PHY_MESSAGE_MAX`); the firmware rejects a
  longer message with `CMD ok=0`. The console line buffer is 96 bytes.
- Messages are displayed as UTF-8 with `repr()` quoting; non-printable bytes
  are also shown in hex.
- The CLI never sends a packed frame over ESP-NOW and never bypasses the
  firmware. It only asks the firmware to originate frames.
- Offline frame decoding defaults to CRC-8/SMBUS, the v1.0 wire default. If
  both boards are built with another profile, pass `--crc autosar`,
  `--crc sae-j1850` or `--crc maxim-dow` to `frame`/`decode`/`send --dry-run`.
  The verify step still uses the firmware's reported results.
- The frame codec mirrors `main/frame.c`: framing and control-type checks are
  reported before the CRC check, as in `frame_parse_bits`.
- `monitor` is safe to leave running; it does not transmit anything until you
  type a command.

## Tests

```sh
uv run --with pyserial python -m unittest discover -s tools/board-cli -v
```

The tests cover the CRC catalogue vectors, frame layout and lane pairing,
single-bit corruption, bit-string parsing, and an end-to-end run of the CLI
against a fake board on a pseudo-terminal (`send`, `ping`, log display,
received-message assembly, and `--decode-bits`).

## Files

| File | Purpose |
|---|---|
| `board_cli.py` | CLI, serial reader, protocol state, output formatting |
| `phyframe.py` | 36-bit frame/CRC codec shared by the CLI and tests |
| `test_phyframe.py` | offline codec tests |
| `test_cli_e2e.py` | pty-based CLI tests with a fake board |
| `README.md` | this document |
