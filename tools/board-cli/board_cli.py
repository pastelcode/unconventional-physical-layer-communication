#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial>=3.5"]
# ///
"""USB serial CLI for the dual-medium ESP32 PHY link.

Attach to a board's USB-UART, send a text message (one DATA frame per byte),
and watch the frames and the received message on the other board.

The CLI understands a small line protocol (documented in README.md) that the
firmware prints with a ``@PHY `` prefix.  Plain ``printf`` logs - including
frame bit dumps - are displayed too, so the tool is useful before the firmware
implements the protocol.

Examples:
    uv run tools/board-cli/board_cli.py --port /dev/cu.usbmodem1101 monitor
    uv run tools/board-cli/board_cli.py --port /dev/cu.usbmodem1101 send "hello"
    uv run tools/board-cli/board_cli.py --port /dev/cu.usbserial-0001 --decode-bits monitor
    uv run tools/board-cli/board_cli.py frame --type DATA --data 0x41
    uv run tools/board-cli/board_cli.py ports
"""

from __future__ import annotations

import argparse
import json
import queue
import re
import select
import sys
import threading
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

import phyframe

if TYPE_CHECKING:  # pyserial is imported lazily for serial commands only
    import serial

DEFAULT_BAUD = 115200
MAX_MSG_BYTES = 64  # PHY_MESSAGE_MAX in main/protocol_config.h

EXIT_OK = 0
EXIT_BOARD_ERROR = 1
EXIT_USAGE = 2
EXIT_UNCONFIRMED = 3

_KV_RE = re.compile(r'([A-Za-z_][A-Za-z0-9_]*)=("(?:[^"\\]|\\.)*"|[^\s]*)')
_EVENT_RE = re.compile(r"^@phy\b\s*(.*)$", re.IGNORECASE)
_C_ESCAPE_RE = re.compile(r'\\(x[0-9A-Fa-f]{2}|[\\"nrt])')


def clock() -> str:
    now = time.time()
    return time.strftime("%H:%M:%S", time.localtime(now)) + ".%03d" % int(
        now * 1000 % 1000
    )


def unquote(token: str) -> str:
    token = token.strip()
    if len(token) >= 2 and token[0] == '"' and token[-1] == '"':
        try:
            value = json.loads(token)
            if isinstance(value, str):
                return value
        except (ValueError, TypeError):
            pass
        # Firmware print_quoted() emits C escapes such as \x07, which JSON
        # does not accept.  Fall back to a small C-escape decoder.
        body = token[1:-1]

        def replace(match: "re.Match[str]") -> str:
            escape = match.group(1)
            if escape.startswith("x"):
                return chr(int(escape[1:], 16))
            return {"n": "\n", "r": "\r", "t": "\t", "\\": "\\", '"': '"'}[escape]

        return _C_ESCAPE_RE.sub(replace, body)
    return token


def parse_kv(text: str) -> Dict[str, str]:
    return {key: unquote(value) for key, value in _KV_RE.findall(text)}


def as_int(value: Optional[str]) -> Optional[int]:
    if value is None:
        return None
    text = value.strip()
    try:
        if text.lower().startswith("0x"):
            return int(text, 16)
        return int(text, 0)
    except ValueError:
        return None


def printable(value: int) -> str:
    return chr(value) if 0x20 <= value <= 0x7E else "."


class Palette:
    CODES = {
        "tx": "32",
        "rx": "36",
        "msg": "1;35",
        "board": "1;34",
        "info": "34",
        "stats": "35",
        "step": "1;36",
        "warn": "33",
        "err": "1;31",
        "log": "2",
    }

    def __init__(self, enabled: bool) -> None:
        self.enabled = enabled

    def paint(self, code: str, text: str) -> str:
        if not self.enabled:
            return text
        return "\033[%sm%s\033[0m" % (code, text)


@dataclass
class PendingSend:
    text: str
    expected: int
    started: float
    acked: int = 0
    status: Optional[str] = None
    reason: Optional[str] = None

    def note_tx(self, status: str) -> None:
        if status == "ACK":
            self.acked += 1


@dataclass
class BoardState:
    role: Optional[str] = None
    session: Optional[str] = None
    owner: Optional[str] = None
    link: Optional[str] = None
    next_seq: Optional[int] = None
    crc: Optional[str] = None
    ready: bool = False


class LinkClosed(Exception):
    pass


class SerialLink:
    """Background reader over a pyserial port; yields complete decoded lines."""

    def __init__(self, port: str, baud: int, reset: bool = False) -> None:
        self.port = port
        self.baud = baud
        self.reset = reset
        self.ser: Optional[serial.Serial] = None
        self.lines: "queue.Queue[Optional[str]]" = queue.Queue()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._buffer = bytearray()
        self.error: Optional[str] = None

    def open(self) -> None:
        try:
            import serial
        except ImportError:
            raise LinkClosed(
                "pyserial is not installed; run through uv "
                "(`uv run tools/board-cli/board_cli.py ...`)"
            ) from None
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.05,
                write_timeout=2.0,
            )
        except (OSError, ValueError) as exc:
            raise LinkClosed("cannot open %s: %s" % (self.port, exc)) from exc

        # Do not drive the ESP32 auto-reset lines by accident: pyserial raises
        # DTR/RTS on open, which resets most dev boards.  --reset asks for it.
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except OSError:
            pass
        self.ser.reset_input_buffer()

    def open_with_reset(self) -> None:
        self.open()
        assert self.ser is not None
        if not self.reset:
            return
        try:
            self.ser.dtr = False
            self.ser.rts = True
            time.sleep(0.12)
            self.ser.rts = False
        except OSError:
            return
        # Let the ROM bootloader messages go by before the app logs start.
        deadline = time.time() + 1.2
        while time.time() < deadline:
            try:
                self.ser.read(4096)
            except OSError:
                break
            time.sleep(0.05)
        self.ser.reset_input_buffer()

    def start(self) -> None:
        assert self.ser is not None
        self._thread = threading.Thread(
            target=self._reader, name="serial-reader", daemon=True
        )
        self._thread.start()

    def _reader(self) -> None:
        assert self.ser is not None
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except OSError as exc:
                self.error = str(exc)
                self.lines.put(None)
                return
            if not chunk:
                continue
            self._buffer.extend(chunk)
            while True:
                newline = self._buffer.find(b"\n")
                if newline < 0:
                    break
                raw = bytes(self._buffer[:newline])
                del self._buffer[: newline + 1]
                text = raw.decode("utf-8", errors="replace").rstrip("\r")
                self.lines.put(text)
            # Keep memory bounded if a sender never emits a newline.
            if len(self._buffer) > 16384:
                text = self._buffer.decode("utf-8", errors="replace")
                self._buffer.clear()
                self.lines.put(text)

    def get(self, timeout: float) -> Optional[str]:
        try:
            item = self.lines.get(timeout=timeout)
        except queue.Empty:
            return None
        if item is None:
            raise LinkClosed(self.error or "serial port closed")
        return item

    def write_line(self, text: str) -> None:
        assert self.ser is not None
        payload = (text + "\n").encode("utf-8")
        try:
            self.ser.write(payload)
            self.ser.flush()
        except OSError as exc:
            raise LinkClosed("write failed: %s" % exc) from exc

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if self.ser is not None:
            try:
                self.ser.close()
            except OSError:
                pass


class Session:
    """Turns serial lines into readable events and tracks the link state."""

    def __init__(
        self,
        palette: Palette,
        timestamps: bool = True,
        decode_bits: bool = False,
        quiet: bool = False,
        steps: bool = False,
    ) -> None:
        self.palette = palette
        self.timestamps = timestamps
        self.decode_bits = decode_bits
        self.quiet = quiet
        self.steps = steps
        self.state = BoardState()
        self.pending: Optional[PendingSend] = None
        self.rx_buf = bytearray()
        self.rx_next_seq: Optional[int] = None
        self.last_rx_flush: Optional[Tuple[float, bytes]] = None
        self.saw_event = False
        self.last_error: Optional[str] = None

    # -- output -----------------------------------------------------------
    def emit(self, tag: str, text: str) -> None:
        color = Palette.CODES.get(tag, "0")
        label = self.palette.paint(color, "%-5s" % tag)
        prefix = "[%s] " % clock() if self.timestamps else ""
        print(prefix + label + " " + text, flush=True)

    # -- inbound lines ----------------------------------------------------
    def handle(self, raw: str) -> None:
        line = raw.rstrip()
        if not line:
            return
        match = _EVENT_RE.match(line)
        if match:
            self.saw_event = True
            self._event(match.group(1).strip(), line)
            return
        self._log(line)

    def _log(self, line: str) -> None:
        is_step = line.startswith("[phy]") and " step:" in line
        if self.quiet and not (self.steps and is_step) and line.startswith("[phy.]"):
            return
        tag = "log"
        if is_step:
            tag = "step" if self.steps else "log"
        elif line.startswith("[phy!]"):
            tag = "warn"
        elif line.startswith("[phyX]"):
            tag = "err"
        suffix = ""
        if self.decode_bits:
            notes = []
            for bits in phyframe.find_bit_runs(line):
                try:
                    notes.append(phyframe.describe_bits(bits))
                except ValueError:
                    continue
            if notes:
                suffix = "   [" + "; ".join(notes) + "]"
        self.emit(tag, line + suffix)

    def _event(self, body: str, raw: str) -> None:
        tokens = body.split(None, 1)
        name = tokens[0].upper() if tokens else "EVENT"
        rest = tokens[1] if len(tokens) > 1 else ""
        kv = parse_kv(rest)

        handler = getattr(self, "_on_" + name.lower(), None)
        if handler is not None:
            handler(rest, kv)
        else:
            self.emit("board", body)

    def _on_ready(self, rest: str, kv: Dict[str, str]) -> None:
        self.state.ready = True
        self.state.role = kv.get("role")
        self.state.session = kv.get("session")
        self.state.crc = kv.get("crc")
        self.emit("board", "ready" + ((" " + rest) if rest else ""))
        self.emit("info", self._state_line())

    def _on_state(self, rest: str, kv: Dict[str, str]) -> None:
        role = kv.get("role", self.state.role)
        session = kv.get("session", self.state.session)
        self.state.role = role
        self.state.session = session
        self.state.link = kv.get("link", self.state.link)
        self.state.owner = kv.get("owner", self.state.owner)
        seq_next = as_int(kv.get("seq_next"))
        if seq_next is not None:
            self.state.next_seq = seq_next
        self.emit("board", "state " + rest)

    def _on_tx(self, rest: str, kv: Dict[str, str]) -> None:
        seq = as_int(kv.get("seq"))
        ftype = kv.get("type", "?").upper()
        status = kv.get("status", "").upper()
        retries = kv.get("retries")
        data = as_int(kv.get("data"))
        text = "%-4s" % ftype
        if data is not None:
            text += " 0x%02X %r" % (data, printable(data))
        if seq is not None:
            text += "  seq=%d" % seq
        if status:
            color = {"ACK": "32", "NACK": "1;31", "TIMEOUT": "33"}.get(status, "0")
            text += "  " + self.palette.paint(color, status)
            if kv.get("reason"):
                reason = as_int(kv["reason"])
                if reason is not None:
                    text += " reason=%s" % phyframe.NACK_REASON_NAMES.get(
                        reason, "0x%02X" % reason
                    )
                else:
                    text += " reason=%s" % kv["reason"]
        if retries is not None:
            text += "  retries=%s" % retries
        self.emit("tx", text)
        if self.pending is not None:
            self.pending.note_tx(status)
            if status == "NACK":
                self.pending.status = "NACK"
                self.pending.reason = kv.get("reason")
            elif status == "TIMEOUT":
                self.pending.status = "TIMEOUT"
                self.pending.reason = "timeout"

    def _on_txdone(self, rest: str, kv: Dict[str, str]) -> None:
        status = kv.get("status", "?").upper()
        first = kv.get("first_seq", "?")
        last = kv.get("last_seq", "?")
        count = kv.get("bytes", kv.get("count", "?"))
        text = "status=%s bytes=%s seq %s..%s" % (status, count, first, last)
        tag = "info" if status == "OK" else "err"
        self.emit(tag, "send " + text)
        if self.pending is not None:
            self.pending.status = status
            self.pending.reason = kv.get("reason")

    def _on_msg(self, rest: str, kv: Dict[str, str]) -> None:
        self._on_rxmsg(rest, kv)

    def _on_rxmsg(self, rest: str, kv: Dict[str, str]) -> None:
        text = kv.get("text")
        count = kv.get("bytes")
        if text is not None:
            # The firmware already assembles messages and prints RXMSG after
            # RXEND.  Drop the copy if the per-frame RX list produced the same
            # message a moment ago, and let RXMSG win if it arrives first.
            payload = text.encode("utf-8", errors="replace")
            if self.rx_buf:
                self.rx_buf.clear()
                self.rx_next_seq = None
            if self.last_rx_flush is not None:
                when, data = self.last_rx_flush
                if data == payload and time.time() - when < 2.0:
                    self.last_rx_flush = None
                    return
            shown = repr(text)
            if count:
                self.emit("msg", "message (%s bytes): %s" % (count, shown))
            else:
                self.emit("msg", "message: %s" % shown)
        else:
            self.emit("msg", rest or "(empty message)")

    def _on_rx(self, rest: str, kv: Dict[str, str]) -> None:
        ftype = kv.get("type", "DATA").upper()
        seq = as_int(kv.get("seq"))
        data = as_int(kv.get("data"))

        if ftype == "DATA":
            if data is None:
                self.emit("warn", "rx DATA without a data byte: " + rest)
                return
            if (
                self.rx_next_seq is not None
                and seq is not None
                and seq != self.rx_next_seq
            ):
                self.flush_rx(
                    "sequence gap (expected %d, got %d)" % (self.rx_next_seq, seq)
                )
            self.rx_buf.append(data & 0xFF)
            self.rx_next_seq = None if seq is None else (seq + 1) & 0xFFFF
            line = "DATA 0x%02X %r" % (data, printable(data))
            if seq is not None:
                line += "  seq=%d" % seq
            self.emit("rx", line)
            return

        if ftype == "END":
            line = ftype
            if seq is not None:
                line += "  seq=%d" % seq
            self.emit("rx", line)
            self.flush_rx(None)
            return
        line = ftype
        if data is not None:
            line += " 0x%02X" % data
        if seq is not None:
            line += "  seq=%d" % seq
        self.emit("rx", line)

    def _on_rxend(self, rest: str, kv: Dict[str, str]) -> None:
        seq = as_int(kv.get("seq"))
        self.emit("rx", "END" + (("  seq=%d" % seq) if seq is not None else ""))
        self.flush_rx(None)

    def _on_reply(self, rest: str, kv: Dict[str, str]) -> None:
        ftype = kv.get("type", "?").upper()
        seq = as_int(kv.get("seq"))
        status = kv.get("status", "").upper()
        data = as_int(kv.get("data"))
        text = "%-4s" % ftype
        if data is not None:
            text += " 0x%02X" % data
        if seq is not None:
            text += "  seq=%d" % seq
        if status:
            text += "  " + status
        self.emit("board", "reply " + text)

    def _on_nack(self, rest: str, kv: Dict[str, str]) -> None:
        reason = as_int(kv.get("reason"))
        text = "NACK"
        if reason is not None:
            text += " %s" % phyframe.NACK_REASON_NAMES.get(reason, "0x%02X" % reason)
        if kv.get("seq"):
            text += " seq=%s" % kv["seq"]
        self.emit("warn", text)
        if self.pending is not None:
            self.pending.status = "NACK"
            self.pending.reason = kv.get("reason")

    def _on_err(self, rest: str, kv: Dict[str, str]) -> None:
        message = kv.get("msg") or rest or "board reported an error"
        self.last_error = message
        self.emit("err", message)
        if self.pending is not None and self.pending.status is None:
            self.pending.status = "ERROR"
            self.pending.reason = message

    def _on_pong(self, rest: str, kv: Dict[str, str]) -> None:
        self.emit("info", "pong" + ((" " + rest) if rest else ""))

    def _on_cmd(self, rest: str, kv: Dict[str, str]) -> None:
        if kv.get("ok", "1").lower() in ("0", "false", "no"):
            message = kv.get("msg") or kv.get("detail") or rest or "command rejected"
            self.last_error = message
            self.emit("err", "command rejected: " + message)
            if self.pending is not None and self.pending.status is None:
                self.pending.status = "REJECTED"
                self.pending.reason = message
        else:
            self.emit("info", "command accepted" + ((" " + rest) if rest else ""))

    def _on_stats(self, rest: str, kv: Dict[str, str]) -> None:
        if not kv:
            self.emit("stats", rest)
            return
        parts = ["%s=%s" % item for item in kv.items()]
        self.emit("stats", " ".join(parts))

    def _state_line(self) -> str:
        bits = []
        if self.state.role:
            bits.append("role=%s" % self.state.role)
        if self.state.session:
            bits.append("session=%s" % self.state.session)
        if self.state.owner:
            bits.append("owner=%s" % self.state.owner)
        if self.state.link:
            bits.append("link=%s" % self.state.link)
        if self.state.next_seq is not None:
            bits.append("seq_next=%d" % self.state.next_seq)
        if self.state.crc:
            bits.append("crc=%s" % self.state.crc)
        return " ".join(bits) if bits else "no state yet"

    # -- message assembly -------------------------------------------------
    def flush_rx(self, note: Optional[str]) -> None:
        if not self.rx_buf:
            self.rx_next_seq = None
            return
        data = bytes(self.rx_buf)
        self.rx_buf.clear()
        self.rx_next_seq = None
        self.last_rx_flush = (time.time(), data)
        text = data.decode("utf-8", errors="replace")
        shown = repr(text)
        detail = "message (%d bytes): %s" % (len(data), shown)
        if any(byte < 0x20 or byte > 0x7E for byte in data):
            detail += "  [%s]" % phyframe.hex_bytes(data)
        if note:
            detail += "  (" + note + ")"
        self.emit("msg", detail)


def send_payload(link: SerialLink, payload: bytes) -> None:
    """Queue one message on the board: plain text when printable, else hex."""
    if all(0x20 <= byte <= 0x7E for byte in payload):
        link.write_line("SEND " + payload.decode("ascii"))
    else:
        link.write_line("SENDHEX " + phyframe.hex_bytes(payload))


def print_frames(payload: bytes, profile: str, type_name: str = "DATA") -> None:
    for offset, byte in enumerate(payload):
        frame = phyframe.build_frame(type_name, byte, profile)
        ir, wifi = phyframe.frame_to_lanes(frame.bits)
        print(
            "%3d  %-4s data=0x%02X %r  crc=0x%02X  bits=%s"
            % (
                offset,
                frame.type_name,
                frame.data,
                printable(frame.data),
                frame.crc,
                phyframe.bits_to_str(frame.bits),
            )
        )
        pairs = " ".join("%d%d" % (a, b) for a, b in zip(ir, wifi))
        print("     lanes (IR,ESP-NOW per cycle): " + pairs)


def frame_report(args: argparse.Namespace) -> int:
    profile = args.crc
    try:
        data = int(str(args.data), 0)
        if not 0 <= data <= 0xFF:
            raise ValueError
    except ValueError:
        print("error: --data must be one byte (0x00..0xFF)", file=sys.stderr)
        return EXIT_USAGE
    try:
        frame = phyframe.build_frame(args.type, data, profile)
    except ValueError as exc:
        print("error: %s" % exc, file=sys.stderr)
        return EXIT_USAGE

    params = phyframe.resolve_profile(profile)
    print(
        "frame   %s (CONTROL=0x%X)  DATA=0x%02X (%d, %r)"
        % (
            frame.type_name,
            frame.control,
            frame.data,
            frame.data,
            printable(frame.data),
        )
    )
    print(
        "crc     %s over %02x %02x = 0x%02X"
        % (params.name, frame.control, frame.data, frame.crc)
    )
    print("bits    " + phyframe.bits_grouped(frame.bits))
    print("         SYN      SFD      CTRL DATA     CRC")
    if frame.control == phyframe.FRAME_NACK:
        reason = phyframe.NACK_REASON_NAMES.get(frame.data, "unknown")
        print("reason  %s" % reason)
    ir, wifi = phyframe.frame_to_lanes(frame.bits)
    print("cycles  " + " ".join("%2d" % k for k in range(phyframe.FRAME_CYCLES)))
    print("IR      " + " ".join("%2d" % bit for bit in ir))
    print("ESP-NOW " + " ".join("%2d" % bit for bit in wifi))
    return EXIT_OK


def decode_report(args: argparse.Namespace) -> int:
    text = " ".join(args.bits) if args.bits else ""
    lines: List[str] = []
    if text in ("", "-"):
        lines = [line.strip() for line in sys.stdin if line.strip()]
    else:
        lines = [text]

    failures = 0
    for line in lines:
        for bits in phyframe.find_bit_runs(line) or (phyframe.normalize_bits(line),):
            try:
                parsed = phyframe.parse_bits(bits, args.crc)
            except ValueError as exc:
                print("error: %s" % exc, file=sys.stderr)
                return EXIT_USAGE
            frame = parsed.frame
            print(
                "%s  %-5s control=0x%X data=0x%02X (%r) crc=0x%02X expected=0x%02X"
                % (
                    parsed.status,
                    frame.type_name,
                    frame.control,
                    frame.data,
                    printable(frame.data),
                    frame.crc,
                    parsed.expected_crc,
                )
            )
            if (
                parsed.status == phyframe.PARSE_OK
                and frame.control == phyframe.FRAME_NACK
            ):
                print(
                    "       reason: %s"
                    % phyframe.NACK_REASON_NAMES.get(frame.data, "unknown")
                )
            elif parsed.detail:
                print("       " + parsed.detail)
            if args.lanes:
                ir, wifi = phyframe.frame_to_lanes(frame.bits)
                print("       " + " ".join("%d%d" % (a, b) for a, b in zip(ir, wifi)))
            failures += 0 if parsed.ok else 1
    return EXIT_OK if failures == 0 else EXIT_BOARD_ERROR


def list_ports() -> int:
    try:
        import serial.tools.list_ports
    except ImportError:
        print(
            "pyserial is not installed; run `uv run tools/board-cli/board_cli.py ports`",
            file=sys.stderr,
        )
        return EXIT_USAGE
    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    if not ports:
        print("no serial ports found")
        return EXIT_OK
    for port in ports:
        description = port.description or ""
        if description == "n/a":
            description = ""
        hwid = ("  [%s]" % port.hwid) if port.hwid else ""
        print("%-28s %s%s" % (port.device, description, hwid))
    return EXIT_OK


def attach(args: argparse.Namespace) -> Tuple[SerialLink, Session]:
    link = SerialLink(args.port, args.baud, reset=args.reset)
    link.open_with_reset()
    palette = Palette(enabled=sys.stdout.isatty() and not args.no_color)
    session = Session(
        palette,
        timestamps=not args.no_timestamps,
        decode_bits=args.decode_bits,
        quiet=args.quiet,
        steps=args.steps,
    )
    link.start()
    return link, session


def monitor_loop(
    link: SerialLink,
    session: Session,
    seconds: Optional[float],
    interactive: bool,
) -> int:
    deadline = time.time() + seconds if seconds else None
    stdin_open = interactive and sys.stdin is not None
    try:
        while True:
            if deadline is not None and time.time() >= deadline:
                break
            try:
                line = link.get(0.1)
            except LinkClosed as exc:
                print("\nserial link lost: %s" % exc, file=sys.stderr)
                return EXIT_BOARD_ERROR
            if line is not None:
                session.handle(line)
            if not stdin_open:
                continue
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0)
            except (OSError, ValueError):
                stdin_open = False
                continue
            if not ready:
                continue
            command = sys.stdin.readline()
            if command == "":
                stdin_open = False
                continue
            if handle_interactive(command.strip(), link, session) == "quit":
                break
    except KeyboardInterrupt:
        print()
    return EXIT_OK


def handle_interactive(line: str, link: SerialLink, session: Session) -> Optional[str]:
    if not line:
        return None
    if not line.startswith("/"):
        payload = line.encode("utf-8")
        if len(payload) > MAX_MSG_BYTES:
            session.emit(
                "warn",
                "message is %d bytes; the board console may truncate above %d"
                % (len(payload), MAX_MSG_BYTES),
            )
        send_payload(link, payload)
        session.emit("info", "queued %d byte(s) for transmission" % len(payload))
        return None

    parts = line.split(None, 1)
    command = parts[0].lower()
    rest = parts[1] if len(parts) > 1 else ""

    if command in ("/quit", "/exit", "/q"):
        return "quit"
    if command in ("/help", "/h", "/?"):
        print(
            "plain text      send it as a message (one DATA frame per byte)\n"
            "/send <text>    same as plain text\n"
            "/hex <bytes>    send raw bytes, e.g. /hex 68 65 6c\n"
            "/turn           request originator handoff\n"
            "/end            end the current message/session\n"
            "/clk <hz>       set the nominal wired-clock rate\n"
            "/irfreq <hz>    set the IR carrier frequency\n"
            "/timing         show the active timings\n"
            "/status         ask the board for link state\n"
            "/reset          reset protocol session and sequence state\n"
            "/ping           liveness check\n"
            "/raw <line>     send an arbitrary line, without the PHY prefix\n"
            "/quit           detach"
        )
        return None
    if command == "/send":
        if not rest:
            session.emit("err", "usage: /send <text>")
            return None
        payload = rest.encode("utf-8")
        send_payload(link, payload)
        session.emit("info", "queued %d byte(s) for transmission" % len(payload))
        return None
    if command == "/hex":
        try:
            payload = phyframe.parse_hex_bytes(rest)
        except ValueError as exc:
            session.emit("err", str(exc))
            return None
        send_payload(link, payload)
        session.emit("info", "queued %d byte(s) for transmission" % len(payload))
        return None
    if command in ("/clk", "/rate", "/irfreq", "/timing"):
        if command in ("/clk", "/rate", "/irfreq") and not rest:
            session.emit("err", "usage: %s <hz>" % command)
            return None
        word = {"clk": "CLK", "rate": "CLK", "irfreq": "IRFREQ", "timing": "TIMING"}
        line_out = (word[command[1:]] + " " + rest).strip()
        link.write_line(line_out)
        session.emit("info", "sent " + line_out)
        return None
    if command in ("/turn", "/end", "/status", "/reset", "/ping"):
        link.write_line(command[1:].upper())
        session.emit("info", "sent " + command[1:].upper())
        return None
    if command == "/raw":
        if not rest:
            session.emit("err", "usage: /raw <line>")
            return None
        link.write_line(rest)
        session.emit("info", "sent raw line")
        return None
    session.emit("err", "unknown command %s (try /help)" % command)
    return None


def wait_for_completion(
    link: SerialLink,
    session: Session,
    pending: PendingSend,
    timeout: float,
) -> int:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pending.status is not None:
            break
        try:
            line = link.get(0.1)
        except LinkClosed as exc:
            print("\nserial link lost: %s" % exc, file=sys.stderr)
            return EXIT_BOARD_ERROR
        if line is not None:
            session.handle(line)

    if pending.status == "OK":
        return EXIT_OK
    if pending.status is None:
        progress = ""
        if pending.acked:
            progress = " (%d/%d byte(s) acknowledged)" % (
                pending.acked,
                pending.expected,
            )
        hint = (
            "try `monitor` to see the board's raw log"
            if not session.saw_event
            else "the board may have stopped before TXDONE; try `monitor`"
        )
        print(
            "\nno completion event within %.0fs%s. The board may not implement the "
            "@PHY protocol yet; %s." % (timeout, progress, hint),
            file=sys.stderr,
        )
        return EXIT_UNCONFIRMED
    reason = ""
    if pending.reason:
        code = as_int(pending.reason)
        if code is not None:
            reason = " (%s)" % phyframe.NACK_REASON_NAMES.get(code, "0x%02X" % code)
        else:
            reason = " (%s)" % pending.reason
    print("transmission failed: %s%s" % (pending.status, reason), file=sys.stderr)
    return EXIT_BOARD_ERROR


def run_command(
    args: argparse.Namespace, command: str, payload: Optional[bytes] = None
) -> int:
    link, session = attach(args)
    pending = None
    try:
        if command == "SEND":
            assert payload is not None
            pending = PendingSend(
                text=payload.decode("utf-8", errors="replace"),
                expected=len(payload),
                started=time.time(),
            )
            session.pending = pending
            send_payload(link, payload)
            session.emit("info", "queued %d byte(s) for transmission" % len(payload))
            if args.no_wait:
                # Short grace period so the first frames are visible.
                end = time.time() + min(2.0, args.timeout)
                while time.time() < end:
                    line = link.get(0.1)
                    if line is not None:
                        session.handle(line)
                return EXIT_OK
            return wait_for_completion(link, session, pending, args.timeout)

        link.write_line(command)
        session.emit("info", "sent " + command)
        if command in ("STATUS", "PING"):
            deadline = time.time() + args.timeout
            while time.time() < deadline:
                line = link.get(0.1)
                if line is not None:
                    session.handle(line)
                    if command == "PING" and "pong" in line.lower():
                        return EXIT_OK
                    if command == "STATUS" and session.state.link:
                        # Give the board a moment to print the rest.
                        end = time.time() + 0.3
                        while time.time() < end:
                            extra = link.get(0.05)
                            if extra is not None:
                                session.handle(extra)
                        return EXIT_BOARD_ERROR if session.last_error else EXIT_OK
            return EXIT_UNCONFIRMED
        # Let immediate replies print before detaching.
        end = time.time() + min(1.0, args.timeout)
        while time.time() < end:
            line = link.get(0.1)
            if line is not None:
                session.handle(line)
        return EXIT_BOARD_ERROR if session.last_error else EXIT_OK
    finally:
        link.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="board_cli.py",
        description="USB serial CLI for the dual-medium ESP32 PHY link.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Run without a subcommand for an interactive session:\n"
            "  plain text sends a message, /help lists commands.\n\n"
            "The board must implement the @PHY serial protocol described in\n"
            "tools/board-cli/README.md.  Until then, `monitor` still shows the\n"
            "board's printf log lines."
        ),
    )
    parser.add_argument("--port", help="serial device, e.g. /dev/cu.usbmodem1101")
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD, help="baud rate (default 115200)"
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="pulse DTR/RTS to reset the board on connect",
    )
    parser.add_argument("--no-color", action="store_true", help="disable ANSI colours")
    parser.add_argument(
        "--no-timestamps", action="store_true", help="omit the wall-clock prefix"
    )
    parser.add_argument(
        "--decode-bits",
        action="store_true",
        help="decode 36-bit bit strings in log lines",
    )
    parser.add_argument(
        "--quiet", action="store_true", help="hide verbose [phy.] log lines"
    )
    parser.add_argument(
        "--steps",
        action="store_true",
        help="tag per-step '[phy] step: ...' log lines and keep them under --quiet",
    )
    sub = parser.add_subparsers(dest="command", metavar="command")

    monitor = sub.add_parser(
        "monitor", help="stream board output and interact on stdin"
    )
    monitor.add_argument(
        "--seconds", type=float, default=None, help="detach after N seconds"
    )
    monitor.add_argument(
        "--no-stdin", action="store_true", help="ignore stdin (pure log view)"
    )

    send = sub.add_parser("send", help="send a text message")
    send.add_argument("text", nargs="*", help="message text (UTF-8)")
    send.add_argument("--hex", help="send raw bytes instead, e.g. '68 65 6c'")
    send.add_argument("--no-wait", action="store_true", help="do not wait for TXDONE")
    send.add_argument(
        "--timeout", type=float, default=60.0, help="completion timeout in seconds"
    )
    send.add_argument(
        "--dry-run", action="store_true", help="print frames only; no port needed"
    )
    send.add_argument(
        "--crc",
        default=phyframe.DEFAULT_CRC_PROFILE,
        choices=sorted(phyframe.CRC_PROFILES),
    )

    for name, help_text in (
        ("status", "print link state and counters"),
        ("ping", "check that the board console is alive"),
        ("turn", "request originator handoff"),
        ("end", "end the current message/session"),
        ("reset", "reset protocol session and sequence state"),
    ):
        cmd = sub.add_parser(name, help=help_text)
        cmd.add_argument(
            "--timeout", type=float, default=10.0, help="reply timeout in seconds"
        )

    clk = sub.add_parser("clk", help="set the nominal wired-clock rate in Hz")
    clk.add_argument("hz", type=int)
    clk.add_argument("--timeout", type=float, default=10.0)

    irfreq = sub.add_parser("irfreq", help="set the IR carrier frequency in Hz")
    irfreq.add_argument("hz", type=int)
    irfreq.add_argument("--timeout", type=float, default=10.0)

    cmd = sub.add_parser("cmd", help="send one raw command line")
    cmd.add_argument("line", nargs="+", help="command text, e.g. STATUS")
    cmd.add_argument("--timeout", type=float, default=10.0)

    frame = sub.add_parser("frame", help="build a frame and show its bits (offline)")
    frame.add_argument("--type", default="DATA", help="DATA, ACK, NACK, TURN or END")
    frame.add_argument("--data", default="0x41", help="DATA byte, e.g. 0x41")
    frame.add_argument(
        "--crc",
        default=phyframe.DEFAULT_CRC_PROFILE,
        choices=sorted(phyframe.CRC_PROFILES),
    )

    decode = sub.add_parser(
        "decode", help="decode 36 bits from a log or argument (offline)"
    )
    decode.add_argument(
        "bits", nargs="*", help="36-bit string; '-' or none reads stdin"
    )
    decode.add_argument(
        "--crc",
        default=phyframe.DEFAULT_CRC_PROFILE,
        choices=sorted(phyframe.CRC_PROFILES),
    )
    decode.add_argument(
        "--lanes", action="store_true", help="print the 18 (IR, ESP-NOW) pairs"
    )

    sub.add_parser("ports", help="list serial ports")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "ports":
        return list_ports()
    if args.command == "frame":
        return frame_report(args)
    if args.command == "decode":
        return decode_report(args)

    if args.command == "send" and args.dry_run:
        try:
            payload = (
                phyframe.parse_hex_bytes(args.hex)
                if args.hex
                else " ".join(args.text).encode("utf-8")
            )
        except ValueError as exc:
            print("error: %s" % exc, file=sys.stderr)
            return EXIT_USAGE
        if not payload:
            print("error: nothing to send", file=sys.stderr)
            return EXIT_USAGE
        print_frames(payload, args.crc)
        return EXIT_OK

    if not args.port:
        print("error: --port is required (see `board_cli.py ports`)", file=sys.stderr)
        return EXIT_USAGE

    if args.command == "send":
        if args.hex:
            try:
                payload = phyframe.parse_hex_bytes(args.hex)
            except ValueError as exc:
                print("error: %s" % exc, file=sys.stderr)
                return EXIT_USAGE
        else:
            payload = " ".join(args.text).encode("utf-8")
        if not payload:
            print("error: nothing to send", file=sys.stderr)
            return EXIT_USAGE
        if len(payload) > MAX_MSG_BYTES:
            print(
                "warning: %d bytes; the firmware console buffers about %d bytes per line"
                % (len(payload), MAX_MSG_BYTES),
                file=sys.stderr,
            )
        return run_command(args, "SEND", payload)

    if args.command == "cmd":
        args.timeout = getattr(args, "timeout", 10.0)
        return run_command(args, " ".join(args.line))

    if args.command in ("status", "ping", "turn", "end", "reset"):
        return run_command(args, args.command.upper())
    if args.command == "clk":
        return run_command(args, "CLK %d" % args.hz)
    if args.command == "irfreq":
        return run_command(args, "IRFREQ %d" % args.hz)

    if args.command == "monitor":
        link, session = attach(args)
        try:
            return monitor_loop(
                link, session, args.seconds, interactive=not args.no_stdin
            )
        finally:
            link.close()

    if args.command is None:
        # Interactive by default.
        args.timeout = 10.0
        args.no_wait = False
        link, session = attach(args)
        try:
            session.emit(
                "info",
                "attached to %s; type a message or /help (Ctrl-C or /quit to exit)"
                % args.port,
            )
            return monitor_loop(link, session, None, interactive=True)
        finally:
            link.close()

    parser.error("unknown command")
    return EXIT_USAGE


if __name__ == "__main__":
    try:
        sys.exit(main())
    except LinkClosed as exc:
        print("serial error: %s" % exc, file=sys.stderr)
        sys.exit(EXIT_BOARD_ERROR)
    except KeyboardInterrupt:
        print()
        sys.exit(130)
