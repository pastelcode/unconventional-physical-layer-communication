"""Integration tests: the firmware's exact ``@PHY`` lines through this CLI.

The transcripts below are transcribed from the printf format strings in
``main/serial_cli.c`` (and the ``[phy] step:`` lines from
``main/protocol_fsm.c``), so a change on the firmware side that breaks the
console contract fails here instead of on the bench.

Run with the other CLI tests:
    uv run --with pyserial python -m unittest discover -s tools/board-cli
"""

import os
import pty
import select
import subprocess
import sys
import threading
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
CLI = os.path.join(HERE, "board_cli.py")

import board_cli  # noqa: E402  (script directory is on sys.path)


class RecordingSession(board_cli.Session):
    """Session that keeps rendered lines instead of printing them."""

    def __init__(self, **kwargs) -> None:
        super().__init__(board_cli.Palette(False), timestamps=False, **kwargs)
        self.out = []

    def emit(self, tag: str, text: str) -> None:
        self.out.append((tag, text))

    def tags(self, tag: str):
        return [text for kind, text in self.out if kind == tag]

    def text(self) -> str:
        return "\n".join(text for _, text in self.out)


def frame_exchange(seq: int, byte: int, ack: str = "ACK") -> list:
    """The firmware's lines for one DATA frame, as main/serial_cli.c prints them."""
    return [
        "[phy]  step: LINK_IDLE -> TX_FRAME",
        "[phy]  TX DATA seq=%d data=0x%02x (attempt 1 of 4)" % (seq, byte),
        "[phy]  step: TX_FRAME -> WAIT_RESPONSE",
        "@PHY TX type=DATA seq=%d data=0x%02x status=%s retries=0" % (seq, byte, ack),
        "[phy]  ACK seq=%d for DATA after 312 ms" % seq,
        "[phy]  step: WAIT_RESPONSE -> LINK_IDLE",
    ]


def receive_exchange(seq: int, byte: int) -> list:
    """The responder's lines for one DATA frame plus its REPLY."""
    return [
        "[phy]  step: LINK_IDLE -> RX_FRAME",
        "@PHY RX type=DATA seq=%d data=0x%02x" % (seq, byte),
        "[phy]  TX ACK  seq=%d" % seq,
        "@PHY REPLY type=ACK seq=%d data=0x00" % seq,
        "[phy]  step: RX_FRAME -> LINK_IDLE",
    ]


class FirmwareContractTests(unittest.TestCase):
    """One host session fed the exact lines the firmware prints."""

    def setUp(self) -> None:
        self.session = RecordingSession(steps=True)

    def feed(self, lines) -> None:
        for line in lines:
            self.session.handle(line)

    def test_ready_and_stats(self) -> None:
        self.feed(
            [
                "@PHY READY role=A session=0x0001 crc=CRC-8/SMBUS channel=6",
                "@PHY STATE link=IDLE owner=A seq_next=7 role=A session=0x0001",
                "@PHY STATS lane_tx=126 lane_rx=126 frames_tx=6 frames_rx=120 data=5 "
                "crc_err=0 missing=0 timeouts=0 retries=0 clk_hz=250 ir_hz=38000",
            ]
        )
        self.assertTrue(self.session.state.ready)
        self.assertEqual(self.session.state.role, "A")
        self.assertEqual(self.session.state.link, "IDLE")
        self.assertEqual(self.session.state.next_seq, 7)
        self.assertIn("lane_tx=126", self.session.text())
        self.assertIn("clk_hz=250", self.session.text())

    def test_send_transcript_is_exactly_once(self) -> None:
        message = b"hello"
        lines = [
            "@PHY CMD ok=1 cmd=SEND bytes=5",
            '["app] queued 5 byte(s) for transmission',
        ]
        for offset, byte in enumerate(message):
            lines += frame_exchange(41 + offset, byte)
        lines.append("@PHY TXDONE first_seq=41 last_seq=45 bytes=5 status=OK")
        self.feed(lines)

        tx = self.session.tags("tx")
        self.assertEqual(len(tx), 5)
        self.assertIn("DATA 0x68 'h'  seq=41  ACK  retries=0", tx[0])
        self.assertIn("DATA 0x6F 'o'  seq=45  ACK  retries=0", tx[4])
        info = self.session.tags("info")
        self.assertIn("command accepted", info[0])
        self.assertIn("status=OK bytes=5 seq 41..45", info[-1])

    def test_receive_message_is_shown_once(self) -> None:
        """RX lines + RXEND + RXMSG must not print the message twice."""
        lines = []
        for offset, byte in enumerate(b"hello"):
            lines += receive_exchange(41 + offset, byte)
        lines.append("@PHY RXEND seq=46")
        lines.append('@PHY RXMSG bytes=5 text="hello"')
        self.feed(lines)

        rx = self.session.tags("rx")
        self.assertEqual(len([line for line in rx if line.startswith("DATA")]), 5)
        self.assertEqual(rx[-1], "END  seq=46")
        messages = self.session.tags("msg")
        self.assertEqual(len(messages), 1, messages)
        self.assertEqual(messages[0], "message (5 bytes): 'hello'")

    def test_binary_message_is_assembled_by_the_host(self) -> None:
        """The firmware omits RXMSG for binary payloads, so RX + RXEND win."""
        self.feed(
            [
                "@PHY RX type=DATA seq=1 data=0x00",
                "@PHY RX type=DATA seq=2 data=0x01",
                "@PHY RXEND seq=3",
            ]
        )
        messages = self.session.tags("msg")
        self.assertEqual(len(messages), 1, messages)
        self.assertIn("[00 01]", messages[0])

    def test_rxmsg_wins_when_it_arrives_first(self) -> None:
        self.feed(['@PHY RXMSG bytes=2 text="hi"'])
        self.assertEqual(self.session.tags("msg"), ["message (2 bytes): 'hi'"])

    def test_step_lines_are_tagged_and_survive_quiet(self) -> None:
        quiet = RecordingSession(steps=True, quiet=True)
        for line in frame_exchange(1, 0x41):
            quiet.handle(line)
        steps = quiet.tags("step")
        self.assertEqual(len(steps), 3, quiet.out)
        self.assertIn("LINK_IDLE -> TX_FRAME", steps[0])

        plain = RecordingSession(steps=False)
        for line in frame_exchange(1, 0x41):
            plain.handle(line)
        self.assertEqual(plain.tags("step"), [])

    def test_eventual_nack_reports_reason(self) -> None:
        self.feed(
            [
                "@PHY NACK reason=0x01 seq=7",
                "@PHY TX type=DATA seq=7 data=0x41 status=FAIL retries=4",
                "@PHY TXDONE bytes=0 status=FAIL reason=RETRIES",
            ]
        )
        self.assertIn("NACK CRC seq=7", self.session.tags("warn")[0])
        self.assertIn("FAIL", self.session.tags("tx")[0])
        self.assertEqual(len(self.session.tags("err")), 1)
        self.assertIn("send status=FAIL", self.session.tags("err")[0])

    def test_not_owner_completes_the_send(self) -> None:
        self.feed(
            [
                '@PHY CMD ok=0 cmd=SEND detail="not the DATA originator; send TURN first"',
                "@PHY TXDONE bytes=0 status=FAIL reason=NOT_OWNER",
            ]
        )
        self.assertIn("command rejected", self.session.tags("err")[0])
        self.assertEqual(self.session.pending, None)  # no send was pending here


class FakeFirmware:
    """A pseudo-terminal board that answers with the firmware's own formats."""

    def __init__(self) -> None:
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.received = []
        self._stop = threading.Event()
        self._buffer = b""
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def emit(self, line: str) -> None:
        os.write(self.master, (line + "\r\n").encode("utf-8"))

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([self.master], [], [], 0.05)
            except (OSError, ValueError):
                return
            if not ready:
                continue
            try:
                chunk = os.read(self.master, 4096)
            except OSError:
                return
            self._buffer += chunk
            while b"\n" in self._buffer:
                raw, self._buffer = self._buffer.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").strip()
                if line:
                    self.received.append(line)
                    self._handle(line)

    def _handle(self, line: str) -> None:
        if line.startswith("SEND "):
            payload = line[5:]
            self.emit("@PHY CMD ok=1 cmd=SEND bytes=%d" % len(payload))
            for offset, char in enumerate(payload):
                for entry in frame_exchange(1 + offset, ord(char)):
                    self.emit(entry)
            self.emit(
                "@PHY TXDONE first_seq=1 last_seq=%d bytes=%d status=OK"
                % (len(payload), len(payload))
            )
        elif line.startswith("SENDHEX "):
            self.emit('@PHY CMD ok=0 cmd=SENDHEX detail="not the DATA originator"')
            self.emit("@PHY TXDONE bytes=0 status=FAIL reason=NOT_OWNER")
        elif line == "PING":
            self.emit("@PHY PONG uptime_ms=4242 role=A")
        elif line == "STATUS":
            self.emit("@PHY STATE link=IDLE owner=A seq_next=3 role=A session=0x0001")
            self.emit("@PHY STATS lane_tx=18 lane_rx=18 clk_hz=250 ir_hz=38000")
        elif line.startswith("CLK "):
            value = line.split(None, 1)[1]
            self.emit(
                '@PHY CMD ok=1 cmd=CLK detail="%s Hz nominal, 25000 us high, 25000 us low"'
                % value
            )
        elif line.startswith("IRFREQ "):
            value = line.split(None, 1)[1]
            self.emit('@PHY CMD ok=1 cmd=IRFREQ detail="%s Hz carrier"' % value)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


def run_cli(args, timeout=25):
    return subprocess.run(
        [sys.executable, CLI] + args,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


class CliAgainstFirmwareFormatsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.board = FakeFirmware()
        self.addCleanup(self.board.close)

    def test_send_waits_for_txdone(self) -> None:
        result = run_cli(["--port", self.board.path, "send", "hi"])
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertIn("status=OK bytes=2", result.stdout)
        self.assertIn("step", result.stdout)
        self.assertEqual(self.board.received[0], "SEND hi")

    def test_not_owner_is_reported_without_waiting(self) -> None:
        # Non-printable bytes make the CLI use SENDHEX, which this fake board
        # answers with the firmware's NOT_OWNER rejection.
        result = run_cli(
            ["--port", self.board.path, "send", "--hex", "00 01", "--timeout", "4"]
        )
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        # The CLI stops at CMD ok=0 and reports the firmware's own detail, so a
        # rejected send never waits out its timeout.
        self.assertIn("not the DATA originator", result.stderr)

    def test_ping_and_status(self) -> None:
        ping = run_cli(["--port", self.board.path, "ping"])
        self.assertEqual(ping.returncode, 0, ping.stderr)
        status = run_cli(["--port", self.board.path, "status"])
        self.assertEqual(status.returncode, 0, status.stderr)
        self.assertIn("link=IDLE", status.stdout)
        self.assertIn("lane_tx=18", status.stdout)

    def test_frequency_commands(self) -> None:
        clk = run_cli(["--port", self.board.path, "clk", "20"])
        self.assertEqual(clk.returncode, 0, clk.stderr)
        self.assertIn("20 Hz nominal", clk.stdout)
        irfreq = run_cli(["--port", self.board.path, "irfreq", "40000"])
        self.assertEqual(irfreq.returncode, 0, irfreq.stderr)
        self.assertIn("40000 Hz carrier", irfreq.stdout)
        self.assertEqual(self.board.received[-2:], ["CLK 20", "IRFREQ 40000"])


if __name__ == "__main__":
    unittest.main()
