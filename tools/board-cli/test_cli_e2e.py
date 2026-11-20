"""End-to-end tests: run the CLI against a fake board on a pseudo-terminal.

The fake board implements just enough of the ``@PHY`` line protocol from
README.md to exercise send, monitor, message assembly and frame decoding.

Run with:  uv run --with pyserial python -m unittest discover -s tools/board-cli
"""

import os
import pty
import select
import subprocess
import sys
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
CLI = os.path.join(HERE, "board_cli.py")

import phyframe  # noqa: E402  (script directory is on sys.path)
import board_cli  # noqa: E402


class ProtocolParsingTests(unittest.TestCase):
    def test_quoted_values_with_spaces(self) -> None:
        kv = board_cli.parse_kv(r'ok=0 cmd=SEND detail="not the originator"')
        self.assertEqual(kv["cmd"], "SEND")
        self.assertEqual(kv["detail"], "not the originator")

    def test_json_and_c_escapes(self) -> None:
        self.assertEqual(board_cli.unquote(r'"line\nbreak"'), "line\nbreak")
        self.assertEqual(board_cli.unquote(r'"say \"hi\""'), 'say "hi"')
        self.assertEqual(board_cli.unquote(r'"bell\x07"'), "bell\a")
        self.assertEqual(board_cli.unquote("plain"), "plain")


class FakeBoard:
    """A board that answers the host console protocol over a pty."""

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
            if not chunk:
                return
            self._buffer += chunk
            while b"\n" in self._buffer:
                raw, self._buffer = self._buffer.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").strip()
                if line:
                    self.received.append(line)
                    self._handle(line)

    def _handle(self, line: str) -> None:
        if line == "SEND failme":
            self.emit('@PHY CMD ok=0 cmd=SEND detail="not the originator"')
            return
        if line.startswith("SEND "):
            payload = line[5:].encode("utf-8")
            self.emit("@PHY READY role=B session=0x0001 crc=CRC-8/SMBUS")
            self.emit("@PHY CMD ok cmd=SEND bytes=%d first_seq=1" % len(payload))
            for offset, byte in enumerate(payload):
                self.emit(
                    "@PHY TX seq=%d type=DATA data=0x%02X status=ACK retries=0"
                    % (offset + 1, byte)
                )
            self.emit(
                "@PHY TXDONE first_seq=1 last_seq=%d bytes=%d status=OK"
                % (len(payload), len(payload))
            )
        elif line == "PING":
            self.emit("@PHY PONG uptime_ms=1234")
        elif line == "STATUS":
            self.emit("@PHY STATE link=IDLE owner=A seq_next=42 session=0x0001")

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


class CliEndToEndTests(unittest.TestCase):
    def setUp(self) -> None:
        self.board = FakeBoard()
        self.addCleanup(self.board.close)

    def test_ports_does_not_need_a_board(self) -> None:
        result = run_cli(["ports"])
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_frame_command_is_offline(self) -> None:
        result = run_cli(["frame", "--type", "DATA", "--data", "0x41"])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("01100110", result.stdout)
        self.assertIn("CRC-8/SMBUS", result.stdout)

    def test_decode_command(self) -> None:
        bits = phyframe.bits_to_str(phyframe.build_frame("DATA", 0x41).bits)
        result = run_cli(["decode", bits])
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("OK", result.stdout)
        self.assertIn("DATA", result.stdout)

    def test_send_waits_for_completion(self) -> None:
        result = run_cli(
            [
                "--port",
                self.board.path,
                "--no-timestamps",
                "--no-color",
                "send",
                "hello",
            ]
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        self.assertIn("SEND hello", self.board.received)
        self.assertIn("DATA 0x68", result.stdout)
        self.assertIn("send status=OK", result.stdout)

    def test_rejected_send_fails_fast(self) -> None:
        started = time.time()
        result = run_cli(
            [
                "--port",
                self.board.path,
                "--no-timestamps",
                "--no-color",
                "send",
                "failme",
            ]
        )
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertIn("not the originator", result.stderr)
        self.assertLess(time.time() - started, 5.0)

    def test_status(self) -> None:
        result = run_cli(
            ["--port", self.board.path, "--no-timestamps", "--no-color", "status"]
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("link=IDLE", result.stdout)

    def test_monitor_displays_logs_and_received_message(self) -> None:
        proc = subprocess.Popen(
            [
                sys.executable,
                CLI,
                "--port",
                self.board.path,
                "--no-timestamps",
                "--no-color",
                "monitor",
                "--seconds",
                "2",
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(0.6)
        self.board.emit("[phy]  link idle, owner A")
        for offset, char in enumerate("hello"):
            self.board.emit(
                "@PHY RX type=DATA seq=%d data=0x%02X" % (10 + offset, ord(char))
            )
        # This is what main/serial_cli.c emits at the end of a message.
        self.board.emit("@PHY RXEND seq=15")
        self.board.emit('@PHY RXMSG bytes=5 text="hello"')
        out, err = proc.communicate(timeout=15)
        self.assertEqual(proc.returncode, 0, err)
        self.assertIn("link idle", out)
        self.assertEqual(out.count("message (5 bytes)"), 1, out)
        self.assertIn("'hello'", out)

    def test_monitor_decodes_bit_dumps(self) -> None:
        bits = phyframe.bits_to_str(phyframe.build_frame("DATA", 0x41).bits)
        proc = subprocess.Popen(
            [
                sys.executable,
                CLI,
                "--port",
                self.board.path,
                "--no-timestamps",
                "--no-color",
                "--decode-bits",
                "monitor",
                "--seconds",
                "2",
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(0.6)
        self.board.emit("[phy.] tx bits=%s" % bits)
        out, err = proc.communicate(timeout=15)
        self.assertEqual(proc.returncode, 0, err)
        self.assertIn("bits=" + bits, out)
        self.assertIn("DATA data=0x41 'A'", out)

    def test_ping(self) -> None:
        result = run_cli(
            ["--port", self.board.path, "--no-timestamps", "--no-color", "ping"]
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("pong", result.stdout)

    def test_interactive_stdin_sends_message(self) -> None:
        proc = subprocess.Popen(
            [
                sys.executable,
                CLI,
                "--port",
                self.board.path,
                "--no-timestamps",
                "--no-color",
                "monitor",
                "--seconds",
                "2",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(0.6)
        assert proc.stdin is not None
        proc.stdin.write("hello\n")
        proc.stdin.flush()
        out, err = proc.communicate(timeout=15)
        self.assertEqual(proc.returncode, 0, err)
        self.assertIn("SEND hello", self.board.received)
        self.assertIn("queued 5 byte(s)", out)
        self.assertIn("send status=OK", out)


if __name__ == "__main__":
    unittest.main()
