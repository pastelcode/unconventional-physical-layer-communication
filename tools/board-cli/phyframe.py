"""Host-side codec for the 36-bit dual-medium PHY frame.

This mirrors ``main/frame.h``, ``main/frame.c`` and ``main/crc8.c`` so frames
can be built and decoded without a board attached.  Layout, MSB-first field
ordering, lane pairing and the CRC input order are exactly those of the
implementation specification (sections 3 and 4):

    SYN[8] | SFD[8] | CONTROL[4] | DATA[8] | CRC[8]     36 bits, 18 cycles

Cycle ``k`` carries IR = logical bit ``2*k`` and ESP-NOW = ``2*k + 1``.
CRC input is ``bytes([CONTROL & 0x0F, DATA])`` only; SYN, SFD and the CRC
byte are excluded.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Dict, Iterable, Sequence, Tuple, Union

FRAME_BITS = 36
FRAME_CYCLES = 18

FRAME_SYN = 0x66
FRAME_SFD = 0xD5

FRAME_DATA = 0x1
FRAME_ACK = 0x2
FRAME_NACK = 0x3
FRAME_TURN = 0x4
FRAME_END = 0x5

FRAME_TYPE_NAMES: Dict[int, str] = {
    FRAME_DATA: "DATA",
    FRAME_ACK: "ACK",
    FRAME_NACK: "NACK",
    FRAME_TURN: "TURN",
    FRAME_END: "END",
}
FRAME_TYPE_IDS = {name: value for value, name in FRAME_TYPE_NAMES.items()}

NACK_REASON_NAMES: Dict[int, str] = {
    0x01: "CRC",
    0x02: "framing",
    0x03: "missing lane bit",
    0x04: "timeout",
    0x05: "unexpected state",
    0x06: "unsupported type",
}

# Parse status names mirror the firmware's frame_parse_result_t.
PARSE_OK = "OK"
PARSE_BAD_FRAMING = "BAD_FRAMING"
PARSE_BAD_CRC = "BAD_CRC"
PARSE_UNSUPPORTED = "UNSUPPORTED"


@dataclass(frozen=True)
class CrcProfile:
    name: str
    poly: int
    init: int
    refin: bool
    refout: bool
    xorout: int
    check: int  # CRC-8 of ASCII "123456789"


CRC_PROFILES: Dict[str, CrcProfile] = {
    "smbus": CrcProfile("CRC-8/SMBUS", 0x07, 0x00, False, False, 0x00, 0xF4),
    "autosar": CrcProfile("CRC-8/AUTOSAR", 0x2F, 0xFF, False, False, 0xFF, 0xDF),
    "sae-j1850": CrcProfile("CRC-8/SAE-J1850", 0x1D, 0xFF, False, False, 0xFF, 0x4B),
    "maxim-dow": CrcProfile("CRC-8/MAXIM-DOW", 0x31, 0x00, True, True, 0x00, 0xA1),
}
DEFAULT_CRC_PROFILE = "smbus"

BitLike = Union[int, bool]


def reflect8(value: int) -> int:
    value &= 0xFF
    value = ((value & 0xF0) >> 4) | ((value & 0x0F) << 4)
    value = ((value & 0xCC) >> 2) | ((value & 0x33) << 2)
    value = ((value & 0xAA) >> 1) | ((value & 0x55) << 1)
    return value & 0xFF


def resolve_profile(profile: Union[str, CrcProfile, None]) -> CrcProfile:
    if isinstance(profile, CrcProfile):
        return profile
    key = (profile or DEFAULT_CRC_PROFILE).lower()
    try:
        return CRC_PROFILES[key]
    except KeyError:
        raise ValueError(
            "unknown CRC profile %r (choose from %s)"
            % (profile, ", ".join(sorted(CRC_PROFILES)))
        ) from None


def crc8(
    data: bytes, profile: Union[str, CrcProfile, None] = DEFAULT_CRC_PROFILE
) -> int:
    """CRC-8 with the catalogue parameters of the requested profile."""
    params = resolve_profile(profile)
    crc = params.init & 0xFF
    for byte in data:
        if params.refin:
            byte = reflect8(byte)
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ params.poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    if params.refout:
        crc = reflect8(crc)
    return (crc ^ params.xorout) & 0xFF


def crc8_frame_checksum(
    control: int, data: int, profile: Union[str, CrcProfile, None] = DEFAULT_CRC_PROFILE
) -> int:
    return crc8(bytes(((control & 0x0F), data & 0xFF)), profile)


def byte_to_bits(value: int) -> Tuple[int, ...]:
    return tuple((value >> (7 - i)) & 1 for i in range(8))


def bits_to_byte(bits: Sequence[BitLike]) -> int:
    value = 0
    for bit in bits:
        value = (value << 1) | (1 if bit else 0)
    return value


@dataclass(frozen=True)
class Frame:
    control: int
    data: int
    crc: int
    bits: Tuple[int, ...]

    @property
    def type_name(self) -> str:
        return FRAME_TYPE_NAMES.get(self.control, "INVALID")

    @property
    def supported(self) -> bool:
        return self.control in FRAME_TYPE_NAMES


@dataclass(frozen=True)
class ParsedFrame:
    status: str
    frame: Frame
    expected_crc: int
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.status == PARSE_OK


def frame_control(frame_type: Union[int, str]) -> int:
    if isinstance(frame_type, str):
        key = frame_type.upper()
        if key not in FRAME_TYPE_IDS:
            raise ValueError(
                "unknown frame type %r (choose from %s)"
                % (frame_type, ", ".join(FRAME_TYPE_IDS))
            )
        return FRAME_TYPE_IDS[key]
    value = int(frame_type)
    if not 0 <= value <= 0x0F:
        raise ValueError("control must fit in 4 bits, got %r" % frame_type)
    return value


def build_frame(
    frame_type: Union[int, str],
    data: int,
    profile: Union[str, CrcProfile, None] = DEFAULT_CRC_PROFILE,
) -> Frame:
    """Build a complete 36-bit frame from a CONTROL value and a DATA byte."""
    control = frame_control(frame_type)
    if not 0 <= int(data) <= 0xFF:
        raise ValueError("data must be one byte, got %r" % data)
    data = int(data)
    crc = crc8_frame_checksum(control, data, profile)

    bits = (
        byte_to_bits(FRAME_SYN)
        + byte_to_bits(FRAME_SFD)
        + tuple((control >> (3 - i)) & 1 for i in range(4))
        + byte_to_bits(data)
        + byte_to_bits(crc)
    )
    assert len(bits) == FRAME_BITS
    return Frame(control=control, data=data, crc=crc, bits=bits)


def frame_to_lanes(bits: Sequence[BitLike]) -> Tuple[Tuple[int, ...], Tuple[int, ...]]:
    """Split the serialized bit stream into (IR bits, ESP-NOW bits) per cycle."""
    bits = normalize_bits(bits)
    if len(bits) != FRAME_BITS:
        raise ValueError("need %d bits, got %d" % (FRAME_BITS, len(bits)))
    ir = tuple(1 if bits[2 * k] else 0 for k in range(FRAME_CYCLES))
    wifi = tuple(1 if bits[2 * k + 1] else 0 for k in range(FRAME_CYCLES))
    return ir, wifi


def frame_from_lanes(
    ir_bits: Sequence[BitLike], wifi_bits: Sequence[BitLike]
) -> Tuple[int, ...]:
    """Rebuild the 36-bit stream, IR first within each cycle."""
    if len(ir_bits) != FRAME_CYCLES or len(wifi_bits) != FRAME_CYCLES:
        raise ValueError("each lane must carry %d bits" % FRAME_CYCLES)
    bits = []
    for ir, wifi in zip(ir_bits, wifi_bits):
        bits.append(1 if ir else 0)
        bits.append(1 if wifi else 0)
    return tuple(bits)


def normalize_bits(value: Union[str, Iterable[BitLike]]) -> Tuple[int, ...]:
    """Accept 0/1 ints or a string of 0/1 with optional separators/spaces."""
    if isinstance(value, str):
        text = re.sub(r"[\s_,:.\-]", "", value)
        if not text or any(ch not in "01" for ch in text):
            raise ValueError("bit string must contain only 0/1 (spaces allowed)")
        return tuple(int(ch) for ch in text)
    bits = tuple(1 if bit else 0 for bit in value)
    return bits


def bits_to_str(bits: Sequence[BitLike]) -> str:
    return "".join("1" if bit else "0" for bit in bits)


def bits_grouped(bits: Sequence[BitLike]) -> str:
    """36 bits grouped per field: SYN SFD CONTROL DATA CRC."""
    bits = normalize_bits(bits)
    if len(bits) != FRAME_BITS:
        raise ValueError("need %d bits, got %d" % (FRAME_BITS, len(bits)))
    text = bits_to_str(bits)
    return " ".join((text[0:8], text[8:16], text[16:20], text[20:28], text[28:36]))


def parse_bits(
    bits: Union[str, Iterable[BitLike]],
    profile: Union[str, CrcProfile, None] = DEFAULT_CRC_PROFILE,
) -> ParsedFrame:
    """Decode a 36-bit stream the same way ``frame_parse_bits`` does.

    Order of checks matches the firmware: framing first, then the control
    type, then the CRC.  ``frame`` is always populated so callers can show
    what was actually received even when a check failed.
    """
    bits = normalize_bits(bits)
    if len(bits) != FRAME_BITS:
        raise ValueError("need %d bits, got %d" % (FRAME_BITS, len(bits)))

    control = bits_to_byte(bits[16:20])
    data = bits_to_byte(bits[20:28])
    crc = bits_to_byte(bits[28:36])
    frame = Frame(control=control, data=data, crc=crc, bits=bits)
    expected = crc8_frame_checksum(control, data, profile)

    if bits_to_byte(bits[0:8]) != FRAME_SYN or bits_to_byte(bits[8:16]) != FRAME_SFD:
        detail = "SYN/SFD mismatch (expected 0x%02X 0x%02X)" % (FRAME_SYN, FRAME_SFD)
        return ParsedFrame(PARSE_BAD_FRAMING, frame, expected, detail)
    if control not in FRAME_TYPE_NAMES:
        detail = "CONTROL 0x%X is reserved" % control
        return ParsedFrame(PARSE_UNSUPPORTED, frame, expected, detail)
    if crc != expected:
        detail = "CRC 0x%02X, expected 0x%02X" % (crc, expected)
        return ParsedFrame(PARSE_BAD_CRC, frame, expected, detail)
    return ParsedFrame(PARSE_OK, frame, expected, "")


def printable_char(value: int) -> str:
    return chr(value) if 0x20 <= value <= 0x7E else "."


def frame_data_note(frame: Frame) -> str:
    """Human meaning of the DATA byte for the given frame type."""
    if frame.control == FRAME_DATA:
        return "0x%02X %r" % (frame.data, printable_char(frame.data))
    if frame.control == FRAME_NACK:
        reason = NACK_REASON_NAMES.get(frame.data, "unknown reason")
        return "0x%02X (%s)" % (frame.data, reason)
    if frame.control in (FRAME_ACK, FRAME_TURN, FRAME_END):
        return "0x%02X (reserved)" % frame.data
    return "0x%02X" % frame.data


def describe_bits(
    bits: Union[str, Iterable[BitLike]],
    profile: Union[str, CrcProfile, None] = DEFAULT_CRC_PROFILE,
) -> str:
    parsed = parse_bits(bits, profile)
    frame = parsed.frame
    if parsed.ok:
        return "%s data=%s crc=0x%02X" % (
            frame.type_name,
            frame_data_note(frame),
            frame.crc,
        )
    return "%s: %s (control=0x%X data=0x%02X)" % (
        parsed.status,
        parsed.detail,
        frame.control,
        frame.data,
    )


_BIT_RUN_RE = re.compile(r"(?<![01])[01]{36}(?![01])")


def find_bit_runs(text: str) -> Tuple[Tuple[int, ...], ...]:
    """Find bare 36-bit bit strings in a log line (e.g. ``bits=0110...``)."""
    return tuple(normalize_bits(match.group(0)) for match in _BIT_RUN_RE.finditer(text))


def hex_bytes(data: bytes) -> str:
    return " ".join("%02x" % byte for byte in data)


def parse_hex_bytes(text: str) -> bytes:
    """Parse ``68 65 6c`` / ``0x68,0x65`` / ``68656c`` into bytes."""
    cleaned = re.sub(r"0x", "", text, flags=re.IGNORECASE)
    tokens = [token for token in re.split(r"[\s,;:._-]+", cleaned) if token]
    if len(tokens) == 1 and len(tokens[0]) > 2:
        token = tokens[0]
        if len(token) % 2:
            raise ValueError("hex string has an odd number of digits")
        tokens = [token[i : i + 2] for i in range(0, len(token), 2)]
    try:
        return bytes(int(token, 16) for token in tokens)
    except ValueError:
        raise ValueError("not a hex byte sequence: %r" % text) from None
