"""Unit tests for the host-side frame codec.

Run with:  uv run --with pyserial python -m unittest discover -s tools/board-cli
"""

import unittest

import phyframe


class CrcTests(unittest.TestCase):
    def test_catalogue_check_values(self) -> None:
        # Each profile must reproduce its published check value for the
        # ASCII string "123456789" (spec section 4 and main/crc8.c).
        for key, params in phyframe.CRC_PROFILES.items():
            with self.subTest(profile=key):
                self.assertEqual(phyframe.crc8(b"123456789", key), params.check)

    def test_smbus_is_the_default(self) -> None:
        self.assertEqual(phyframe.DEFAULT_CRC_PROFILE, "smbus")
        self.assertEqual(phyframe.crc8(b"123456789"), 0xF4)

    def test_frame_checksum_input_order(self) -> None:
        # CONTROL in the low nibble (high nibble zero), then DATA.
        self.assertEqual(
            phyframe.crc8_frame_checksum(0x1, 0x41),
            phyframe.crc8(bytes([0x01, 0x41])),
        )
        self.assertEqual(
            phyframe.crc8_frame_checksum(0xF1, 0x00),
            phyframe.crc8(bytes([0x01, 0x00])),
        )

    def test_unknown_profile_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            phyframe.crc8(b"x", "not-a-crc")


class FrameLayoutTests(unittest.TestCase):
    def test_field_placement_and_sizes(self) -> None:
        frame = phyframe.build_frame("DATA", 0x41)
        self.assertEqual(len(frame.bits), phyframe.FRAME_BITS)
        self.assertEqual(len(frame.bits), 36)
        self.assertEqual(
            phyframe.bits_to_str(frame.bits[0:8]), "01100110", "SYN must be 0x66"
        )
        self.assertEqual(
            phyframe.bits_to_str(frame.bits[8:16]), "11010101", "SFD must be 0xD5"
        )
        self.assertEqual(phyframe.bits_to_str(frame.bits[16:20]), "0001")
        self.assertEqual(phyframe.bits_to_str(frame.bits[20:28]), "01000001")
        self.assertEqual(
            phyframe.bits_to_str(frame.bits[28:36]),
            phyframe.bits_to_str(phyframe.byte_to_bits(frame.crc)),
        )
        self.assertEqual(frame.control, phyframe.FRAME_DATA)

    def test_data_0x41_lane_pairs(self) -> None:
        # Spec section 3 table: DATA 0x41 occupies global cycles 10-13 with
        # pairs (IR, ESP-NOW) = 01, 00, 00, 01.
        frame = phyframe.build_frame("DATA", 0x41)
        ir, wifi = phyframe.frame_to_lanes(frame.bits)
        pairs = ["%d%d" % (a, b) for a, b in zip(ir[10:14], wifi[10:14])]
        self.assertEqual(pairs, ["01", "00", "00", "01"])

    def test_lane_round_trip(self) -> None:
        for frame_type in ("DATA", "ACK", "NACK", "TURN", "END"):
            with self.subTest(type=frame_type):
                frame = phyframe.build_frame(frame_type, 0x00)
                ir, wifi = phyframe.frame_to_lanes(frame.bits)
                self.assertEqual(phyframe.frame_from_lanes(ir, wifi), frame.bits)

    def test_all_field_boundaries_round_trip(self) -> None:
        for frame_type in (1, 2, 3, 4, 5):
            for data in (0x00, 0x01, 0x7F, 0x80, 0xFF, 0x41, 0x55, 0xAA):
                with self.subTest(control=frame_type, data=data):
                    frame = phyframe.build_frame(frame_type, data)
                    parsed = phyframe.parse_bits(frame.bits)
                    self.assertTrue(parsed.ok, parsed.detail)
                    self.assertEqual(parsed.frame.control, frame_type)
                    self.assertEqual(parsed.frame.data, data)
                    self.assertEqual(parsed.frame.crc, frame.crc)

    def test_grouped_bits(self) -> None:
        frame = phyframe.build_frame("ACK", 0x00)
        groups = phyframe.bits_grouped(frame.bits).split()
        self.assertEqual([len(g) for g in groups], [8, 8, 4, 8, 8])
        self.assertEqual("".join(groups), phyframe.bits_to_str(frame.bits))

    def test_normalize_accepts_separators(self) -> None:
        bits = phyframe.build_frame("DATA", 0x41).bits
        text = phyframe.bits_to_str(bits)
        spaced = " ".join(text[i : i + 8] for i in range(0, 36, 8))
        self.assertEqual(phyframe.normalize_bits(spaced), bits)
        self.assertEqual(phyframe.normalize_bits(text.replace("0", "0_")), bits)


class ParseTests(unittest.TestCase):
    def test_parse_reports_fields(self) -> None:
        frame = phyframe.build_frame("NACK", 0x03)
        parsed = phyframe.parse_bits(frame.bits)
        self.assertEqual(parsed.status, phyframe.PARSE_OK)
        self.assertEqual(parsed.frame.type_name, "NACK")
        self.assertEqual(
            phyframe.NACK_REASON_NAMES[parsed.frame.data], "missing lane bit"
        )

    def test_bad_syn_and_sfd(self) -> None:
        frame = phyframe.build_frame("DATA", 0x41)
        for position in (0, 8):
            bits = list(frame.bits)
            bits[position] ^= 1
            parsed = phyframe.parse_bits(bits)
            self.assertEqual(parsed.status, phyframe.PARSE_BAD_FRAMING)

    def test_single_bit_flips_are_detected(self) -> None:
        frame = phyframe.build_frame("DATA", 0x41)
        for position in range(phyframe.FRAME_BITS):
            bits = list(frame.bits)
            bits[position] ^= 1
            parsed = phyframe.parse_bits(bits)
            self.assertNotEqual(
                parsed.status,
                phyframe.PARSE_OK,
                "flip at bit %d was accepted" % position,
            )

    def test_reserved_control_is_unsupported(self) -> None:
        frame = phyframe.build_frame("DATA", 0x41)
        bits = list(frame.bits)
        # Clear the CONTROL nibble: 0x0 is reserved.
        for position in range(16, 20):
            bits[position] = 0
        parsed = phyframe.parse_bits(bits)
        self.assertEqual(parsed.status, phyframe.PARSE_UNSUPPORTED)

    def test_wrong_crc_is_rejected(self) -> None:
        frame = phyframe.build_frame("DATA", 0x41)
        bits = list(frame.bits)
        bits[35] ^= 1
        parsed = phyframe.parse_bits(bits)
        self.assertEqual(parsed.status, phyframe.PARSE_BAD_CRC)
        self.assertNotEqual(parsed.frame.crc, parsed.expected_crc)

    def test_wrong_length_is_an_error(self) -> None:
        with self.assertRaises(ValueError):
            phyframe.parse_bits("0" * 35)


class TextHelpersTests(unittest.TestCase):
    def test_find_bit_runs(self) -> None:
        frame = phyframe.build_frame("DATA", 0x41)
        text = "rx bits=%s seq=3" % phyframe.bits_to_str(frame.bits)
        runs = phyframe.find_bit_runs(text)
        self.assertEqual(len(runs), 1)
        self.assertEqual(runs[0], frame.bits)
        self.assertEqual(phyframe.find_bit_runs("no bits here"), ())

    def test_hex_parsing(self) -> None:
        self.assertEqual(phyframe.parse_hex_bytes("68 65 6c"), b"hel")
        self.assertEqual(phyframe.parse_hex_bytes("0x68,0x65"), b"he")
        self.assertEqual(phyframe.parse_hex_bytes("68656c"), b"hel")
        with self.assertRaises(ValueError):
            phyframe.parse_hex_bytes("nope")

    def test_describe_bits(self) -> None:
        frame = phyframe.build_frame("DATA", 0x41)
        self.assertIn("DATA", phyframe.describe_bits(frame.bits))


if __name__ == "__main__":
    unittest.main()
