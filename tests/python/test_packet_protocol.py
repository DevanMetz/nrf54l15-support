"""Packet reference, bounded stream parsing, and live-probe transport tests."""
import argparse
import os
from pathlib import Path
import random
import subprocess
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pc-app"))
from packet_protocol import BadPacket, Decoder, MAX_ENCODED, PacketTooLarge, decode_body, encode
from uart_packet_probe import _positive_int, _reply, _timeout, verify_echo


class PacketTests(unittest.TestCase):
    def test_known_vectors(self):
        cases = [
            (b"", "00010101010100"),
            (b"123456789", "000e3132333435363738392639f4cb00"),
            (bytes([0, 1, 0, 255, 128, 0]), "0001020103ff8005b524b90900"),
        ]
        for payload, hex_frame in cases:
            self.assertEqual(encode(payload).hex(), hex_frame)
            self.assertEqual(decode_body(bytes.fromhex(hex_frame)[1:-1]), payload)

    def test_lengths_chunks_and_back_to_back_frames(self):
        for size in range(257):
            payload = bytes(index & 255 for index in range(size))
            wire = encode(payload)
            self.assertLessEqual(len(wire), 264)
            self.assertNotIn(0, wire[1:-1])
            for chunk_size in (1, 2, 7, 64, 512):
                decoder = Decoder()
                received = []
                data = b"junk before boundary" + wire + wire
                for offset in range(0, len(data), chunk_size):
                    received.extend(decoder.feed(data[offset:offset + chunk_size]))
                    self.assertLessEqual(decoder.pending_bytes, MAX_ENCODED)
                self.assertEqual(received, [payload, payload])
                self.assertEqual(decoder.bad_frames + decoder.oversized_frames, 0)

    def test_corruption_and_recovery(self):
        wire = encode(b"123456789")
        for position in range(len(wire)):
            for bit in range(8):
                damaged = bytearray(wire)
                damaged[position] ^= 1 << bit
                decoder = Decoder()
                self.assertEqual(decoder.feed(damaged), [])
                self.assertEqual(decoder.feed(wire), [b"123456789"])
        for position in range(1, len(wire) - 1):
            decoder = Decoder()
            self.assertEqual(decoder.feed(wire[:position] + wire[position + 1:]), [])
            self.assertEqual(decoder.feed(wire), [b"123456789"])

    def test_reset_drops_partial_packets_and_preserves_counts(self):
        wire = encode(b"partial")
        for cut in range(1, len(wire)):
            decoder = Decoder()
            self.assertEqual(decoder.feed(wire[:cut]), [])
            decoder.reset()
            self.assertEqual(decoder.feed(wire[cut:]), [])
            self.assertEqual(decoder.feed(wire), [b"partial"])
        decoder.feed(b"\0\xff\0")
        decoder.reset()
        self.assertEqual(decoder.bad_frames, 1)
        self.assertEqual(decoder.feed(encode(b"")), [b""])

    def test_limits_and_malformed_bodies(self):
        with self.assertRaises(PacketTooLarge):
            encode(bytes(257))
        for malformed in (b"", b"\0", b"\xff", b"\x01", b"\x02\x01", b"\x03\x01"):
            with self.assertRaises(BadPacket):
                decode_body(malformed)
        for oversized in (b"\x01" * 263, b"\x01" * 262):
            with self.assertRaises(PacketTooLarge):
                decode_body(oversized)
        decoder = Decoder()
        self.assertEqual(decoder.feed(b"\0" + b"\x01" * 100000), [])
        self.assertEqual(decoder.oversized_frames, 1)
        self.assertEqual(decoder.pending_bytes, 0)
        self.assertEqual(decoder.feed(encode(b"recovered")), [b"recovered"])
        self.assertEqual(decoder.feed(b"\0" * 100), [])

    def test_c_reference_interoperability(self):
        explicit = os.environ.get("NRF54L15_PACKET_TEST_BIN")
        candidates = ([Path(explicit)] if explicit else [
            ROOT / "build" / directory / executable
            for directory in ("native/tests/host", "native/tests/host/Debug", "host-clang", "host-tests")
            for executable in ("packet_tests.exe", "packet_tests")
        ])
        executable = next((path for path in candidates if path.is_file()), None)
        if executable is None:
            self.skipTest("build packet_tests first or set NRF54L15_PACKET_TEST_BIN")
        generator = random.Random(54715)
        payloads = []
        for size in range(257):
            payloads.extend((bytes(size), bytes([255]) * size,
                             bytes(index & 255 for index in range(size)), generator.randbytes(size)))
        fixtures = "".join(f"{payload.hex()}:{encode(payload).hex()}\n" for payload in payloads)
        result = subprocess.run([str(executable), "--interop"], input=fixtures, text=True,
                                capture_output=True, timeout=15, check=False)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(f"Verified {len(payloads)} Python/C packet vectors", result.stdout)


class EchoPort:
    """Serial-shaped reference peer, not a hardware timing model."""
    def __init__(self, fault=None):
        self.peer = Decoder()
        self.queue = bytearray()
        self.timeout = None
        self.write_timeout = None
        self.fault = fault

    @property
    def in_waiting(self):
        return len(self.queue)

    def reset_input_buffer(self):
        self.queue.clear()

    def write(self, data):
        if self.fault == "partial_write":
            return len(data) - 1
        for payload in self.peer.feed(data):
            if self.fault == "silent":
                continue
            if self.fault == "mismatch":
                payload = b"wrong"
            reply = encode(payload)
            if self.fault == "corrupt":
                reply = reply[:-2] + b"\xfe\0"
            if self.fault == "duplicate":
                reply += reply
            if self.fault == "partial_tail":
                reply += b"\x05\x12"
            self.queue.extend(reply)
        if self.fault == "raw_loopback":
            self.queue.extend(data)
        return len(data)

    def read(self, count):
        result = bytes(self.queue[:count])
        del self.queue[:count]
        return result


class ProbeTests(unittest.TestCase):
    def test_reference_echo_all_payload_sizes_and_error_types(self):
        port = EchoPort()
        self.assertEqual(verify_echo(port), (257, 16))
        self.assertGreater(port.peer.bad_frames, 0)
        self.assertGreater(port.peer.oversized_frames, 0)
        self.assertEqual(port.write_timeout, 1.0)

    def test_probe_detects_errors(self):
        for fault in ("partial_write", "mismatch", "corrupt", "duplicate", "partial_tail", "raw_loopback"):
            with self.subTest(fault=fault), self.assertRaises(OSError):
                verify_echo(EchoPort(fault), rounds=1)

    def test_reply_deadline_and_invalid_arguments(self):
        with patch("uart_packet_probe.time.monotonic", side_effect=[0.0, 0.5, 1.1]):
            with self.assertRaises(TimeoutError):
                _reply(EchoPort("silent"), Decoder(), 1.0)
        for timeout in (0, -1, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                verify_echo(EchoPort(), timeout=timeout)
        with self.assertRaises(ValueError):
            verify_echo(EchoPort(), rounds=0)
        for timeout in ("0", "-1", "inf", "nan"):
            with self.assertRaises(argparse.ArgumentTypeError):
                _timeout(timeout)
        for count in ("0", "-2"):
            with self.assertRaises(argparse.ArgumentTypeError):
                _positive_int(count)


if __name__ == "__main__":
    unittest.main()
