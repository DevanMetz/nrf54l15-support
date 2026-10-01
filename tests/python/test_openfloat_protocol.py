import dataclasses
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pc-app"))
from openfloat_protocol import (BleSample, DeviceStats, LiveSample, ShotEvent, decode_ble,
                                decode_text, missing_notifications, text_lines)


class WireTests(unittest.TestCase):
    def test_serial_fragmentation(self):
        frame = b"OFSHOT,1,2,1234567,12000,-500,1000,2\r\n"
        for split in range(len(frame) + 1):
            lines = list(text_lines([frame[:split], b"", frame[split:]]))
            self.assertEqual(len(lines), 1)
            self.assertIsInstance(decode_text(lines[0]), ShotEvent)
        self.assertEqual(list(text_lines([frame[:-1]])), [])

    def test_serial_resynchronization(self):
        bad = b"OFRAW," + b"0" * 1000 + b"\n"
        good = b"OFSHOT,1,1,1000,12000,0,0,1\n"
        lines = list(text_lines([bad[:500], bad[500:] + good]))
        self.assertEqual(len(lines), 2)
        self.assertEqual(len(lines[0]), 513)
        with self.assertRaises(ValueError):
            decode_text(lines[0])
        self.assertIsInstance(decode_text(lines[1]), ShotEvent)

    def test_firmware_golden_frame(self):
        data = bytes.fromhex("4f46010134126309e80318fc00001000e0ff0000")
        sample = decode_ble(data)
        self.assertEqual(sample, BleSample(0x1234, 2403, (1000, -1000, 0), (16, -32, 0)))
        self.assertEqual(sample.gyro_dps, (1.0, -2.0, 0.0))
        self.assertEqual(decode_ble(bytearray(data)), sample)
        self.assertEqual(decode_ble(memoryview(data)), sample)

    def test_signed_limits(self):
        frame = struct.pack("<2sBBHH6h", b"OF", 1, 1, 65535, 65535,
                            -32768, 32767, 0, 32767, -32768, 0)
        sample = decode_ble(frame)
        self.assertEqual(sample.accel_mg, (-32768, 32767, 0))
        self.assertEqual(sample.gyro_dps_q4, (32767, -32768, 0))

    def test_bad_binary(self):
        for frame in (b"", bytes(19), bytes(21), bytes(20),
                      b"OF\x02\x01" + bytes(16), b"OF\x01\x02" + bytes(16)):
            with self.subTest(frame=frame), self.assertRaises(ValueError):
                decode_ble(frame)

    def test_text(self):
        line = "OFRAW,1,8,1234567,2403,1000,-1000,0,1000,-2000,0,123,-456,0,1000000,0,0,0,2\r\n"
        sample = decode_text(line)
        self.assertEqual(sample, LiveSample(8, 1234567, 2403, (1000, -1000, 0),
                                           (1000, -2000, 0), (123, -456, 0),
                                           (1000000, 0, 0, 0), 2))
        with self.assertRaises(dataclasses.FrozenInstanceError):
            sample.sequence = 0
        self.assertEqual(decode_text("OFSHOT,1,2,1234567,12000,-500,1000,2"),
                         ShotEvent(2, 1234567, (12000, -500, 1000), 2))

    def test_bad_text_and_logs(self):
        for line in ("# Calibrated: cant=0 pitch=0", "", "*** Booting Zephyr ***", "ORIENT,1,2"):
            self.assertIsNone(decode_text(line))
        for line in ("OFRAW", "OFRAW,2,0", "OFSHOT,1,1,2,3,4,5",
                     "OFSHOT,1,-1,2,3,4,5,6", "OFSHOT,1,1,2,3,4,5,4294967296",
                     "OFSHOT,1,1,18446744073709551616,3,4,5,6",
                     "OFSHOT,1,1,2,nan,4,5,6", "OFSHOT,1,1,2,1_000,4,5,6",
                     "OFSHOT,1,1,2,2147483648,4,5,6", "OFSHOT," + "1" * 600):
            with self.subTest(line=line), self.assertRaises(ValueError):
                decode_text(line)

    def test_device_counters(self):
        self.assertEqual(decode_text(
            "# OFSTATS,1,queue_drops=2,missed_ticks=3,sample_errors=4,ble_errors=4294967295"
        ), DeviceStats(2, 3, 4, 2**32 - 1))
        self.assertEqual(decode_text(
            "# OFSTATS,1,ble_errors=0,sample_errors=4,missed_ticks=3,queue_drops=2"
        ), DeviceStats(2, 3, 4, 0))
        for line in (
            "# OFSTATS,1,sample_errors=0",
            "# OFSTATS,2,queue_drops=0,missed_ticks=0,sample_errors=0,ble_errors=0",
            "# OFSTATS,1,queue_drops=0,queue_drops=0,sample_errors=0,ble_errors=0",
            "# OFSTATS,1,queue_drops=0,missed_ticks=0,sample_errors=0,unknown=0",
            "# OFSTATS,1,queue_drops=0,missed_ticks=0,sample_errors=0,ble_errors=-1",
            "# OFSTATS,1,queue_drops=0,missed_ticks=0,sample_errors=0,ble_errors=4294967296",
            "# OFSTATS,1,queue_drops=0,missed_ticks=0,sample_errors=0,ble_errors",
        ):
            with self.subTest(line=line), self.assertRaises(ValueError):
                decode_text(line)

    def test_decimation_and_counter_wrap(self):
        self.assertEqual(missing_notifications(0, 8), 0)
        self.assertEqual(missing_notifications(0, 24), 2)
        self.assertEqual(missing_notifications(65528, 0), 0)
        self.assertEqual(missing_notifications(65520, 8), 2)
        self.assertEqual(missing_notifications(2**32 - 8, 0, bits=32), 0)
        self.assertIsNone(missing_notifications(16, 16))
        self.assertIsNone(missing_notifications(1000, 0))
        self.assertIsNone(missing_notifications(0, 7))
        self.assertIsNone(missing_notifications(0, 32768))
        with self.assertRaises(ValueError):
            missing_notifications(0, 8, stride=0)
        with self.assertRaises(ValueError):
            missing_notifications(-1, 8)


if __name__ == "__main__":
    unittest.main()
