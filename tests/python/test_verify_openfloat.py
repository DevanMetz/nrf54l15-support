import argparse
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pc-app"))
from verify_openfloat import analyze_lines, main, positive_seconds


def sample(sequence, uptime_us, quaternion="1000000,0,0,0"):
    return f"OFRAW,1,{sequence},{uptime_us},2404,0,0,1000,0,0,0,0,0,0,{quaternion},0"


class CaptureTests(unittest.TestCase):
    def test_rate_gaps_and_comments(self):
        report = analyze_lines(iter([
            "# Booting", sample(0, 100000), sample(8, 119232),
            sample(24, 157696), "OFSHOT,1,1,150000,13000,0,0,1",
        ]))
        self.assertEqual(report["live_frames"], 3)
        self.assertEqual(report["missing_notifications"], 1)
        self.assertEqual(report["ignored_lines"], 1)
        self.assertEqual(report["shot_events"], 1)
        self.assertEqual(report["dt_us_mean"], 2404)
        self.assertAlmostEqual(report["received_rate_hz"], 2e6 / 57696)

    def test_reboot_and_corruption(self):
        report = analyze_lines([
            sample(80, 200000), sample(0, 100000), sample(8, 119232),
            sample(8, 119232), "OFRAW,1,bad", sample(16, 138464, "0,0,0,0"),
        ])
        self.assertEqual(report["sequence_discontinuities"], 2)
        self.assertEqual(report["malformed_frames"], 1)
        self.assertEqual(report["nonunit_quaternions"], 1)
        self.assertEqual(report["missing_notifications"], 0)
        self.assertAlmostEqual(report["received_rate_hz"], 1e6 / 19232)

    def test_empty(self):
        report = analyze_lines([])
        self.assertEqual(report["live_frames"], 0)
        self.assertIsNone(report["received_rate_hz"])
        self.assertIsNone(report["dt_us_mean"])
        self.assertIsNone(report["latest_device_counters"])

    def test_firmware_counters_and_reset(self):
        report = analyze_lines([
            "# OFSTATS,1,queue_drops=2,missed_ticks=3,sample_errors=4,ble_errors=5",
            sample(0, 100000),
            "# OFSTATS,1,queue_drops=0,missed_ticks=0,sample_errors=0,ble_errors=0",
            "# OFSTATS,1,sample_errors=0",  # Truncation counts as corruption.
        ])
        self.assertEqual(report["device_stats_frames"], 2)
        self.assertEqual(report["latest_device_counters"], {
            "queue_drops": 0, "missed_ticks": 0, "sample_errors": 0, "ble_errors": 0,
        })
        self.assertEqual(report["malformed_frames"], 1)
        self.assertEqual(report["ignored_lines"], 0)

    def test_invalid_cli_values(self):
        for text in ("nan", "inf", "0", "-1"):
            with self.subTest(text=text), self.assertRaises(argparse.ArgumentTypeError):
                positive_seconds(text)

    def test_offline_cli(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / "capture.log"
            capture.write_text(sample(0, 100000) + "\n" + sample(8, 119232) + "\n", encoding="ascii")
            output = io.StringIO()
            with redirect_stdout(output):
                status = main(["--input", str(capture), "--min-frames", "2"])
            self.assertEqual(status, 0)
            self.assertEqual(json.loads(output.getvalue())["live_frames"], 2)
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--input", str(capture)]), 1)


if __name__ == "__main__":
    unittest.main()
