import csv
import tempfile
import unittest
from pathlib import Path

from motor_trace import MotorTraceLogger, TRACE_FIELDS


class MotorTraceLoggerTests(unittest.TestCase):
    def test_background_writer_preserves_records_and_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.csv"
            logger = MotorTraceLogger(path)
            logger.record(
                "x",
                {"event": "pulse_batch", "batch_delta": 8,
                 "total_pulses": 8, "monotonic_ns": 123},
            )
            logger.record(
                "x", event="point_saved", row=4,
                target_position_steps=250,
            )
            logger.close()

            with path.open(newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                rows = list(reader)

        self.assertEqual(tuple(reader.fieldnames), TRACE_FIELDS)
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["event"], "pulse_batch")
        self.assertEqual(rows[0]["batch_delta"], "8")
        self.assertEqual(rows[1]["event"], "point_saved")
        self.assertEqual(rows[1]["row"], "4")


if __name__ == "__main__":
    unittest.main()
