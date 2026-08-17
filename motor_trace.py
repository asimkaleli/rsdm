"""Non-blocking CSV trace writer for motor/backlash diagnostics."""

import csv
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue
from threading import Thread


TRACE_FIELDS = (
    "timestamp", "event", "axis", "row", "target_kind", "move_id",
    "requested_steps", "batch_delta", "direction", "total_pulses",
    "raw_position_steps", "logical_position_steps", "gap_steps",
    "backlash_steps", "target_position_steps", "monotonic_ns", "message",
)


def _timestamp_ms() -> str:
    return datetime.now().isoformat(sep=" ", timespec="milliseconds")


class MotorTraceLogger:
    """Write trace records on a background thread, away from motor timing."""

    _STOP = object()

    def __init__(self, path=None):
        default_path = Path.cwd() / (
            f"motor_trace_{datetime.now():%Y%m%d_%H%M%S}.csv"
        )
        self.path = Path(path or default_path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Open synchronously so startup can report an unwritable destination
        # instead of silently failing inside the background thread.
        self._handle = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=TRACE_FIELDS)
        self._writer.writeheader()
        self._handle.flush()
        self._queue = Queue()
        self._closed = False
        self._thread = Thread(
            target=self._writer_loop,
            name="motor-trace-writer",
            daemon=True,
        )
        self._thread.start()

    def record(self, axis: str, trace_record=None, **values):
        if self._closed:
            return
        row = {field: "" for field in TRACE_FIELDS}
        row["timestamp"] = _timestamp_ms()
        row["axis"] = str(axis)
        if trace_record:
            for key, value in dict(trace_record).items():
                if key in row:
                    row[key] = "" if value is None else value
        for key, value in values.items():
            if key in row:
                row[key] = "" if value is None else value
        self._queue.put(row)

    def _writer_loop(self):
        try:
            while True:
                try:
                    item = self._queue.get(timeout=0.5)
                except Empty:
                    self._handle.flush()
                    continue
                if item is self._STOP:
                    self._handle.flush()
                    return
                self._writer.writerow(item)
        finally:
            self._handle.close()

    def close(self, timeout_s: float = 5.0):
        if self._closed:
            return
        self._closed = True
        self._queue.put(self._STOP)
        self._thread.join(timeout=max(0.0, float(timeout_s)))
