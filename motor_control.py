"""Thread-safe, two-axis friendly stepper motor control.

Only the worker thread touches STEP/DIR GPIO. Calls made by the GUI thread
append commands to a protected queue, so direction and step count cannot be
separated by a race.
"""

import time
from collections import deque
from dataclasses import dataclass
from threading import Condition, Lock
from typing import Optional

import gpiod
from PySide2.QtCore import QObject, QThread, Signal, Slot, QEventLoop, QTimer


class _Chip:
    """Singleton holder for gpiochip0."""

    _chip = None

    @classmethod
    def get(cls) -> gpiod.Chip:
        if cls._chip is None:
            cls._chip = gpiod.Chip("gpiochip0")
        return cls._chip


def _req_out(pin: Optional[int], default_val: int):
    if pin is None:
        return None
    line = _Chip.get().get_line(pin)
    line.request(
        consumer="stepper",
        type=gpiod.LINE_REQ_DIR_OUT,
        default_val=1 if default_val else 0,
    )
    return line


_MICROSTEP_TABLE = {
    "FULL": (0, 0, 0),
    "HALF": (1, 0, 0),
    "QUARTER": (0, 1, 0),
    "EIGHTH": (1, 1, 0),
    "SIXTEENTH": (1, 1, 1),
}


class SharedPins:
    """Manage EN/RESET/SLEEP/MS pins shared by both motor drivers."""

    def __init__(self, en: int, reset: int, sleep: int, ms1: int, ms2: int, ms3: int):
        self._en = _req_out(en, 1)       # A4988: EN=1 disabled
        self._reset = _req_out(reset, 1)
        self._sleep = _req_out(sleep, 1)
        self._ms1 = _req_out(ms1, 0)
        self._ms2 = _req_out(ms2, 0)
        self._ms3 = _req_out(ms3, 0)

    def set_enable(self, enabled: bool):
        self._en.set_value(0 if enabled else 1)

    def set_sleep(self, awake: bool):
        self._sleep.set_value(1 if awake else 0)

    def pulse_reset(self):
        self._reset.set_value(0)
        time.sleep(0.002)
        self._reset.set_value(1)
        time.sleep(0.002)

    def set_microstep(self, mode: str):
        ms = _MICROSTEP_TABLE.get(mode.strip().upper())
        if ms is None:
            return
        self._ms1.set_value(ms[0])
        self._ms2.set_value(ms[1])
        self._ms3.set_value(ms[2])


@dataclass(frozen=True)
class MotorPins:
    step: int
    dir: int
    dir_inverted: bool = False
    en: Optional[int] = None
    sleep: Optional[int] = None
    reset: Optional[int] = None
    ms1: Optional[int] = None
    ms2: Optional[int] = None
    ms3: Optional[int] = None


class StepperWorker(QObject):
    finished = Signal()
    progress = Signal(int)
    error = Signal(str)
    step = Signal(int)
    busyChanged = Signal(bool)
    moveStarted = Signal(int, int)   # move_id, signed_steps
    moveFinished = Signal(int, bool)  # move_id, completed; False means cancelled
    timingReport = Signal(int, float, float)  # pulse intervals, mean period ms, max jitter ms
    pulsePhaseReport = Signal(int, float, float, float, float)
    # samples, min HIGH ms, max HIGH ms, min LOW ms, max LOW ms

    def __init__(self, pins: MotorPins, shared: Optional[SharedPins] = None, parent=None):
        super().__init__(parent)
        self.pins = pins
        self.shared = shared
        self._step_line = _req_out(pins.step, 0)
        self._dir_line = _req_out(pins.dir, 0)

        self._edge_s = 0.005
        self._forward = True
        # [move_id, remaining_steps]
        self._active_move = None
        self._continuous_requested_direction = 0
        self._continuous_direction = 0
        self._moves = deque()     # (move_id, signed_steps)
        self._commands = deque()
        self._condition = Condition()
        self._state_lock = Lock()
        self._run = True
        self._total = 0
        self._position_steps = 0
        self._busy = False
        self._next_move_id = 1
        self._timing_count = 0
        self._timing_period_sum_s = 0.0
        self._timing_max_jitter_s = 0.0
        self._timing_last_start = None
        self._timing_last_expected_period_s = None
        self._phase_count = 0
        self._phase_high_min_s = None
        self._phase_high_max_s = 0.0
        self._phase_low_min_s = None
        self._phase_low_max_s = 0.0
        self._pending_step_delta = 0
        self._last_step_emit_s = 0.0
        self._step_emit_batch = 8
        self._step_emit_interval_s = 0.020

        self._apply_dir()
        self._wake(True)

    def _enable(self, on: bool):
        if self.shared:
            self.shared.set_enable(on)

    def _wake(self, awake: bool):
        if self.shared:
            self.shared.set_sleep(awake)

    def _apply_dir(self):
        pin_forward = self._forward != self.pins.dir_inverted
        self._dir_line.set_value(1 if pin_forward else 0)
        time.sleep(0.002)

    @staticmethod
    def _sleep_until(deadline: float):
        """Sleep only for the time left until an absolute monotonic deadline.

        Two consecutive ``sleep(edge_s)`` calls accumulate the scheduler
        overshoot of both calls on every motor step.  Using absolute deadlines
        lets the low phase absorb normal Python/Linux wake-up latency instead
        of permanently adding it to the requested step period.
        """
        remaining = deadline - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)

    def _pulse_once(self, edge_s: Optional[float] = None):
        pulse_edge_s = self._edge_s if edge_s is None else max(0.0005, float(edge_s))
        pulse_start = time.monotonic()
        self._record_pulse_timing(pulse_start, 2.0 * pulse_edge_s)
        self._step_line.set_value(1)
        high_started = time.monotonic()
        self._sleep_until(high_started + pulse_edge_s)
        self._step_line.set_value(0)
        low_started = time.monotonic()
        delta = 1 if self._forward else -1
        self._total += 1
        with self._state_lock:
            self._position_steps += delta
        self._pending_step_delta += delta
        self._emit_step_update()
        # LOW has its own deadline. Scheduler overrun during HIGH must never
        # be "recovered" by shortening the driver's minimum LOW pulse width.
        self._sleep_until(low_started + pulse_edge_s)
        low_finished = time.monotonic()
        self._record_pulse_phases(
            high_s=low_started - high_started,
            low_s=low_finished - low_started,
        )

    def _emit_step_update(self, force: bool = False):
        """Publish exact position deltas without flooding the Qt event queue."""
        if self._pending_step_delta == 0:
            return
        now = time.monotonic()
        due = now - self._last_step_emit_s >= self._step_emit_interval_s
        if not (force or due or abs(self._pending_step_delta) >= self._step_emit_batch):
            return
        delta = self._pending_step_delta
        self._pending_step_delta = 0
        self._last_step_emit_s = now
        self.progress.emit(self._total)
        self.step.emit(delta)

    def _reset_timing(self):
        self._timing_count = 0
        self._timing_period_sum_s = 0.0
        self._timing_max_jitter_s = 0.0
        self._timing_last_start = None
        self._timing_last_expected_period_s = None
        self._phase_count = 0
        self._phase_high_min_s = None
        self._phase_high_max_s = 0.0
        self._phase_low_min_s = None
        self._phase_low_max_s = 0.0

    def _record_pulse_phases(self, high_s: float, low_s: float):
        high_s = max(0.0, float(high_s))
        low_s = max(0.0, float(low_s))
        self._phase_count += 1
        self._phase_high_min_s = (
            high_s if self._phase_high_min_s is None
            else min(self._phase_high_min_s, high_s)
        )
        self._phase_high_max_s = max(self._phase_high_max_s, high_s)
        self._phase_low_min_s = (
            low_s if self._phase_low_min_s is None
            else min(self._phase_low_min_s, low_s)
        )
        self._phase_low_max_s = max(self._phase_low_max_s, low_s)

    def _record_pulse_timing(self, pulse_start: float, expected_period_s: float):
        if self._timing_last_start is not None:
            actual_period_s = pulse_start - self._timing_last_start
            expected_s = self._timing_last_expected_period_s
            jitter_s = actual_period_s - expected_s
            self._timing_count += 1
            self._timing_period_sum_s += actual_period_s
            self._timing_max_jitter_s = max(
                self._timing_max_jitter_s, abs(jitter_s)
            )
        self._timing_last_start = pulse_start
        self._timing_last_expected_period_s = expected_period_s

    def _emit_timing_report(self):
        if self._timing_count > 0:
            mean_period_ms = 1000.0 * self._timing_period_sum_s / self._timing_count
            max_jitter_ms = 1000.0 * self._timing_max_jitter_s
            self.timingReport.emit(
                self._timing_count, mean_period_ms, max_jitter_ms
            )
        if self._phase_count > 0:
            self.pulsePhaseReport.emit(
                self._phase_count,
                1000.0 * self._phase_high_min_s,
                1000.0 * self._phase_high_max_s,
                1000.0 * self._phase_low_min_s,
                1000.0 * self._phase_low_max_s,
            )
        self._reset_timing()

    def _movement_edge_s(self) -> float:
        """Use the selected fixed speed for manual and planned movements."""
        return self._edge_s

    def _set_busy(self, busy: bool):
        busy = bool(busy)
        with self._state_lock:
            changed = busy != self._busy
            self._busy = busy
        if changed:
            self.busyChanged.emit(busy)

    def _queue_command(self, name: str, *args):
        """Queue without touching GPIO from the caller's thread."""
        with self._condition:
            self._commands.append((name, args))
            if name in ("move", "continuous_start"):
                self._set_busy(True)
            self._condition.notify()

    def submit_move(self, signed_steps: int) -> int:
        """Atomically queue direction and distance and return a movement id."""
        signed_steps = int(signed_steps)
        if signed_steps == 0:
            return 0
        with self._condition:
            move_id = self._next_move_id
            self._next_move_id += 1
            self._commands.append(("move", (move_id, signed_steps)))
            self._set_busy(True)
            self._condition.notify()
        return move_id

    def start_continuous(self, direction: int):
        """Start a worker-timed manual move; direction must be -1 or +1."""
        direction = 1 if int(direction) > 0 else -1
        self._queue_command("continuous_start", direction)

    def stop_continuous(self):
        """Stop after the currently executing pulse has completed."""
        self._queue_command("continuous_stop")

    def _stop_continuous_in_worker(self):
        self._continuous_requested_direction = 0
        if not self._continuous_direction:
            return
        self._continuous_direction = 0
        self._emit_step_update(force=True)
        self._emit_timing_report()

    def _start_requested_continuous_in_worker(self):
        direction = self._continuous_requested_direction
        if not direction or self._continuous_direction:
            return
        self._continuous_direction = direction
        self._reset_timing()
        self._wake(True)
        self._forward = direction > 0
        self._apply_dir()

    def _cancel_moves_in_worker(self):
        cancelled = []
        if self._active_move is not None:
            cancelled.append(self._active_move[0])
            self._active_move = None
            self._emit_step_update(force=True)
            self._emit_timing_report()
        while self._moves:
            cancelled.append(self._moves.popleft()[0])

        # A cancel may arrive just before another producer queues a move.
        # Commands already ahead of this cancel are handled in order by
        # _handle_commands; commands queued afterwards remain valid.
        for move_id in cancelled:
            self.moveFinished.emit(move_id, False)

    def _handle_commands(self):
        with self._condition:
            commands = list(self._commands)
            self._commands.clear()

        for name, args in commands:
            if name == "speed":
                self._edge_s = max(0.0005, float(args[0]) / 1000.0)
            elif name == "microstep":
                if self.shared:
                    self.shared.set_microstep(args[0])
                else:
                    self.error.emit("SharedPins yok: microstep ortak pinleri yonetilemiyor.")
            elif name == "move":
                move_id, signed_steps = int(args[0]), int(args[1])
                self._moves.append((move_id, signed_steps))
            elif name == "continuous_start":
                direction = 1 if int(args[0]) > 0 else -1
                if self._continuous_direction and self._continuous_direction != direction:
                    self._stop_continuous_in_worker()
                self._continuous_requested_direction = direction
            elif name == "continuous_stop":
                self._stop_continuous_in_worker()
            elif name == "cancel_moves":
                self._stop_continuous_in_worker()
                self._cancel_moves_in_worker()
            elif name == "emergency_stop":
                self._stop_continuous_in_worker()
                self._cancel_moves_in_worker()
            elif name == "reset":
                if self.shared:
                    self.shared.pulse_reset()
                else:
                    self.error.emit("SharedPins yok: reset ortak pini yok.")
            elif name == "sleep":
                if self.shared:
                    self.shared.set_sleep(not bool(args[0]))
                if args[0]:
                    self._stop_continuous_in_worker()
                    self._cancel_moves_in_worker()
            elif name == "enable":
                self._enable(bool(args[0]))
            elif name == "shutdown":
                self._stop_continuous_in_worker()
                self._cancel_moves_in_worker()
                self._emit_step_update(force=True)
                self._run = False

    @Slot(float)
    def set_speed_ms(self, ms_per_edge: float):
        self._queue_command("speed", float(ms_per_edge))

    @Slot(str)
    def set_microstep(self, mode: str):
        self._queue_command("microstep", str(mode))

    @Slot()
    def cancel_moves(self):
        self._queue_command("cancel_moves")

    @Slot()
    def emergency_stop(self):
        self._queue_command("emergency_stop")

    @Slot()
    def reset_pulse(self):
        self._queue_command("reset")

    @Slot(bool)
    def sleep(self, do_sleep: bool):
        self._queue_command("sleep", bool(do_sleep))

    @Slot(bool)
    def enable(self, on: bool):
        self._queue_command("enable", bool(on))

    @Slot()
    def run(self):
        try:
            while self._run:
                self._handle_commands()
                if not self._run:
                    break

                if self._active_move is None and self._moves:
                    move_id, signed_steps = self._moves.popleft()
                    total_steps = abs(signed_steps)
                    self._active_move = [move_id, total_steps]
                    self._reset_timing()
                    self._wake(True)
                    self._forward = signed_steps > 0
                    self._apply_dir()
                    self.moveStarted.emit(move_id, signed_steps)

                if (self._active_move is None and not self._moves
                        and not self._continuous_direction):
                    self._start_requested_continuous_in_worker()

                if self._active_move is not None:
                    self._pulse_once(edge_s=self._movement_edge_s())
                    self._active_move[1] -= 1
                    if self._active_move[1] <= 0:
                        move_id = self._active_move[0]
                        self._active_move = None
                        self._emit_step_update(force=True)
                        self._emit_timing_report()
                        self.moveFinished.emit(move_id, True)
                elif self._continuous_direction:
                    self._pulse_once(edge_s=self._movement_edge_s())
                else:
                    with self._condition:
                        if not self._commands:
                            self._condition.wait(timeout=0.05)

                # Use the same lock order as producers to avoid a busy-state race.
                with self._condition:
                    pending_commands = bool(self._commands)
                    self._set_busy(bool(
                        self._active_move is not None or self._moves
                        or self._continuous_direction
                        or self._continuous_requested_direction
                        or pending_commands
                    ))
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            # Never leave the driver STEP input asserted after an exception or
            # shutdown. A later restart must begin from a known LOW idle state.
            try:
                self._step_line.set_value(0)
            except Exception:
                pass
            self._set_busy(False)
            self.finished.emit()

    def shutdown_now(self):
        self._queue_command("shutdown")

    def is_busy(self) -> bool:
        with self._state_lock:
            return self._busy

    def position_steps(self) -> int:
        """Return the exact signed pulse position maintained by the worker."""
        with self._state_lock:
            return int(self._position_steps)


class MotorController(QObject):
    """Public controller; all GPIO work is delegated to StepperWorker."""

    progress = Signal(int)
    error = Signal(str)
    busyChanged = Signal(bool)
    moveStarted = Signal(int, int)
    moveFinished = Signal(int, bool)
    timingReport = Signal(int, float, float)
    pulsePhaseReport = Signal(int, float, float, float, float)

    def __init__(self, pins: MotorPins, shared: Optional[SharedPins] = None, parent=None):
        super().__init__(parent)
        self.worker = StepperWorker(pins, shared=shared)
        self.th = QThread()
        self.worker.moveToThread(self.th)

        self.worker.progress.connect(self.progress)
        self.worker.error.connect(self.error)
        self.worker.busyChanged.connect(self.busyChanged)
        self.worker.moveStarted.connect(self.moveStarted)
        self.worker.moveFinished.connect(self.moveFinished)
        self.worker.timingReport.connect(self.timingReport)
        self.worker.pulsePhaseReport.connect(self.pulsePhaseReport)
        self.th.started.connect(self.worker.run)
        self.th.start()

    def set_speed_ms(self, ms: float):
        self.worker.set_speed_ms(ms)

    def set_microstep(self, mode: str):
        self.worker.set_microstep(mode)

    def cancel_moves(self):
        self.worker.cancel_moves()

    def emergency_stop(self):
        self.worker.emergency_stop()

    def move_signed_steps(self, signed_steps: int) -> int:
        """Preferred atomic movement API."""
        return self.worker.submit_move(int(signed_steps))

    def start_continuous(self, direction: int):
        self.worker.start_continuous(direction)

    def stop_continuous(self):
        self.worker.stop_continuous()

    def reset_pulse(self):
        self.worker.reset_pulse()

    def sleep(self, do_sleep: bool):
        self.worker.sleep(do_sleep)

    def enable(self, on: bool):
        self.worker.enable(on)

    def set_step_callback(self, callback):
        self.worker.step.connect(callback)

    def is_busy(self) -> bool:
        return self.worker.is_busy()

    def position_steps(self) -> int:
        return self.worker.position_steps()

    def wait_until_idle(self, timeout_ms: int = 8000) -> bool:
        if not self.is_busy():
            return True

        loop = QEventLoop()
        timed_out = {"value": False}

        def on_busy_changed(busy: bool):
            if not busy and loop.isRunning():
                loop.quit()

        self.busyChanged.connect(on_busy_changed)
        timer = QTimer()
        timer.setSingleShot(True)
        timer.timeout.connect(
            lambda: (timed_out.update(value=True), loop.quit())
        )
        timer.start(max(1, int(timeout_ms)))

        if not self.is_busy():
            try:
                self.busyChanged.disconnect(on_busy_changed)
            except Exception:
                pass
            timer.stop()
            return True

        loop.exec_()
        try:
            self.busyChanged.disconnect(on_busy_changed)
        except Exception:
            pass
        timer.stop()
        return not timed_out["value"] and not self.is_busy()

    def shutdown(self):
        self.worker.shutdown_now()
        self.th.quit()
        self.th.wait()
