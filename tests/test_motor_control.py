import importlib
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch


class _BoundSignal:
    def __init__(self):
        self.callbacks = []

    def connect(self, callback):
        self.callbacks.append(callback)

    def disconnect(self, callback):
        self.callbacks.remove(callback)

    def emit(self, *args):
        for callback in list(self.callbacks):
            callback(*args)


class _Signal:
    def __set_name__(self, owner, name):
        self.name = "_signal_" + name

    def __get__(self, instance, owner):
        if instance is None:
            return self
        signal = instance.__dict__.get(self.name)
        if signal is None:
            signal = _BoundSignal()
            instance.__dict__[self.name] = signal
        return signal


class _QObject:
    def __init__(self, parent=None):
        self.parent = parent

    def moveToThread(self, thread):
        self.thread = thread


class _UnusedQtClass:
    def __init__(self, *args, **kwargs):
        pass


class _FakeLine:
    def __init__(self):
        self.values = []

    def request(self, **kwargs):
        self.request_args = kwargs

    def set_value(self, value):
        self.values.append(value)


class _FakeChip:
    def __init__(self, name):
        self.name = name
        self.lines = {}

    def get_line(self, pin):
        return self.lines.setdefault(pin, _FakeLine())


def _slot(*types_):
    def decorate(function):
        return function
    return decorate


def _install_fakes():
    qtcore = types.ModuleType("PySide2.QtCore")
    qtcore.QObject = _QObject
    qtcore.QThread = _UnusedQtClass
    qtcore.Signal = lambda *args: _Signal()
    qtcore.Slot = _slot
    qtcore.QEventLoop = _UnusedQtClass
    qtcore.QTimer = _UnusedQtClass
    pyside = types.ModuleType("PySide2")
    pyside.QtCore = qtcore

    gpiod = types.ModuleType("gpiod")
    gpiod.Chip = _FakeChip
    gpiod.LINE_REQ_DIR_OUT = 1

    sys.modules["PySide2"] = pyside
    sys.modules["PySide2.QtCore"] = qtcore
    sys.modules["gpiod"] = gpiod


_install_fakes()
motor_control = importlib.import_module("motor_control")


class StepperWorkerTests(unittest.TestCase):
    def setUp(self):
        motor_control._Chip._chip = None
        self.worker = motor_control.StepperWorker(
            motor_control.MotorPins(step=12, dir=5)
        )
        self.worker.set_speed_ms(0.5)
        self.worker.set_motion_profile(1000, 100000)
        self.thread = threading.Thread(target=self.worker.run, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.worker.shutdown_now()
        self.thread.join(timeout=2)
        self.assertFalse(self.thread.is_alive())

    def test_signed_moves_are_atomic_and_finish(self):
        steps = []
        finished = []
        timing_reports = []
        done = threading.Event()
        self.worker.step.connect(steps.append)
        self.worker.timingReport.connect(
            lambda count, mean_ms, jitter_ms: timing_reports.append(
                (count, mean_ms, jitter_ms)
            )
        )

        def on_finished(move_id, completed):
            finished.append((move_id, completed))
            if len(finished) == 2:
                done.set()

        self.worker.moveFinished.connect(on_finished)
        forward_id = self.worker.submit_move(5)
        reverse_id = self.worker.submit_move(-3)

        self.assertTrue(done.wait(2))
        self.assertEqual(sum(steps), 2)
        self.assertEqual(sum(step for step in steps if step > 0), 5)
        self.assertEqual(sum(step for step in steps if step < 0), -3)
        self.assertEqual(finished, [(forward_id, True), (reverse_id, True)])
        self.assertEqual([report[0] for report in timing_reports], [4, 2])
        self.assertTrue(all(report[1] > 0 for report in timing_reports))
        self.assertEqual(self.worker.position_steps(), 2)
        self.assertFalse(self.worker.is_busy())

    def test_single_step_move_outputs_exactly_one_complete_pulse(self):
        steps = []
        finished = []
        done = threading.Event()
        self.worker.step.connect(steps.append)

        def on_finished(move_id, completed):
            finished.append((move_id, completed))
            done.set()

        self.worker.moveFinished.connect(on_finished)
        move_id = self.worker.submit_move(1)

        self.assertTrue(done.wait(1))
        self.assertEqual(finished, [(move_id, True)])
        self.assertEqual(steps, [1])
        self.assertEqual(self.worker.position_steps(), 1)
        self.assertEqual(self.worker._step_line.values[-2:], [1, 0])
        self.assertFalse(self.worker.is_busy())

    def test_controller_routes_signed_move_to_atomic_worker_api(self):
        class WorkerStub:
            def __init__(self):
                self.moves = []
                self.continuous = []

            def submit_move(self, signed_steps):
                self.moves.append(signed_steps)
                return 42

            def start_continuous(self, direction):
                self.continuous.append(("start", direction))

            def stop_continuous(self):
                self.continuous.append(("stop",))

        controller = object.__new__(motor_control.MotorController)
        controller.worker = WorkerStub()

        move_id = controller.move_signed_steps(-1)

        self.assertEqual(move_id, 42)
        self.assertEqual(controller.worker.moves, [-1])

        controller.start_continuous(1)
        controller.stop_continuous()
        self.assertEqual(
            controller.worker.continuous,
            [("start", 1), ("stop",)],
        )

    def test_continuous_move_runs_until_stop_without_partial_pulse(self):
        self.worker.start_continuous(-1)
        deadline = time.monotonic() + 1.0
        while self.worker.position_steps() > -8 and time.monotonic() < deadline:
            time.sleep(0.001)

        self.assertLessEqual(self.worker.position_steps(), -8)
        self.worker.stop_continuous()
        deadline = time.monotonic() + 1.0
        while self.worker.is_busy() and time.monotonic() < deadline:
            time.sleep(0.001)

        self.assertFalse(self.worker.is_busy())
        stopped_position = self.worker.position_steps()
        time.sleep(0.02)
        self.assertEqual(self.worker.position_steps(), stopped_position)
        self.assertEqual(self.worker._step_line.values[-1], 0)

    def test_continuous_direction_waits_for_queued_atomic_move(self):
        atomic_finished_at = []
        done = threading.Event()

        def on_finished(_move_id, completed):
            if completed:
                atomic_finished_at.append(self.worker.position_steps())
                done.set()

        self.worker.moveFinished.connect(on_finished)
        self.worker.submit_move(-5)
        self.worker.start_continuous(1)

        self.assertTrue(done.wait(1))
        self.assertEqual(atomic_finished_at, [-5])

        deadline = time.monotonic() + 1.0
        while self.worker.position_steps() <= -2 and time.monotonic() < deadline:
            time.sleep(0.001)
        self.worker.stop_continuous()
        deadline = time.monotonic() + 1.0
        while self.worker.is_busy() and time.monotonic() < deadline:
            time.sleep(0.001)

        self.assertFalse(self.worker.is_busy())
        self.assertGreater(self.worker.position_steps(), -5)

    def test_cancel_reports_incomplete_and_stops_early(self):
        steps = []
        result = []
        started = threading.Event()
        done = threading.Event()
        self.worker.step.connect(steps.append)
        self.worker.moveStarted.connect(lambda *_: started.set())

        def on_finished(move_id, completed):
            result.append((move_id, completed))
            done.set()

        self.worker.moveFinished.connect(on_finished)
        move_id = self.worker.submit_move(500)
        self.assertTrue(self.worker.is_busy())
        self.assertTrue(started.wait(1))
        time.sleep(0.01)
        self.worker.cancel_moves()

        self.assertTrue(done.wait(1))
        self.assertEqual(result, [(move_id, False)])
        self.assertLess(sum(abs(step) for step in steps), 500)

    def test_emergency_stop_cancels_active_and_queued_moves(self):
        results = []
        started = threading.Event()
        done = threading.Event()
        self.worker.moveStarted.connect(lambda *_: started.set())

        def on_finished(move_id, completed):
            results.append((move_id, completed))
            if len(results) == 2:
                done.set()

        self.worker.moveFinished.connect(on_finished)
        first_id = self.worker.submit_move(500)
        second_id = self.worker.submit_move(-50)
        self.assertTrue(started.wait(1))
        self.worker.emergency_stop()

        self.assertTrue(done.wait(1))
        self.assertEqual(
            sorted(results), sorted([(first_id, False), (second_id, False)])
        )

    def test_motion_profile_accelerates_and_decelerates(self):
        self.worker._edge_s = 0.0005  # 1000 step/s target
        self.worker._start_sps = 50
        self.worker._acceleration_sps2 = 400
        first = self.worker._profile_edge_s(completed=0, remaining=2000)
        middle = self.worker._profile_edge_s(completed=1000, remaining=1000)
        last = self.worker._profile_edge_s(completed=1999, remaining=1)

        self.assertGreater(first, middle)
        self.assertAlmostEqual(first, last)

    def test_inverted_dir_pin_preserves_logical_step_sign(self):
        worker = motor_control.StepperWorker(
            motor_control.MotorPins(step=13, dir=6, dir_inverted=True)
        )
        logical_steps = []
        worker.step.connect(logical_steps.append)
        worker._forward = True
        worker._apply_dir()
        worker._pulse_once(edge_s=0.0005)

        self.assertEqual(logical_steps, [1])
        self.assertEqual(worker.position_steps(), 1)
        self.assertEqual(worker._dir_line.values[-1], 0)

    def test_pulse_guarantees_full_high_and_low_waits(self):
        worker = motor_control.StepperWorker(
            motor_control.MotorPins(step=14, dir=7)
        )
        clock = {"now": 10.0}
        sleep_deadlines = []
        phase_reports = []
        worker.pulsePhaseReport.connect(
            lambda *values: phase_reports.append(values)
        )

        def monotonic():
            return clock["now"]

        def sleep(seconds):
            sleep_deadlines.append(seconds)
            # Simulate 0.2 ms operating-system wake-up latency. LOW must still
            # get a complete independent wait instead of being shortened.
            clock["now"] += seconds + 0.0002

        with patch.object(motor_control.time, "monotonic", side_effect=monotonic), \
             patch.object(motor_control.time, "sleep", side_effect=sleep):
            worker._pulse_once(edge_s=0.001)
            worker._emit_timing_report()

        self.assertAlmostEqual(sleep_deadlines[0], 0.001, places=6)
        self.assertAlmostEqual(sleep_deadlines[1], 0.001, places=6)
        self.assertAlmostEqual(clock["now"], 10.0024, places=6)
        self.assertEqual(len(phase_reports), 1)
        samples, min_high, max_high, min_low, max_low = phase_reports[0]
        self.assertEqual(samples, 1)
        self.assertAlmostEqual(min_high, 1.2, places=6)
        self.assertAlmostEqual(max_high, 1.2, places=6)
        self.assertAlmostEqual(min_low, 1.2, places=6)
        self.assertAlmostEqual(max_low, 1.2, places=6)

    def test_worker_forces_step_low_after_exception(self):
        worker = motor_control.StepperWorker(
            motor_control.MotorPins(step=16, dir=9)
        )
        errors = []
        worker.error.connect(errors.append)

        def fail_with_step_asserted():
            worker._step_line.set_value(1)
            raise RuntimeError("forced test failure")

        worker._handle_commands = fail_with_step_asserted
        worker.run()

        self.assertEqual(errors, ["forced test failure"])
        self.assertEqual(worker._step_line.values[-2:], [1, 0])
        self.assertFalse(worker.is_busy())

    def test_step_updates_are_batched_without_losing_position(self):
        worker = motor_control.StepperWorker(
            motor_control.MotorPins(step=15, dir=8)
        )
        updates = []
        worker.step.connect(updates.append)
        worker._forward = True
        clock = {"now": 10.0}

        def monotonic():
            return clock["now"]

        def sleep(seconds):
            clock["now"] += seconds

        worker._last_step_emit_s = clock["now"]
        with patch.object(motor_control.time, "monotonic", side_effect=monotonic), \
             patch.object(motor_control.time, "sleep", side_effect=sleep):
            for _ in range(16):
                worker._pulse_once(edge_s=0.0005)
            worker._emit_step_update(force=True)

        self.assertEqual(updates, [8, 8])
        self.assertEqual(sum(updates), 16)


if __name__ == "__main__":
    unittest.main()
