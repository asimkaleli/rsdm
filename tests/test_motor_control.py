import importlib
import sys
import threading
import time
import types
import unittest


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
        done = threading.Event()
        self.worker.step.connect(steps.append)

        def on_finished(move_id, completed):
            finished.append((move_id, completed))
            if len(finished) == 2:
                done.set()

        self.worker.moveFinished.connect(on_finished)
        forward_id = self.worker.submit_move(5)
        reverse_id = self.worker.submit_move(-3)

        self.assertTrue(done.wait(2))
        self.assertEqual(steps, [1] * 5 + [-1] * 3)
        self.assertEqual(finished, [(forward_id, True), (reverse_id, True)])
        self.assertFalse(self.worker.is_busy())

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
        self.assertLess(len(steps), 500)

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
        self.assertEqual(worker._dir_line.values[-1], 0)


if __name__ == "__main__":
    unittest.main()
