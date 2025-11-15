# motor_control.py
# Step motor sürücüsü (A4988/DRV8825/TMC..) için libgpiod + QThread tabanlı kontrol.
# - İki katman: SharedPins (EN/RESET/SLEEP/MS* ortak hatlar) + MotorController (tek motor STEP/DIR).
# - Jog (basılı tut) ve "N adım" hareketi.
# - Hız ayarı: ms/kenar (HIGH ve LOW ayrı ayrı). Küçük ms ⇒ daha hızlı.
# - Pi 5 uyumlu, pigpio gerektirmez.

import time
from dataclasses import dataclass
from typing import Optional

import gpiod
from PySide2.QtCore import QObject, QThread, Signal, Slot, QEventLoop, QTimer


# -------------------- Yardımcılar --------------------

class _Chip:
    """gpiochip0 için basit tekil (singleton) tutucu."""
    _chip = None

    @classmethod
    def get(cls) -> gpiod.Chip:
        if cls._chip is None:
            cls._chip = gpiod.Chip("gpiochip0")
        return cls._chip


def _req_out(pin: Optional[int], default_val: int):
    """Bir GPIO hattını çıkış olarak talep et. None ise None döner."""
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
    "FULL":      (0, 0, 0),
    "HALF":      (1, 0, 0),
    "QUARTER":   (0, 1, 0),
    "EIGHTH":    (1, 1, 0),
    "SIXTEENTH": (1, 1, 1),
}


# -------------------- Ortak Hatlar --------------------

class SharedPins:
    """
    EN / RESET / SLEEP / MS1 / MS2 / MS3 gibi ortak hatları tek yerden yönetir.
    İki (veya daha fazla) motor aynı SharedPins örneğini paylaşır.
    """
    def __init__(self, en: int, reset: int, sleep: int, ms1: int, ms2: int, ms3: int):
        self._en    = _req_out(en,    1)  # A4988: EN=1 devre dışı
        self._reset = _req_out(reset, 1)  # 1 = normal
        self._sleep = _req_out(sleep, 1)  # 1 = uyanık
        self._ms1   = _req_out(ms1,   0)
        self._ms2   = _req_out(ms2,   0)
        self._ms3   = _req_out(ms3,   0)

    def set_enable(self, enabled: bool):
        """True ⇒ EN=LOW (etkin), False ⇒ EN=HIGH (devre dışı)."""
        self._en.set_value(0 if enabled else 1)

    def set_sleep(self, awake: bool):
        """True ⇒ uyanık (SLEEP=1), False ⇒ uyku (SLEEP=0)."""
        self._sleep.set_value(1 if awake else 0)

    def pulse_reset(self):
        """Reset hattına kısa darbe."""
        self._reset.set_value(0); time.sleep(0.002)
        self._reset.set_value(1); time.sleep(0.002)

    def set_microstep(self, mode: str):
        """Mikroadım modunu ayarla (her iki/sistemdeki tüm sürücülere etki eder)."""
        ms = _MICROSTEP_TABLE.get(mode.strip().upper())
        if not ms:
            return
        self._ms1.set_value(ms[0])
        self._ms2.set_value(ms[1])
        self._ms3.set_value(ms[2])


# -------------------- Pin Tanımı --------------------

@dataclass(frozen=True)
class MotorPins:
    step: int  # ZORUNLU
    dir:  int  # ZORUNLU
    # Aşağıdakiler SharedPins ile ortak yönetildiği için burada None bırakabilirsiniz
    en:    Optional[int] = None
    sleep: Optional[int] = None
    reset: Optional[int] = None
    ms1:   Optional[int] = None
    ms2:   Optional[int] = None
    ms3:   Optional[int] = None


# -------------------- Worker (QThread içinde koşar) --------------------

class StepperWorker(QObject):
    finished     = Signal()
    progress     = Signal(int)   # toplam atılan adım
    error        = Signal(str)
    step         = Signal(int)   # her adımda +1 (ileri/sağa/yukarı), -1 (geriye/sola/aşağı)
    busyChanged  = Signal(bool)  # YENİ: meşguliyet durumu değiştiğinde

    def __init__(self, pins: MotorPins, shared: Optional[SharedPins] = None, parent=None):
        super().__init__(parent)
        self.pins   = pins
        self.shared = shared

        # Sadece STEP/DIR hatlarını talep et (ortak hatlar SharedPins tarafından tutulur)
        self._step_line = _req_out(pins.step, 0)
        self._dir_line  = _req_out(pins.dir,  0)

        # Durum
        self._edge_s  = 0.005   # her kenar için süre (s). 5ms ⇒ ~100 adım/sn
        self._forward = True
        self._jog     = False
        self._nsteps  = 0
        self._run     = True
        self._total   = 0
        self._busy    = False    # YENİ

        # Güvenli başlangıç
        self._apply_dir()
        self._wake(True)         # uyanık

    # ---- düşük seviye yardımcılar ----
    def _enable(self, on: bool):
        if self.shared:
            self.shared.set_enable(on)

    def _wake(self, awake: bool):
        if self.shared:
            self.shared.set_sleep(awake)

    def _apply_dir(self):
        self._dir_line.set_value(1 if self._forward else 0)
        time.sleep(0.002)

    def _pulse_once(self):
        """Tek tam adım (HIGH ve LOW kenarları)."""
        self._step_line.set_value(1); time.sleep(self._edge_s)
        self._step_line.set_value(0); time.sleep(self._edge_s)

        delta = +1 if self._forward else -1
        self._total += 1
        self.progress.emit(self._total)
        self.step.emit(delta)

    # ---- dış API (slotlar) ----
    @Slot(float)
    def set_speed_ms(self, ms_per_edge: float):
        """Her kenar süresi (ms). 1.0 ms ⇒ ~500 adım/sn. Küçük ⇒ hızlı."""
        self._edge_s = max(0.0005, float(ms_per_edge) / 1000.0)

    @Slot(bool)
    def set_direction(self, forward: bool):
        self._forward = bool(forward)
        self._apply_dir()

    @Slot(str)
    def set_microstep(self, mode: str):
        if self.shared:
            self.shared.set_microstep(mode)
        else:
            self.error.emit("SharedPins yok: microstep ortak pinleri yönetilemiyor.")

    @Slot()
    def start_jog(self):
        """Sürekli jog (buton basılı tut)."""
        self._wake(True)
        self._jog = True

    @Slot()
    def stop(self):
        """Jog'u durdurur. EN'i değiştirmez (ortak olduğu için)."""
        self._jog = False

    @Slot(int)
    def move_steps(self, n: int):
        """Tam n adım (asenkron; işçi döngüsünde tüketilir)."""
        if n <= 0:
            return
        self._wake(True)
        self._nsteps += int(n)

    @Slot()
    def reset_pulse(self):
        if self.shared:
            self.shared.pulse_reset()
        else:
            self.error.emit("SharedPins yok: reset ortak pini yok.")

    @Slot(bool)
    def sleep(self, do_sleep: bool):
        if self.shared:
            self.shared.set_sleep(not do_sleep)
        if do_sleep:
            self._jog = False

    @Slot(bool)
    def enable(self, on: bool):
        self._enable(on)

    # ---- thread döngüsü ----
    @Slot()
    def run(self):
        try:
            while self._run:
                did = False

                if self._jog:
                    self._pulse_once()
                    did = True

                if self._nsteps > 0:
                    self._pulse_once()
                    self._nsteps -= 1
                    did = True

                # --- busy state takibi ---
                new_busy = self._jog or (self._nsteps > 0)
                if new_busy != self._busy:
                    self._busy = new_busy
                    self.busyChanged.emit(self._busy)

                if not did:
                    time.sleep(0.004)  # boşta CPU'yu yorma
        except Exception as e:
            self.error.emit(str(e))
        finally:
            # döngüden çıkarken idle'a geçtiğimizi bildir
            if self._busy:
                self._busy = False
                self.busyChanged.emit(False)
            self.finished.emit()

    # dışarıdan güvenli kapatma
    def shutdown_now(self):
        self._run = False
        self._jog = False

    # Durum sorgusu (opsiyonel)
    def is_busy(self) -> bool:
        return self._busy


# -------------------- Yüksek Seviye Sarmalayıcı --------------------

class MotorController(QObject):
    """
    Tek motor kontrol sınıfı.
    - STEP/DIR: bireysel
    - EN/RESET/SLEEP/MS*: SharedPins üzerinden ortak
    """
    progress = Signal(int)
    error    = Signal(str)

    def __init__(self, pins: MotorPins, shared: Optional[SharedPins] = None, parent=None):
        super().__init__(parent)
        self.worker = StepperWorker(pins, shared=shared)
        self.th = QThread()
        self.worker.moveToThread(self.th)

        # köprü sinyaller
        self.worker.progress.connect(self.progress)
        self.worker.error.connect(self.error)

        self.th.started.connect(self.worker.run)
        self.th.start()

    # Dış API (rsdm.py buradan çağırır)
    def set_speed_ms(self, ms: float):    self.worker.set_speed_ms(ms)
    def set_direction(self, fwd: bool):   self.worker.set_direction(fwd)
    def set_microstep(self, mode: str):   self.worker.set_microstep(mode)
    def start_jog(self):                  self.worker.start_jog()
    def stop(self):                       self.worker.stop()
    def move_steps(self, n: int):         self.worker.move_steps(n)
    def reset_pulse(self):                self.worker.reset_pulse()
    def sleep(self, do_sleep: bool):      self.worker.sleep(do_sleep)
    def enable(self, on: bool):           self.worker.enable(on)

    # Adım başına callback
    def set_step_callback(self, cb):
        """
        cb(delta:int) -> None
        delta = +1 (ileri/sağ/yukarı), -1 (geri/sol/aşağı)
        """
        self.worker.step.connect(cb)

    # Busy/idle izleme
    def is_busy(self) -> bool:
        return bool(self.worker.is_busy())

    def wait_until_idle(self, timeout_ms: int = 8000) -> bool:
        """
        Motor boşta (idle) olana dek bekler. True=başarılı, False=timeout.
        """
        if not self.is_busy():
            return True

        loop = QEventLoop()
        timed_out = {"v": False}

        def on_busy_changed(busy: bool):
            if not busy and loop.isRunning():
                loop.quit()

        self.worker.busyChanged.connect(on_busy_changed)

        timer = QTimer()
        timer.setSingleShot(True)
        timer.timeout.connect(lambda: (timed_out.update(v=True), loop.quit()))
        timer.start(max(1, int(timeout_ms)))

        # Çağrıda idle olduysa hemen çık
        if not self.is_busy():
            try:
                self.worker.busyChanged.disconnect(on_busy_changed)
            except Exception:
                pass
            timer.stop()
            return True

        loop.exec_()

        try:
            self.worker.busyChanged.disconnect(on_busy_changed)
        except Exception:
            pass
        timer.stop()

        return not timed_out["v"]

    def shutdown(self):
        """Thread'i düzgün kapat."""
        self.worker.shutdown_now()
        self.th.quit()
        self.th.wait()
