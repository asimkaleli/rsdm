# -*- coding: utf-8 -*-
"""
Dimetix D-Series iş parçacığı
- Distance modu:
    * s0h+<ms> komutu BİR KERE gönderilir
    * Dimetix <ms> periyoduyla otomatik distance gönderir
    * Thread gelen satırları okuyup distance olarak parse eder
    * s0c komutuyla auto-output durdurulur (cevap beklenmez)
- Strength modu:
    * Her döngüde s0m+0 (veya verilen strength_cmd) gönderilir
    * Gelen cevap strength olarak parse edilir
    * Döngü periyodu: interval_ms (sadece strength'te kullanılır)
"""

import re
import threading
from typing import Callable, Optional, Tuple, Iterable
from PySide2.QtCore import QThread, Signal, Slot
from serial import SerialException
from dimetix import find_and_open_port, send_cmd_open, close_port


# --- basit yardımcılar ---
def _parse_signed_int(s: str) -> Optional[int]:
    ms = re.findall(r'[+\-]?\d+', (s or "").strip())
    if not ms:
        return None
    m = max(ms, key=lambda t: len(t.lstrip('+-')))
    return int(m)


def _default_parse_distance(s: str) -> Tuple[Optional[float], str]:
    n = _parse_signed_int(s)
    return (n / 10000.0, "m") if n is not None else (None, "")


def _default_parse_strength(s: str) -> Tuple[Optional[float], str]:
    n = _parse_signed_int(s)
    return (float(n), "") if n is not None else (None, "")


class DimetixWorker(QThread):
    # sinyaller
    distance = Signal(float, str)   # (değer, birim)
    strength = Signal(float, str)   # (değer, birim)
    raw      = Signal(str)
    error    = Signal(str)

    def __init__(
        self,
        interval_ms: int = 500,
        *,
        pre_open_cmds: Iterable[str] = ("s0o",),  # lazeri aç vb.
        parse_distance: Callable[[str], Tuple[Optional[float], str]] = _default_parse_distance,
        parse_strength: Callable[[str], Tuple[Optional[float], str]] = _default_parse_strength,
        strength_cmd: str = "s0m+0",  # strength için gönderilecek komut
        parent=None
    ):
        super().__init__(parent)
        self.interval_ms = max(50, int(interval_ms))
        self._running = False
        self.ser = None
        self.port = None

        # parametrik ayarlar
        self._pre_open_cmds = tuple(pre_open_cmds) if pre_open_cmds else ()
        self._parse_distance = parse_distance
        self._parse_strength = parse_strength

        # distance auto-output period (ms) -> s0h+<ms>
        self._distance_period_ms = self.interval_ms
        self._applied_distance_period_ms = None
        self._config_lock = threading.Lock()

        # strength komutu (default: s0m+0)
        self._strength_cmd = (strength_cmd or "s0m+0").strip()

        # aktif mod: "distance" | "strength"
        self._mode = "distance"

        # distance auto-output başlatıldı mı?
        self._auto_distance_started = False

        self._open_timeout = 1.5
        self._cmd_timeout  = 1.0

    # ---- dış API ----
    def stop(self):
        self._running = False

    @Slot(str)
    def set_distance_command(self, cmd: str):
        """
        Distance periyodunu ayarla.
        Örnek:
            set_distance_command("500")      -> s0h+500 gönderecek
            set_distance_command("s0h+750")  -> s0h+750 gönderecek
        """
        cmd = (cmd or "").strip()
        if not cmd:
            return

        if cmd.startswith("s0h+"):
            val = cmd[4:]
        else:
            val = cmd

        try:
            ms = int(val)
            if ms <= 0:
                raise ValueError
            with self._config_lock:
                self._distance_period_ms = max(50, ms)
        except ValueError:
            self.error.emit(f"Geçersiz distance periyodu: '{cmd}'")

    @Slot(int)
    def set_interval_ms(self, interval_ms: int):
        """Strength döngüsünün periyodunu çalışma sırasında güncelle."""
        try:
            self.interval_ms = max(50, int(interval_ms))
        except (TypeError, ValueError):
            self.error.emit(f"Geçersiz ölçüm periyodu: '{interval_ms}'")

    @Slot(str)
    def set_strength_command(self, cmd: str):
        """
        Strength ölçümü için gönderilecek komutu değiştir (varsayılan: s0m+0).
        """
        cmd = (cmd or "").strip()
        if cmd:
            self._strength_cmd = cmd

    @Slot()
    def set_mode_distance(self):
        """Aktif modu MESAFE yap."""
        self._mode = "distance"

    @Slot()
    def set_mode_strength(self):
        """Aktif modu SİNYAL yap."""
        self._mode = "strength"

    @Slot()
    def set_mode_off(self):
        """Hiçbir ölçüm yapma (idle / kapalı)."""
        self._mode = "off"

    # ---- dahili ----
    def _reopen(self, delay_ms=800):
        # portu kapat-aç
        try:
            if self.ser:
                close_port(self.ser)
        except Exception:
            pass
        self.ser = None
        self.port = None

        self.msleep(max(0, int(delay_ms)))

        # port arama (s0g sadece port bulma için kullanılıyor)
        self.ser, self.port = find_and_open_port(cmd="s0g", timeout=self._open_timeout)
        if self.ser:
            for c in self._pre_open_cmds:
                try:
                    send_cmd_open(self.ser, cmd=c, timeout=1.0)
                except Exception as e:
                    self.error.emit(f"Ön-komut '{c}' hata: {e}")
            # auto-distance yeniden başlatılması gerektiğini belirt
            self._auto_distance_started = False
            self._applied_distance_period_ms = None
            return True
        return False

    def _send_and_emit(self, cmd: str,
                       parser: Callable[[str], Tuple[Optional[float], str]],
                       which: str):
        ok, resp = send_cmd_open(self.ser, cmd=cmd, timeout=self._cmd_timeout)
        if not ok or not resp:
            return
        self.raw.emit(resp)
        val, unit = parser(resp)
        if val is None:
            return
        if which == "distance":
            self.distance.emit(val, unit)
        else:
            self.strength.emit(val, unit)

    def _start_auto_distance_if_needed(self):
        """
        Distance modunda s0h+<ms> gönder; periyot değiştiyse otomatik yenile.
        """
        if not self.ser:
            return

        with self._config_lock:
            requested_period_ms = int(self._distance_period_ms)

        if (self._auto_distance_started
                and self._applied_distance_period_ms == requested_period_ms):
            return
        if self._auto_distance_started:
            self._stop_auto_distance_if_running()

        cmd = f"s0h+{requested_period_ms}"
        try:
            ok, resp = send_cmd_open(self.ser, cmd=cmd, timeout=self._cmd_timeout)
            if not ok:
                self.error.emit(f"Auto-distance başlatılamadı: {cmd}")
                return
            # cevabı istersen görmek için:
            if resp:
                self.raw.emit(resp)
            self._auto_distance_started = True
            self._applied_distance_period_ms = requested_period_ms
        except Exception as e:
            self.error.emit(f"Auto-distance komut hatası ({cmd}): {e}")

    def _stop_auto_distance_if_running(self):
        """
        Auto-distance aktif ise s0c ile durdur.
        Dimetix s0c'ye cevap göndermediği için cevap beklemiyoruz.
        """
        if not self._auto_distance_started or not self.ser:
            return

        try:
            # Cevabı umursamadan sadece komutu gönder.
            send_cmd_open(self.ser, cmd="s0c", timeout=self._cmd_timeout)
        except Exception as e:
            self.error.emit(f"Auto-distance durdurma hatası (s0c): {e}")
        finally:
            self._auto_distance_started = False
            self._applied_distance_period_ms = None

    def stop_auto_distance(self):
        self._stop_auto_distance_if_running()

    def run(self):
        self._running = True

        if not self._reopen(delay_ms=0):
            self.error.emit("Dimetix bulunamadı veya cevap vermedi.")
            self._running = False
            return

        try:
            while self._running:
                try:
                    if self._mode == "distance":
                        # Distance modunda: auto-output başlat, sonra gelen satırı oku
                        self._start_auto_distance_if_needed()

                        if not self.ser:
                            if not self._reopen(delay_ms=1200):
                                self.msleep(1500)
                                continue

                        try:
                            line = self.ser.readline().decode(errors="ignore").strip()
                        except Exception as e:
                            self.error.emit(f"Okuma hatası: {e}")
                            if not self._reopen(delay_ms=1200):
                                self.msleep(1500)
                            continue

                        if line:
                            self.raw.emit(line)
                            val, unit = self._parse_distance(line)
                            if val is not None:
                                self.distance.emit(val, unit)

                        # BURADA EXTRA SLEEP YOK (periyodu Dimetix belirliyor)

                    elif self._mode == "strength":
                        # Strength moduna geçince auto-distance varsa durdur
                        self._stop_auto_distance_if_running()

                        if not self.ser:
                            if not self._reopen(delay_ms=1200):
                                self.msleep(1500)
                                continue

                        if self._strength_cmd:
                            self._send_and_emit(self._strength_cmd,
                                                self._parse_strength,
                                                "strength")

                        # Strength modunda kendi periyodumuz
                        self.msleep(self.interval_ms)

                    else:
                        # OFF / idle mod
                        # Her ihtimale karşı auto-distance kapalı olsun
                        self._stop_auto_distance_if_running()
                        # Cihazı yormadan ufak bir bekleme
                        self.msleep(100)
                        continue


                except SerialException as e:
                    self.error.emit(f"Seri hata: {e}. Yeniden bağlanılıyor...")
                    self._auto_distance_started = False
                    if not self._reopen(delay_ms=1200):
                        self.msleep(1500)

                except Exception as e:
                    self.error.emit(f"Dimetix hata: {e}")
                    self._auto_distance_started = False
                    if not self._reopen(delay_ms=1200):
                        self.msleep(1500)

        finally:
            # Thread kapanırken auto-distance açıksa kapat
            try:
                self._stop_auto_distance_if_running()
            except Exception:
                pass
            if self.ser:
                close_port(self.ser)
                self.ser = None
                self.port = None
