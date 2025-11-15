# -*- coding: utf-8 -*-
"""
LaserGPIO
- BCM 24 hattını (varsayılan) libgpiod v2 varsa v2 API ile, yoksa v1 API ile sürer.
- set_enabled(True/False) ile hattı 1/0 yapar.
- release() kaynakları serbest bırakır.
"""

try:
    import gpiod
except ImportError:
    gpiod = None


class LaserGPIO:
    """
    GPIO24 (BCM) kontrolü. libgpiod v2 varsa onu, yoksa v1'e düşer.
    """
    def __init__(self, line=24, chipname="gpiochip0"):
        if gpiod is None:
            raise RuntimeError("python3-gpiod kurulu değil. Kur: sudo apt update && sudo apt install -y python3-gpiod")

        self.line = line
        self.api = None
        self.chip = None
        self.req = None
        self.line_obj = None

        # Önce v2 dene
        try:
            self.chip = gpiod.Chip(chipname)

            # libgpiod v2 API
            ls = gpiod.LineSettings(
                direction=gpiod.LineDirection.OUTPUT,
                output_value=getattr(gpiod, "LineValue").INACTIVE
            )
            self.req = self.chip.request_lines(
                consumer="rsdm-laser",
                config={self.line: ls}
            )
            self.api = 2
        except Exception:
            # v1'e düş
            try:
                # v1 API
                self.chip = gpiod.Chip(chipname)
                self.line_obj = self.chip.get_line(self.line)
                self.line_obj.request(
                    consumer="rsdm-laser",
                    type=gpiod.LINE_REQ_DIR_OUT,
                    default_vals=[0]
                )
                self.api = 1
            except Exception as e:
                raise RuntimeError(f"GPIO init hatası: {e}")

    def set_enabled(self, enabled: bool):
        val = 1 if enabled else 0
        try:
            if self.api == 2 and self.req:
                self.req.set_value(self.line, val)
            elif self.api == 1 and self.line_obj:
                self.line_obj.set_value(val)
            else:
                raise RuntimeError("GPIO henüz doğru biçimde tahsis edilmemiş.")
        except Exception as e:
            raise RuntimeError(f"GPIO set hatası: {e}")

    def release(self):
        try:
            if self.api == 2 and self.req:
                try:
                    self.req.release()
                except Exception:
                    pass
            if self.api == 1 and self.line_obj:
                try:
                    self.line_obj.set_value(0)
                    self.line_obj.release()
                except Exception:
                    pass
        finally:
            if self.chip:
                try:
                    self.chip.close()
                except Exception:
                    pass
