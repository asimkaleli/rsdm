# -*- coding: utf-8 -*-
from PySide2.QtGui import QImage, QPixmap
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QLabel

from picamera2 import Picamera2
from libcamera import Transform


class CameraController:
    """
    RPi Camera Module 2 (IMX219) preview'ünü bir QLabel üzerinde gösterir.
    Kullanım:
        self.cam = CameraController(self.label, width=854, height=480, fps=30)
        self.cam.start()
        ...
        self.cam.stop()
    """
    def __init__(self, label: QLabel, width=854, height=480, fps=30,
                 vflip=False, hflip=False):
        self.label = label
        self.width = width
        self.height = height
        self.fps = max(1, int(fps))

        self.timer = QTimer()
        self.timer.timeout.connect(self._update_frame)

        self.picam2 = Picamera2()
        transform = Transform(vflip=vflip, hflip=hflip)

        cfg = self.picam2.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"},
            queue=False,
            transform=transform
        )
        self.picam2.configure(cfg)
        self.is_running = False

    def start(self):
        if self.is_running:
            return
        self.picam2.start()
        interval_ms = max(1, int(1000 / self.fps))
        self.timer.start(interval_ms)
        self.is_running = True

    def stop(self):
        if not self.is_running:
            return
        self.timer.stop()
        try:
            self.picam2.stop()
        except Exception:
            pass
        self.is_running = False

    def _update_frame(self):
        try:
            frame = self.picam2.capture_array("main")
            if frame is None:
                return
            h, w, ch = frame.shape
            # Güvenli renk kanalı terslemesi (BGR<->RGB olasılığı için)
            frame = frame[..., ::-1].copy()
            qimg = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
            self.label.setPixmap(QPixmap.fromImage(qimg))
        except Exception:
            # Kameranın geçici hatalarında sessizce atla
            pass

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass
