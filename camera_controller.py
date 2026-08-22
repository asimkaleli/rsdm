# -*- coding: utf-8 -*-
import glob
import os

from PySide2.QtGui import QImage, QPixmap
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QLabel

from picamera2 import Picamera2
from libcamera import Transform

try:
    import cv2
except ImportError:
    cv2 = None


class CameraController:
    """CSI kamera, yoksa UVC USB kamera önizlemesini QLabel'da gösterir."""

    def __init__(self, label: QLabel, width=854, height=480, fps=30,
                 vflip=False, hflip=False):
        self.label = label
        self.width = width
        self.height = height
        self.fps = max(1, int(fps))
        self.vflip = bool(vflip)
        self.hflip = bool(hflip)
        self.backend = None
        self.picam2 = None
        self.capture = None
        self.device = None
        self.is_running = False

        self.timer = QTimer()
        self.timer.timeout.connect(self._update_frame)

        csi_camera = self._find_csi_camera()
        if csi_camera is not None:
            self._init_csi(csi_camera["Num"])
        else:
            self._init_usb()

    @staticmethod
    def _find_csi_camera():
        for info in Picamera2.global_camera_info():
            camera_id = str(info.get("Id", "")).lower()
            model = str(info.get("Model", "")).lower()
            if "/usb@" not in camera_id and "uvc" not in model:
                return info
        return None

    def _init_csi(self, camera_num):
        self.picam2 = Picamera2(camera_num)
        transform = Transform(vflip=self.vflip, hflip=self.hflip)
        cfg = self.picam2.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"},
            queue=False,
            transform=transform
        )
        self.picam2.configure(cfg)
        self.backend = "csi"

    @staticmethod
    def _find_usb_camera():
        # by-id yolu, yeniden başlatmalarda /dev/video numarasından daha kararlıdır.
        for path in sorted(glob.glob("/dev/v4l/by-id/*-video-index0")):
            if os.path.exists(path):
                return path
        if os.path.exists("/dev/video0"):
            return "/dev/video0"
        return None

    def _init_usb(self):
        if cv2 is None:
            raise RuntimeError(
                "CSI kamera bulunamadı ve USB kamera desteği için python3-opencv "
                "kurulu değil."
            )

        self.device = self._find_usb_camera()
        if self.device is None:
            raise RuntimeError("Kullanılabilir CSI veya USB kamera bulunamadı.")

        self.capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.capture.release()
            self.capture = None
            raise RuntimeError("USB kamera açılamadı: {}".format(self.device))

        # Full-HD UVC kameralarda yüksek çözünürlükte 30 FPS için sıkıştırılmış
        # MJPEG akışı gerekir; YUYV aynı çözünürlükte USB bant genişliğini aşar.
        self.capture.set(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")
        )
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.backend = "usb"
        actual_width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.capture.get(cv2.CAP_PROP_FPS)
        print(
            "[camera] USB kamera kullanılıyor: {}  {}x{} @ {:.1f} FPS".format(
                self.device, actual_width, actual_height, actual_fps
            )
        )

    def start(self):
        if self.is_running:
            return
        if self.backend == "csi":
            self.picam2.start()
        interval_ms = max(1, int(1000 / self.fps))
        self.timer.start(interval_ms)
        self.is_running = True

    def stop(self):
        self.timer.stop()
        if self.backend == "csi" and self.picam2 is not None:
            try:
                if self.is_running:
                    self.picam2.stop()
            except Exception:
                pass
        elif self.backend == "usb" and self.capture is not None:
            self.capture.release()
            self.capture = None
        self.is_running = False

    def _update_frame(self):
        try:
            if self.backend == "csi":
                frame = self.picam2.capture_array("main")
            else:
                ok, frame = self.capture.read()
                if not ok:
                    return
                if self.vflip and self.hflip:
                    frame = cv2.flip(frame, -1)
                elif self.vflip:
                    frame = cv2.flip(frame, 0)
                elif self.hflip:
                    frame = cv2.flip(frame, 1)

            if frame is None:
                return
            h, w, ch = frame.shape
            frame = frame[..., ::-1].copy()
            qimg = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
            self.label.setPixmap(QPixmap.fromImage(qimg))
        except Exception:
            # Kameranın geçici okuma hatalarında arayüzü durdurma.
            pass

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass
