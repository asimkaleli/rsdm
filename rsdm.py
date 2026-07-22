import os
from pathlib import Path
import sys
from types import SimpleNamespace
from typing import Optional, Type, TypeVar

import time
import re  # <<<< saniye parser'ı için eklendi

# -------- HiDPI'yi ZORLA kapat (PySide2 importlarından ÖNCE!) --------
# Ortam zaten ölçeklemeyi açmış olabilir; setdefault yetmez → override/popup.
for _k in ("QT_AUTO_SCREEN_SCALE_FACTOR", "QT_ENABLE_HIGHDPI_SCALING",
           "QT_DEVICE_PIXEL_RATIO", "QT_SCREEN_SCALE_FACTORS"):
    os.environ.pop(_k, None)
os.environ["QT_SCALE_FACTOR"] = "1"          # sabit 1.0 ölçek
os.environ["QT_QPA_PLATFORMTHEME"] = ""      # öncekiyle tutarlı
os.environ["QT_STYLE_OVERRIDE"] = "Fusion"   # stil aynı kalsın

from PySide2.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QMessageBox,
    QFileDialog, QTableWidget, QTableWidgetItem, QHeaderView,
    QHBoxLayout, QLineEdit, QSpinBox, QStyleFactory, QCheckBox,
    QComboBox, QStyle
)
from PySide2.QtGui import QPixmap, QIcon
from PySide2.QtCore import QFile, Qt, QCoreApplication, QTimer, QThread, QSize
from PySide2.QtUiTools import QUiLoader

# --- Ayrı modüller ---
from camera_controller import CameraController
from clickable_label import ClickableLabel
from dimetix_worker import DimetixWorker
from dimetix import close_port
from laser_gpio import LaserGPIO

from orientation import OrientationWorker
from motor_control import MotorController, MotorPins, SharedPins

# Yalnızca planlayıcıyı kullanacağız (fallback yok)
from laser_path_planner import plan_laser_path, StepperConfig

from datetime import datetime

T = TypeVar("T")

class rsdm(QWidget):
    # ----------- KALİBRASYON (kendine göre güncelle) -----------
    # Görsel kısım için piksel→adım dursun ama kullanılmıyor (planlayıcı kullanıyoruz)
    PIXEL_TO_STEP_X = 0.0
    PIXEL_TO_STEP_Y = 0.0

    # Mekanik parametreler (her eksen için ayrı)
    # Not: planlayıcı StepperConfig(step_angle_deg, microstep_div, gear_ratio) bekliyor.
    STEP_ANGLE_DEG_X = 1.8
    MICROSTEP_DIV_X = 16
    GEAR_RATIO_X = 10

    STEP_ANGLE_DEG_Y = 1.8
    MICROSTEP_DIV_Y = 16
    GEAR_RATIO_Y = 10

    # Motion profile. The combo box remains the target speed; motors ramp from
    # this start speed using the configured acceleration.
    MOTOR_START_SPS = 50.0
    MOTOR_ACCELERATION_SPS2 = 400.0

    def __init__(self):
        super(rsdm, self).__init__()

        self.setFocusPolicy(Qt.StrongFocus)

        # 1) UI'yi yükle ve özel label swap yap
        self.load_ui()

        # --- Dimetix ölçüm worker ---
        self._last_distance = None            # anlık ölçüm (m ya da mm → stringte birim var)
        self._first_distance = None           # Select First'te yakalanan
        self._last_distance_at_select = None  # Select Last'ta yakalanan

        self.dim_worker = DimetixWorker(interval_ms=5000, parent=self)
        she = self.dim_worker
        she.distance.connect(self._on_dim_distance)
        she.strength.connect(self._on_dim_strength)
        she.error.connect(self._on_dim_error)
        she.set_distance_command("s0h+500")
        she.set_strength_command("s0m+0")
        she.set_mode_distance()

        # 2) Widget referanslarını bağla
        self.bind_widgets()
        # 3) Sinyal/slot bağlantıları
        self.connect_signals()
        # 4) Tablo ayarları
        self.setup_table()

        # Genel stil (grup başlıklarını ortala)
        self.setStyleSheet(
            """
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top center;
    padding: 0 6px;
}
"""
        )

        # --- Kamera ---
        self.cam = CameraController(self.label, width=854, height=480, fps=30,
                                    vflip=False, hflip=False)
        self.cam.start()

        # --- IMU ---
        self.ori_thread = QThread(self)
        self.ori_worker = OrientationWorker(i2c_addr=0x68, hz=50, use_mag=True,
                                            yaw_alpha=0.6, declination_deg=6.0)
        self.ori_worker.moveToThread(self.ori_thread)
        self.ori_thread.started.connect(self.ori_worker.start)
        self.ori_worker.orientation.connect(self.on_orientation)
        self.ori_worker.error.connect(lambda m: QMessageBox.critical(self, "IMU Hatası", m))
        self.ori_worker.finished.connect(self.ori_thread.quit)
        self.ori_thread.start()

        # --- Motor step sayaçları (mutlak + referans için) ---
        self._steps_x_abs = 0   # motorX (Yaw) mutlak step sayacı
        self._steps_y_abs = 0   # motorY (Pitch) mutlak step sayacı
        self._origin_steps_x = None  # pbStorePoint ile alınan referans X
        self._origin_steps_y = None  # pbStorePoint ile alınan referans Y

        # pbStorePoint sonrası marker bekleme durumu
        self._awaiting_marker_click = False
        self._last_store_row = None

        # Açısal scan için sıra index'i (angle satırlarına göre)
        self._scan_angle_index = 0

        # --- Motorlar ve lazer ---
        try:
            # 1) Ortak pin yöneticisi
            self.shared = SharedPins(
                en=7, reset=8, sleep=25,
                ms1=16, ms2=20, ms3=21
            )

            # 2) Motorlar
            self.motorX = MotorController(MotorPins(step=12, dir=5), shared=self.shared)  # Yaw ~ sağ/sol
            self.motorY = MotorController(
                MotorPins(step=13, dir=6, dir_inverted=True), shared=self.shared
            )  # Pitch ~ yukarı/aşağı; sürücünün DIR polaritesi ters
            self.motorX.set_motion_profile(self.MOTOR_START_SPS, self.MOTOR_ACCELERATION_SPS2)
            self.motorY.set_motion_profile(self.MOTOR_START_SPS, self.MOTOR_ACCELERATION_SPS2)

            # Adım sayaç/durum (seçim aralığını ölçmek için)
            self._track_steps = False
            self._sx_right = 0
            self._sx_left = 0
            self._sy_up = 0
            self._sy_down = 0

            # Step callback
            self.motorX.set_step_callback(self._on_motorX_step)
            self.motorY.set_step_callback(self._on_motorY_step)

            # Microstep (her iki motora etki eder)
            self.motorX.set_microstep("SIXTEENTH")

            # Güç ver (EN=LOW)
            self.shared.set_enable(True)

            # Lazer GPIO (BCM 24)
            self.laser = LaserGPIO(line=24)
            self.laser.set_enabled(False)
        except Exception as e:
            QMessageBox.critical(self, "Motor/Lazer Hatası", str(e))

        self._x_left_pressed = False
        self._x_right_pressed = False
        self._y_up_pressed = False
        self._y_down_pressed = False
        self._x_last_dir = None   # "left" / "right"
        self._y_last_dir = None   # "up" / "down"

        # --- Plan çıktısı ---
        self._planner_result = None  # dict: xyz, dpy, pitch_steps_delta, yaw_steps_delta
        self._scan_step_index = 0    # manuel tarama için segment index

        # --- Loglama (pitch, yaw, mesafe, zaman) ---
        self._logging_enabled = False
        self._log_file = None
        self._log_path = None
        self._last_distance_unit = ""    # Dimetix'ten gelen birim ("m" vb.)

        # Hangi satırda beklerken zaman serisi loglanacağını tut
        self._current_log_row = None
        self._last_log_row = None
        # Sequential tabloda X/Y piksel değerleri bulunduğundan, CSV için
        # planner'ın hesapladığı hedef açıları ayrıca taşıyoruz.
        self._current_log_pitch = None
        self._current_log_yaw = None

        # --- Hız combobox varsayılanı uygulansın (UI hazır olduğunda) ---
        if self.ui.cbMotorSpeed:
            QTimer.singleShot(0, self._apply_initial_speed_from_combo)


    # ---------- UI yükleme ve widget bağlama ----------
    def load_ui(self):
        loader = QUiLoader()
        path = os.fspath(Path(__file__).resolve().parent / "form.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        loader.load(ui_file, self)
        ui_file.close()

        # Görüntü label'ını ClickableLabel ile değiştir
        old_label = self.findChild(QLabel, "imageLabel")
        self.label = ClickableLabel(old_label.parent())
        self.label.setGeometry(old_label.geometry())
        self.label.setObjectName("imageLabel")
        self.label.setScaledContents(True)
        self.label.show()
        old_label.hide()

    def w(self, typ: Type[T], name: str, *, required: bool = True) -> Optional[T]:
        obj = self.findChild(typ, name)
        if not obj and required:
            raise RuntimeError(f"UI öğesi bulunamadı: {name} ({typ.__name__})")
        return obj

    def bind_widgets(self):
        self.ui = SimpleNamespace()

        # Zorunlu widget'lar
        self.ui.table = self.w(QTableWidget, "coordTable")
        self.ui.btnSelect = self.w(QPushButton, "selectButton")
        self.ui.btnSeqFirst = self.w(QPushButton, "seqSelFirstPb")
        self.ui.btnSeqLast = self.w(QPushButton, "seqSelLastPb")
        self.ui.btnSeqCreate = self.w(QPushButton, "seqCreatePb")
        self.ui.pbNextPoint = self.w(QPushButton, "pbNextPoint")
        self.ui.countSpin = self.w(QSpinBox, "seqCntSb")

        # Yön tuşları (senin UI eşleşmene göre)
        self.ui.btnDown = self.w(QPushButton, "btnDown")
        self.ui.btnUp = self.w(QPushButton, "btnUp")
        self.ui.btnRight = self.w(QPushButton, "btnRight")
        self.ui.btnLeft = self.w(QPushButton, "btnLeft")

        # Opsiyonel widget'lar
        self.ui.cbLazer = self.w(QCheckBox, "cbLazerActivate", required=False)
        self.ui.cbManualMeasure = self.w(QCheckBox, "cbManualMeasure", required=False)
        self.ui.cbModeDistance = self.w(QCheckBox, "cbModeDistance", required=False)
        self.ui.cbModeSignalQuality = self.w(QCheckBox, "cbModeSignalQuality", required=False)

        self.ui.leManualMeasure = self.w(QLineEdit, "leManualMeasure", required=False)
        self.ui.leDistance = self.w(QLineEdit, "leDistance", required=False)
        self.ui.leDistanceInterval = self.w(QLineEdit, "leDistanceInterval", required=False)
        self.ui.leSignalQuality = self.w(QLineEdit, "leSignalQuality", required=False)
        self.ui.leSignalInterval = self.w(QLineEdit, "leSignalInterval", required=False)

        self.ui.yawLe = self.w(QLineEdit, "yawLe", required=False)
        self.ui.pitchLe = self.w(QLineEdit, "pitchLe", required=False)
        self.ui.rollLe = self.w(QLineEdit, "rollLe", required=False)
        self.ui.tmpLe = self.w(QLineEdit, "tmpLe", required=False)

        self.ui.xInput = self.w(QLineEdit, "xInput", required=False)
        self.ui.yInput = self.w(QLineEdit, "yInput", required=False)
        self.ui.xyAddButton = self.w(QPushButton, "xyAddButton", required=False)

        self.ui.cbMotorSpeed = self.w(QComboBox, "cbMotorSpeed", required=False)

        # Manuel nokta kaydetme butonu (Pitch/Yaw)
        self.ui.pbStorePoint = self.w(QPushButton, "pbStorePoint", required=False)

        # Label ↔ tablo bağlantısı
        self.label.set_table(self.ui.table)

        if self.ui.leDistance:
            self.ui.leDistance.setReadOnly(True)
            self.ui.leDistance.setText("--")
        if self.ui.leSignalQuality:
            self.ui.leSignalQuality.setReadOnly(True)
            self.ui.leSignalQuality.setText("--")
        if self.ui.leDistanceInterval:
            self.ui.leDistanceInterval.setText("500")
        if self.ui.leSignalInterval:
            self.ui.leSignalInterval.setText("500")

        # Geriye uyumluluk
        self.table = self.ui.table

        # --- Tarama UI öğeleri ---
        self.ui.pbScanPoints = self.w(QPushButton, "pbScanPoints", required=False)
        self.ui.leScanInterval = self.w(QLineEdit,  "leScanInterval", required=False)
        if self.ui.leScanInterval and not self.ui.leScanInterval.text().strip():
            self.ui.leScanInterval.setText("5")  # saniye varsayılan

        self.ui.pbStartLoging = self.w(QPushButton, "pbStartLoging", required=False)
        self.ui.pbStopLoging = self.w(QPushButton, "pbStopLoging", required=False)

        self.ui.test_l_btn = self.w(QPushButton, "test_l_btn", required=False)
        self.ui.test_r_btn = self.w(QPushButton, "test_r_btn", required=False)
        self.ui.test_l_label = self.w(QLineEdit,  "test_l_label", required=False)
        self.ui.test_r_label = self.w(QLineEdit,  "test_r_label", required=False)






    def connect_signals(self):
        # Dosya seç
        self.ui.btnSelect.clicked.connect(self.select_button_clicked)
        # Label tıklama
        self.label.clicked.connect(self.on_label_clicked)
        # Elle marker ekleme
        self.ui.xyAddButton.clicked.connect(self.add_manual_marker)

        # Sıralı seçim
        self.ui.btnSeqFirst.clicked.connect(self.on_select_first_clicked)
        self.ui.btnSeqLast.clicked.connect(self.on_select_last_clicked)
        self.ui.btnSeqCreate.clicked.connect(self.on_clicked_create_btn)
        self.ui.pbNextPoint.clicked.connect(self.on_clicked_next_point_btn)

        # Tablo-sil senkronu
        self.ui.table.model().rowsRemoved.connect(self.on_rows_removed)

        # Motor butonları
        self.ui.btnRight.pressed.connect(self._x_right_press)
        self.ui.btnRight.released.connect(self._x_stop)
        self.ui.btnLeft.pressed.connect(self._x_left_press)
        self.ui.btnLeft.released.connect(self._x_stop)
        self.ui.btnUp.pressed.connect(self._y_up_press)
        self.ui.btnUp.released.connect(self._y_stop)
        self.ui.btnDown.pressed.connect(self._y_down_press)
        self.ui.btnDown.released.connect(self._y_stop)

        self.ui.cbLazer.toggled.connect(self.on_laser_toggled)
        self.ui.cbManualMeasure.toggled.connect(self.on_manual_measure_select)
        self.ui.cbModeDistance.toggled.connect(self.on_mode_distance_select)
        self.ui.cbModeDistance.setChecked(True)
        self.ui.cbModeSignalQuality.toggled.connect(self.on_signal_quality_select)
        # Hız combobox
        self.init_speed_combo()
        # Tarama butonu
        self.ui.pbScanPoints.clicked.connect(self.on_scan_points_clicked)
        # Pitch/Yaw nokta kaydetme (pbStorePoint)
        self.ui.pbStorePoint.clicked.connect(self.on_pb_store_point)

        self.ui.pbStartLoging.clicked.connect(self.on_pb_start_logging)
        self.ui.pbStopLoging.clicked.connect(self.on_pb_stop_logging)

        self.ui.test_l_btn.clicked.connect(self.on_test_l_btn)
        self.ui.test_r_btn.clicked.connect(self.on_test_r_btn)


    def on_test_l_btn(self):

        self.motorX.set_direction(True)
        self.motorX.move_steps(int(self.ui.test_l_label.text()))


    def on_test_r_btn(self):
        self.motorX.set_direction(False)
        self.motorX.move_steps(int(self.ui.test_r_label.text()))

    # ---------- Tablo ayarı ----------
    def setup_table(self):
        header = self.table.horizontalHeader()
        for col in range(self.table.columnCount()):
            if col in [0, 1]:
                header.setSectionResizeMode(col, QHeaderView.Fixed)
                self.table.setColumnWidth(col, 60)
            elif col == 2:
                header.setSectionResizeMode(col, QHeaderView.Stretch)

    # ---------- Dimetix callbacks ----------
    def _on_dim_distance(self, value: float, unit: str):
        if self.ui.cbModeDistance and not self.ui.cbModeDistance.isChecked():
            return

        self._last_distance = float(value)
        self._last_distance_unit = unit or ""

        if self.ui.leDistance:
            self.ui.leDistance.setText(f"{value:.4f} {unit}")

        # --- LOG KISMI ---
        # Sadece:
        # - Log açıkken
        # - Dosya varken
        # - Her iki motor da BUSY DEĞİLKEN (yani durağan haldeyken)
        # log al.
        if not (self._logging_enabled and self._log_file):
            return

        try:
            # Eğer motor objeleri yoksa (init hata vs.) loglama
            if not hasattr(self, "motorX") or not hasattr(self, "motorY"):
                return

            if self.motorX.is_busy() or self.motorY.is_busy():
                # Hareket sırasında → hiç log alma
                return
        except Exception:
            # Motor is_busy() çağrısında bir problem olursa güvenlik için loglama
            return

        # Buraya geldiysek: motorlar duruyor.
        # Açısal modda isek _current_log_row o anki hedef row’u temsil eder.
        row_idx = self._current_log_row

        # Eğer tanımlı bir row yoksa log alma (boş row istemiyorsan)
        if row_idx is None:
            return

        self._log_current_state(row_index=row_idx)

    def _clear_log_target(self):
        """Hareket sırasında eski hedefe ölçüm yazılmasını engelle."""
        self._current_log_row = None
        self._current_log_pitch = None
        self._current_log_yaw = None

    def _set_log_target(self, row: int, pitch=None, yaw=None):
        """Motor hedefe ulaştıktan sonra ölçümlerin bağlanacağı noktayı seç."""
        self._current_log_row = int(row)
        self._last_log_row = int(row)
        self._current_log_pitch = None if pitch is None else float(pitch)
        self._current_log_yaw = None if yaw is None else float(yaw)

    def _on_dim_strength(self, value: float, unit: str):
        if self.ui.cbModeSignalQuality and not self.ui.cbModeSignalQuality.isChecked():
            return

        if self.ui.leSignalQuality:
            self.ui.leSignalQuality.setText(f"{value:.0f}{(' ' + unit) if unit else ''}")

    def _on_dim_error(self, msg: str):
        print(f"Hata: {msg}")

    # ---------- Lazer ve mod seçimleri ----------
    def on_laser_toggled(self, checked: bool):
        try:
            if hasattr(self, "laser") and self.laser:
                self.laser.set_enabled(checked)
        except Exception as e:
            QMessageBox.critical(self, "Lazer GPIO", str(e))
            if self.ui.cbLazer:
                self.ui.cbLazer.blockSignals(True)
                self.ui.cbLazer.setChecked(not checked)
                self.ui.cbLazer.blockSignals(False)
            return

        try:
            if checked:
                if not self.dim_worker.isRunning():
                    QTimer.singleShot(1500, lambda: self.dim_worker.start())

                self.ui.leManualMeasure.setEnabled(False)
            else:
                if self.dim_worker.isRunning():
                    self.dim_worker.stop()
                    self.dim_worker.wait(1000)
                    if self.dim_worker.ser:
                        close_port(self.dim_worker.ser)
                        self.dim_worker.ser = None

                    self.dim_worker.stop_auto_distance()
                    self.dim_worker.set_mode_off()
                    self.ui.leDistance.setText("--")
                    self.ui.leSignalQuality.setText("--")
                    self.ui.cbModeDistance.setChecked(False)
                    self.ui.cbModeSignalQuality.setChecked(False)
                    self.ui.leManualMeasure.setEnabled(True)
        except Exception as e:
            QMessageBox.critical(self, "Dimetix Worker", str(e))

    def on_mode_distance_select(self, checked: bool):
        if checked:
            self.ui.cbModeSignalQuality.setChecked(False)

            # interval'i lineEdit'ten al
            txt = self.ui.leDistanceInterval.text().strip() if self.ui.leDistanceInterval else ""
            distance_interval = int(txt) if txt.isdigit() else 500

            # worker interval ve komut
            self.dim_worker.interval_ms = distance_interval
            self.dim_worker.set_distance_command(f"s0h+{distance_interval}")
            self.dim_worker.set_mode_distance()

        else:
            self.dim_worker.stop_auto_distance()
            self.dim_worker.set_mode_off()
            if self.ui.leDistance:
                self.ui.leDistance.setText("--")

    def on_signal_quality_select(self, checked: bool):
        if checked:
            self.ui.cbModeDistance.setChecked(False)

            txt = self.ui.leSignalInterval.text().strip()
            strength_interval = int(txt) if txt.isdigit() else 500

            self.dim_worker.interval_ms = strength_interval
            self.dim_worker.stop_auto_distance()
            self.dim_worker.set_mode_strength()

        else:
            self.dim_worker.set_mode_off()
            self.ui.leSignalQuality.setText("--")

    def on_manual_measure_select(self, checked: bool):
        if checked:
            self.dim_worker.stop_auto_distance()
            self.dim_worker.set_mode_off()
            self.ui.cbModeDistance.setChecked(False)
            self.ui.cbModeSignalQuality.setChecked(False)

            self.ui.leDistance.setText("--")
            self.ui.leSignalQuality.setText("--")

            self.ui.leDistance.setEnabled(False)
            self.ui.leSignalQuality.setEnabled(False)

        else:
            self.dim_worker.stop_auto_distance()
            self.ui.leDistance.setEnabled(True)
            self.ui.leSignalQuality.setEnabled(True)

    # ---------- IMU callback ----------
    def on_orientation(self, roll, pitch, yaw, temp):
        if self.ui.rollLe:
            self.ui.rollLe.setText(f"{roll:.2f}")
        if self.ui.pitchLe:
            self.ui.pitchLe.setText(f"{pitch:.2f}")
        if self.ui.yawLe:
            self.ui.yawLe.setText(f"{yaw:.2f}")
        if self.ui.tmpLe:
            self.ui.tmpLe.setText(f"{temp:.2f}")

    # ---------- Klavye olayları ----------
    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return

        key = event.key()

        if key == Qt.Key_Right:
            self._x_right_pressed = True
            self._x_last_dir = "right"
            self._update_x_from_keys()
        elif key == Qt.Key_Left:
            self._x_left_pressed = True
            self._x_last_dir = "left"
            self._update_x_from_keys()
        elif key == Qt.Key_Up:
            self._y_up_pressed = True
            self._y_last_dir = "up"
            self._update_y_from_keys()
        elif key == Qt.Key_Down:
            self._y_down_pressed = True
            self._y_last_dir = "down"
            self._update_y_from_keys()
        else:
            super(rsdm, self).keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return

        key = event.key()

        if key == Qt.Key_Right:
            self._x_right_pressed = False
            self._update_x_from_keys()
        elif key == Qt.Key_Left:
            self._x_left_pressed = False
            self._update_x_from_keys()
        elif key == Qt.Key_Up:
            self._y_up_pressed = False
            self._update_y_from_keys()
        elif key == Qt.Key_Down:
            self._y_down_pressed = False
            self._update_y_from_keys()
        else:
            super(rsdm, self).keyReleaseEvent(event)

    def _update_x_from_keys(self):
        """
        X ekseni için klavye tuşlarına göre motor durumunu güncelle.
        - Sadece sağ basılı: sağa jog
        - Sadece sol basılı: sola jog
        - İkisi de değil: durdur
        - İkisi birden basılı: son basılan yönü kullan
        """
        if self._x_right_pressed and not self._x_left_pressed:
            self._x_right_press()
        elif self._x_left_pressed and not self._x_right_pressed:
            self._x_left_press()
        elif not self._x_left_pressed and not self._x_right_pressed:
            self._x_stop()
        else:
            # Her ikisi de basılı → son basılan yönü kullan
            if self._x_last_dir == "right":
                self._x_right_press()
            elif self._x_last_dir == "left":
                self._x_left_press()

    def _update_y_from_keys(self):
        """
        Y ekseni için klavye tuşlarına göre motor durumunu güncelle.
        """
        if self._y_up_pressed and not self._y_down_pressed:
            self._y_up_press()
        elif self._y_down_pressed and not self._y_up_pressed:
            self._y_down_press()
        elif not self._y_up_pressed and not self._y_down_pressed:
            self._y_stop()
        else:
            if self._y_last_dir == "up":
                self._y_up_press()
            elif self._y_last_dir == "down":
                self._y_down_press()

    # ---------- Motor handler'ları ----------
    def _x_right_press(self):
        try:
            self.motorX.set_direction(False)
            self.motorX.start_jog()
        except Exception as e:
            QMessageBox.critical(self, "Motor X", str(e))

    def _x_left_press(self):
        try:
            self.motorX.set_direction(True)
            self.motorX.start_jog()
        except Exception as e:
            QMessageBox.critical(self, "Motor X", str(e))

    def _x_stop(self):
        try:
            self.motorX.stop()
        except Exception as e:
            QMessageBox.critical(self, "Motor X", str(e))

    def _y_up_press(self):
        try:
            self.motorY.set_direction(True)
            self.motorY.start_jog()
        except Exception as e:
            QMessageBox.critical(self, "Motor Y", str(e))

    def _y_down_press(self):
        try:
            self.motorY.set_direction(False)
            self.motorY.start_jog()
        except Exception as e:
            QMessageBox.critical(self, "Motor Y", str(e))

    def _y_stop(self):
        try:
            self.motorY.stop()
        except Exception as e:
            QMessageBox.critical(self, "Motor Y", str(e))

    # ---------- Hız / speed combo ----------
    def init_speed_combo(self):
        if not self.ui.cbMotorSpeed:
            return
        # Sadece LUT'e göre çalışıyoruz; combobox item metinlerine bağlı değiliz.
        self._speed_lut_sps = [25, 50, 100, 150, 200, 300, 400, 600, 800, 1000]
        self.ui.cbMotorSpeed.currentIndexChanged.connect(self.on_speed_combo_changed)
        # Eğer combo henüz bir seçim taşımıyorsa güvenli varsayılan index uygula (2 ⇒ ~100 sps)
        if self.ui.cbMotorSpeed.currentIndex() < 0:
            self.ui.cbMotorSpeed.setCurrentIndex(2)
        # Yine de o anki index'i uygula
        self.on_speed_combo_changed(self.ui.cbMotorSpeed.currentIndex())

    def _apply_initial_speed_from_combo(self):
        """UI gösterildikten sonra combobox seçimine göre hızı tekrar uygula."""
        if not self.ui.cbMotorSpeed:
            return
        idx = self.ui.cbMotorSpeed.currentIndex()
        if idx < 0:
            idx = 2  # güvenli varsayılan
            self.ui.cbMotorSpeed.setCurrentIndex(idx)
        self.on_speed_combo_changed(idx)

    def on_speed_combo_changed(self, index: int):
        try:
            if not hasattr(self, "_speed_lut_sps"):
                return
            if index < 0 or index >= len(self._speed_lut_sps):
                return
            sps = float(self._speed_lut_sps[index])  # 0→25 ... 9→1000
            edge_ms = 1000.0 / (2.0 * sps)          # 1 adım = 2 kenar
            if hasattr(self, "motorX") and self.motorX:
                self.motorX.set_speed_ms(edge_ms)
            if hasattr(self, "motorY") and self.motorY:
                self.motorY.set_speed_ms(edge_ms)
            # Debug
            print(f"[speed] index={index}  sps={sps:.0f}  edge_ms={edge_ms:.3f}")
        except Exception as e:
            QMessageBox.critical(self, "Speed", f"Hız uygulanamadı: {e}")

    # ---------- Dosya seçimi ----------
    def select_button_clicked(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Resim Seç", "", "Images (*.png *.jpg *.bmp *.gif *.jpeg)"
        )
        if not file_path:
            return
        pixmap = QPixmap(file_path)
        if pixmap.isNull():
            QMessageBox.warning(self, "Hata", "Resim yüklenemedi!")
            return
        self.label.setPixmap(pixmap)

    # ---------- Tabloya satır ekleme / silme ----------
    def _make_delete_btn(self, *, framed=True) -> QPushButton:
        btn = QPushButton()
        btn.setCursor(Qt.PointingHandCursor)
        btn.setToolTip("Satırı sil")
        btn.setFixedSize(28, 24)
        btn.setIconSize(QSize(16, 16))

        icon = QApplication.style().standardIcon(QStyle.SP_DialogCloseButton)
        if icon.isNull():
            icon = QIcon.fromTheme("window-close")
        if not icon.isNull():
            btn.setIcon(icon)
        else:
            btn.setText("✖")
            btn.setStyleSheet("color: red; font-weight: bold;")

        if framed:
            btn.setFlat(False)
            btn.setStyleSheet(btn.styleSheet() + """
                QPushButton {
                    border: 1px solid palette(mid);
                    border-radius: 4px;
                    padding: 1px 6px;
                }
                QPushButton:hover {
                    background: palette(button);
                    border-color: palette(dark);
                }
                QPushButton:pressed {
                    background: palette(dark);
                    color: white;
                }
            """)
        else:
            btn.setFlat(True)

        return btn

    def on_label_clicked(self, x, y):
        if not self.label.pixmap():
            return

        # Eğer pvStorePoint ile yeni bir nokta kaydedildi ve
        # bu nokta için ekranda marker bekliyorsak:
        if getattr(self, "_awaiting_marker_click", False) and self._last_store_row is not None:
            row = self._last_store_row

            # Sadece marker ekle, tabloya yeni satır ekleme!
            self.label.add_marker(row, x, y)

            # Bu marker atanmış oldu, bekleme bitti
            self._awaiting_marker_click = False
            self._last_store_row = None

            return

        # Aksi halde: klasik davranış (x,y tablosu için)
        row_position = self.table.rowCount()
        self.table.insertRow(row_position)
        self.table.setItem(row_position, 0, QTableWidgetItem(str(x)))
        self.table.setItem(row_position, 1, QTableWidgetItem(str(y)))

        btn = self._make_delete_btn(framed=True)
        btn.setFocusPolicy(Qt.NoFocus)

        cell_widget = QWidget()
        layout = QHBoxLayout(cell_widget)
        layout.addWidget(btn)
        layout.setAlignment(Qt.AlignCenter)
        layout.setContentsMargins(0, 0, 0, 0)
        self.table.setCellWidget(row_position, 2, cell_widget)

        self.label.add_marker(row_position, x, y)
        btn.clicked.connect(self.delete_row_by_button)


    def delete_row_by_button(self):
        button = self.sender()
        if button is None:
            return
        for row in range(self.table.rowCount()):
            cell_widget = self.table.cellWidget(row, 2)
            if cell_widget:
                layout = cell_widget.layout()
                if layout and layout.itemAt(0).widget() == button:
                    self.table.removeRow(row)
                    break

    def add_manual_marker(self):
        if not self.label.pixmap():
            return
        try:
            x = int(self.ui.xInput.text())
            y = int(self.ui.yInput.text())
        except ValueError:
            QMessageBox.warning(self, "Hata", "Geçerli bir sayı girin!")
            return

        max_x = self.label.width()
        max_y = self.label.height()
        if x < 0 or x > max_x or y < 0 or y > max_y:
            QMessageBox.warning(self, "Hata", f"Koordinatlar 0-{max_x} ve 0-{max_y} arasında olmalıdır!")
            return

        row_position = self.table.rowCount()
        self.table.insertRow(row_position)
        self.table.setItem(row_position, 0, QTableWidgetItem(str(x)))
        self.table.setItem(row_position, 1, QTableWidgetItem(str(y)))

        btn = self._make_delete_btn(framed=True)
        cell_widget = QWidget()
        layout = QHBoxLayout(cell_widget)
        layout.addWidget(btn)
        layout.setAlignment(Qt.AlignCenter)
        layout.setContentsMargins(0, 0, 0, 0)
        self.table.setCellWidget(row_position, 2, cell_widget)
        self.label.add_marker(row_position, x, y)
        btn.clicked.connect(self.delete_row_by_button)

        self.ui.xInput.clear()
        self.ui.yInput.clear()

    def on_rows_removed(self, parent_index, first, last):
        if not self.label.markers:
            return
        kept = []
        for idx, x, y in self.label.markers:
            if first <= idx <= last:
                continue
            kept.append((idx, x, y))
        self.label.markers = kept
        shift = (last - first + 1)
        new_markers = []
        for idx, x, y in self.label.markers:
            if idx > last:
                new_markers.append((idx - shift, x, y))
            else:
                new_markers.append((idx, x, y))
        self.label.markers = new_markers

        def exists_in_markers(pt):
            return pt is not None and any((pt[0] == mx and pt[1] == my) for _, mx, my in self.label.markers)
        if not exists_in_markers(self.label.first_point):
            self.label.first_point = None
        if not exists_in_markers(self.label.last_point):
            self.label.last_point = None

    def _get_angle_rows(self):
        """
        coordTable içindeki 'angle' satırlarının row index listesi.
        (pvStorePoint ile doldurulan satırlar.)
        """
        rows = []
        table = self.ui.table
        for r in range(table.rowCount()):
            item = table.item(r, 0)
            if item is not None and item.data(Qt.UserRole) == "angle":
                rows.append(r)
        return rows

    def _goto_angle_row(self, row: int, wait_s: float = 0.0) -> bool:
        """
        Verilen açı satırına (Pitch,Yaw) gidecek şekilde motorları hareket ettirir.
        Pitch/Yaw → step hesabını yapar, delta kadar hareket eder.
        Başarılıysa True, hata / timeout durumda False döner.
        """
        table = self.ui.table
        pitch_item = table.item(row, 0)
        yaw_item   = table.item(row, 1)
        if pitch_item is None or yaw_item is None:
            return False

        try:
            pitch_deg = float(pitch_item.text())
            yaw_deg   = float(yaw_item.text())
        except ValueError:
            QMessageBox.warning(self, "Hata", f"Row {row} için geçersiz açı değeri.")
            return False

        if self._origin_steps_x is None or self._origin_steps_y is None:
            QMessageBox.warning(self, "Hata", "Önce en az bir pvStorePoint ile referans belirleyin.")
            return False

        # step/deg oranları
        deg_per_step_x = float(self.STEP_ANGLE_DEG_X) / float(self.MICROSTEP_DIV_X) / float(self.GEAR_RATIO_X)
        deg_per_step_y = float(self.STEP_ANGLE_DEG_Y) / float(self.MICROSTEP_DIV_Y) / float(self.GEAR_RATIO_Y)

        # Hedef step (origin'e göre)
        target_dx_steps = int(round(yaw_deg   / deg_per_step_x))
        target_dy_steps = int(round(pitch_deg / deg_per_step_y))

        # Mevcut step (origin'e göre)
        cur_dx_steps = self._steps_x_abs - self._origin_steps_x
        cur_dy_steps = self._steps_y_abs - self._origin_steps_y

        # Gidilmesi gereken delta
        delta_x = target_dx_steps - cur_dx_steps  # MotorX (Yaw)
        delta_y = target_dy_steps - cur_dy_steps  # MotorY (Pitch)

        print(f"[goto_angle_row] row={row}  Pitch={pitch_deg:.4f}°  Yaw={yaw_deg:.4f}°")
        print(f"    cur_dx={cur_dx_steps}  target_dx={target_dx_steps}  delta_x={delta_x}")
        print(f"    cur_dy={cur_dy_steps}  target_dy={target_dy_steps}  delta_y={delta_y}")
        sys.stdout.flush()

        # --- HAREKETTEN ÖNCE: log satırını temizle ---
        self._clear_log_target()

        # Hareketleri sırayla gönder
        self._move_signed_steps(self.motorX, delta_x)
        self._move_signed_steps(self.motorY, delta_y)

        if not self._wait_both_idle(timeout_ms=300000):
            QMessageBox.critical(self, "Tarama Hatası",
                                 f"Row {row} konumuna giderken zaman aşımı.")
            return False

        # --- HAREKET BİTTİ: Artık bu row'dayız → loglar bu row'a yazılsın ---
        self._set_log_target(row)

        # Buradan sonra gelen tüm mesafe ölçümleri bu satıra loglanacak
        if wait_s > 0:
            self._wait_seconds(wait_s)

        return True


    def _log_angle_row(self, row: int):
        """
        Verilen angle satırı için:
        - Pitch (deg)
        - Yaw   (deg)
        - Son mesafe (self._last_distance)
        - Zaman (ISO)
        değerlerini log dosyasına yazar.
        """
        if not (self._logging_enabled and self._log_file):
            return

        table = self.ui.table
        pitch_item = table.item(row, 0)
        yaw_item   = table.item(row, 1)
        if pitch_item is None or yaw_item is None:
            return

        try:
            pitch = float(pitch_item.text())
            yaw   = float(yaw_item.text())
        except ValueError:
            return

        dist = self._last_distance
        unit = self._last_distance_unit or ""
        ts   = datetime.now().isoformat(timespec="seconds")

        # CSV satırı: row,pitch,yaw,distance,unit,timestamp
        line = f"{row},{pitch:.4f},{yaw:.4f},"
        if dist is not None:
            line += f"{dist:.4f},{unit}"
        else:
            line += ","  # mesafe yoksa boş bırak
        line += f",{ts}\n"

        try:
            self._log_file.write(line)
            self._log_file.flush()
        except Exception as e:
            self._logging_enabled = False
            QMessageBox.critical(self, "Log Hatası",
                                 f"Log dosyasına yazarken hata oluştu, log durduruldu:\n{e}")

    # ---------- Seçim ve adım sayacı ----------
    def _reset_step_counters(self):
        self._sx_right = 0
        self._sx_left = 0
        self._sy_up = 0
        self._sy_down = 0

    def on_select_first_clicked(self):
        self.label.start_select_first()
        self._reset_step_counters()
        self._track_steps = True
        self._planner_result = None
        # O anki D'yi yakala
        self._first_distance = self._last_distance

    def on_select_last_clicked(self):
        self.label.start_select_last()
        self._track_steps = False
        # O anki D'yi yakala
        self._last_distance_at_select = self._last_distance

    def _on_motorX_step(self, delta: int):
        """
        MotorX (Yaw) için step callback.
        delta işaretine göre mutlak step sayacını ve (gerekirse) seçim sayaçlarını günceller.
        """
        step = 1 if delta >= 0 else -1
        self._steps_x_abs += step

        if not self._track_steps:
            return

        if step > 0:
            self._sx_right += 1
        else:
            self._sx_left += 1

    def _on_motorY_step(self, delta: int):
        """
        MotorY (Pitch) için step callback.
        """
        step = 1 if delta >= 0 else -1
        self._steps_y_abs += step

        if not self._track_steps:
            return

        if step > 0:
            self._sy_up += 1
        else:
            self._sy_down += 1

    def on_pb_store_point(self):
        """
        pbStorePoint:
        1) coordTable BOŞSA:
           - Bu konumu referans (ilk nokta) olarak alır.
           - Pitch=0, Yaw=0 yazar.
           - _scan_angle_index sıfırlanır (Next Point yeni listede baştan başlar).
        2) coordTable BOŞ DEĞİLSE:
           - Mevcut referansa göre Pitch/Yaw hesaplar ve yeni satır ekler.
        3) Satır tipini 'angle' olarak işaretler (Next/Scan Points buna göre çalışır).
        """
        table = self.ui.table

        # Mevcut mutlak step değerleri
        sx = self._steps_x_abs   # MotorX → Yaw
        sy = self._steps_y_abs   # MotorY → Pitch

        row_count = table.rowCount()

        # --- 1) Tabloda hiç satır yoksa: ilk nokta (referans) ---
        if row_count == 0:
            # Bu çağrıyı "ilk nokta" olarak kabul et
            self._origin_steps_x = sx
            self._origin_steps_y = sy
            self._scan_angle_index = 0  # Next Point sıfırdan başlasın

            pitch_deg = 0.0
            yaw_deg = 0.0

        else:
            # Referans henüz set edilmemişse, bu çağrıda set et
            if self._origin_steps_x is None or self._origin_steps_y is None:
                self._origin_steps_x = sx
                self._origin_steps_y = sy

            dx_steps = sx - self._origin_steps_x
            dy_steps = sy - self._origin_steps_y

            # step → derece dönüşümü
            deg_per_step_x = float(self.STEP_ANGLE_DEG_X) / float(self.MICROSTEP_DIV_X) / float(self.GEAR_RATIO_X)
            deg_per_step_y = float(self.STEP_ANGLE_DEG_Y) / float(self.MICROSTEP_DIV_Y) / float(self.GEAR_RATIO_Y)

            # Motor X → Yaw (sağ+), Motor Y → Pitch (yukarı+)
            yaw_deg = dx_steps * deg_per_step_x
            pitch_deg = dy_steps * deg_per_step_y

        # --- Ortak kısım: tabloya satır ekle ---
        row = table.rowCount()
        table.insertRow(row)

        pitch_item = QTableWidgetItem(f"{pitch_deg:.4f}")
        yaw_item   = QTableWidgetItem(f"{yaw_deg:.4f}")

        # Bu satırın "açı satırı" olduğunu işaretle
        pitch_item.setData(Qt.UserRole, "angle")
        yaw_item.setData(Qt.UserRole, "angle")

        table.setItem(row, 0, pitch_item)
        table.setItem(row, 1, yaw_item)

        # Silme butonu ekle (3. sütun)
        btn = self._make_delete_btn(framed=True)
        btn.setFocusPolicy(Qt.NoFocus)
        cell_widget = QWidget()
        layout = QHBoxLayout(cell_widget)
        layout.addWidget(btn)
        layout.setAlignment(Qt.AlignCenter)
        layout.setContentsMargins(0, 0, 0, 0)
        table.setCellWidget(row, 2, cell_widget)
        btn.clicked.connect(self.delete_row_by_button)

        # Bu satır için bir sonraki resim tıklamasında marker beklendiğini işaretle
        self._last_store_row = row
        self._awaiting_marker_click = True

        # === LOG İÇİN AKTİF SATIR OLARAK İŞARETLE ===
        # Artık bu açı noktasına "erişmiş" sayıyoruz; log açıksa,
        # bu row için gelen tüm mesafe ölçümleri CSV'ye yazılacak.
        self._set_log_target(row)


    def on_pb_start_logging(self):
        """
        Start Log:
        - Kullanıcıdan bir CSV dosya yolu ister.
        - Dosyayı açar, gerekiyorsa başlık yazar.
        - _logging_enabled True yapılır.
        """
        if self._logging_enabled:
            QMessageBox.information(self, "Log", "Loglama zaten açık.")
            return

        # Bazı Raspberry Pi masaüstü/portal kombinasyonlarında native dialog
        # görünmesine rağmen klasör ve dosya adı alanları etkileşim almıyor.
        # Qt'nin kendi dialog'u bu bağımlılığı ortadan kaldırır.
        dialog_options = QFileDialog.Options()
        dialog_options |= QFileDialog.DontUseNativeDialog
        default_name = datetime.now().strftime("rsdm_log_%Y%m%d_%H%M%S.csv")
        default_path = os.fspath(Path.home() / default_name)
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Log dosyası seç (CSV)",
            default_path,
            "CSV Files (*.csv);;All Files (*)",
            options=dialog_options,
        )
        if not path:
            return
        if not Path(path).suffix:
            path += ".csv"

        try:
            f = open(path, "w", encoding="utf-8", newline="")
        except Exception as e:
            QMessageBox.critical(self, "Log", f"Dosya açılamadı:\n{e}")
            return

        # Dosya boşsa başlık satırı yaz
        if f.tell() == 0:
            f.write("row,pitch_deg,yaw_deg,distance,unit,timestamp\n")

        self._log_file = f
        self._log_path = path
        self._logging_enabled = True

        QMessageBox.information(self, "Log", f"Loglama başlatıldı:\n{path}")

    def on_pb_stop_logging(self):
        """
        Stop Log:
        - Loglamayı kapatır, dosyayı flush + close yapar.
        """
        if not self._logging_enabled:
            return

        self._logging_enabled = False
        if self._log_file:
            try:
                self._log_file.flush()
                self._log_file.close()
            except Exception:
                pass
        self._log_file = None

        QMessageBox.information(self, "Log", "Loglama durduruldu.")

    def _log_current_state(self, row_index=None):
        """
        Şu anki durumu loglar:
        - row: coordTable satır numarası (1..N) → row_index + 1
        - pitch / yaw: coordTable satırından
        - distance / unit: self._last_distance, self._last_distance_unit
        - timestamp: now() (okunabilir format: YYYY-MM-DD HH:MM:SS)
        """
        if not (self._logging_enabled and self._log_file):
            return

        if self.motorX.is_busy() or self.motorY.is_busy():
            return  # motor hareketliyken loglama

        # Sequential hedeflerde tablo piksel X/Y tutar; bu durumda planner
        # hedefleri kullanılır. Store Point satırlarında override None kalır
        # ve değerler doğrudan tablodan okunur.
        pitch = self._current_log_pitch
        yaw = self._current_log_yaw

        # 1) Eğer tablo satırı verilmişse oradan okumayı dene
        if row_index is not None:
            try:
                table = self.ui.table
                if 0 <= row_index < table.rowCount():
                    p_item = table.item(row_index, 0)
                    y_item = table.item(row_index, 1)
                    if pitch is None and p_item is not None:
                        pitch = float(p_item.text())
                    if yaw is None and y_item is not None:
                        yaw = float(y_item.text())
            except Exception:
                # Planner override'ları geçerliyse tablo okuma hatası bunları
                # silmemeli; eksik değerler aşağıda IMU fallback'ine bırakılır.
                pass

        # 2) Olmadıysa IMU textbox'lardan oku
        if pitch is None and self.ui.pitchLe:
            try:
                pitch = float(self.ui.pitchLe.text())
            except Exception:
                pitch = None

        if yaw is None and self.ui.yawLe:
            try:
                yaw = float(self.ui.yawLe.text())
            except Exception:
                yaw = None

        dist = self._last_distance
        unit = self._last_distance_unit or ""

        # Daha okunabilir zaman: "2025-11-16 15:27:15"
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # CSV satırı: row,pitch_deg,yaw_deg,distance,unit,timestamp
        parts = []

        # row (coordTable satır numarası + 1)
        if row_index is not None:
            parts.append(str(row_index + 1))
        else:
            parts.append("")

        # pitch
        if pitch is not None:
            parts.append(f"{pitch:.4f}")
        else:
            parts.append("")

        # yaw
        if yaw is not None:
            parts.append(f"{yaw:.4f}")
        else:
            parts.append("")

        # distance + unit
        if dist is not None:
            parts.append(f"{dist:.4f}")
            parts.append(unit)
        else:
            parts.append("")
            parts.append("")

        # timestamp
        parts.append(ts)

        line = ",".join(parts) + "\n"

        try:
            self._log_file.write(line)
            self._log_file.flush()
        except Exception as e:
            self._logging_enabled = False
            QMessageBox.critical(
                self,
                "Log Hatası",
                f"Log dosyasına yazarken hata oluştu, log durduruldu:\n{e}",
            )


    # ---------- Planlama & tarama ----------
    def on_clicked_create_btn(self):
        """
        Görsel: Eskisi gibi aradaki piksel noktalarını üretir.
        Açısal: plan_laser_path ile (xyz, dpy, pitch/yaw step deltaları) hesaplar ve stdout'a döker.
        """
        if not self.label.pixmap():
            return

        if not self.label.first_point or not self.label.last_point:
            QMessageBox.information(self, "Bilgi", "Önce 'Select First Point' ve 'Select Last Point' ile iki nokta seçin.")
            return

        n = 10
        if self.ui.countSpin:
            try:
                n = int(self.ui.countSpin.value())
            except Exception:
                pass
        if n < 2:
            n = 2

        # --- Görsel kısmı (UI tablo ve marker'lar) ---
        (x1, y1) = self.label.first_point
        (x2, y2) = self.label.last_point

        self.table.setRowCount(0)
        self.label.markers.clear()

        for i in range(n):
            t = i / (n - 1)
            x = int(round(x1 + (x2 - x1) * t))
            y = int(round(y1 + (y2 - y1) * t))
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.table.setItem(row, 0, QTableWidgetItem(str(x)))
            self.table.setItem(row, 1, QTableWidgetItem(str(y)))
            btn = self._make_delete_btn(framed=True)
            cell_widget = QWidget()
            layout = QHBoxLayout(cell_widget)
            layout.addWidget(btn)
            layout.setAlignment(Qt.AlignCenter)
            layout.setContentsMargins(0, 0, 0, 0)
            self.table.setCellWidget(row, 2, cell_widget)
            btn.clicked.connect(self.delete_row_by_button)
            self.label.add_marker(row, x, y)

        self.label.update()
        self.label.first_point = (x2, y2)
        self.label.last_point = None
        self.label.update()

        # --- Açısal plan için ΔYaw/ΔPitch (derece) hesapla ---
        # Motor X → YAW (sağ+), Motor Y → PITCH (yukarı+)
        deg_per_step_x = float(self.STEP_ANGLE_DEG_X) / float(self.MICROSTEP_DIV_X) / float(self.GEAR_RATIO_X)
        deg_per_step_y = float(self.STEP_ANGLE_DEG_Y) / float(self.MICROSTEP_DIV_Y) / float(self.GEAR_RATIO_Y)

        dx_steps = self._sx_right - self._sx_left
        dy_steps = self._sy_up - self._sy_down
        dYaw_deg = dx_steps * deg_per_step_x
        dPitch_deg = dy_steps * deg_per_step_y

        # D1/D2: Dimetix'ten; yoksa varsayılan
        D1 = self._first_distance if self._first_distance is not None else 1000.0
        D2 = self._last_distance_at_select if self._last_distance_at_select is not None else D1

        print("\n=== plan_laser_path GİRDİ ===")
        print(f"D1={D1:.3f}, Pitch_1=0.000, Yaw_1=0.000")
        print(f"D2={D2:.3f}, Pitch_2={dPitch_deg:.4f}, Yaw_2={dYaw_deg:.4f}")
        print(f"N={n}")
        print(f"X (Yaw): steps={dx_steps} → {dYaw_deg:.6f}°  |  Y (Pitch): steps={dy_steps} → {dPitch_deg:.6f}°")
        print(f"Stepper X: step_angle={self.STEP_ANGLE_DEG_X}°, µstep=/{self.MICROSTEP_DIV_X}, gear={self.GEAR_RATIO_X}")
        print(f"Stepper Y: step_angle={self.STEP_ANGLE_DEG_Y}°, µstep=/{self.MICROSTEP_DIV_Y}, gear={self.GEAR_RATIO_Y}")

        try:
            cfg_x = StepperConfig(
                step_angle_deg=float(self.STEP_ANGLE_DEG_X),
                microstep_div=int(self.MICROSTEP_DIV_X),
                gear_ratio=float(self.GEAR_RATIO_X),
            )
            cfg_y = StepperConfig(
                step_angle_deg=float(self.STEP_ANGLE_DEG_Y),
                microstep_div=int(self.MICROSTEP_DIV_Y),
                gear_ratio=float(self.GEAR_RATIO_Y),
            )

            result = plan_laser_path(
                D1=float(D1),
                Pitch_1=0.0,
                Yaw_1=0.0,
                D2=float(D2),
                Pitch_2=float(dPitch_deg),
                Yaw_2=float(dYaw_deg),
                N=int(n - 1),  # plan fonksiyonu N segmente göre (N+1 nokta) döndürüyor
                stepper_pitch=cfg_y,
                stepper_yaw=cfg_x,
            )
        except Exception as e:
            QMessageBox.critical(self, "Planlama Hatası", f"plan_laser_path başarısız: {e}")
            import traceback; traceback.print_exc()
            return

        # Çıktıları yazdır
        xyz = result.get("xyz", [])
        dpy = result.get("dpy", [])
        dp = result.get("pitch_steps_delta", [])
        dy = result.get("yaw_steps_delta", [])

        print("\n--- Noktalar: XYZ ---")
        for i, p in enumerate(xyz):
            x, y, z = p
            print(f"[{i:02d}]  X={x:.4f}  Y={y:.4f}  Z={z:.4f}")

        print("\n--- Noktalar: D / Pitch / Yaw (deg) ---")
        for i, a in enumerate(dpy):
            D, P, Y = a
            print(f"[{i:02d}]  D={D:.4f}  Pitch={P:.4f}°  Yaw={Y:.4f}°")

        print("\n--- Adım Delta Listeleri (ileri yön: 0→N) ---")
        for i, (pstep, ystep) in enumerate(zip(dp, dy)):
            print(f"[seg {i:02d}]  dPitch_steps={pstep:+d}   dYaw_steps={ystep:+d}")

        print("=== plan_laser_path BİTTİ ===\n")

        # Hafızada sakla (scan için)
        self._planner_result = result
        self._scan_step_index = 0

        QMessageBox.information(self, "Planlama",
            "plan_laser_path tamamlandı.\n"
            "XYZ ve D/Pitch/Yaw noktaları ile step deltaları terminale yazdırıldı."
        )

    def on_clicked_next_point_btn(self):
        """
        İKİ MOD:
        1) Eğer tabloda pvStorePoint ile kaydedilmiş 'angle' satırları varsa:
           - Bunlara göre SON NOKTADAN İLK NOKTAYA DOĞRU ilerler.
             (Son satırda olduğun varsayılır; ilk hareket ikinci sondan başlar.)
        2) Eğer hiç angle satırı yoksa:
           - Eski davranış: planner_result içindeki step deltalarına göre çalışır.
        """
        angle_rows = self._get_angle_rows()

        # --- 1) Yeni mod: angle satırlarına göre (pvStorePoint) ---
        if angle_rows:
            # 0 veya 1 nokta varsa gezilecek yer yok
            if len(angle_rows) <= 1:
                QMessageBox.information(self, "Bilgi",
                                        "En az 2 açı noktası kaydetmelisiniz.")
                return

            # Son noktaya zaten kendin gelmiş kabul ediyoruz.
            # Gezilecek gerçek adım sayısı (sondan ilk noktaya kadar) = len - 1
            max_steps = len(angle_rows) - 1

            # Tüm noktalar gezildiyse
            if self._scan_angle_index >= max_steps:
                QMessageBox.information(self, "Bilgi",
                                        "Tüm açı noktalarına son noktadan ilk noktaya kadar gidildi.")
                # İstersen burada reset de edebiliriz:
                # self._scan_angle_index = 0
                return

            # Sondan → başa giderken ilk hedef:
            # scan_index = 0 iken idx = (max_steps - 1) = len-2 (yani sondan bir önceki satır)
            idx = max_steps - 1 - self._scan_angle_index
            row = angle_rows[idx]

            print(f"[NextPoint-angle] step_index={self._scan_angle_index}  "
                  f"idx={idx}  row={row}")
            sys.stdout.flush()

            ok = self._goto_angle_row(row, wait_s=0.0)
            if ok:
                print(f"[NextPoint-angle] row {row} tamamlandı.")
                sys.stdout.flush()
                self._scan_angle_index += 1
            return

        # --- 2) Eski mod: planner_result kullan (hiç angle yoksa) ---
        if not self._planner_result:
            QMessageBox.warning(self, "Plan yok",
                                "Önce Select First/Last → Create Points ile plan oluştur "
                                "veya pvStorePoint ile açı noktaları ekleyin.")
            return

        pitch_d = list(self._planner_result.get("pitch_steps_delta", []))
        yaw_d   = list(self._planner_result.get("yaw_steps_delta",   []))
        dpy = list(self._planner_result.get("dpy", []))
        if not pitch_d or not yaw_d or len(pitch_d) != len(yaw_d):
            QMessageBox.critical(self, "Hata", "Planner step delta boyutları uyumsuz.")
            return

        total_seg = len(pitch_d)
        if total_seg == 0:
            QMessageBox.information(self, "Bilgi", "Hiç segment yok.")
            return
        if len(dpy) != total_seg + 1:
            QMessageBox.critical(self, "Hata", "Planner hedef açı listesi uyumsuz.")
            return

        # Tüm segmentler bitmişse:
        if self._scan_step_index >= total_seg:
            QMessageBox.information(self, "Bilgi", "Tüm segmentler tarandı.")
            return

        # Eski mantık: SON → İLK segment sırası
        i = total_seg - 1 - self._scan_step_index

        inv_y = -int(yaw_d[i])    # YAW → motorX (ters işaret)
        inv_p = -int(pitch_d[i])  # PITCH → motorY (ters işaret)

        print(f"[MANUAL seg {i:02d}] apply  dYaw_steps={inv_y:+d}  dPitch_steps={inv_p:+d} "
              f"(orijinal ileri yönde: yaw={yaw_d[i]:+d}, pitch={pitch_d[i]:+d})")
        sys.stdout.flush()

        self._clear_log_target()
        self._move_signed_steps(self.motorX, inv_y)
        self._move_signed_steps(self.motorY, inv_p)

        if not self._wait_both_idle(timeout_ms=300000):
            QMessageBox.critical(self, "Tarama Hatası",
                                 f"Segment {i} tamamlanmadan zaman aşımı.")
            return

        print(f"[MANUAL] segment {i:02d} tamamlandı.")
        sys.stdout.flush()

        _, target_pitch, target_yaw = dpy[i]
        self._set_log_target(i, target_pitch, target_yaw)

        self._scan_step_index += 1


    def on_scan_points_clicked(self):
        """
        İKİ MOD:
        1) Eğer tabloda pvStorePoint ile kaydedilmiş 'angle' satırları varsa:
           - Bunların tamamına sırayla gider (row0 → row1 → ...).
        2) Eğer hiç angle satırı yoksa:
           - Eski davranış: planner_result içindeki step deltalarına göre otomatik scan.
        """
        angle_rows = self._get_angle_rows()

        # --- 1) Yeni mod: angle satırlarına göre ---
        if angle_rows:
            # Bekleme süresi (s)
            if self.ui.leScanInterval:
                raw = (self.ui.leScanInterval.text() or "").strip()
                wait_s = self._parse_seconds(raw) if hasattr(self, "_parse_seconds") else 0.0
            else:
                wait_s = 0.0
            wait_s = max(0.0, wait_s)

            print("\n=== AÇISAL SCAN BAŞLIYOR (pvStorePoint noktaları) ===")
            print(f"Toplam nokta: {len(angle_rows)}, interval={wait_s:.3f} s")
            sys.stdout.flush()

            # Başlamadan önce kuyruk temizle
            try:
                self.motorX.emergency_stop()
                self.motorY.emergency_stop()
            except Exception:
                pass
            for _ in range(3):
                QApplication.processEvents()
                time.sleep(0.01)

            for idx, row in enumerate(reversed(angle_rows)):
                print(f"[angle scan] {idx+1}/{len(angle_rows)}  row={row}")
                sys.stdout.flush()
                ok = self._goto_angle_row(row, wait_s=wait_s)
                if not ok:
                    QMessageBox.critical(self, "Tarama Hatası",
                                         f"Row {row} noktasına giderken hata oluştu.")
                    return


            print("=== AÇISAL SCAN BİTTİ ===\n")
            sys.stdout.flush()
            QMessageBox.information(self, "Scanning", "Açısal noktalar için tarama tamamlandı.")
            return

        if not self._planner_result:
            QMessageBox.warning(self, "Plan yok",
                                "Önce Select First/Last → Create Points ile plan oluştur.")
            return

        # Bekleme süresi (s)
        if self.ui.leScanInterval:
            raw = (self.ui.leScanInterval.text() or "").strip()
            wait_s = self._parse_seconds(raw) if hasattr(self, "_parse_seconds") else 0.0
        else:
            wait_s = 0.0
        wait_s = max(0.0, wait_s)

        try:
            pitch_d = list(self._planner_result.get("pitch_steps_delta", []))
            yaw_d   = list(self._planner_result.get("yaw_steps_delta",   []))
            dpy = list(self._planner_result.get("dpy", []))
            if not pitch_d or not yaw_d or len(pitch_d) != len(yaw_d):
                raise RuntimeError("Planner step delta boyutları uyumsuz.")

            total_seg = len(pitch_d)
            if len(dpy) != total_seg + 1:
                raise RuntimeError("Planner hedef açı listesi uyumsuz.")

            # --- BAŞLAMADAN ÖNCE: olası kuyrukları temizle ---
            # (stop() genelde kuyruğu iptal eder; motor_control tarafında flush varsa onu çağır.)
            try:
                self.motorX.emergency_stop()
                self.motorY.emergency_stop()
            except Exception:
                pass
            # kısa arm
            for _ in range(3):
                QApplication.processEvents()
                time.sleep(0.01)

            print("\n=== TARAMA BAŞLIYOR (SON → İLK) ===")
            print(f"Segment sayısı: {total_seg}")
            print(f"İlk hareketten önce bekleme (idle + {wait_s:.3f}s): {wait_s:.3f} s")
            sys.stdout.flush()

            # Create Points sonrasında sistem son seçilen noktadadır.
            _, start_pitch, start_yaw = dpy[total_seg]
            self._set_log_target(total_seg, start_pitch, start_yaw)

            # 0) İlk girişte bekle: busy ise önce idle, sonra wait_s
            if wait_s > 0:
                if not self._wait_both_idle(timeout_ms=120000):
                    raise RuntimeError("Başlangıçta idle beklerken zaman aşımı.")
                print(f"[idle] başlangıç idle tamam. {wait_s:.3f}s bekleniyor...")
                sys.stdout.flush()
                self._wait_seconds(wait_s)

            # 1) Planner ileri yönde 0→N segmentleri veriyor.
            #    Biz SON→İLK gideceğimiz için ters sırada ve ters işaretle uygula:
            cum_y = 0
            cum_p = 0
            for i in range(total_seg - 1, -1, -1):
                self._clear_log_target()
                inv_y = -int(yaw_d[i])    # YAW → motorX (ters işaret)
                inv_p = -int(pitch_d[i])  # PITCH → motorY (ters işaret)

                print(f"[rev seg {i:02d}] apply  dYaw_steps={inv_y:+d}  dPitch_steps={inv_p:+d} "
                      f"(orijinal ileri yönde: yaw={yaw_d[i]:+d}, pitch={pitch_d[i]:+d})")
                sys.stdout.flush()

                # Hareketleri sırayla kuyrukla
                self._move_signed_steps(self.motorX, inv_y)
                self._move_signed_steps(self.motorY, inv_p)

                # Her iki motorun da bitirmesini bekle (busy → idle)
                if not self._wait_both_idle(timeout_ms=300000):  # 5dk üst sınır; gerekirse artır
                    raise RuntimeError(f"Segment {i} tamamlanmadan zaman aşımı.")

                cum_y += inv_y
                cum_p += inv_p
                print(f"            cumulative  yaw={cum_y:+d}  pitch={cum_p:+d}")
                print("            [idle] iki motor da idle.")
                sys.stdout.flush()

                _, target_pitch, target_yaw = dpy[i]
                self._set_log_target(i, target_pitch, target_yaw)

                # Segmentler arası bekleme, sadece idle olduktan sonra başlar
                if wait_s > 0:
                    print(f"            [wait] {wait_s:.3f}s bekleniyor...")
                    sys.stdout.flush()
                    self._wait_seconds(wait_s)

            print("=== TARAMA BİTTİ ===\n")
            sys.stdout.flush()
            QMessageBox.information(self, "Scannig", "Scanning is done.")

        except Exception as e:
            QMessageBox.critical(self, "Tarama Hatası", str(e))

    # ---------- Hareket / zaman yardımcıları ----------
    def _move_signed_steps(self, motor, steps: int):
        """Pozitif → ileri/sağ/yukarı, negatif → geri/sol/aşağı."""
        steps = int(steps)
        if steps == 0 or motor is None:
            return
        motor.move_signed_steps(steps)

    def _wait_seconds(self, seconds: float):
        """UI’yi dondurmadan bekle."""
        try:
            seconds = float(seconds)
        except Exception:
            seconds = 0.0
        if seconds <= 0:
            return
        import time as _t
        end = _t.monotonic() + seconds
        while _t.monotonic() < end:
            QApplication.processEvents()
            _t.sleep(0.01)

    def _parse_seconds(self, txt: str) -> float:
        """'1,5', '1.5', '1 s' vb. girdilerden saniye (float) döndürür."""
        if not txt:
            return 0.0
        txt = txt.replace(",", ".")
        m = re.search(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", txt)
        if not m:
            return 0.0
        try:
            val = float(m.group(0))
            return max(0.0, val)
        except Exception:
            return 0.0

    def _wait_both_idle(self, timeout_ms: int = 120000, settle_ms: int = 30) -> bool:
        """
        Her iki motorun gerçekten durmasını bekler.
        Yarış durumlarına karşı: kısa bir 'arm' gecikmesi + 'settle' yeniden doğrulaması uygular.
        """
        t_end = time.monotonic() + timeout_ms / 1000.0

        # ARM: is_busy bayraklarının set olabilmesi için 2-3 event döngüsü ve ufak gecikme
        for _ in range(3):
            QApplication.processEvents()
            time.sleep(0.01)

        while time.monotonic() < t_end:
            QApplication.processEvents()
            bx = self.motorX.is_busy()
            by = self.motorY.is_busy()

            if not bx and not by:
                # SETTLE: pulse kuyruğu bitti mi? ufak bekleme sonra tekrar bak
                time.sleep(settle_ms / 1000.0)
                QApplication.processEvents()
                if not self.motorX.is_busy() and not self.motorY.is_busy():
                    return True
            else:
                # Çok sık döngü olmasın
                time.sleep(0.005)

        return False

    # ---------- Kapanış ----------
    def _safe(self, func, *a, **kw):
        try:
            func(*a, **kw)
        except Exception:
            pass

    def closeEvent(self, e):
        self._safe(lambda: self.cam and self.cam.stop())
        self._safe(lambda: self.ori_worker and self.ori_worker.stop())
        self._safe(lambda: self.ori_thread and self.ori_thread.quit())
        self._safe(lambda: self.ori_thread and self.ori_thread.wait())
        self._safe(lambda: self.motorX and self.motorX.shutdown())
        self._safe(lambda: self.motorY and self.motorY.shutdown())
        self._safe(lambda: self.laser and self.laser.set_enabled(False))
        self._safe(lambda: self.laser and self.laser.release())
        self._safe(lambda: self.dim_worker and self.dim_worker.stop_auto_distance())
        if self.dim_worker and self.dim_worker.isRunning():
            self.dim_worker.stop(); self.dim_worker.wait(1000)
            if self.dim_worker.ser:
                close_port(self.dim_worker.ser); self.dim_worker.ser = None
        self._safe(lambda: self.shared and self.shared.set_enable(False))
        self._safe(lambda: self.ori_worker and self.ori_worker.stop())
        self._safe(lambda: self._log_file and self._log_file.close())

        return super().closeEvent(e)


# --------------------- main ---------------------
if __name__ == "__main__":
    QCoreApplication.setAttribute(Qt.AA_DisableHighDpiScaling, True)
    QCoreApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, False)

    app = QApplication(sys.argv)
    app.setStyle(QStyleFactory.create("Fusion"))

    widget = rsdm()
    widget.show()
    sys.exit(app.exec_())
