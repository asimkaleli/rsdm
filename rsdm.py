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
from PySide2.QtGui import QIcon
from PySide2.QtCore import QFile, Qt, QCoreApplication, QEvent, QTimer, QSize
from PySide2.QtUiTools import QUiLoader

# --- Ayrı modüller ---
from camera_controller import CameraController
from clickable_label import ClickableLabel
from dimetix_worker import DimetixWorker
from laser_gpio import LaserGPIO

from orientation import OrientationWorker
from motor_control import MotorController, MotorPins, SharedPins

# Yalnızca planlayıcıyı kullanacağız (fallback yok)
from laser_path_planner import (
    plan_grid_path, plan_laser_path, planned_move_steps, StepperConfig,
)

from datetime import datetime

T = TypeVar("T")

class rsdm(QWidget):
    # ----------- KALİBRASYON (kendine göre güncelle) -----------
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
    DISTANCE_MIN_FRESHNESS_S = 2.0
    MANUAL_NUDGE_STEPS = 1

    def __init__(self):
        super(rsdm, self).__init__()

        self.setFocusPolicy(Qt.StrongFocus)

        # 1) UI'yi yükle ve özel label swap yap
        self.load_ui()

        # --- Dimetix ölçüm worker ---
        self._last_distance = None            # anlık ölçüm (m ya da mm → stringte birim var)
        self._last_distance_received_at = None
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
        self.ori_worker = OrientationWorker(i2c_addr=0x68, hz=50, use_mag=True,
                                            yaw_alpha=0.6, declination_deg=6.0)
        self.ori_worker.orientation.connect(self.on_orientation)
        self.ori_worker.error.connect(lambda m: QMessageBox.critical(self, "IMU Hatası", m))
        self.ori_worker.start()

        # --- Motor step sayaçları (mutlak + referans için) ---
        self._steps_x_abs = 0   # motorX (Yaw) mutlak step sayacı
        self._steps_y_abs = 0   # motorY (Pitch) mutlak step sayacı
        self._origin_steps_x = None  # pbStorePoint ile alınan referans X
        self._origin_steps_y = None  # pbStorePoint ile alınan referans Y
        self._origin_angle_row = None  # Store Point serisinin 0,0 referans satırı

        # pbStorePoint sonrası marker bekleme durumu
        self._awaiting_marker_click = False
        self._last_store_row = None

        self._current_point_kind = None  # "stored_angle" | "sequential" | "grid"
        self._current_point_row = None
        self._programmatic_motion_active = False
        self._scan_sequence_active = False
        self._scan_cancel_requested = False
        self._area_corners = {}

        # --- Motorlar ve lazer ---
        try:
            # 1) Ortak pin yöneticisi
            self.shared = SharedPins(
                en=7, reset=8, sleep=25,
                ms1=16, ms2=20, ms3=21
            )

            # 2) Motorlar
            self.motorX = MotorController(
                MotorPins(step=13, dir=6), shared=self.shared
            )  # Yaw ~ sağ/sol; fiziksel motor artık ikinci sürücü kanalında
            self.motorY = MotorController(
                MotorPins(step=12, dir=5, dir_inverted=True), shared=self.shared
            )  # Pitch ~ yukarı/aşağı; fiziksel motor artık birinci sürücü kanalında
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
            self.motorX.timingReport.connect(
                lambda count, mean_ms, jitter_ms: self._on_motor_timing(
                    "X", count, mean_ms, jitter_ms
                )
            )
            self.motorY.timingReport.connect(
                lambda count, mean_ms, jitter_ms: self._on_motor_timing(
                    "Y", count, mean_ms, jitter_ms
                )
            )

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
        self._key_release_tokens = {
            Qt.Key_Left: 0,
            Qt.Key_Right: 0,
            Qt.Key_Up: 0,
            Qt.Key_Down: 0,
        }

        # Sequential endpoint positions are read directly from the motor
        # workers, independently of queued Qt step-update signals.
        self._seq_first_steps = None
        self._seq_last_steps = None

        # --- Plan çıktısı ---
        self._planner_result = None  # dict: xyz, dpy, pitch_steps_delta, yaw_steps_delta

        # --- Loglama (pitch, yaw, mesafe, zaman) ---
        self._logging_enabled = False
        self._log_file = None
        self._log_path = None
        self._last_distance_unit = ""    # Dimetix'ten gelen birim ("m" vb.)

        # Hangi satırda beklerken zaman serisi loglanacağını tut
        self._current_log_row = None
        self._last_log_row = None
        # Planlı hedeflerde CSV için planner'ın hesapladığı açıları ayrıca taşıyoruz.
        self._current_log_pitch = None
        self._current_log_yaw = None

        # --- Hız combobox varsayılanı uygulansın (UI hazır olduğunda) ---
        if self.ui.cbMotorSpeed:
            QTimer.singleShot(0, self._apply_initial_speed_from_combo)

        app = QApplication.instance()
        if app:
            app.installEventFilter(self)


    # ---------- UI yükleme ve widget bağlama ----------
    def load_ui(self):
        loader = QUiLoader()
        app_dir = Path(__file__).resolve().parent
        path = os.fspath(app_dir / "form.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        loaded_ui = loader.load(ui_file, self)
        ui_file.close()
        if loaded_ui is None:
            raise RuntimeError(f"Arayüz yüklenemedi: {path}")

        style = loaded_ui.styleSheet()
        icon_paths = {
            "__CHEVRON_UP_ICON__": app_dir / "ui_assets" / "chevron_up.svg",
            "__CHEVRON_DOWN_ICON__": app_dir / "ui_assets" / "chevron_down.svg",
        }
        for placeholder, icon_path in icon_paths.items():
            if not icon_path.is_file():
                raise RuntimeError(f"Arayüz ikonu bulunamadı: {icon_path}")
            style = style.replace(placeholder, icon_path.as_posix())
        loaded_ui.setStyleSheet(style)

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
        self.ui.btnSeqFirst = self.w(QPushButton, "seqSelFirstPb")
        self.ui.btnSeqLast = self.w(QPushButton, "seqSelLastPb")
        self.ui.btnSeqCreate = self.w(QPushButton, "seqCreatePb")
        self.ui.pbNextPoint = self.w(QPushButton, "pbNextPoint")
        self.ui.countSpin = self.w(QSpinBox, "seqCntSb")
        self.ui.seqTotalLabel = self.w(QLabel, "seqTotalLabel")

        # Dört köşeli alan taraması
        self.ui.areaSelectA = self.w(QPushButton, "areaSelectAPb")
        self.ui.areaSelectB = self.w(QPushButton, "areaSelectBPb")
        self.ui.areaSelectC = self.w(QPushButton, "areaSelectCPb")
        self.ui.areaSelectD = self.w(QPushButton, "areaSelectDPb")
        self.ui.areaXDiv = self.w(QSpinBox, "areaXDivSb")
        self.ui.areaYDiv = self.w(QSpinBox, "areaYDivSb")
        self.ui.areaTotalLabel = self.w(QLabel, "areaTotalLabel")
        self.ui.areaCreate = self.w(QPushButton, "areaCreatePb")

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

        self.ui.cbMotorSpeed = self.w(QComboBox, "cbMotorSpeed", required=False)

        # Manuel nokta kaydetme butonu (Pitch/Yaw)
        self.ui.pbStorePoint = self.w(QPushButton, "pbStorePoint", required=False)

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
        self.ui.pbEmergencyStop = self.w(QPushButton, "pbEmergencyStop", required=False)
        self.ui.leScanInterval = self.w(QLineEdit,  "leScanInterval", required=False)
        if self.ui.leScanInterval and not self.ui.leScanInterval.text().strip():
            self.ui.leScanInterval.setText("5")  # saniye varsayılan

        self.ui.pbStartLogging = self.w(QPushButton, "pbStartLogging", required=False)
        self.ui.pbStopLogging = self.w(QPushButton, "pbStopLogging", required=False)

    def connect_signals(self):
        # Label tıklama
        self.label.clicked.connect(self.on_label_clicked)
        self.label.selectionCompleted.connect(self.on_selection_completed)
        # Sıralı seçim
        self.ui.btnSeqFirst.clicked.connect(self.on_select_first_clicked)
        self.ui.btnSeqLast.clicked.connect(self.on_select_last_clicked)
        self.ui.btnSeqCreate.clicked.connect(self.on_clicked_create_btn)
        self.ui.countSpin.valueChanged.connect(self._update_sequential_total)
        self._update_sequential_total()
        self.ui.pbNextPoint.clicked.connect(self.on_clicked_next_point_btn)
        self.ui.areaSelectA.clicked.connect(lambda: self.on_area_select_clicked("A"))
        self.ui.areaSelectB.clicked.connect(lambda: self.on_area_select_clicked("B"))
        self.ui.areaSelectC.clicked.connect(lambda: self.on_area_select_clicked("C"))
        self.ui.areaSelectD.clicked.connect(lambda: self.on_area_select_clicked("D"))
        self.ui.areaXDiv.valueChanged.connect(self._update_area_total)
        self.ui.areaYDiv.valueChanged.connect(self._update_area_total)
        self.ui.areaCreate.clicked.connect(self.on_create_area_points)
        self._update_area_total()

        # Tablo-sil senkronu
        self.ui.table.model().rowsRemoved.connect(self.on_rows_removed)

        # Motor butonları
        self.ui.btnRight.clicked.connect(
            lambda: self._queue_manual_nudge("x", "right")
        )
        self.ui.btnLeft.clicked.connect(
            lambda: self._queue_manual_nudge("x", "left")
        )
        self.ui.btnUp.clicked.connect(
            lambda: self._queue_manual_nudge("y", "up")
        )
        self.ui.btnDown.clicked.connect(
            lambda: self._queue_manual_nudge("y", "down")
        )

        self.ui.cbLazer.toggled.connect(self.on_laser_toggled)
        self.ui.cbManualMeasure.toggled.connect(self.on_manual_measure_select)
        self.ui.cbModeDistance.toggled.connect(self.on_mode_distance_select)
        self.ui.cbModeDistance.setChecked(False)
        self.ui.cbModeSignalQuality.toggled.connect(self.on_signal_quality_select)
        # Hız combobox
        self.init_speed_combo()
        # Tarama butonu
        self.ui.pbScanPoints.clicked.connect(self.on_scan_points_clicked)
        self.ui.pbEmergencyStop.clicked.connect(self.on_emergency_stop_clicked)
        # Pitch/Yaw nokta kaydetme (pbStorePoint)
        self.ui.pbStorePoint.clicked.connect(self.on_pb_store_point)

        self.ui.pbStartLogging.clicked.connect(self.on_pb_start_logging)
        self.ui.pbStopLogging.clicked.connect(self.on_pb_stop_logging)

    def on_emergency_stop_clicked(self):
        """Bekleyen/planlı hareketleri kes ve yazılımsal konumu geçersizleştir."""
        self._scan_cancel_requested = True
        self._x_left_pressed = False
        self._x_right_pressed = False
        self._y_up_pressed = False
        self._y_down_pressed = False
        try:
            if hasattr(self, "motorX") and self.motorX:
                self.motorX.emergency_stop()
            if hasattr(self, "motorY") and self.motorY:
                self.motorY.emergency_stop()
        finally:
            self._planner_result = None
            self._origin_steps_x = None
            self._origin_steps_y = None
            self._origin_angle_row = None
            self._first_distance = None
            self._last_distance_at_select = None
            self._seq_first_steps = None
            self._seq_last_steps = None
            self._track_steps = False
            self._awaiting_marker_click = False
            self._last_store_row = None
            self.label.clear_selection()
            self._reset_sequential_selection_labels()
            self._reset_area_selection()
            self._mark_position_unknown()
        print("[EMERGENCY STOP] Motor komutları iptal edildi; konum bilinmiyor.")
        sys.stdout.flush()
        QMessageBox.warning(
            self,
            "Emergency Stop",
            "Motor hareketleri durduruldu. Konum artık bilinmiyor; "
            "mevcut hedefler yeniden kullanılmayacak. Store Point serisinde eski "
            "satırları silip yeniden referans belirleyin; Sequential/Area "
            "noktalarını ise yeniden seçin.",
        )

    # ---------- Tablo ayarı ----------
    def setup_table(self):
        self.table.setHorizontalHeaderLabels(["Pitch", "Yaw", "Del"])
        header = self.table.horizontalHeader()
        for col in range(self.table.columnCount()):
            if col in [0, 1]:
                header.setSectionResizeMode(col, QHeaderView.Fixed)
                self.table.setColumnWidth(col, 60)
            elif col == 2:
                header.setSectionResizeMode(col, QHeaderView.Stretch)

    # ---------- Dimetix callbacks ----------
    def _on_dim_distance(self, value: float, unit: str):
        if self.ui.cbLazer and not self.ui.cbLazer.isChecked():
            return
        if self.ui.cbModeDistance and not self.ui.cbModeDistance.isChecked():
            return

        self._last_distance = float(value)
        self._last_distance_received_at = time.monotonic()
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

    def _set_current_point(self, kind: str, row: int):
        self._current_point_kind = str(kind)
        self._current_point_row = int(row)

    def _mark_position_unknown(self):
        self._current_point_kind = None
        self._current_point_row = None
        self._clear_log_target()

    def _on_dim_strength(self, value: float, unit: str):
        if self.ui.cbModeSignalQuality and not self.ui.cbModeSignalQuality.isChecked():
            return

        if self.ui.leSignalQuality:
            self.ui.leSignalQuality.setText(f"{value:.0f}{(' ' + unit) if unit else ''}")

    def _on_dim_error(self, msg: str):
        print(f"Hata: {msg}")

    def _distance_max_age_seconds(self) -> float:
        """Allow a few measurement periods, with a safe lower bound."""
        interval_ms = 500
        if self.ui.leDistanceInterval:
            text = self.ui.leDistanceInterval.text().strip()
            if text.isdigit():
                interval_ms = max(1, int(text))
        return max(
            float(self.DISTANCE_MIN_FRESHNESS_S),
            3.0 * interval_ms / 1000.0,
        )

    def _fresh_distance_measurement(self):
        """Return the current (distance, unit), or None when it is not usable."""
        if self.ui.cbLazer and not self.ui.cbLazer.isChecked():
            return None
        if self.ui.cbModeDistance and not self.ui.cbModeDistance.isChecked():
            return None
        if self._last_distance is None or self._last_distance_received_at is None:
            return None
        age = time.monotonic() - float(self._last_distance_received_at)
        if age < 0.0 or age > self._distance_max_age_seconds():
            return None
        return float(self._last_distance), self._last_distance_unit or ""

    def _invalidate_distance_selections(self):
        """Discard measurements/selections that must not survive laser-off."""
        self._last_distance = None
        self._last_distance_received_at = None
        self._last_distance_unit = ""
        self._first_distance = None
        self._last_distance_at_select = None
        self._seq_first_steps = None
        self._seq_last_steps = None
        self._track_steps = False

        if hasattr(self, "label") and self.label:
            self.label.clear_selection()
        if hasattr(self, "ui"):
            self._reset_sequential_selection_labels()
        if hasattr(self, "_area_corners"):
            self._reset_area_selection()

    def _start_dimetix_if_requested(self):
        """Start after the laser warm-up delay only if it is still requested."""
        if not self.ui.cbLazer or not self.ui.cbLazer.isChecked():
            return
        if not self.dim_worker.isRunning():
            self.dim_worker.start()

    def _stop_dimetix_worker(self, timeout_ms: int = 3000) -> bool:
        """Request a cooperative stop; the worker closes its own serial port."""
        if not self.dim_worker:
            return True
        self.dim_worker.stop()
        if not self.dim_worker.isRunning():
            return True
        stopped = self.dim_worker.wait(max(1, int(timeout_ms)))
        if not stopped:
            print("[Dimetix] Worker belirtilen sürede durmadı.")
            sys.stdout.flush()
        return bool(stopped)

    # ---------- Lazer ve mod seçimleri ----------
    def on_laser_toggled(self, checked: bool):
        if not checked:
            self._invalidate_distance_selections()
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
                if self.ui.cbModeDistance and not self.ui.cbModeDistance.isChecked():
                    self.ui.cbModeDistance.setChecked(True)
                QTimer.singleShot(1500, self._start_dimetix_if_requested)

                self.ui.leManualMeasure.setEnabled(False)
            else:
                self.dim_worker.set_mode_off()
                self._stop_dimetix_worker()
                self.ui.leDistance.setText("--")
                self.ui.leSignalQuality.setText("--")
                self.ui.cbModeDistance.setChecked(False)
                self.ui.cbModeSignalQuality.setChecked(False)
                self.ui.leManualMeasure.setEnabled(True)
        except Exception as e:
            QMessageBox.critical(self, "Dimetix Worker", str(e))

    def on_mode_distance_select(self, checked: bool):
        if checked:
            if self.ui.cbLazer and not self.ui.cbLazer.isChecked():
                self.ui.cbModeDistance.blockSignals(True)
                self.ui.cbModeDistance.setChecked(False)
                self.ui.cbModeDistance.blockSignals(False)
                self.dim_worker.set_mode_off()
                return
            self.ui.cbModeSignalQuality.setChecked(False)

            # interval'i lineEdit'ten al
            txt = self.ui.leDistanceInterval.text().strip() if self.ui.leDistanceInterval else ""
            distance_interval = int(txt) if txt.isdigit() else 500

            # worker interval ve komut
            self.dim_worker.interval_ms = distance_interval
            self.dim_worker.set_distance_command(f"s0h+{distance_interval}")
            self.dim_worker.set_mode_distance()

        else:
            self._last_distance = None
            self._last_distance_received_at = None
            self._last_distance_unit = ""
            self.dim_worker.stop_auto_distance()
            self.dim_worker.set_mode_off()
            if self.ui.leDistance:
                self.ui.leDistance.setText("--")

    def on_signal_quality_select(self, checked: bool):
        if checked:
            if self.ui.cbLazer and not self.ui.cbLazer.isChecked():
                self.ui.cbModeSignalQuality.blockSignals(True)
                self.ui.cbModeSignalQuality.setChecked(False)
                self.ui.cbModeSignalQuality.blockSignals(False)
                self.dim_worker.set_mode_off()
                return
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
    def _handle_motion_key(self, key: int, pressed: bool) -> bool:
        """Queue one pulse on each physical key press; holding does nothing."""
        key_map = {
            Qt.Key_Right: ("_x_right_pressed", "x", "right"),
            Qt.Key_Left: ("_x_left_pressed", "x", "left"),
            Qt.Key_Up: ("_y_up_pressed", "y", "up"),
            Qt.Key_Down: ("_y_down_pressed", "y", "down"),
        }
        mapping = key_map.get(key)
        if mapping is None:
            return False
        attr, axis, direction = mapping
        was_pressed = bool(getattr(self, attr))
        setattr(self, attr, bool(pressed))
        if pressed and not was_pressed:
            self._queue_manual_nudge(axis, direction)
        return True

    def _release_motion_keys(self):
        """Clear key latches if focus/application state changes."""
        for key in self._key_release_tokens:
            self._key_release_tokens[key] += 1
        self._x_left_pressed = False
        self._x_right_pressed = False
        self._y_up_pressed = False
        self._y_down_pressed = False

    def _schedule_motion_key_release(self, key: int):
        """Debounce Linux key-repeat release/press pairs without extra pulses."""
        self._key_release_tokens[key] += 1
        token = self._key_release_tokens[key]

        def finish_release():
            if self._key_release_tokens.get(key) == token:
                self._handle_motion_key(key, False)

        QTimer.singleShot(40, finish_release)

    def eventFilter(self, watched, event):
        event_type = event.type()
        if event_type in (QEvent.ApplicationDeactivate, QEvent.WindowDeactivate):
            self._release_motion_keys()
        elif event_type in (QEvent.KeyPress, QEvent.KeyRelease):
            key = event.key()
            if key in (Qt.Key_Left, Qt.Key_Right, Qt.Key_Up, Qt.Key_Down):
                # Auto-repeat is consumed: holding an arrow never creates a
                # pulse train. Every new physical press queues exactly 1 pulse.
                if event_type == QEvent.KeyRelease:
                    if not event.isAutoRepeat():
                        self._schedule_motion_key_release(key)
                    return True
                if self.isActiveWindow() and QApplication.activeModalWidget() is None:
                    # Any following press cancels a pending synthetic release,
                    # even on platforms that mislabel repeat events.
                    self._key_release_tokens[key] += 1
                    if not event.isAutoRepeat():
                        self._handle_motion_key(key, True)
                    return True
        return super(rsdm, self).eventFilter(watched, event)

    def keyPressEvent(self, event):
        key = event.key()
        if key in self._key_release_tokens:
            self._key_release_tokens[key] += 1
        if (not event.isAutoRepeat()
                and self._handle_motion_key(key, True)):
            event.accept()
            return
        super(rsdm, self).keyPressEvent(event)

    def keyReleaseEvent(self, event):
        key = event.key()
        if not event.isAutoRepeat() and key in self._key_release_tokens:
            self._schedule_motion_key_release(key)
            event.accept()
            return
        super(rsdm, self).keyReleaseEvent(event)

    def _axis_motor(self, axis: str):
        return self.motorX if axis == "x" else self.motorY

    def _queue_manual_nudge(self, axis: str, direction: str):
        """Queue exactly one driver pulse for a short button/key press."""
        if self._programmatic_motion_active or self._scan_sequence_active:
            return
        sign = {
            ("x", "right"): -1,
            ("x", "left"): 1,
            ("y", "up"): 1,
            ("y", "down"): -1,
        }[(axis, direction)]
        self._mark_position_unknown()
        self._axis_motor(axis).move_signed_steps(
            sign * max(1, int(self.MANUAL_NUDGE_STEPS))
        )

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

        # Store Point marker beklemiyorsa tıklama tabloyu değiştirmez.
        return


    def delete_row_by_button(self):
        button = self.sender()
        if button is None:
            return
        for row in range(self.table.rowCount()):
            cell_widget = self.table.cellWidget(row, 2)
            if cell_widget:
                layout = cell_widget.layout()
                if layout and layout.itemAt(0).widget() == button:
                    if row == getattr(self, "_origin_angle_row", None):
                        other_angle_rows = [
                            angle_row for angle_row in self._get_angle_rows()
                            if angle_row != row
                        ]
                        if other_angle_rows:
                            QMessageBox.warning(
                                self,
                                "Referans Noktası",
                                "Bu satır açı serisinin referansıdır. "
                                "Önce referansa bağlı diğer açı noktalarını silin.",
                            )
                            return
                    self.table.removeRow(row)
                    break

    def on_rows_removed(self, parent_index, first, last):
        shift = last - first + 1

        def shifted_row(row):
            if row is None:
                return None
            if first <= row <= last:
                return None
            return row - shift if row > last else row

        current_row = getattr(self, "_current_log_row", None)
        new_current_row = shifted_row(current_row)
        if current_row is not None and new_current_row is None:
            self._clear_log_target()
        elif new_current_row is not None:
            self._current_log_row = new_current_row

        self._last_log_row = shifted_row(getattr(self, "_last_log_row", None))
        self._last_store_row = shifted_row(getattr(self, "_last_store_row", None))
        if self._last_store_row is None:
            self._awaiting_marker_click = False

        # Satır sırası değiştiğinde mevcut tarama indeksleri ve planner-row
        # eşleşmesi artık güvenilir değildir.
        self._planner_result = None

        current_kind = getattr(self, "_current_point_kind", None)
        current_point_row = shifted_row(getattr(self, "_current_point_row", None))
        if current_kind == "stored_angle" and current_point_row is not None:
            self._current_point_row = current_point_row
        else:
            # Sequential plan satır değişikliğinde geçersizdir; aktif stored
            # satır silindiyse de fiziksel hedef artık tabloyla eşleşmez.
            self._mark_position_unknown()

        old_origin_row = getattr(self, "_origin_angle_row", None)
        self._origin_angle_row = shifted_row(old_origin_row)
        if old_origin_row is not None and self._origin_angle_row is None:
            self._origin_steps_x = None
            self._origin_steps_y = None

        kept = []
        for idx, x, y in self.label.markers:
            if first <= idx <= last:
                continue
            kept.append((idx, x, y))
        self.label.markers = kept
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
            if item is not None and item.data(Qt.UserRole) == "stored_angle":
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

        if self.motorX.is_busy() or self.motorY.is_busy():
            QMessageBox.warning(self, "Motor Meşgul", "Motor hareketi devam ediyor.")
            return False
        self._sync_absolute_steps()

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

        if not self._move_both_signed_and_wait(delta_x, delta_y, timeout_ms=300000):
            QMessageBox.critical(self, "Tarama Hatası",
                                 f"Row {row} konumuna hareket tamamlanamadı.")
            return False

        # --- HAREKET BİTTİ: Artık bu row'dayız → loglar bu row'a yazılsın ---
        self._set_log_target(row)
        self._set_current_point("stored_angle", row)

        # Buradan sonra gelen tüm mesafe ölçümleri bu satıra loglanacak
        if wait_s > 0:
            if not self._wait_seconds(wait_s):
                return False

        return True


    # ---------- Seçim ve adım sayacı ----------
    def _sync_absolute_steps(self):
        """Read exact signed pulse counters directly from both workers."""
        self._steps_x_abs = int(self.motorX.position_steps())
        self._steps_y_abs = int(self.motorY.position_steps())

    def _position_capture_ready(self, title: str) -> bool:
        if self.motorX.is_busy() or self.motorY.is_busy():
            QMessageBox.warning(
                self, title,
                "Motor hareketi henüz tamamlanmadı. Noktayı kaydetmeden önce bekleyin.",
            )
            return False
        QApplication.processEvents()
        if self.motorX.is_busy() or self.motorY.is_busy():
            return False
        self._sync_absolute_steps()
        return True

    def _reset_step_counters(self):
        self._sx_right = 0
        self._sx_left = 0
        self._sy_up = 0
        self._sy_down = 0

    def on_select_first_clicked(self):
        if self._scan_sequence_active:
            return
        measurement = self._fresh_distance_measurement()
        if measurement is None:
            QMessageBox.warning(
                self,
                "Sequential Scan",
                "İlk noktayı seçmek için lazeri açın ve güncel bir mesafe ölçümü bekleyin.",
            )
            return
        if not self._position_capture_ready("Sequential Scan"):
            return
        self._reset_area_selection()
        self._reset_sequential_selection_labels()
        self.label.start_select_first()
        self._reset_step_counters()
        self._track_steps = True
        self._planner_result = None
        # O anki D'yi yakala
        self._first_distance = measurement[0]
        self._seq_first_steps = (self._steps_x_abs, self._steps_y_abs)

    def on_select_last_clicked(self):
        if self._scan_sequence_active:
            return
        measurement = self._fresh_distance_measurement()
        if measurement is None:
            QMessageBox.warning(
                self,
                "Sequential Scan",
                "Son noktayı seçmek için lazeri açın ve güncel bir mesafe ölçümü bekleyin.",
            )
            return
        if not self._position_capture_ready("Sequential Scan"):
            return
        self.label.start_select_last()
        self._track_steps = False
        # O anki D'yi yakala
        self._last_distance_at_select = measurement[0]
        self._seq_last_steps = (self._steps_x_abs, self._steps_y_abs)

    def _update_sequential_total(self, *_):
        total = int(self.ui.countSpin.value()) + 1
        self.ui.seqTotalLabel.setText(f"Generated Points: {total}")

    def _reset_sequential_selection_labels(self):
        self.ui.btnSeqFirst.setText("Select First Point")
        self.ui.btnSeqLast.setText("Select Last Point")

    # ---------- Dört köşeli alan seçimi ----------
    def _update_area_total(self, *_):
        total = (int(self.ui.areaXDiv.value()) + 1) * (
            int(self.ui.areaYDiv.value()) + 1
        )
        self.ui.areaTotalLabel.setText(f"Generated Points: {total}")

    def _area_button(self, corner: str):
        return {
            "A": self.ui.areaSelectA,
            "B": self.ui.areaSelectB,
            "C": self.ui.areaSelectC,
            "D": self.ui.areaSelectD,
        }[corner]

    def _reset_area_selection(self):
        self._area_corners.clear()
        self.label.area_points.clear()
        labels = {
            "A": "A - Top Left",
            "B": "B - Top Right",
            "C": "C - Bottom Right",
            "D": "D - Bottom Left",
        }
        for corner, text in labels.items():
            self._area_button(corner).setText(text)
        self.label.update()

    def on_area_select_clicked(self, corner: str):
        """Arm one corner selection; capture happens on the camera click."""
        if self._scan_sequence_active or self._programmatic_motion_active:
            return
        if not self.label.pixmap():
            QMessageBox.warning(self, "Area Scan", "Kamera görüntüsü hazır değil.")
            return
        if not self._position_capture_ready("Area Scan"):
            return
        if self._fresh_distance_measurement() is None:
            QMessageBox.warning(
                self,
                "Area Scan",
                "Köşeyi seçmek için lazeri açın ve güncel bir mesafe ölçümü bekleyin.",
            )
            return
        if corner == "A":
            self._reset_area_selection()
        self._track_steps = False
        self.label.first_point = None
        self.label.last_point = None
        self.label.start_select_area(corner)

    def on_selection_completed(self, mode: str, x: int, y: int):
        if mode == "first":
            self.ui.btnSeqFirst.setText("First Point Selected")
            return
        if mode == "last":
            self.ui.btnSeqLast.setText("Last Point Selected")
            return
        if not mode.startswith("area:"):
            return
        corner = mode.split(":", 1)[1]
        measurement = self._fresh_distance_measurement()
        if measurement is None:
            self.label.area_points.pop(corner, None)
            self.label.update()
            QMessageBox.warning(
                self,
                "Area Scan",
                "Köşe kaydedilemedi: lazer açık değil veya güncel mesafe ölçümü yok.",
            )
            return
        if self.motorX.is_busy() or self.motorY.is_busy():
            self.label.area_points.pop(corner, None)
            self.label.update()
            QMessageBox.warning(self, "Area Scan", "Köşe kaydedilemedi: motor hareket ediyor.")
            return

        self._sync_absolute_steps()

        self._area_corners[corner] = {
            "pixel": (int(x), int(y)),
            "steps_x": int(self._steps_x_abs),
            "steps_y": int(self._steps_y_abs),
            "distance": measurement[0],
            "unit": measurement[1],
        }
        self._area_button(corner).setText(f"{corner} Selected")
        self._planner_result = None
        self._mark_position_unknown()

    @staticmethod
    def _area_pixel_geometry_valid(corners) -> bool:
        points = [corners[name]["pixel"] for name in ("A", "B", "C", "D")]
        if len(set(points)) != 4:
            return False

        def orientation(p, q, r):
            value = (q[0] - p[0]) * (r[1] - p[1]) - (
                q[1] - p[1]
            ) * (r[0] - p[0])
            return 1 if value > 0 else -1 if value < 0 else 0

        def crosses(p1, p2, q1, q2):
            return (orientation(p1, p2, q1) * orientation(p1, p2, q2) < 0
                    and orientation(q1, q2, p1) * orientation(q1, q2, p2) < 0)

        if crosses(points[0], points[1], points[2], points[3]):
            return False
        if crosses(points[1], points[2], points[3], points[0]):
            return False
        twice_area = 0
        for current, following in zip(points, points[1:] + points[:1]):
            twice_area += current[0] * following[1] - following[0] * current[1]
        return abs(twice_area) >= 20

    def on_create_area_points(self):
        """Build a D-origin serpentine grid and publish it to the common planner."""
        if self._scan_sequence_active or self._programmatic_motion_active:
            return
        self._scan_cancel_requested = False
        missing = [name for name in ("A", "B", "C", "D")
                   if name not in self._area_corners]
        if missing:
            QMessageBox.warning(
                self, "Area Scan", "Eksik köşeler: " + ", ".join(missing)
            )
            return
        total_points = (int(self.ui.areaXDiv.value()) + 1) * (
            int(self.ui.areaYDiv.value()) + 1
        )
        if total_points > 2500:
            QMessageBox.warning(
                self, "Area Scan",
                "En fazla 2500 alan noktası oluşturulabilir. X/Y bölme sayılarını azaltın.",
            )
            return
        if not self._area_pixel_geometry_valid(self._area_corners):
            QMessageBox.warning(
                self, "Area Scan",
                "Köşe işaretleri farklı ve A-B-C-D sırasında geçerli bir alan oluşturmalıdır.",
            )
            return

        units = {data["unit"] for data in self._area_corners.values() if data["unit"]}
        if len(units) > 1:
            QMessageBox.warning(self, "Area Scan", "Köşe mesafelerinin birimleri aynı değil.")
            return

        # Normal seçim A→B→C→D şeklindedir ve tarama D'den başlar.
        # D seçildikten sonra manuel hareket olduysa fiziksel başlangıç bilinmez.
        d_corner = self._area_corners["D"]
        if (int(self._steps_x_abs) != d_corner["steps_x"]
                or int(self._steps_y_abs) != d_corner["steps_y"]):
            QMessageBox.warning(
                self, "Area Scan",
                "Tarama D köşesinden başlar. Motoru D noktasına getirip D köşesini yeniden seçin.",
            )
            return

        deg_per_step_x = (float(self.STEP_ANGLE_DEG_X)
                          / float(self.MICROSTEP_DIV_X)
                          / float(self.GEAR_RATIO_X))
        deg_per_step_y = (float(self.STEP_ANGLE_DEG_Y)
                          / float(self.MICROSTEP_DIV_Y)
                          / float(self.GEAR_RATIO_Y))
        a_corner = self._area_corners["A"]
        corner_dpy = {}
        for name in ("A", "B", "C", "D"):
            data = self._area_corners[name]
            yaw = (data["steps_x"] - a_corner["steps_x"]) * deg_per_step_x
            pitch = (data["steps_y"] - a_corner["steps_y"]) * deg_per_step_y
            corner_dpy[name] = (data["distance"], pitch, yaw)

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
        try:
            result = plan_grid_path(
                corners=corner_dpy,
                x_segments=int(self.ui.areaXDiv.value()),
                y_segments=int(self.ui.areaYDiv.value()),
                stepper_pitch=cfg_y,
                stepper_yaw=cfg_x,
            )
        except Exception as exc:
            QMessageBox.critical(self, "Area Scan", f"Alan planı oluşturulamadı: {exc}")
            return

        dpy = list(result.get("dpy", []))
        grid_indices = list(result.get("grid_indices", []))
        if not dpy or len(dpy) != len(grid_indices):
            QMessageBox.critical(self, "Area Scan", "Alan planı hedef listesi geçersiz.")
            return

        self.table.setRowCount(0)
        self.label.markers.clear()
        pixels = {name: self._area_corners[name]["pixel"]
                  for name in ("A", "B", "C", "D")}
        x_segments = int(self.ui.areaXDiv.value())
        y_segments = int(self.ui.areaYDiv.value())

        for row, ((_, pitch, yaw), (ix, iy)) in enumerate(zip(dpy, grid_indices)):
            pitch_item = QTableWidgetItem(f"{pitch:.4f}")
            yaw_item = QTableWidgetItem(f"{yaw:.4f}")
            pitch_item.setData(Qt.UserRole, "grid")
            yaw_item.setData(Qt.UserRole, "grid")
            self.table.insertRow(row)
            self.table.setItem(row, 0, pitch_item)
            self.table.setItem(row, 1, yaw_item)
            btn = self._make_delete_btn(framed=True)
            cell_widget = QWidget()
            layout = QHBoxLayout(cell_widget)
            layout.addWidget(btn)
            layout.setAlignment(Qt.AlignCenter)
            layout.setContentsMargins(0, 0, 0, 0)
            self.table.setCellWidget(row, 2, cell_widget)
            btn.clicked.connect(self.delete_row_by_button)

            u = ix / float(x_segments)
            v = iy / float(y_segments)
            bottom_x = pixels["D"][0] + u * (pixels["C"][0] - pixels["D"][0])
            bottom_y = pixels["D"][1] + u * (pixels["C"][1] - pixels["D"][1])
            top_x = pixels["A"][0] + u * (pixels["B"][0] - pixels["A"][0])
            top_y = pixels["A"][1] + u * (pixels["B"][1] - pixels["A"][1])
            marker_x = int(round(bottom_x + v * (top_x - bottom_x)))
            marker_y = int(round(bottom_y + v * (top_y - bottom_y)))
            self.label.add_marker(row, marker_x, marker_y)

        self.label.area_points.clear()
        self.label.first_point = None
        self.label.last_point = None
        self.label.update()
        self._planner_result = result
        self._set_current_point("grid", 0)
        _, pitch, yaw = dpy[0]
        self._set_log_target(0, pitch, yaw)
        QMessageBox.information(
            self, "Area Scan",
            f"{len(dpy)} alan noktası oluşturuldu. Tarama D köşesinden başlayacak.",
        )

    def _on_motorX_step(self, delta: int):
        """
        MotorX (Yaw) için step callback.
        delta işaretine göre mutlak step sayacını ve (gerekirse) seçim sayaçlarını günceller.
        """
        steps = int(delta)
        if steps == 0:
            return
        # Worker owns the authoritative pulse counter. Reading it here keeps
        # delayed/batched Qt signals from making the UI-side position drift.
        self._steps_x_abs = int(self.motorX.position_steps())

        if not self._track_steps:
            return

        if steps > 0:
            self._sx_right += steps
        else:
            self._sx_left += -steps

    def _on_motorY_step(self, delta: int):
        """
        MotorY (Pitch) için step callback.
        """
        steps = int(delta)
        if steps == 0:
            return
        self._steps_y_abs = int(self.motorY.position_steps())

        if not self._track_steps:
            return

        if steps > 0:
            self._sy_up += steps
        else:
            self._sy_down += -steps

    def _on_motor_timing(self, axis: str, count: int,
                         mean_period_ms: float, max_jitter_ms: float):
        effective_sps = 1000.0 / mean_period_ms if mean_period_ms > 0 else 0.0
        print(
            f"[timing-{axis}] intervals={count} "
            f"mean_period={mean_period_ms:.3f} ms "
            f"effective={effective_sps:.1f} step/s "
            f"max_jitter={max_jitter_ms:.3f} ms"
        )
        sys.stdout.flush()

    def on_pb_store_point(self):
        """
        pbStorePoint:
        1) Tabloda hiç angle satırı yoksa:
           - Bu konumu referans (ilk nokta) olarak alır.
           - Pitch=0, Yaw=0 yazar.
        2) En az bir angle satırı varsa:
           - Mevcut referansa göre Pitch/Yaw hesaplar ve yeni satır ekler.
        3) Satır tipini 'angle' olarak işaretler (Next/Scan Points buna göre çalışır).
        """
        if self._scan_sequence_active or self._programmatic_motion_active:
            return
        if not self._position_capture_ready("Store Point"):
            return
        self._scan_cancel_requested = False
        table = self.ui.table

        # Mevcut mutlak step değerleri
        sx = self._steps_x_abs   # MotorX → Yaw
        sy = self._steps_y_abs   # MotorY → Pitch

        angle_rows = self._get_angle_rows()
        if not angle_rows and table.rowCount() > 0:
            # Sequential/Area hedefleri ile Store Point satırlarını aynı
            # tabloda karıştırma; Store Point yeni bir açı serisi başlatır.
            table.setRowCount(0)
            self.label.markers.clear()
            self.label.first_point = None
            self.label.last_point = None
            self.label.update()
            self._planner_result = None
            self._reset_sequential_selection_labels()
            self._reset_area_selection()
            angle_rows = []
        is_new_series = not angle_rows

        # --- 1) Hiç angle satırı yoksa: ilk nokta (referans) ---
        if is_new_series:
            # Bu çağrıyı "ilk nokta" olarak kabul et
            self._origin_steps_x = sx
            self._origin_steps_y = sy

            pitch_deg = 0.0
            yaw_deg = 0.0

        else:
            if self._origin_steps_x is None or self._origin_steps_y is None:
                QMessageBox.warning(
                    self,
                    "Referans Noktası",
                    "Açı satırları var fakat motor referansı geçersiz. "
                    "Yeni seri için önce mevcut açı satırlarını silin.",
                )
                return

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
        if is_new_series:
            self._origin_angle_row = row

        pitch_item = QTableWidgetItem(f"{pitch_deg:.4f}")
        yaw_item   = QTableWidgetItem(f"{yaw_deg:.4f}")

        # Bu satırın "açı satırı" olduğunu işaretle
        pitch_item.setData(Qt.UserRole, "stored_angle")
        yaw_item.setData(Qt.UserRole, "stored_angle")

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
        self._set_current_point("stored_angle", row)


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

        try:
            # Dosya "w" modunda açıldığı için her yeni log başlıkla başlar.
            f.write("row,pitch_deg,yaw_deg,distance,unit,timestamp\n")
            f.flush()
        except Exception as e:
            try:
                f.close()
            except Exception:
                pass
            QMessageBox.critical(self, "Log", f"Dosya başlatılamadı:\n{e}")
            return

        self._log_file = f
        self._log_path = path
        self._logging_enabled = True

        QMessageBox.information(self, "Log", f"Loglama başlatıldı:\n{path}")

    def _close_log_file(self):
        """Close the active log exactly once, including after write errors."""
        self._logging_enabled = False
        log_file = self._log_file
        self._log_file = None
        self._log_path = None
        if not log_file:
            return
        try:
            log_file.flush()
        except Exception:
            pass
        try:
            log_file.close()
        except Exception:
            pass

    def on_pb_stop_logging(self):
        """
        Stop Log:
        - Loglamayı kapatır, dosyayı flush + close yapar.
        """
        if not self._logging_enabled and not self._log_file:
            return

        self._close_log_file()

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

        # Planlı hedeflerde planner açıları kullanılır. Store Point
        # satırlarında override None kalır
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

        # Milisaniye hassasiyetli zaman: "2026-07-22 20:39:28.123"
        ts = datetime.now().isoformat(sep=" ", timespec="milliseconds")

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
            self._close_log_file()
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
        if self._scan_sequence_active:
            return
        self._scan_cancel_requested = False
        if not self.label.pixmap():
            return

        if not self.label.first_point or not self.label.last_point:
            QMessageBox.information(self, "Bilgi", "Önce 'Select First Point' ve 'Select Last Point' ile iki nokta seçin.")
            return

        divisions = 1
        if self.ui.countSpin:
            try:
                divisions = int(self.ui.countSpin.value())
            except Exception:
                pass
        divisions = max(1, divisions)
        point_count = divisions + 1

        # --- Görsel kısmı (UI tablo ve marker'lar) ---
        (x1, y1) = self.label.first_point
        (x2, y2) = self.label.last_point

        self.table.setRowCount(0)
        self.label.markers.clear()

        for i in range(point_count):
            t = i / divisions
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

        if self._seq_first_steps is None or self._seq_last_steps is None:
            QMessageBox.warning(
                self,
                "Sequential Scan",
                "İlk ve son motor konumları geçerli değil. Noktaları yeniden seçin.",
            )
            return
        dx_steps = self._seq_last_steps[0] - self._seq_first_steps[0]
        dy_steps = self._seq_last_steps[1] - self._seq_first_steps[1]
        dYaw_deg = dx_steps * deg_per_step_x
        dPitch_deg = dy_steps * deg_per_step_y

        # D1/D2 yalnızca seçim anında alınmış geçerli Dimetix ölçümleridir.
        if self._first_distance is None or self._last_distance_at_select is None:
            QMessageBox.warning(
                self,
                "Sequential Scan",
                "İlk ve son nokta için geçerli mesafe ölçümü bulunmuyor. "
                "Lazeri açıp iki noktayı yeniden seçin.",
            )
            return
        D1 = float(self._first_distance)
        D2 = float(self._last_distance_at_select)

        print("\n=== plan_laser_path GİRDİ ===")
        print(f"D1={D1:.3f}, Pitch_1=0.000, Yaw_1=0.000")
        print(f"D2={D2:.3f}, Pitch_2={dPitch_deg:.4f}, Yaw_2={dYaw_deg:.4f}")
        print(f"Divisions={divisions}, Generated Points={point_count}")
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
                N=int(divisions),
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

        if len(dpy) != self.table.rowCount():
            self.table.setRowCount(0)
            self.label.markers.clear()
            self.label.update()
            QMessageBox.critical(
                self,
                "Planlama Hatası",
                "Planner hedef sayısı tablo satır sayısıyla eşleşmiyor.",
            )
            return

        # Fotoğraf marker'ları piksel konumlarını ayrı listede tutar. Tabloda
        # ise kullanıcının göreceği gerçek hedef Pitch/Yaw açıları bulunur.
        for row, (_, pitch, yaw) in enumerate(dpy):
            pitch_item = QTableWidgetItem(f"{pitch:.4f}")
            yaw_item = QTableWidgetItem(f"{yaw:.4f}")
            pitch_item.setData(Qt.UserRole, "sequential")
            yaw_item.setData(Qt.UserRole, "sequential")
            self.table.setItem(row, 0, pitch_item)
            self.table.setItem(row, 1, yaw_item)

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
        self._set_current_point("sequential", len(dpy) - 1)

        QMessageBox.information(self, "Planlama",
            "plan_laser_path tamamlandı.\n"
            "XYZ ve D/Pitch/Yaw noktaları ile step deltaları terminale yazdırıldı."
        )

    def on_clicked_next_point_btn(self):
        """Aktif planın tarama yönündeki bir sonraki bilinen hedefe git."""
        if self._scan_sequence_active:
            return
        self._scan_cancel_requested = False
        angle_rows = self._get_angle_rows()

        if angle_rows:
            if len(angle_rows) <= 1:
                QMessageBox.information(self, "Bilgi", "En az 2 açı noktası kaydetmelisiniz.")
                return
            if (self._current_point_kind != "stored_angle"
                    or self._current_point_row not in angle_rows):
                QMessageBox.warning(
                    self, "Konum Bilinmiyor",
                    "Motor bilinen bir Store Point hedefinde değil. "
                    "Oklarla hedefe gidip Store Point ile konumu yeniden kaydedin.",
                )
                return
            current_index = angle_rows.index(self._current_point_row)
            if current_index == 0:
                QMessageBox.information(self, "Bilgi", "İlk açı noktasındasınız.")
                return
            row = angle_rows[current_index - 1]
            print(f"[NextPoint-angle] current={self._current_point_row} target={row}")
            sys.stdout.flush()
            ok = self._goto_angle_row(row, wait_s=0.0)
            if ok:
                print(f"[NextPoint-angle] row {row} tamamlandı.")
                sys.stdout.flush()
            else:
                self._mark_position_unknown()
                if not self._scan_cancel_requested:
                    QMessageBox.critical(self, "Tarama Hatası", "Hareket tamamlanamadı.")
            return

        if not self._planner_result:
            QMessageBox.warning(
                self, "Plan yok",
                "Önce Sequential veya Area Scan noktalarını oluşturun.",
            )
            return

        pitch_d = list(self._planner_result.get("pitch_steps_delta", []))
        yaw_d = list(self._planner_result.get("yaw_steps_delta", []))
        dpy = list(self._planner_result.get("dpy", []))
        plan_type = str(self._planner_result.get("plan_type", "sequential"))
        scan_step = int(self._planner_result.get("scan_step", -1))
        if scan_step not in (-1, 1):
            QMessageBox.critical(self, "Hata", "Planner tarama yönü geçersiz.")
            return
        if not pitch_d or not yaw_d or len(pitch_d) != len(yaw_d):
            QMessageBox.critical(self, "Hata", "Planner step delta boyutları uyumsuz.")
            return

        total_seg = len(pitch_d)
        if len(dpy) != total_seg + 1:
            QMessageBox.critical(self, "Hata", "Planner hedef açı listesi uyumsuz.")
            return

        if (self._current_point_kind != plan_type
                or not isinstance(self._current_point_row, int)
                or not 0 <= self._current_point_row <= total_seg):
            QMessageBox.warning(
                self, "Konum Bilinmiyor",
                "Motor bu planın bilinen bir hedefinde değil. Noktaları yeniden oluşturun.",
            )
            return

        end_row = 0 if scan_step < 0 else total_seg
        if self._current_point_row == end_row:
            QMessageBox.information(self, "Bilgi", "Planın son tarama noktasındasınız.")
            return

        current_row = self._current_point_row
        target_row = current_row + scan_step
        move_yaw, move_pitch, segment = planned_move_steps(
            pitch_d, yaw_d, current_row, target_row
        )

        print(
            f"[NextPoint-{plan_type}] {current_row}→{target_row} "
            f"dYaw_steps={move_yaw:+d} dPitch_steps={move_pitch:+d}"
        )
        sys.stdout.flush()

        self._clear_log_target()
        if not self._move_both_signed_and_wait(
            move_yaw, move_pitch, timeout_ms=300000
        ):
            self._mark_position_unknown()
            if not self._scan_cancel_requested:
                QMessageBox.critical(
                    self, "Tarama Hatası", f"Segment {segment} tamamlanamadı."
                )
            return

        _, target_pitch, target_yaw = dpy[target_row]
        self._set_log_target(target_row, target_pitch, target_yaw)
        self._set_current_point(plan_type, target_row)


    def on_scan_points_clicked(self):
        """
        ÜÇ MOD:
        1) Eğer tabloda pvStorePoint ile kaydedilmiş 'angle' satırları varsa:
           - Bunların tamamına sırayla gider (row0 → row1 → ...).
        2) Sequential planner sonucunu Last→First tarar.
        3) Area planner sonucunu D'den başlayarak serpantin sırada tarar.
        """
        if self._scan_sequence_active:
            return
        self._scan_cancel_requested = False
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

            if (self._current_point_kind != "stored_angle"
                    or self._current_point_row not in angle_rows):
                QMessageBox.warning(
                    self, "Konum Bilinmiyor",
                    "Otomatik tarama için motor bilinen bir Store Point hedefinde olmalıdır.",
                )
                return
            if self.motorX.is_busy() or self.motorY.is_busy():
                QMessageBox.warning(self, "Motor Meşgul", "Motor hareketi devam ediyor.")
                return
            current_index = angle_rows.index(self._current_point_row)
            scan_rows = list(reversed(angle_rows[:current_index + 1]))

            print("\n=== AÇISAL SCAN BAŞLIYOR (pvStorePoint noktaları) ===")
            print(f"Kalan nokta: {len(scan_rows)}, yön=Last→First, interval={wait_s:.3f} s")
            sys.stdout.flush()

            self._scan_sequence_active = True
            try:
                for idx, row in enumerate(scan_rows):
                    print(f"[angle scan] {idx+1}/{len(scan_rows)}  row={row}")
                    sys.stdout.flush()
                    ok = self._goto_angle_row(row, wait_s=wait_s)
                    if not ok:
                        self._mark_position_unknown()
                        if not self._scan_cancel_requested:
                            QMessageBox.critical(self, "Tarama Hatası",
                                                 f"Row {row} noktasına giderken hata oluştu.")
                        return
            finally:
                self._scan_sequence_active = False


            self._clear_log_target()
            print("=== AÇISAL SCAN BİTTİ ===\n")
            sys.stdout.flush()
            QMessageBox.information(self, "Scanning", "Açısal noktalar için tarama tamamlandı.")
            return

        if not self._planner_result:
            QMessageBox.warning(
                self, "Plan yok",
                "Önce Sequential veya Area Scan noktalarını oluşturun.",
            )
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
            yaw_d = list(self._planner_result.get("yaw_steps_delta", []))
            dpy = list(self._planner_result.get("dpy", []))
            plan_type = str(self._planner_result.get("plan_type", "sequential"))
            scan_step = int(self._planner_result.get("scan_step", -1))
            if scan_step not in (-1, 1):
                raise RuntimeError("Planner tarama yönü geçersiz.")
            if not pitch_d or not yaw_d or len(pitch_d) != len(yaw_d):
                raise RuntimeError("Planner step delta boyutları uyumsuz.")

            total_seg = len(pitch_d)
            if len(dpy) != total_seg + 1:
                raise RuntimeError("Planner hedef açı listesi uyumsuz.")

            if (self._current_point_kind != plan_type
                    or not isinstance(self._current_point_row, int)
                    or not 0 <= self._current_point_row <= total_seg):
                raise RuntimeError(
                    "Motor bu planın bilinen bir hedefinde değil. "
                    "Plan noktalarını yeniden oluşturun."
                )
            if self.motorX.is_busy() or self.motorY.is_busy():
                raise RuntimeError("Motor hareketi devam ediyor.")
            start_row = self._current_point_row
            end_row = 0 if scan_step < 0 else total_seg
            self._scan_sequence_active = True

            direction_text = "Last→First" if scan_step < 0 else "D→Serpentine End"
            print(f"\n=== {plan_type.upper()} TARAMA BAŞLIYOR ({direction_text}) ===")
            print(f"Başlangıç satırı: {start_row}, bitiş satırı: {end_row}")
            print(f"İlk hareketten önce bekleme (idle + {wait_s:.3f}s): {wait_s:.3f} s")
            sys.stdout.flush()

            _, start_pitch, start_yaw = dpy[start_row]
            self._set_log_target(start_row, start_pitch, start_yaw)

            # 0) İlk girişte bekle: busy ise önce idle, sonra wait_s
            if wait_s > 0:
                if not self._wait_both_idle(timeout_ms=120000):
                    raise RuntimeError("Başlangıçta idle beklerken zaman aşımı.")
                print(f"[idle] başlangıç idle tamam. {wait_s:.3f}s bekleniyor...")
                sys.stdout.flush()
                if not self._wait_seconds(wait_s):
                    self._scan_sequence_active = False
                    self._clear_log_target()
                    return

            cum_y = 0
            cum_p = 0
            current_row = start_row
            while current_row != end_row:
                target_row = current_row + scan_step
                move_yaw, move_pitch, segment = planned_move_steps(
                    pitch_d, yaw_d, current_row, target_row
                )

                self._clear_log_target()
                print(
                    f"[seg {segment:02d}] {current_row}→{target_row} "
                    f"dYaw_steps={move_yaw:+d} dPitch_steps={move_pitch:+d}"
                )
                sys.stdout.flush()

                if not self._move_both_signed_and_wait(
                    move_yaw, move_pitch, timeout_ms=300000
                ):
                    if self._scan_cancel_requested:
                        self._scan_sequence_active = False
                        self._clear_log_target()
                        return
                    raise RuntimeError(f"Segment {segment} hareketi tamamlanamadı.")

                cum_y += move_yaw
                cum_p += move_pitch
                print(f"            cumulative  yaw={cum_y:+d}  pitch={cum_p:+d}")
                print("            [idle] iki motor da idle.")
                sys.stdout.flush()

                current_row = target_row
                _, target_pitch, target_yaw = dpy[current_row]
                self._set_log_target(current_row, target_pitch, target_yaw)
                self._set_current_point(plan_type, current_row)

                # Segmentler arası bekleme, sadece idle olduktan sonra başlar
                if wait_s > 0:
                    print(f"            [wait] {wait_s:.3f}s bekleniyor...")
                    sys.stdout.flush()
                    if not self._wait_seconds(wait_s):
                        self._scan_sequence_active = False
                        self._clear_log_target()
                        return

            self._clear_log_target()
            self._scan_sequence_active = False
            print(f"=== {plan_type.upper()} TARAMA BİTTİ ===\n")
            sys.stdout.flush()
            QMessageBox.information(self, "Scanning", "Scanning is complete.")

        except Exception as e:
            self._scan_sequence_active = False
            self._mark_position_unknown()
            if not self._scan_cancel_requested:
                QMessageBox.critical(self, "Tarama Hatası", str(e))

    # ---------- Hareket / zaman yardımcıları ----------
    def _move_both_signed_and_wait(self, x_steps: int, y_steps: int,
                                   timeout_ms: int = 120000) -> bool:
        """İki hareket kimliğinin de başarıyla tamamlanmasını bekle."""
        if self._programmatic_motion_active:
            return False
        self._programmatic_motion_active = True
        x_results = {}
        y_results = {}

        def on_x_finished(move_id: int, completed: bool):
            x_results[int(move_id)] = bool(completed)

        def on_y_finished(move_id: int, completed: bool):
            y_results[int(move_id)] = bool(completed)

        self.motorX.moveFinished.connect(on_x_finished)
        self.motorY.moveFinished.connect(on_y_finished)
        try:
            x_id = self.motorX.move_signed_steps(int(x_steps))
            y_id = self.motorY.move_signed_steps(int(y_steps))

            if x_id == 0:
                x_results[0] = True
            if y_id == 0:
                y_results[0] = True

            deadline = time.monotonic() + max(1, int(timeout_ms)) / 1000.0
            while time.monotonic() < deadline:
                QApplication.processEvents()
                x_done = x_id in x_results
                y_done = y_id in y_results
                if (x_done and not x_results[x_id]) or (y_done and not y_results[y_id]):
                    self._mark_position_unknown()
                    self.motorX.emergency_stop()
                    self.motorY.emergency_stop()
                    return False
                if x_done and y_done:
                    completed = x_results[x_id] and y_results[y_id]
                    if completed:
                        self._sync_absolute_steps()
                    return completed
                time.sleep(0.005)

            self._mark_position_unknown()
            self.motorX.emergency_stop()
            self.motorY.emergency_stop()
            return False
        except Exception:
            self._mark_position_unknown()
            self.motorX.emergency_stop()
            self.motorY.emergency_stop()
            return False
        finally:
            self._programmatic_motion_active = False
            try:
                self.motorX.moveFinished.disconnect(on_x_finished)
            except Exception:
                pass
            try:
                self.motorY.moveFinished.disconnect(on_y_finished)
            except Exception:
                pass

    def _wait_seconds(self, seconds: float):
        """UI’yi dondurmadan bekle."""
        try:
            seconds = float(seconds)
        except Exception:
            seconds = 0.0
        if seconds <= 0:
            return not self._scan_cancel_requested
        import time as _t
        end = _t.monotonic() + seconds
        while _t.monotonic() < end:
            QApplication.processEvents()
            if self._scan_cancel_requested:
                return False
            _t.sleep(0.01)
        return True

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
        self._scan_cancel_requested = True
        self._release_motion_keys()
        app = QApplication.instance()
        if app:
            app.removeEventFilter(self)
        self._safe(lambda: self.motorX and self.motorX.emergency_stop())
        self._safe(lambda: self.motorY and self.motorY.emergency_stop())
        self._safe(self._close_log_file)
        self._safe(lambda: self.cam and self.cam.stop())
        self._safe(lambda: self.laser and self.laser.set_enabled(False))
        self._safe(lambda: self.dim_worker and self.dim_worker.set_mode_off())
        self._safe(lambda: self._stop_dimetix_worker(timeout_ms=5000))
        self._safe(lambda: self.ori_worker and self.ori_worker.stop())
        self._safe(lambda: self.ori_worker and self.ori_worker.wait(3000))
        self._safe(lambda: self.motorX and self.motorX.shutdown())
        self._safe(lambda: self.motorY and self.motorY.shutdown())
        self._safe(lambda: self.laser and self.laser.release())
        self._safe(lambda: self.shared and self.shared.set_enable(False))

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
