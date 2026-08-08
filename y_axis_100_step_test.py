"""X veya Y eksenini sabit 100 pulse/manual modda güvenle test eder.

Bu araç gerçek donanımı hareket ettirir. Ana RSDM uygulamasını kapatın,
mekanik alanı boşaltın ve yalnız hedef Raspberry Pi üzerinde çalıştırın.
"""

import argparse
import sys

from PySide2.QtCore import Qt
from PySide2.QtWidgets import (
    QApplication,
    QComboBox,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from laser_gpio import LaserGPIO
from motor_control import MotorController, MotorPins, SharedPins


MOVE_STEPS = 100
SPEED_SPS = 100.0


class YAxisStepTest(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RSDM X/Y Axis - 100 Step Test")
        self.setMinimumWidth(360)
        self._closing = False
        self._active_move_id = None
        self._active_axis = None
        self._manual_active_direction = None
        self.shared = None
        self.motors = {}
        self.laser = None

        self.status_label = QLabel(
            "Hazır — Up: +100 step, Down: -100 step"
        )
        self.status_label.setAlignment(Qt.AlignCenter)

        self.axis_combo = QComboBox()
        self.axis_combo.addItem("Y ekseni — STEP 13 / DIR 6", "y")
        self.axis_combo.addItem("X ekseni — STEP 12 / DIR 5", "x")

        self.position_label = QLabel("Yazılımsal Y konumu: 0 step")
        self.position_label.setAlignment(Qt.AlignCenter)
        self.laser_label = QLabel("LAZER AÇIK — göz hizasına yöneltmeyin")
        self.laser_label.setAlignment(Qt.AlignCenter)
        self.laser_label.setStyleSheet(
            "font-weight: bold; color: #b00020;"
        )

        self.up_button = QPushButton("UP  (+100 step)")
        self.down_button = QPushButton("DOWN  (-100 step)")
        self.manual_button = QPushButton("MANUAL MODE: OFF")
        self.manual_button.setCheckable(True)
        self.stop_button = QPushButton("ACİL DURDUR")
        self.stop_button.setStyleSheet(
            "font-weight: bold; color: white; background-color: #b00020;"
        )

        layout = QVBoxLayout(self)
        layout.addWidget(self.status_label)
        layout.addWidget(self.axis_combo)
        layout.addWidget(self.position_label)
        layout.addWidget(self.laser_label)
        layout.addWidget(self.up_button)
        layout.addWidget(self.down_button)
        layout.addWidget(self.manual_button)
        layout.addWidget(self.stop_button)

        # RSDM ana uygulamasındaki gerçek pin ve yön ayarları.
        try:
            self.shared = SharedPins(
                en=7, reset=8, sleep=25,
                ms1=16, ms2=20, ms3=21,
            )
            self.motors["y"] = MotorController(
                MotorPins(step=13, dir=6, dir_inverted=True),
                shared=self.shared,
            )
            self.motors["x"] = MotorController(
                MotorPins(step=12, dir=5, dir_inverted=True),
                shared=self.shared,
            )

            # Ana uygulamayla aynı 1/16 microstep modu ve güvenli 100 step/s hız.
            for motor in self.motors.values():
                motor.set_speed_ms(1000.0 / (2.0 * SPEED_SPS))
            self.motors["y"].set_microstep("SIXTEENTH")
            self.shared.set_enable(True)

            # Lazer ancak motor kurulumu tamamlandıktan sonra açılır.
            self.laser = LaserGPIO(line=24)
            self.laser.set_enabled(False)
            self.laser.set_enabled(True)
        except Exception:
            self._shutdown_hardware()
            raise

        self.up_button.pressed.connect(
            lambda: self._direction_pressed(+1, "UP")
        )
        self.up_button.released.connect(
            lambda: self._direction_released(+1, "UP")
        )
        self.down_button.pressed.connect(
            lambda: self._direction_pressed(-1, "DOWN")
        )
        self.down_button.released.connect(
            lambda: self._direction_released(-1, "DOWN")
        )
        self.manual_button.toggled.connect(self._manual_mode_changed)
        self.axis_combo.currentIndexChanged.connect(self._axis_changed)
        self.stop_button.clicked.connect(self._emergency_stop)
        for axis, motor in self.motors.items():
            motor.moveFinished.connect(
                lambda move_id, completed, a=axis:
                self._on_move_finished(a, move_id, completed)
            )
            motor.busyChanged.connect(
                lambda busy, a=axis: self._on_busy_changed(a, busy)
            )
            motor.error.connect(
                lambda message, a=axis: self._on_motor_error(a, message)
            )
            motor.set_step_callback(
                lambda delta, a=axis: self._on_step_update(a, delta)
            )

    @property
    def selected_axis(self):
        return str(self.axis_combo.currentData())

    @property
    def motor(self):
        return self.motors[self.selected_axis]

    def _set_move_buttons_enabled(self, enabled: bool):
        self.up_button.setEnabled(enabled)
        self.down_button.setEnabled(enabled)
        self.manual_button.setEnabled(enabled)
        self.axis_combo.setEnabled(enabled)

    def _axis_changed(self, _index: int):
        axis = self.selected_axis.upper()
        position = self.motor.position_steps()
        self.position_label.setText(
            f"Yazılımsal {axis} konumu: {position} step"
        )
        self.status_label.setText(f"{axis} ekseni seçildi")

    def _manual_mode_changed(self, checked: bool):
        if checked:
            self.manual_button.setText("MANUAL MODE: ON")
            self.up_button.setText("UP  (basılı tut)")
            self.down_button.setText("DOWN  (basılı tut)")
            self.status_label.setText(
                "Manuel mod — hareket için yön butonunu basılı tutun"
            )
        else:
            self.manual_button.setText("MANUAL MODE: OFF")
            self.up_button.setText("UP  (+100 step)")
            self.down_button.setText("DOWN  (-100 step)")
            self.status_label.setText(
                "100-step modu — Up: +100 step, Down: -100 step"
            )

    def _direction_pressed(self, direction: int, direction_name: str):
        if self.manual_button.isChecked():
            self._start_manual(direction, direction_name)
        else:
            self._move(direction * MOVE_STEPS, direction_name)

    def _direction_released(self, direction: int, direction_name: str):
        if self._manual_active_direction != direction:
            return
        self._manual_active_direction = None
        self.status_label.setText(f"{direction_name}: manuel hareket durduruluyor...")
        self.motor.stop_continuous()

    def _start_manual(self, direction: int, direction_name: str):
        if (self._closing or self.motor.is_busy()
                or self._manual_active_direction is not None):
            return
        self._manual_active_direction = int(direction)
        self._active_axis = self.selected_axis
        self.manual_button.setEnabled(False)
        if direction > 0:
            self.down_button.setEnabled(False)
        else:
            self.up_button.setEnabled(False)
        self.status_label.setText(
            f"{direction_name}: manuel hareket — bırakınca durur"
        )
        print(
            f"[manual-test-{self.selected_axis}] start direction={direction:+d} "
            f"from={self.motor.position_steps()}"
        )
        sys.stdout.flush()
        self.motor.start_continuous(direction)

    def _move(self, signed_steps: int, direction_name: str):
        if self._closing or self.motor.is_busy():
            return
        self._set_move_buttons_enabled(False)
        self.status_label.setText(
            f"{direction_name}: {signed_steps:+d} step hareket ediyor..."
        )
        move_id = self.motor.move_signed_steps(signed_steps)
        if move_id == 0:
            self.status_label.setText("Hareket oluşturulamadı.")
            self._set_move_buttons_enabled(True)
            return
        self._active_move_id = int(move_id)
        self._active_axis = self.selected_axis
        print(
            f"[100-step-test-{self.selected_axis}] start id={move_id} "
            f"command={signed_steps:+d} "
            f"from={self.motor.position_steps()}"
        )
        sys.stdout.flush()

    def _on_step_update(self, axis: str, _delta: int):
        if axis != self.selected_axis:
            return
        self.position_label.setText(
            f"Yazılımsal {axis.upper()} konumu: "
            f"{self.motors[axis].position_steps()} step"
        )

    def _on_move_finished(self, axis: str, move_id: int, completed: bool):
        if axis != self._active_axis:
            return
        if (self._active_move_id is not None
                and int(move_id) != self._active_move_id):
            return
        position = self.motors[axis].position_steps()
        self._active_move_id = None
        self._active_axis = None
        self.position_label.setText(
            f"Yazılımsal {axis.upper()} konumu: {position} step"
        )
        if completed:
            self.status_label.setText(
                f"Tamamlandı — konum: {position} step"
            )
        else:
            self.status_label.setText(
                f"Hareket iptal edildi — konum: {position} step"
            )
        print(
            f"[100-step-test-{axis}] finish id={move_id} "
            f"completed={bool(completed)} position={position}"
        )
        sys.stdout.flush()
        if not self._closing:
            self._set_move_buttons_enabled(True)

    def _on_busy_changed(self, axis: str, busy: bool):
        if self._active_axis is not None and axis != self._active_axis:
            return
        if busy or self._closing or self._manual_active_direction is not None:
            return
        position = self.motors[axis].position_steps()
        self._active_axis = None
        self.position_label.setText(
            f"Yazılımsal {axis.upper()} konumu: {position} step"
        )
        if self.manual_button.isChecked():
            self.status_label.setText(
                f"Manuel hareket durdu — konum: {position} step"
            )
            print(f"[manual-test-{axis}] stop position={position}")
            sys.stdout.flush()
        self._set_move_buttons_enabled(True)

    def _emergency_stop(self):
        self._manual_active_direction = None
        self._set_move_buttons_enabled(False)
        self.status_label.setText("Acil durdurma istendi; lazer kapatıldı...")
        self._laser_off()
        for motor in self.motors.values():
            motor.emergency_stop()

    def _on_motor_error(self, axis: str, message: str):
        self._set_move_buttons_enabled(False)
        self.status_label.setText("Motor hatası")
        self._laser_off()
        QMessageBox.critical(
            self, f"{axis.upper()} Motor Hatası", str(message)
        )

    def _laser_off(self):
        if self.laser is None:
            return
        try:
            self.laser.set_enabled(False)
        except Exception:
            pass
        self.laser_label.setText("Lazer kapalı")

    def _shutdown_hardware(self):
        self._laser_off()
        if self.motors:
            for motor in self.motors.values():
                try:
                    motor.emergency_stop()
                except Exception:
                    pass
            for motor in self.motors.values():
                try:
                    motor.shutdown()
                except Exception:
                    pass
        if self.shared is not None:
            try:
                self.shared.set_enable(False)
            except Exception:
                pass
        if self.laser is not None:
            try:
                self.laser.release()
            except Exception:
                pass

    def closeEvent(self, event):
        self._closing = True
        self._set_move_buttons_enabled(False)
        self._shutdown_hardware()
        event.accept()


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Seçilen RSDM X/Y motorunu buton başına tam 100 step hareket "
            "ettirir ve "
            "BCM 24 lazerini test boyunca açık tutar."
        )
    )
    parser.add_argument(
        "--confirm-motion",
        action="store_true",
        help="Mekanik alanın ve lazer yönünün güvenli olduğunu onaylar.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.confirm_motion:
        print(
            "HAREKET ENGELLENDİ: Önce ana RSDM uygulamasını kapatın, "
            "mekanik alanı ve lazer yönünü güvenli hale getirin, ardından "
            "--confirm-motion ekleyin."
        )
        return 2

    app = QApplication(sys.argv[:1])
    try:
        window = YAxisStepTest()
    except Exception as exc:
        print(f"Motor/lazer başlatılamadı: {exc}")
        return 1
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
