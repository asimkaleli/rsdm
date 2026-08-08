"""Y eksenini sabit 100 pulse ile güvenli biçimde ileri/geri test eder.

Bu araç gerçek donanımı hareket ettirir. Ana RSDM uygulamasını kapatın,
mekanik alanı boşaltın ve yalnız hedef Raspberry Pi üzerinde çalıştırın.
"""

import argparse
import sys

from PySide2.QtCore import Qt
from PySide2.QtWidgets import (
    QApplication,
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
        self.setWindowTitle("RSDM Y Axis - 100 Step Test")
        self.setMinimumWidth(360)
        self._closing = False
        self._active_move_id = None
        self.shared = None
        self.motor = None
        self.laser = None

        self.status_label = QLabel(
            "Hazır — Up: +100 step, Down: -100 step"
        )
        self.status_label.setAlignment(Qt.AlignCenter)

        self.position_label = QLabel("Yazılımsal Y konumu: 0 step")
        self.position_label.setAlignment(Qt.AlignCenter)
        self.laser_label = QLabel("LAZER AÇIK — göz hizasına yöneltmeyin")
        self.laser_label.setAlignment(Qt.AlignCenter)
        self.laser_label.setStyleSheet(
            "font-weight: bold; color: #b00020;"
        )

        self.up_button = QPushButton("UP  (+100 step)")
        self.down_button = QPushButton("DOWN  (-100 step)")
        self.stop_button = QPushButton("ACİL DURDUR")
        self.stop_button.setStyleSheet(
            "font-weight: bold; color: white; background-color: #b00020;"
        )

        layout = QVBoxLayout(self)
        layout.addWidget(self.status_label)
        layout.addWidget(self.position_label)
        layout.addWidget(self.laser_label)
        layout.addWidget(self.up_button)
        layout.addWidget(self.down_button)
        layout.addWidget(self.stop_button)

        # RSDM ana uygulamasındaki gerçek pin ve yön ayarları.
        try:
            self.shared = SharedPins(
                en=7, reset=8, sleep=25,
                ms1=16, ms2=20, ms3=21,
            )
            self.motor = MotorController(
                MotorPins(step=12, dir=5, dir_inverted=True),
                shared=self.shared,
            )

            # Ana uygulamayla aynı 1/16 microstep modu ve güvenli 100 step/s hız.
            self.motor.set_microstep("SIXTEENTH")
            self.motor.set_speed_ms(1000.0 / (2.0 * SPEED_SPS))
            self.shared.set_enable(True)

            # Lazer ancak motor kurulumu tamamlandıktan sonra açılır.
            self.laser = LaserGPIO(line=24)
            self.laser.set_enabled(False)
            self.laser.set_enabled(True)
        except Exception:
            self._shutdown_hardware()
            raise

        self.up_button.clicked.connect(lambda: self._move(+MOVE_STEPS, "UP"))
        self.down_button.clicked.connect(
            lambda: self._move(-MOVE_STEPS, "DOWN")
        )
        self.stop_button.clicked.connect(self._emergency_stop)
        self.motor.moveFinished.connect(self._on_move_finished)
        self.motor.error.connect(self._on_motor_error)
        self.motor.set_step_callback(self._on_step_update)

    def _set_move_buttons_enabled(self, enabled: bool):
        self.up_button.setEnabled(enabled)
        self.down_button.setEnabled(enabled)

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
        print(
            f"[100-step-test] start id={move_id} command={signed_steps:+d} "
            f"from={self.motor.position_steps()}"
        )
        sys.stdout.flush()

    def _on_step_update(self, _delta: int):
        self.position_label.setText(
            f"Yazılımsal Y konumu: {self.motor.position_steps()} step"
        )

    def _on_move_finished(self, move_id: int, completed: bool):
        if self._active_move_id is not None and int(move_id) != self._active_move_id:
            return
        position = self.motor.position_steps()
        self._active_move_id = None
        self.position_label.setText(f"Yazılımsal Y konumu: {position} step")
        if completed:
            self.status_label.setText(
                f"Tamamlandı — konum: {position} step"
            )
        else:
            self.status_label.setText(
                f"Hareket iptal edildi — konum: {position} step"
            )
        print(
            f"[100-step-test] finish id={move_id} "
            f"completed={bool(completed)} position={position}"
        )
        sys.stdout.flush()
        if not self._closing:
            self._set_move_buttons_enabled(True)

    def _emergency_stop(self):
        self._set_move_buttons_enabled(False)
        self.status_label.setText("Acil durdurma istendi; lazer kapatıldı...")
        self._laser_off()
        self.motor.emergency_stop()

    def _on_motor_error(self, message: str):
        self._set_move_buttons_enabled(False)
        self.status_label.setText("Motor hatası")
        self._laser_off()
        QMessageBox.critical(self, "Motor Hatası", str(message))

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
        if self.motor is not None:
            try:
                self.motor.emergency_stop()
                self.motor.shutdown()
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
            "RSDM Y motorunu buton başına tam 100 step hareket ettirir ve "
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
