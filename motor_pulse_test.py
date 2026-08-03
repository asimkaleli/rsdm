"""Standalone pulse-return diagnostic for the RSDM motor wiring.

Close rsdm.py before running this script. It intentionally moves real hardware.
The CSV proves what the software emitted; physical return must be checked with
the laser spot, a dial indicator, or another external reference.
"""

import argparse
import csv
from datetime import datetime
from pathlib import Path
import sys
import time

from PySide2.QtCore import QCoreApplication

from laser_gpio import LaserGPIO
from motor_control import MotorController, MotorPins, SharedPins


AXES = {
    "x": {
        "pins": MotorPins(step=13, dir=6),
        "positive_name": "left",
        "negative_name": "right",
    },
    "y": {
        "pins": MotorPins(step=12, dir=5, dir_inverted=True),
        "positive_name": "up",
        "negative_name": "down",
    },
}


def timestamp_ms() -> str:
    return datetime.now().isoformat(sep=" ", timespec="milliseconds")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Send a fixed pulse count in one direction and the same count "
            "back while recording software pulse/timing data."
        )
    )
    parser.add_argument(
        "--axis", choices=("both", "x", "y"), default="both",
        help="Test both axes by default, or select a single axis.",
    )
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument("--speed-sps", type=float, default=100.0)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--timeout-seconds", type=float, default=180.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--confirm-motion",
        action="store_true",
        help="Required acknowledgement that the mechanism is clear to move.",
    )
    return parser.parse_args()


def wait_for_move(app, results, result_key, timeout_seconds: float) -> bool:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        app.processEvents()
        if result_key in results:
            return bool(results[result_key])
        time.sleep(0.002)
    return False


def main() -> int:
    args = parse_args()
    if not args.confirm_motion:
        print(
            "HAREKET ENGELLENDI: Alanin guvenli oldugunu kontrol edip "
            "--confirm-motion parametresiyle tekrar calistirin."
        )
        return 2
    if args.steps <= 0 or args.cycles <= 0:
        print("--steps ve --cycles sifirdan buyuk olmalidir.")
        return 2
    if not 1.0 <= args.speed_sps <= 1000.0:
        print("--speed-sps 1 ile 1000 arasinda olmalidir.")
        return 2

    output = args.output or Path(
        f"pulse_test_{args.axis}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    )
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    selected_axes = ("x", "y") if args.axis == "both" else (args.axis,)
    app = QCoreApplication.instance() or QCoreApplication(sys.argv[:1])
    shared = None
    motors = {}
    laser = None
    rows = []
    move_results = {}
    active_state = {"axis": selected_axes[0], "phase": "setup"}

    def record(event, **values):
        axis_name = values.get("axis_name", active_state["axis"])
        event_motor = motors.get(axis_name)
        rows.append({
            "timestamp": timestamp_ms(),
            "event": event,
            "axis": axis_name,
            "phase": active_state["phase"],
            "requested_steps": values.get("requested_steps", ""),
            "step_delta": values.get("step_delta", ""),
            "software_position_steps": (
                event_motor.position_steps() if event_motor is not None else ""
            ),
            "timing_intervals": values.get("timing_intervals", ""),
            "mean_period_ms": values.get("mean_period_ms", ""),
            "max_jitter_ms": values.get("max_jitter_ms", ""),
            "phase_samples": values.get("phase_samples", ""),
            "min_high_ms": values.get("min_high_ms", ""),
            "max_high_ms": values.get("max_high_ms", ""),
            "min_low_ms": values.get("min_low_ms", ""),
            "max_low_ms": values.get("max_low_ms", ""),
            "completed": values.get("completed", ""),
        })

    try:
        shared = SharedPins(
            en=7, reset=8, sleep=25,
            ms1=16, ms2=20, ms3=21,
        )
        for axis_name in selected_axes:
            axis_config = AXES[axis_name]
            motor = MotorController(axis_config["pins"], shared=shared)
            motors[axis_name] = motor
            motor.set_speed_ms(1000.0 / (2.0 * args.speed_sps))
            motor.set_microstep("SIXTEENTH")
            motor.set_step_callback(
                lambda delta, a=axis_name: record(
                    "step_batch", axis_name=a, step_delta=int(delta)
                )
            )
            motor.moveFinished.connect(
                lambda move_id, completed, a=axis_name: move_results.__setitem__(
                    (a, int(move_id)), bool(completed)
                )
            )
            motor.timingReport.connect(
                lambda count, mean_ms, jitter_ms, a=axis_name: record(
                    "timing",
                    axis_name=a,
                    timing_intervals=int(count),
                    mean_period_ms=f"{mean_ms:.6f}",
                    max_jitter_ms=f"{jitter_ms:.6f}",
                )
            )
            motor.pulsePhaseReport.connect(
                lambda count, min_high, max_high, min_low, max_low, a=axis_name: record(
                    "pulse_phases",
                    axis_name=a,
                    phase_samples=int(count),
                    min_high_ms=f"{min_high:.6f}",
                    max_high_ms=f"{max_high:.6f}",
                    min_low_ms=f"{min_low:.6f}",
                    max_low_ms=f"{max_low:.6f}",
                )
            )
        shared.set_enable(True)
        laser = LaserGPIO(line=24)
        laser.set_enabled(False)

        movements = []
        for axis_name in selected_axes:
            axis_config = AXES[axis_name]
            movements.append(
                f"{axis_name.upper()}: {args.steps} {axis_config['positive_name']} "
                f"+ {args.steps} {axis_config['negative_name']}"
            )
        print("UYARI: " + "; ".join(movements))
        print("UYARI: GPIO24 lazeri test boyunca acik kalacak; goz hizasina bakmayin.")
        print(f"Hiz: {args.speed_sps:g} step/s  Log: {output}")
        laser.set_enabled(True)
        record("laser_on")
        print(
            "Lazer acildi. Baslangic noktasini isaretleyin; hareket 3 saniye "
            "sonra baslayacak. Ctrl+C ile iptal edebilirsiniz."
        )
        for remaining in (3, 2, 1):
            print(remaining)
            time.sleep(1.0)

        record("test_start")
        final_positions = {}
        for axis_name in selected_axes:
            axis_config = AXES[axis_name]
            motor = motors[axis_name]
            active_state["axis"] = axis_name
            for cycle in range(1, args.cycles + 1):
                for phase_name, signed_steps in (
                    (f"cycle_{cycle}_{axis_config['positive_name']}", args.steps),
                    (f"cycle_{cycle}_{axis_config['negative_name']}", -args.steps),
                ):
                    active_state["phase"] = phase_name
                    record("move_requested", requested_steps=signed_steps)
                    move_id = motor.move_signed_steps(signed_steps)
                    result_key = (axis_name, move_id)
                    completed = wait_for_move(
                        app, move_results, result_key, args.timeout_seconds
                    )
                    record(
                        "move_finished",
                        requested_steps=signed_steps,
                        completed=completed,
                    )
                    if not completed:
                        motor.emergency_stop()
                        raise RuntimeError(
                            f"Hareket tamamlanamadi: {axis_name}/{phase_name}"
                        )
                    end = time.monotonic() + max(0.0, args.settle_seconds)
                    while time.monotonic() < end:
                        app.processEvents()
                        time.sleep(0.01)

            final_positions[axis_name] = motor.position_steps()
            active_state["phase"] = "axis_complete"
            record(
                "axis_complete",
                completed=(final_positions[axis_name] == 0),
            )

        active_state["phase"] = "complete"
        all_zero = all(position == 0 for position in final_positions.values())
        record("test_complete", completed=all_zero)

        for axis_name, final_position in final_positions.items():
            print(
                f"{axis_name.upper()} yazilim son pulse konumu: "
                f"{final_position} (beklenen: 0)"
            )
        print("Fiziksel isaretin ayni noktaya donup donmedigini kontrol edin.")
        return 0 if all_zero else 1
    except KeyboardInterrupt:
        for motor in motors.values():
            motor.emergency_stop()
        active_state["phase"] = "cancelled"
        record("cancelled")
        print("Test kullanici tarafindan durduruldu.")
        return 130
    except Exception as exc:
        for motor in motors.values():
            motor.emergency_stop()
        active_state["phase"] = "error"
        record("error", completed=False)
        print(f"Test hatasi: {exc}")
        return 1
    finally:
        if laser is not None:
            try:
                laser.set_enabled(False)
                record("laser_off")
            except Exception as exc:
                print(f"Lazer kapatma hatasi: {exc}")
            try:
                laser.release()
            except Exception:
                pass
        for motor in motors.values():
            motor.shutdown()
        if rows:
            fieldnames = list(rows[0])
            with output.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            print(f"Log kaydedildi: {output}")


if __name__ == "__main__":
    raise SystemExit(main())
