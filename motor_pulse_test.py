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
    parser.add_argument("--axis", choices=sorted(AXES), default="x")
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


def wait_for_move(app, results, move_id: int, timeout_seconds: float) -> bool:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        app.processEvents()
        if move_id in results:
            return bool(results[move_id])
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

    axis = AXES[args.axis]
    app = QCoreApplication.instance() or QCoreApplication(sys.argv[:1])
    shared = None
    motor = None
    rows = []
    move_results = {}
    active_phase = {"name": "setup"}

    def record(event, **values):
        rows.append({
            "timestamp": timestamp_ms(),
            "event": event,
            "axis": args.axis,
            "phase": active_phase["name"],
            "requested_steps": values.get("requested_steps", ""),
            "step_delta": values.get("step_delta", ""),
            "software_position_steps": (
                motor.position_steps() if motor is not None else ""
            ),
            "timing_intervals": values.get("timing_intervals", ""),
            "mean_period_ms": values.get("mean_period_ms", ""),
            "max_jitter_ms": values.get("max_jitter_ms", ""),
            "completed": values.get("completed", ""),
        })

    try:
        shared = SharedPins(
            en=7, reset=8, sleep=25,
            ms1=16, ms2=20, ms3=21,
        )
        motor = MotorController(axis["pins"], shared=shared)
        motor.set_motion_profile(50.0, 400.0)
        motor.set_speed_ms(1000.0 / (2.0 * args.speed_sps))
        motor.set_microstep("SIXTEENTH")
        shared.set_enable(True)

        motor.set_step_callback(
            lambda delta: record("step_batch", step_delta=int(delta))
        )
        motor.moveFinished.connect(
            lambda move_id, completed: move_results.__setitem__(
                int(move_id), bool(completed)
            )
        )
        motor.timingReport.connect(
            lambda count, mean_ms, jitter_ms: record(
                "timing",
                timing_intervals=int(count),
                mean_period_ms=f"{mean_ms:.6f}",
                max_jitter_ms=f"{jitter_ms:.6f}",
            )
        )

        print(
            f"UYARI: {args.axis.upper()} ekseni {args.steps} pulse "
            f"{axis['positive_name']}, sonra {args.steps} pulse "
            f"{axis['negative_name']} hareket edecek."
        )
        print(f"Hiz: {args.speed_sps:g} step/s  Log: {output}")
        print("Hareket 3 saniye sonra baslayacak. Ctrl+C ile iptal edebilirsiniz.")
        for remaining in (3, 2, 1):
            print(remaining)
            time.sleep(1.0)

        record("test_start")
        for cycle in range(1, args.cycles + 1):
            for phase_name, signed_steps in (
                (f"cycle_{cycle}_{axis['positive_name']}", args.steps),
                (f"cycle_{cycle}_{axis['negative_name']}", -args.steps),
            ):
                active_phase["name"] = phase_name
                record("move_requested", requested_steps=signed_steps)
                move_id = motor.move_signed_steps(signed_steps)
                completed = wait_for_move(
                    app, move_results, move_id, args.timeout_seconds
                )
                record(
                    "move_finished",
                    requested_steps=signed_steps,
                    completed=completed,
                )
                if not completed:
                    motor.emergency_stop()
                    raise RuntimeError(f"Hareket tamamlanamadi: {phase_name}")
                end = time.monotonic() + max(0.0, args.settle_seconds)
                while time.monotonic() < end:
                    app.processEvents()
                    time.sleep(0.01)

        active_phase["name"] = "complete"
        final_position = motor.position_steps()
        record("test_complete", completed=(final_position == 0))

        print(f"Yazilim son pulse konumu: {final_position} (beklenen: 0)")
        print("Fiziksel isaretin ayni noktaya donup donmedigini kontrol edin.")
        return 0 if final_position == 0 else 1
    except KeyboardInterrupt:
        if motor is not None:
            motor.emergency_stop()
        active_phase["name"] = "cancelled"
        record("cancelled")
        print("Test kullanici tarafindan durduruldu.")
        return 130
    except Exception as exc:
        active_phase["name"] = "error"
        record("error", completed=False)
        print(f"Test hatasi: {exc}")
        return 1
    finally:
        if motor is not None:
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
