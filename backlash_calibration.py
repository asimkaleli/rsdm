"""Interactive backlash measurement tool for the RSDM axes.

Close rsdm.py before running this script. The tool never moves on startup;
every motion is an explicit terminal command from the operator.
"""

import argparse
import csv
from datetime import datetime
from pathlib import Path
import shlex
import sys
import time

from PySide2.QtCore import QCoreApplication

from laser_gpio import LaserGPIO
from motor_control import MotorController, MotorPins, SharedPins


# Keep this mapping identical to the main application (rsdm.py).
AXES = {
    "x": {
        "pins": MotorPins(step=12, dir=5),
        "positive_name": "left",
        "negative_name": "right",
    },
    "y": {
        "pins": MotorPins(step=13, dir=6, dir_inverted=True),
        "positive_name": "up",
        "negative_name": "down",
    },
}


def timestamp_ms() -> str:
    return datetime.now().isoformat(sep=" ", timespec="milliseconds")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Interactively send exact pulses to one axis and record backlash "
            "measurements. No motion occurs until a '+' or '-' command is entered."
        )
    )
    parser.add_argument(
        "--axis",
        choices=("x", "y"),
        required=True,
        help="Axis to calibrate.",
    )
    parser.add_argument("--speed-sps", type=float, default=100.0)
    parser.add_argument("--default-step", type=int, default=1)
    parser.add_argument("--settle-ms", type=int, default=500)
    parser.add_argument(
        "--max-command-steps",
        type=int,
        default=100,
        help="Safety limit for a single terminal motion command.",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--confirm-motion",
        action="store_true",
        help="Required acknowledgement that the mechanism is clear to move.",
    )
    return parser.parse_args()


def process_for(app, duration_s: float):
    deadline = time.monotonic() + max(0.0, float(duration_s))
    while time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.005)


def wait_for_move(app, results, move_id: int, timeout_s: float) -> bool:
    deadline = time.monotonic() + max(1.0, float(timeout_s))
    while time.monotonic() < deadline:
        app.processEvents()
        if move_id in results:
            return bool(results.pop(move_id))
        time.sleep(0.002)
    return False


def print_help(axis_config, default_step: int, max_steps: int):
    print()
    print("Komutlar:")
    print(
        f"  + [N]       + yonde hareket "
        f"({axis_config['positive_name']}); N verilmezse {default_step} pulse"
    )
    print(
        f"  - [N]       - yonde hareket "
        f"({axis_config['negative_name']}); N verilmezse {default_step} pulse"
    )
    print("  step N      Varsayilan pulse grubunu degistir")
    print("  arm +       Mekanizma + tarafa yasli; + -> - olcumunu baslat")
    print("  arm -       Mekanizma - tarafa yasli; - -> + olcumunu baslat")
    print("  mark        Lazer ilk net hareket ettiginde olcumu kaydet")
    print("  cancel      Aktif olcumu kaydetmeden iptal et")
    print("  status      Sayaclari ve aktif olcumu goster")
    print("  laser on|off")
    print("  help")
    print("  quit")
    print(f"Tek komut guvenlik siniri: {max_steps} pulse")
    print()


def main() -> int:
    args = parse_args()
    if not args.confirm_motion:
        print(
            "HAREKET ENGELLENDI: Mekanizmayi orta/guvenli konuma alip "
            "--confirm-motion ile tekrar calistirin."
        )
        return 2
    if not 1.0 <= args.speed_sps <= 1000.0:
        print("--speed-sps 1 ile 1000 arasinda olmalidir.")
        return 2
    if args.default_step <= 0 or args.max_command_steps <= 0:
        print("--default-step ve --max-command-steps sifirdan buyuk olmalidir.")
        return 2
    if not 0 <= args.settle_ms <= 10000:
        print("--settle-ms 0 ile 10000 arasinda olmalidir.")
        return 2
    if args.default_step > args.max_command_steps:
        print("--default-step, --max-command-steps degerini asamaz.")
        return 2

    output = args.output or Path(
        f"backlash_{args.axis}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    )
    output = output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    app = QCoreApplication.instance() or QCoreApplication(sys.argv[:1])
    axis_config = AXES[args.axis]
    shared = None
    motor = None
    laser = None
    rows = []
    move_results = {}
    default_step = int(args.default_step)
    active_measurement = None
    exit_code = 0

    def record(event: str, **values):
        rows.append({
            "timestamp": timestamp_ms(),
            "event": event,
            "axis": args.axis,
            "loaded_side": values.get("loaded_side", ""),
            "transition": values.get("transition", ""),
            "movement_sign": values.get("movement_sign", ""),
            "requested_steps": values.get("requested_steps", ""),
            "reverse_pulses": values.get("reverse_pulses", ""),
            "lower_bound_pulses": values.get("lower_bound_pulses", ""),
            "upper_bound_pulses": values.get("upper_bound_pulses", ""),
            "estimated_backlash_pulses": values.get(
                "estimated_backlash_pulses", ""
            ),
            "raw_position_steps": (
                motor.position_steps() if motor is not None else ""
            ),
            "speed_sps": f"{args.speed_sps:g}",
            "settle_ms": int(args.settle_ms),
            "message": values.get("message", ""),
        })

    def show_status():
        print(f"Raw pulse konumu: {motor.position_steps():+d}")
        if active_measurement is None:
            print("Aktif backlash olcumu yok.")
            return
        print(
            f"Aktif olcum: {active_measurement['transition']}  "
            f"ters yon pulse={active_measurement['reverse_pulses']}"
        )

    try:
        shared = SharedPins(
            en=7,
            reset=8,
            sleep=25,
            ms1=16,
            ms2=20,
            ms3=21,
        )
        motor = MotorController(axis_config["pins"], shared=shared)
        motor.set_speed_ms(1000.0 / (2.0 * args.speed_sps))
        motor.set_microstep("SIXTEENTH")
        motor.moveFinished.connect(
            lambda move_id, completed: move_results.__setitem__(
                int(move_id), bool(completed)
            )
        )
        motor.error.connect(lambda message: print(f"[Motor] {message}"))
        shared.set_enable(True)

        laser = LaserGPIO(line=24)
        laser.set_enabled(True)
        record("session_start", message="laser_on")

        print("UYARI: Lazer acik; isin yoluna ve goz hizasina bakmayin.")
        print("Program acilista motor hareketi yapmadi.")
        print(
            f"Eksen {args.axis.upper()}: +="
            f"{axis_config['positive_name']}, -="
            f"{axis_config['negative_name']}, hiz={args.speed_sps:g} pulse/s"
        )
        print(f"CSV: {output}")
        print_help(axis_config, default_step, args.max_command_steps)

        while True:
            try:
                command_line = input(f"backlash-{args.axis}> ").strip()
            except EOFError:
                command_line = "quit"
            if not command_line:
                continue

            try:
                parts = shlex.split(command_line)
            except ValueError as exc:
                print(f"Komut okunamadi: {exc}")
                continue
            command = parts[0].lower()

            if command in ("quit", "exit", "q"):
                break
            if command in ("help", "h", "?"):
                print_help(axis_config, default_step, args.max_command_steps)
                continue
            if command == "status":
                show_status()
                continue
            if command == "cancel":
                if active_measurement is not None:
                    record(
                        "measurement_cancelled",
                        loaded_side=active_measurement["loaded_side"],
                        transition=active_measurement["transition"],
                        reverse_pulses=active_measurement["reverse_pulses"],
                    )
                active_measurement = None
                print("Aktif olcum iptal edildi.")
                continue
            if command == "step":
                if len(parts) != 2:
                    print("Kullanim: step N")
                    continue
                try:
                    requested_default = int(parts[1])
                except ValueError:
                    print("N tam sayi olmalidir.")
                    continue
                if not 1 <= requested_default <= args.max_command_steps:
                    print(
                        f"N, 1 ile {args.max_command_steps} arasinda olmalidir."
                    )
                    continue
                default_step = requested_default
                print(f"Varsayilan hareket: {default_step} pulse")
                continue
            if command == "laser":
                if len(parts) != 2 or parts[1].lower() not in ("on", "off"):
                    print("Kullanim: laser on veya laser off")
                    continue
                enabled = parts[1].lower() == "on"
                laser.set_enabled(enabled)
                record("laser_on" if enabled else "laser_off")
                print("Lazer acildi." if enabled else "Lazer kapatildi.")
                continue
            if command == "arm":
                if len(parts) != 2 or parts[1] not in ("+", "-"):
                    print("Kullanim: arm + veya arm -")
                    continue
                loaded_sign = parts[1]
                reverse_sign = "-" if loaded_sign == "+" else "+"
                transition = f"{loaded_sign}_to_{reverse_sign}"
                active_measurement = {
                    "loaded_side": loaded_sign,
                    "reverse_sign": reverse_sign,
                    "transition": transition,
                    "reverse_pulses": 0,
                    "before_last_batch": 0,
                    "last_batch_steps": 0,
                }
                record(
                    "measurement_armed",
                    loaded_side=loaded_sign,
                    transition=transition,
                )
                print(
                    f"{transition} olcumu basladi. Lazer noktasini isaretleyin; "
                    f"bundan sonra yalnizca '{reverse_sign}' hareketine izin verilecek."
                )
                continue
            if command == "mark":
                if active_measurement is None:
                    print("Once arm + veya arm - ile olcumu baslatin.")
                    continue
                if active_measurement["reverse_pulses"] <= 0:
                    print("Henuz ters yonde pulse gonderilmedi.")
                    continue
                lower_bound = active_measurement["before_last_batch"]
                upper_bound = active_measurement["reverse_pulses"] - 1
                exact_value = (
                    lower_bound if lower_bound == upper_bound else ""
                )
                record(
                    "measurement",
                    loaded_side=active_measurement["loaded_side"],
                    transition=active_measurement["transition"],
                    reverse_pulses=active_measurement["reverse_pulses"],
                    lower_bound_pulses=lower_bound,
                    upper_bound_pulses=upper_bound,
                    estimated_backlash_pulses=exact_value,
                    message="first_visible_motion",
                )
                if exact_value != "":
                    print(
                        f"KAYDEDILDI: {active_measurement['transition']} "
                        f"backlash = {exact_value} pulse"
                    )
                else:
                    print(
                        f"KAYDEDILDI: {active_measurement['transition']} "
                        f"backlash araligi = {lower_bound}..{upper_bound} pulse. "
                        "Kesin sonuc icin son yaklasimi 1 pulse ile tekrarlayin."
                    )
                active_measurement = None
                continue

            if command not in ("+", "-"):
                print("Bilinmeyen komut. 'help' yazin.")
                continue
            if len(parts) > 2:
                print("Kullanim: + [N] veya - [N]")
                continue
            try:
                pulse_count = int(parts[1]) if len(parts) == 2 else default_step
            except ValueError:
                print("Pulse sayisi tam sayi olmalidir.")
                continue
            if not 1 <= pulse_count <= args.max_command_steps:
                print(
                    f"Tek komutta pulse sayisi 1 ile "
                    f"{args.max_command_steps} arasinda olmalidir."
                )
                continue
            if (
                active_measurement is not None
                and command != active_measurement["reverse_sign"]
            ):
                print(
                    "Olcum sirasinda yalniz ters yone hareket edilebilir. "
                    "Yon degistirmek icin once 'cancel' yazin."
                )
                continue

            signed_steps = pulse_count if command == "+" else -pulse_count
            record(
                "move_requested",
                loaded_side=(
                    active_measurement["loaded_side"]
                    if active_measurement is not None else ""
                ),
                transition=(
                    active_measurement["transition"]
                    if active_measurement is not None else ""
                ),
                movement_sign=command,
                requested_steps=signed_steps,
                reverse_pulses=(
                    active_measurement["reverse_pulses"]
                    if active_measurement is not None else ""
                ),
            )
            move_id = motor.move_signed_steps(signed_steps)
            timeout_s = max(10.0, pulse_count / args.speed_sps * 3.0 + 5.0)
            if not wait_for_move(app, move_results, move_id, timeout_s):
                motor.emergency_stop()
                raise RuntimeError("Motor hareketi tamamlanamadi veya zaman asimina ugradi.")

            if active_measurement is not None:
                active_measurement["before_last_batch"] = active_measurement[
                    "reverse_pulses"
                ]
                active_measurement["last_batch_steps"] = pulse_count
                active_measurement["reverse_pulses"] += pulse_count
            process_for(app, args.settle_ms / 1000.0)
            record(
                "move_finished",
                loaded_side=(
                    active_measurement["loaded_side"]
                    if active_measurement is not None else ""
                ),
                transition=(
                    active_measurement["transition"]
                    if active_measurement is not None else ""
                ),
                movement_sign=command,
                requested_steps=signed_steps,
                reverse_pulses=(
                    active_measurement["reverse_pulses"]
                    if active_measurement is not None else ""
                ),
            )
            direction_name = (
                axis_config["positive_name"]
                if command == "+" else axis_config["negative_name"]
            )
            print(
                f"{pulse_count} pulse {direction_name}; "
                f"raw={motor.position_steps():+d}"
            )
            if active_measurement is not None:
                print(
                    f"Ters yon toplam: "
                    f"{active_measurement['reverse_pulses']} pulse"
                )

    except KeyboardInterrupt:
        exit_code = 130
        if motor is not None:
            motor.emergency_stop()
        record("cancelled", message="keyboard_interrupt")
        print("Kalibrasyon kullanici tarafindan durduruldu.")
    except Exception as exc:
        exit_code = 1
        if motor is not None:
            motor.emergency_stop()
        record("error", message=str(exc))
        print(f"Kalibrasyon hatasi: {exc}")
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
        if motor is not None:
            motor.shutdown()
        if shared is not None:
            try:
                shared.set_enable(False)
            except Exception:
                pass
        if rows:
            with output.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            print(f"Log kaydedildi: {output}")

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
