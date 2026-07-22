# laser_path_planner.py
from dataclasses import dataclass
from typing import Dict, List, Mapping, Tuple
import math


@dataclass
class StepperConfig:
    """
    Step motor kinematik parametreleri.
    - step_angle_deg : Tam adım başına mekanik açı (ör. 1.8)
    - microstep_div  : Mikro-adım bölüntüsü (ör. 16 ⇒ 1/16)
    - gear_ratio     : Redüksiyon oranı (ör. 10.0 ⇒ motor 10 tur = eksen 1 tur)
    """
    step_angle_deg: float = 1.8
    microstep_div: int = 16
    gear_ratio: float = 1.0


def _yaw_norm_deg(y: float) -> float:
    """Yaw değerini [0, 360) aralığına al."""
    y = float(y) % 360.0
    if y < 0:
        y += 360.0
    return y


def _xyz_to_d_pitch_yaw(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """
    Konvansiyon:
    +X: Doğu, +Y: Kuzey, +Z: Yukarı
    Pitch = atan2(z, hypot(x,y))  (deg)
    Yaw   = atan2(x, y)           (deg), [0,360)
    """
    D = math.sqrt(x * x + y * y + z * z)
    Pitch = math.degrees(math.atan2(z, math.hypot(x, y)))
    Yaw = math.degrees(math.atan2(x, y))
    Yaw = _yaw_norm_deg(Yaw)
    return D, Pitch, Yaw


def _d_pitch_yaw_to_xyz(
    distance: float, pitch_deg: float, yaw_deg: float
) -> Tuple[float, float, float]:
    """Convert laser-centric distance/elevation/north-yaw to XYZ."""
    distance = float(distance)
    pitch = math.radians(float(pitch_deg))
    yaw = math.radians(float(yaw_deg))
    radial_xy = distance * math.cos(pitch)
    return (
        radial_xy * math.sin(yaw),
        radial_xy * math.cos(yaw),
        distance * math.sin(pitch),
    )


def _deg_to_steps(delta_deg: float, cfg: StepperConfig) -> int:
    """
    Açı farkını (deg) step sayısına çevir.
    Pozitif → ileri/sağ/yukarı, negatif → geri/sol/aşağı (işaret korunur).
    """
    if cfg.step_angle_deg <= 0 or cfg.microstep_div <= 0 or cfg.gear_ratio <= 0:
        return 0
    steps_per_rev = 360.0 / cfg.step_angle_deg
    microsteps_per_rev = steps_per_rev * float(cfg.microstep_div) * float(cfg.gear_ratio)
    steps = delta_deg * (microsteps_per_rev / 360.0)
    return int(round(steps))


def planned_move_steps(
    pitch_steps_delta, yaw_steps_delta, current_row: int, target_row: int
) -> Tuple[int, int, int]:
    """Return ``(yaw_steps, pitch_steps, segment_index)`` for adjacent rows."""
    pitch_steps_delta = list(pitch_steps_delta)
    yaw_steps_delta = list(yaw_steps_delta)
    if len(pitch_steps_delta) != len(yaw_steps_delta):
        raise ValueError("Planner step delta lengths do not match.")
    current_row = int(current_row)
    target_row = int(target_row)
    if target_row == current_row + 1:
        segment = current_row
        sign = 1
    elif target_row == current_row - 1:
        segment = target_row
        sign = -1
    else:
        raise ValueError("Planned movement must be between adjacent rows.")
    if not 0 <= segment < len(pitch_steps_delta):
        raise IndexError("Planned segment index is outside the path.")
    return (
        sign * int(yaw_steps_delta[segment]),
        sign * int(pitch_steps_delta[segment]),
        segment,
    )


def _dpy_to_step_deltas(dpy_list, stepper_pitch, stepper_yaw):
    """Quantize absolute targets first so rounding cannot accumulate."""
    if len(dpy_list) < 2:
        return [], []
    _, first_pitch, first_yaw = dpy_list[0]
    pitch_absolute = [0]
    yaw_absolute = [0]
    unwrapped_yaw_delta = 0.0
    previous_yaw = first_yaw
    for _, pitch, yaw in dpy_list[1:]:
        pitch_absolute.append(
            _deg_to_steps(float(pitch) - float(first_pitch), stepper_pitch)
        )
        unwrapped_yaw_delta += (
            (float(yaw) - float(previous_yaw) + 180.0) % 360.0 - 180.0
        )
        yaw_absolute.append(_deg_to_steps(unwrapped_yaw_delta, stepper_yaw))
        previous_yaw = yaw

    pitch_delta = [
        following - current
        for current, following in zip(pitch_absolute, pitch_absolute[1:])
    ]
    yaw_delta = [
        following - current
        for current, following in zip(yaw_absolute, yaw_absolute[1:])
    ]
    return pitch_delta, yaw_delta


def plan_laser_path(
    *,
    D1: float,
    Pitch_1: float,
    Yaw_1: float,
    D2: float,
    Pitch_2: float,
    Yaw_2: float,
    N: int,
    stepper_pitch: StepperConfig,
    stepper_yaw: StepperConfig,
) -> Dict[str, List]:
    """
    MATLAB taslağındaki mantığın Python uyarlaması.
    A referansı: Pitch1=0, Yaw1=0 varsayılarak A = (0, D1, 0).
    B, sadece farklarla (DeltaPitch = Pitch2-Pitch1, DeltaYaw = Yaw2-Yaw1) tanımlanır.

    Dönüş:
      {
        "xyz": [(x0,y0,z0), ..., (xN,yN,zN)],
        "dpy": [(D0,Pitch0,Yaw0), ..., (DN,PitchN,YawN)],
        "pitch_steps_delta": [dp0, dp1, ..., dp{N-1}],
        "yaw_steps_delta":   [dy0, dy1, ..., dy{N-1}],
      }
    """
    if N <= 0:
        N = 1

    # --- A noktası (referans hizalama) ---
    A_xyz = (0.0, float(D1), 0.0)

    # --- B yönü: yalnızca farklarla belirlenir ---
    dPitch = float(Pitch_2) - float(Pitch_1)
    dYaw = float(Yaw_2) - float(Yaw_1)
    p2 = math.radians(dPitch)
    y2 = math.radians(dYaw)

    # Kuzey referanslı azimut:
    # x = D*cos(pitch)*sin(yaw)
    # y = D*cos(pitch)*cos(yaw)
    # z = D*sin(pitch)
    Bx = float(D2) * math.cos(p2) * math.sin(y2)
    By = float(D2) * math.cos(p2) * math.cos(y2)
    Bz = float(D2) * math.sin(p2)
    B_xyz = (Bx, By, Bz)

    # --- AB doğrusu üzerinde N eş segment (N+1 nokta) ---
    xyz_list: List[Tuple[float, float, float]] = []
    for i in range(N + 1):
        t = i / float(N)
        x = A_xyz[0] + t * (B_xyz[0] - A_xyz[0])
        y = A_xyz[1] + t * (B_xyz[1] - A_xyz[1])
        z = A_xyz[2] + t * (B_xyz[2] - A_xyz[2])
        xyz_list.append((x, y, z))

    # --- Her noktanın D/Pitch/Yaw hesabı ---
    dpy_list: List[Tuple[float, float, float]] = []
    for (x, y, z) in xyz_list:
        dpy_list.append(_xyz_to_d_pitch_yaw(x, y, z))

    # Mutlak hedefleri önce quantize etmek, çok sayıda noktada yuvarlama
    # hatasının birikmesini engeller.
    pitch_steps_delta, yaw_steps_delta = _dpy_to_step_deltas(
        dpy_list, stepper_pitch, stepper_yaw
    )

    return {
        "plan_type": "sequential",
        "scan_step": -1,
        "xyz": xyz_list,               # (N+1) nokta
        "dpy": dpy_list,               # (N+1) nokta
        "pitch_steps_delta": pitch_steps_delta,  # N adet
        "yaw_steps_delta": yaw_steps_delta,      # N adet
    }


def plan_grid_path(
    *,
    corners: Mapping[str, Tuple[float, float, float]],
    x_segments: int,
    y_segments: int,
    stepper_pitch: StepperConfig,
    stepper_yaw: StepperConfig,
) -> Dict[str, List]:
    """Plan a four-corner surface scan in a continuous serpentine order.

    ``corners`` contains ``A`` (top-left), ``B`` (top-right), ``C``
    (bottom-right) and ``D`` (bottom-left).  Each value is
    ``(distance, pitch_deg, yaw_deg)`` in the laser coordinate system.

    The returned path starts at D.  Vertical grid lines alternate direction,
    avoiding the long diagonal return present in the original MATLAB script.
    ``x_segments`` and ``y_segments`` are division counts, therefore the
    result contains ``(x_segments + 1) * (y_segments + 1)`` points.
    """
    required = ("A", "B", "C", "D")
    if any(name not in corners for name in required):
        raise ValueError("Grid corners A, B, C and D are required.")
    if int(x_segments) != x_segments or int(y_segments) != y_segments:
        raise ValueError("Grid division counts must be integers.")
    x_segments = int(x_segments)
    y_segments = int(y_segments)
    if x_segments < 1 or y_segments < 1:
        raise ValueError("Grid division counts must be at least 1.")

    xyz_corners = {}
    for name in required:
        values = tuple(corners[name])
        if len(values) != 3:
            raise ValueError(f"Corner {name} must contain distance, pitch and yaw.")
        distance, pitch, yaw = map(float, values)
        if not all(math.isfinite(value) for value in (distance, pitch, yaw)):
            raise ValueError(f"Corner {name} contains a non-finite value.")
        if distance <= 0.0:
            raise ValueError(f"Corner {name} distance must be positive.")
        xyz_corners[name] = _d_pitch_yaw_to_xyz(distance, pitch, yaw)

    A = xyz_corners["A"]
    B = xyz_corners["B"]
    C = xyz_corners["C"]
    D = xyz_corners["D"]

    xyz_list: List[Tuple[float, float, float]] = []
    grid_indices: List[Tuple[int, int]] = []
    for ix in range(x_segments + 1):
        u = ix / float(x_segments)
        bottom = tuple(D[k] + u * (C[k] - D[k]) for k in range(3))
        top = tuple(A[k] + u * (B[k] - A[k]) for k in range(3))
        y_indices = range(y_segments + 1)
        if ix % 2:
            y_indices = range(y_segments, -1, -1)
        for iy in y_indices:
            v = iy / float(y_segments)
            point = tuple(
                bottom[k] + v * (top[k] - bottom[k]) for k in range(3)
            )
            xyz_list.append(point)
            grid_indices.append((ix, iy))

    dpy_list = [_xyz_to_d_pitch_yaw(*point) for point in xyz_list]
    pitch_steps_delta, yaw_steps_delta = _dpy_to_step_deltas(
        dpy_list, stepper_pitch, stepper_yaw
    )

    return {
        "plan_type": "grid",
        "scan_step": 1,
        "xyz": xyz_list,
        "dpy": dpy_list,
        "grid_indices": grid_indices,
        "pitch_steps_delta": pitch_steps_delta,
        "yaw_steps_delta": yaw_steps_delta,
    }
