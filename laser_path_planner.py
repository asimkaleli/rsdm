# laser_path_planner.py
from dataclasses import dataclass
from typing import Dict, List, Tuple
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

    # --- Ardışık noktalar arası açı farklarını step'e çevir ---
    pitch_steps_delta: List[int] = []
    yaw_steps_delta: List[int] = []

    for i in range(N):
        _, P0, Y0 = dpy_list[i]
        _, P1, Y1 = dpy_list[i + 1]

        dP = P1 - P0
        dY = Y1 - Y0
        # Yaw farkını -180..+180’e katla ki kısa yoldan dönsün
        dY = (dY + 180.0) % 360.0 - 180.0

        pitch_steps_delta.append(_deg_to_steps(dP, stepper_pitch))
        yaw_steps_delta.append(_deg_to_steps(dY, stepper_yaw))

    return {
        "xyz": xyz_list,               # (N+1) nokta
        "dpy": dpy_list,               # (N+1) nokta
        "pitch_steps_delta": pitch_steps_delta,  # N adet
        "yaw_steps_delta": yaw_steps_delta,      # N adet
    }
