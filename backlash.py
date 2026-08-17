"""GPIO-independent backlash state tracking.

The state describes the estimated load position and where the drive currently
sits inside the mechanical clearance. Motor GPIO code will own and update one
instance per axis once integration is enabled.
"""

from dataclasses import dataclass
import operator
from typing import Optional


DEFAULT_BACKLASH_STEPS = {
    "x": 77,
    "y": 4,
}


def _integer(value, name: str) -> int:
    try:
        return operator.index(value)
    except TypeError as exc:
        raise TypeError(f"{name} must be an integer") from exc


@dataclass(frozen=True)
class BacklashSnapshot:
    backlash_steps: int
    logical_position_steps: Optional[int]
    gap_steps: Optional[int]
    initialized: bool


class BacklashAxisState:
    """Track logical load position across motor pulses and direction changes.

    ``gap_steps == 0`` means the drive is loaded against the negative flank.
    ``gap_steps == backlash_steps`` means it is loaded against the positive
    flank. Intermediate values mean emitted pulses are still crossing the
    clearance and have not moved the load.
    """

    def __init__(self, backlash_steps: int):
        backlash_steps = _integer(backlash_steps, "backlash_steps")
        if backlash_steps < 0:
            raise ValueError("backlash_steps cannot be negative")
        self._backlash_steps = backlash_steps
        self._logical_position_steps = None
        self._gap_steps = None

    @property
    def backlash_steps(self) -> int:
        return self._backlash_steps

    @property
    def initialized(self) -> bool:
        return (
            self._logical_position_steps is not None
            and self._gap_steps is not None
        )

    @property
    def logical_position_steps(self) -> Optional[int]:
        return self._logical_position_steps

    @property
    def gap_steps(self) -> Optional[int]:
        return self._gap_steps

    def snapshot(self) -> BacklashSnapshot:
        return BacklashSnapshot(
            backlash_steps=self._backlash_steps,
            logical_position_steps=self._logical_position_steps,
            gap_steps=self._gap_steps,
            initialized=self.initialized,
        )

    def initialize(self, loaded_direction: int, logical_position_steps: int = 0):
        """Declare a known loaded flank after an operator-controlled preload.

        ``loaded_direction`` must be +1 for the positive flank or -1 for the
        negative flank. This method does not move hardware.
        """
        loaded_direction = _integer(loaded_direction, "loaded_direction")
        logical_position_steps = _integer(
            logical_position_steps, "logical_position_steps"
        )
        if loaded_direction not in (-1, 1):
            raise ValueError("loaded_direction must be -1 or +1")
        self._gap_steps = self._backlash_steps if loaded_direction > 0 else 0
        self._logical_position_steps = logical_position_steps

    def invalidate(self):
        """Discard state after power loss, reset or untracked physical motion."""
        self._logical_position_steps = None
        self._gap_steps = None

    def _require_initialized(self):
        if not self.initialized:
            raise RuntimeError("backlash state is not initialized")

    def apply_pulses(self, signed_pulses: int) -> int:
        """Apply pulses actually emitted and return resulting logical movement.

        Updating from emitted pulses rather than requested moves keeps the
        state valid if an automatic movement is cancelled part-way through.
        """
        self._require_initialized()
        signed_pulses = _integer(signed_pulses, "signed_pulses")
        old_position = self._logical_position_steps

        if signed_pulses > 0:
            clearance_pulses = min(
                signed_pulses, self._backlash_steps - self._gap_steps
            )
            self._gap_steps += clearance_pulses
            self._logical_position_steps += signed_pulses - clearance_pulses
        elif signed_pulses < 0:
            pulse_count = -signed_pulses
            clearance_pulses = min(pulse_count, self._gap_steps)
            self._gap_steps -= clearance_pulses
            self._logical_position_steps -= pulse_count - clearance_pulses

        return self._logical_position_steps - old_position

    def pulses_to_target(self, target_position_steps: int) -> int:
        """Return the motor pulses needed to reach an absolute logical target.

        The state is not changed here. It changes only as physical pulses are
        reported through :meth:`apply_pulses`.
        """
        self._require_initialized()
        target = _integer(target_position_steps, "target_position_steps")
        delta = target - self._logical_position_steps
        if delta > 0:
            return delta + (self._backlash_steps - self._gap_steps)
        if delta < 0:
            return delta - self._gap_steps
        return 0

