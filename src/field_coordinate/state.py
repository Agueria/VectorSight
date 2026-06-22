from __future__ import annotations

from dataclasses import dataclass, field

from field_coordinate.models import Attitude


@dataclass(frozen=True)
class AttitudeChange:
    is_new: bool
    attitude: Attitude


@dataclass
class AttitudeChangeFilter:
    max_history: int = 3
    history: list[Attitude] = field(default_factory=list)

    def update(self, *, roll_deg: float, pitch_deg: float, yaw_deg: float) -> AttitudeChange:
        attitude = Attitude(roll_deg=roll_deg, pitch_deg=pitch_deg, yaw_deg=yaw_deg)
        is_new = not self.history or self.history[-1] != attitude
        self.history = [*self.history, attitude][-self.max_history :]
        return AttitudeChange(is_new=is_new, attitude=attitude)
