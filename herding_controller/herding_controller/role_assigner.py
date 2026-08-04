# herding_controller/herding_controller/role_assigner.py
"""Dynamic Driver/Blocker role assignment with swap hysteresis."""
from dataclasses import dataclass

import numpy as np


@dataclass
class RoleAssignerConfig:
    """Hysteresis and separation thresholds for role swapping."""
    role_swap_margin: float
    role_swap_cooldown_sec: float
    min_robot_separation_m: float
    role_cost_turn_weight: float


class RoleAssigner:
    """Assigns which robot (1 or 2) is Driver, with hysteresis to prevent oscillation."""

    def __init__(self, config: RoleAssignerConfig) -> None:
        self.config = config
        self._driver_id = 1
        self._last_swap_time = None

    def assign(
        self,
        robot1_pos: np.ndarray,
        robot2_pos: np.ndarray,
        robot1_heading: np.ndarray,
        robot2_heading: np.ndarray,
        driving_point_candidate: np.ndarray,
        current_time_sec: float,
    ) -> tuple[int, int]:
        """Return (driver_id, blocker_id), swapping only past margin+cooldown thresholds."""
        if self._last_swap_time is None:
            # Anchor the cooldown clock to the first assignment call so an
            # initial candidate flip can't bypass the cooldown via `-inf`.
            self._last_swap_time = current_time_sec

        cost1 = self._cost(robot1_pos, robot1_heading, driving_point_candidate)
        cost2 = self._cost(robot2_pos, robot2_heading, driving_point_candidate)
        candidate_driver = 1 if cost1 <= cost2 else 2

        if candidate_driver != self._driver_id:
            cost_diff = abs(cost1 - cost2)
            time_since_swap = current_time_sec - self._last_swap_time
            if cost_diff >= self.config.role_swap_margin and time_since_swap >= self.config.role_swap_cooldown_sec:
                self._driver_id = candidate_driver
                self._last_swap_time = current_time_sec

        blocker_id = 2 if self._driver_id == 1 else 1
        return self._driver_id, blocker_id

    def _cost(self, robot_pos: np.ndarray, robot_heading: np.ndarray, target_point: np.ndarray) -> float:
        distance = float(np.linalg.norm(target_point - robot_pos))
        desired = target_point - robot_pos
        norm = np.linalg.norm(desired)
        if norm < 1e-6:
            return distance
        desired = desired / norm
        cos_angle = float(np.clip(np.dot(desired, robot_heading), -1.0, 1.0))
        turn_cost = float(np.arccos(cos_angle))
        return distance + self.config.role_cost_turn_weight * turn_cost


def resolve_separation(
    driving_point: np.ndarray, blocking_point: np.ndarray, config: RoleAssignerConfig
) -> np.ndarray:
    """Push the Blocker's goal away from the Driver's goal to keep min separation."""
    delta = blocking_point - driving_point
    dist = np.linalg.norm(delta)
    if dist >= config.min_robot_separation_m:
        return blocking_point
    direction = delta / dist if dist > 1e-6 else np.array([1.0, 0.0])
    # Nudge slightly past the threshold so floating-point rounding in the
    # add/subtract round-trip can't leave the result a hair short of
    # min_robot_separation_m (norm(a+b) is not exactly ||b|| in float64).
    push_distance = config.min_robot_separation_m * (1.0 + 1e-9) + 1e-9
    return driving_point + direction * push_distance
