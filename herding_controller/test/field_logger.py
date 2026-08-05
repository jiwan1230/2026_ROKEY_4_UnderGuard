"""Blinded capture-zone selection and per-trial CSV logging for the minicar field protocol."""
import csv
import os

import numpy as np

CAPTURE_ZONE_CANDIDATES: list[tuple[float, float]] = [
    (3.0, 3.0), (7.0, 3.0), (3.0, 7.0), (7.0, 7.0),
]

_FIELDNAMES = [
    "trial_id", "condition", "capture_zone_id", "start_time", "end_time", "success",
    "duration_sec", "min_robot_target_dist", "rule_violation_count", "note",
]


def select_capture_zone(rng: np.random.Generator) -> tuple[float, float]:
    """Pick one of the 4 fixed capture-zone candidates. Caller must not print this (operator blinding)."""
    index = int(rng.integers(0, len(CAPTURE_ZONE_CANDIDATES)))
    return CAPTURE_ZONE_CANDIDATES[index]


class FieldLogger:
    """Appends one CSV row per field trial. Never writes the capture zone to the console."""

    def __init__(self, csv_path: str) -> None:
        self.csv_path = csv_path
        parent_dir = os.path.dirname(csv_path)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)
        if not os.path.exists(csv_path):
            with open(csv_path, "w", newline="") as f:
                csv.DictWriter(f, fieldnames=_FIELDNAMES).writeheader()

    def log_trial(
        self, trial_id: int, condition: str, capture_zone_id: int, start_time: float, end_time: float,
        success: bool, min_robot_target_dist: float, rule_violation_count: int, note: str = "",
    ) -> None:
        """Append one trial's outcome. capture_zone_id is written only to this file, never printed."""
        with open(self.csv_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=_FIELDNAMES)
            writer.writerow({
                "trial_id": trial_id, "condition": condition, "capture_zone_id": capture_zone_id,
                "start_time": start_time, "end_time": end_time, "success": int(bool(success)),
                "duration_sec": end_time - start_time, "min_robot_target_dist": min_robot_target_dist,
                "rule_violation_count": rule_violation_count, "note": note,
            })


def detect_rule_violations(
    robot_positions: list, target_positions: list, target_velocities: list,
    panic_distance_m: float, dt: float,
) -> int:
    """Count cycles where a robot is within panic_distance_m but the target moves toward it (rule 2)."""
    if not (len(robot_positions) == len(target_positions) == len(target_velocities)):
        raise ValueError(
            "robot_positions, target_positions, and target_velocities must have the same "
            f"number of time steps, got {len(robot_positions)}, {len(target_positions)}, "
            f"{len(target_velocities)}"
        )
    violations = 0
    for robots_at_t, target_pos, target_vel in zip(robot_positions, target_positions, target_velocities):
        if not robots_at_t:
            continue
        distances = [np.linalg.norm(target_pos - r) for r in robots_at_t]
        closest_index = int(np.argmin(distances))
        closest_dist = distances[closest_index]
        if closest_dist >= panic_distance_m:
            continue
        away = target_pos - robots_at_t[closest_index]
        speed = np.linalg.norm(target_vel)
        if speed < 1e-6:
            continue
        toward_robot = np.dot(target_vel / speed, away) < 0
        if toward_robot:
            violations += 1
    return violations
