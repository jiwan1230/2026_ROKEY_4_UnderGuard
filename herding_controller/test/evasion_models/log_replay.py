# herding_controller/test/evasion_models/log_replay.py
"""Replays a recorded target trajectory CSV (t, x, y) as velocity commands."""
import csv

import numpy as np

from test.evasion_models.base import EvasionModel


class LogReplay(EvasionModel):
    """Feeds back a real minicar trajectory so simulations can be compared to field results."""

    def __init__(self, csv_path: str) -> None:
        self._samples: list[tuple[float, np.ndarray]] = []
        with open(csv_path, newline="") as f:
            for row in csv.DictReader(f):
                self._samples.append((float(row["t"]), np.array([float(row["x"]), float(row["y"])])))
        self._elapsed_sec = 0.0
        self._index = 0

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        self._elapsed_sec += dt
        while self._index < len(self._samples) - 1 and self._samples[self._index + 1][0] <= self._elapsed_sec:
            self._index += 1
        if self._index >= len(self._samples) - 1:
            return np.zeros(2)
        _, current_pos = self._samples[self._index]
        next_t, next_pos = self._samples[self._index + 1]
        remaining = max(next_t - self._elapsed_sec, 1e-6)
        return (next_pos - target_state[:2]) / remaining
