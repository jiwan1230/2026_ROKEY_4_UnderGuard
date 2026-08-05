# herding_controller/test/evasion_models/log_replay.py
"""기록된 타겟 궤적 CSV(t, x, y)를 속도 명령으로 재생한다."""
import csv

import numpy as np

from test.evasion_models.base import EvasionModel


class LogReplay(EvasionModel):
    """실제 미니카 궤적을 되먹임하여 시뮬레이션을 현장 실험 결과와 비교할 수 있게 한다."""

    def __init__(self, csv_path: str, max_speed_mps: float | None = None) -> None:
        """t,x,y 궤적 CSV를 로드한다. max_speed_mps가 주어지면 재생 속도를 그 값으로 제한한다."""
        self._samples: list[tuple[float, np.ndarray]] = []
        with open(csv_path, newline="") as f:
            for row in csv.DictReader(f):
                self._samples.append((float(row["t"]), np.array([float(row["x"]), float(row["y"])])))
        self._elapsed_sec = 0.0
        self._index = 0
        self.max_speed_mps = max_speed_mps

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        """다음 로그 샘플에 도달하는 데 필요한 속도를 반환한다. 제한이 설정된 경우 클리핑한다."""
        self._elapsed_sec += dt
        while self._index < len(self._samples) - 1 and self._samples[self._index + 1][0] <= self._elapsed_sec:
            self._index += 1
        if self._index >= len(self._samples) - 1:
            return np.zeros(2)
        _, current_pos = self._samples[self._index]
        next_t, next_pos = self._samples[self._index + 1]
        remaining = max(next_t - self._elapsed_sec, 1e-6)
        velocity = (next_pos - target_state[:2]) / remaining
        if self.max_speed_mps is not None:
            norm = np.linalg.norm(velocity)
            if norm > self.max_speed_mps:
                velocity = velocity / norm * self.max_speed_mps
        return velocity
