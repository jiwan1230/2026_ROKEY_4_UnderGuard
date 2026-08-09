# herding_controller/test/evasion_models/reactive_flee.py
"""주 검증 모델: 타겟이 가장 가까운 로봇으로부터 곧장 도망친다."""
import numpy as np

from test.evasion_models.base import EvasionModel


class ReactiveFlee(EvasionModel):
    """flee_reaction_distance_m 이내에 있는 로봇으로부터 곧장 도망친다."""

    def __init__(self, max_speed_mps: float, flee_reaction_distance_m: float) -> None:
        """속도 상한과, 로봇이 도망 반응을 유발하는 거리를 저장한다."""
        self.max_speed_mps = max_speed_mps
        self.flee_reaction_distance_m = flee_reaction_distance_m

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        """근처의 모든 로봇에 대해 가중치를 적용한 회피 벡터를 합산하고 결과를 제한한다."""
        target_pos = target_state[:2]
        flee_dir = np.zeros(2)
        for robot_pos in robot_positions:
            away = target_pos - robot_pos
            dist = np.linalg.norm(away)
            if dist < self.flee_reaction_distance_m and dist > 1e-6:
                flee_dir += (away / dist) * (self.flee_reaction_distance_m - dist)
        norm = np.linalg.norm(flee_dir)
        if norm < 1e-9:
            return np.zeros(2)
        return (flee_dir / norm) * self.max_speed_mps
