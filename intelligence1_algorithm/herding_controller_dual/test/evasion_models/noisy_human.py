# herding_controller_dual/test/evasion_models/noisy_human.py
"""인간 RC 조종자를 근사한다: 반응 지연과 노이즈가 추가된 WallHugger 행동."""
import numpy as np

from test.evasion_models.base import EvasionModel
from test.evasion_models.wall_hugger import WallHugger


class NoisyHuman(EvasionModel):
    """실제 환경 성공 여부를 예측하는 모델: 지연되고 노이즈가 섞인 WallHugger 명령."""

    def __init__(
        self,
        max_speed_mps: float,
        flee_reaction_distance_m: float,
        grid_map,
        reaction_delay_range: tuple = (0.3, 0.8),
        noise_std: float = 0.1,
        rng: np.random.Generator | None = None,
    ) -> None:
        """WallHugger를 감싸서 명령에 무작위 반응 지연과 노이즈를 추가한다."""
        self.max_speed_mps = max_speed_mps
        self._wall_hugger = WallHugger(max_speed_mps, flee_reaction_distance_m, grid_map)
        self.reaction_delay_range = reaction_delay_range
        self.noise_std = noise_std
        self.rng = rng or np.random.default_rng()
        self._pending_delay_sec = self.rng.uniform(*reaction_delay_range)
        self._elapsed_since_command_sec = 0.0
        self._held_command = np.zeros(2)

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        """무작위 반응 지연이 지날 때까지 마지막 노이즈 명령을 유지한 뒤 다시 샘플링한다."""
        self._elapsed_since_command_sec += dt
        if self._elapsed_since_command_sec >= self._pending_delay_sec:
            base_command = self._wall_hugger.step(target_state, robot_positions, obstacle_map, dt)
            noise = self.rng.normal(scale=self.noise_std, size=2)
            noisy_command = base_command + noise
            # 노이즈를 더하면 명령 속도가 max_speed_mps를 넘어설 수 있어 시뮬레이터의
            # 상한 고정 가정이 깨진다. 방향은 유지한 채 크기만 다시 클리핑한다.
            noisy_norm = np.linalg.norm(noisy_command)
            if noisy_norm > self.max_speed_mps:
                noisy_command = noisy_command / noisy_norm * self.max_speed_mps
            self._held_command = noisy_command
            self._elapsed_since_command_sec = 0.0
            self._pending_delay_sec = self.rng.uniform(*self.reaction_delay_range)
        return self._held_command
