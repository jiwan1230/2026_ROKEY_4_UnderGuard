# herding_controller_dual/test/evasion_models/base.py
"""모든 타겟 회피 행동 모델이 공유하는 추상 인터페이스."""
from abc import ABC, abstractmethod

import numpy as np


class EvasionModel(ABC):
    """현재 상황(scene)이 주어졌을 때 타겟의 다음 속도 명령을 생성한다."""

    @abstractmethod
    def step(
        self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float
    ) -> np.ndarray:
        """타겟의 다음 [vx, vy] 속도 벡터를 반환한다."""
        raise NotImplementedError
