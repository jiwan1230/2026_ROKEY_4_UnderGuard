# herding_controller/test/evasion_models/base.py
"""Abstract interface shared by every target-evasion behavior model."""
from abc import ABC, abstractmethod

import numpy as np


class EvasionModel(ABC):
    """Produces the target's next velocity command given the current scene."""

    @abstractmethod
    def step(
        self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float
    ) -> np.ndarray:
        """Return the target's next [vx, vy] velocity vector."""
        raise NotImplementedError
