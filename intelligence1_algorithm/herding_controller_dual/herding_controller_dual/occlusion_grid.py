# herding_controller_dual/herding_controller_dual/occlusion_grid.py
"""LOST 상태 재탐색을 위한 베이지안 belief 그리드 (확산 + 감쇠)."""
from dataclasses import dataclass

import numpy as np

from herding_controller_dual.grid_map import GridMap


@dataclass
class OcclusionGridConfig:
    """LOST 복구 belief 그리드의 확산율과 감쇠율."""
    diffusion_rate: float
    decay_factor: float


class OcclusionGrid:
    """LOST 상태 동안 감쇠하고 확산하는 타겟 위치 belief를 추적한다."""

    def __init__(self, config: OcclusionGridConfig, grid_map: GridMap) -> None:
        self.config = config
        self.grid_map = grid_map
        self.belief = np.zeros((grid_map.config.height_cells, grid_map.config.width_cells))

    def seed(self, row: int, col: int) -> None:
        """belief를 마지막으로 알려진 타겟 셀에 있는 단일 점 질량으로 재설정한다."""
        self.belief = np.zeros_like(self.belief)  # 이전 belief 전부 지움
        self.belief[row, col] = 1.0  # 마지막 목격 셀에 확률(질량) 전부를 몰아줌

    def step(self, dt: float) -> None:
        """belief를 4개 이웃으로 확산시키고, 장애물을 마스킹하며, 총 질량을 감쇠시킨다."""
        # 결과뿐 아니라 확산 가중치 자체를 클램프한다: 0.25는 균일 그리드
        # 위의 이 명시적 유한차분 4-이웃 스텐실에 대한 표준 안정성 한계다.
        # 이는 (1 - 4*weight) >= 0을 무조건적으로 보장하므로, 정체/지연된
        # ROS2 타이머 콜백이 비정상적으로 큰 dt를 만들어내는 등 diffusion_rate
        # * dt가 아무리 커지더라도 아래의 확산 단계는 질량 보존적이다(총
        # belief를 절대 부풀리지 않는다).
        weight = min(self.config.diffusion_rate * dt, 0.25)  # 이번 스텝에 이웃 한 칸으로 넘어가는 belief 비율
        # 질량 보존 확산: 각 셀은 자신의 belief 중 (1 - 4*weight)를 유지하고
        # 4개 이웃 각각으로부터 weight * neighbor만큼을 얻는다. 그리드
        # 경계에 있는 셀들은 여전히 4*weight 전체가 유출되지만 받을 수 있는
        # 이웃은 4개보다 적으므로, "맵 경계 밖으로 확산되는" 질량은 그곳에서
        # 사라진다 (아래의 장애물 케이스와 동일한 방식).
        diffused = self.belief * (1.0 - 4.0 * weight)  # 각 셀은 4방향으로 유출하고 남은 몫만 우선 보유
        diffused[1:, :] += self.belief[:-1, :] * weight   # 위쪽(row-1) 이웃에서 들어오는 몫
        diffused[:-1, :] += self.belief[1:, :] * weight   # 아래쪽(row+1) 이웃에서 들어오는 몫
        diffused[:, 1:] += self.belief[:, :-1] * weight   # 왼쪽(col-1) 이웃에서 들어오는 몫
        diffused[:, :-1] += self.belief[:, 1:] * weight   # 오른쪽(col+1) 이웃에서 들어오는 몫
        diffused[self.grid_map.obstacle_mask] = 0.0        # 벽 셀엔 표적이 있을 수 없으므로 belief 강제로 0

        # 이미 질량 보존적인 확산된 belief에 감쇠를 직접 적용한다. 여기서
        # 총합을 다시 1.0으로 재정규화하지 말 것: 그렇게 하면 이전 모든
        # 단계의 감쇠가 지워지고, 관측되지 않은 채로 더 많은 단계가
        # 지나감에 따라 기하급수적으로(decay_factor ** n) 줄어드는 대신
        # 총 질량이 영원히 정확히 decay_factor로 고정될 것이다.
        self.belief = diffused * self.config.decay_factor  # 매 스텝 전체 질량을 decay_factor배로 줄임(오래 안 보일수록 확신도 하락)

    def best_guess_cell(self) -> tuple[int, int]:
        """belief가 가장 높은 셀의 (row, col)을 재탐색 목표로 반환한다."""
        row, col = np.unravel_index(np.argmax(self.belief), self.belief.shape)  # 1차원 최댓값 인덱스 -> (row, col)
        return int(row), int(col)  # numpy 정수 타입 대신 순수 int로(호출부 비교/직렬화 편의)
