# herding_controller_dual/test/evasion_models/cornering_aware_flee.py
"""구석에 몰리지 않으려는 표적 모델 -- 실제 쥐 행동이자, 로봇 두 대가
필요해지는 이론적 조건.

**전제 정정(2026-08-08)**: 앞서 만든 `TrapAvoidingFlee`는 "표적이 트랩을
피한다"를 가정했는데 이건 틀렸다 -- 쥐는 트랩을 구별하지 못한다. 트랩 좌표를
아는 건 우리(알고리즘)뿐이고, 우리가 쥐를 그 좌표로 몰면 된다.

그럼 왜 로봇 한 대로 충분했는가? 주 검증 모델 `ReactiveFlee`가 **로봇 반대
방향으로만** 가기 때문이다 -- 구석으로 밀리면 계속 구석으로 들어간다. 실제
쥐는 그러지 않는다. 궁지에 몰리면 **위협의 옆을 스쳐 빠져나간다**(darting).

이건 이론적으로도 핵심이다. Besicovitch의 고전 결과: **같은 속도라면
추격자 한 명은 회피자를 영원히 못 잡지만, 두 명이면 잡을 수 있다.** 우리
트랩 3곳이 전부 벽 구석(벽에서 0.2m)에 있으므로 "트랩으로 몰기"는 사실상
"구석으로 몰기"이고, 정확히 이 문제와 같은 구조다.

이 모델은 8방향 후보를 이렇게 평가한다:
  - 로봇에서 멀어지는가 (기존과 동일)
  - **그 방향으로 갔을 때 앞이 트여 있는가**(`openness_weight`) -- 막다른
    구석으로 들어가는 선택을 피하고, 필요하면 로봇 옆을 스쳐서라도 트인
    쪽으로 나간다
  - 관성, 벽 막힘 제외
`openness_weight`가 0이면 기존 ReactiveFlee와 비슷해지고, 키우면 "구석에
안 몰리려는" 정도가 세진다 -- 로봇 한 대로는 옆을 못 막으므로 이 값이
커질수록 한 대의 성공률이 떨어지고, 두 대의 가치가 드러날 것으로 기대한다.
"""
import numpy as np

from test.evasion_models.base import EvasionModel

_N = 8
_DIRECTIONS = np.array([[np.cos(2 * np.pi * i / _N), np.sin(2 * np.pi * i / _N)] for i in range(_N)])


class CorneringAwareFlee(EvasionModel):
    """로봇을 피하되, **막다른 구석으로 몰리는 걸 피하는** 표적."""

    def __init__(
        self,
        max_speed_mps: float,
        flee_reaction_distance_m: float,
        grid_map,
        openness_weight: float = 1.0,
        openness_probe_m: float = 1.2,
        lookahead_m: float = 0.4,
        momentum_bonus: float = 0.4,
        temperature: float = 0.3,
        rng=None,
    ) -> None:
        self.max_speed_mps = max_speed_mps
        self.flee_reaction_distance_m = flee_reaction_distance_m
        self.grid_map = grid_map
        self.openness_weight = openness_weight
        self.openness_probe_m = openness_probe_m
        self.lookahead_m = lookahead_m
        self.momentum_bonus = momentum_bonus
        self.temperature = temperature
        self.rng = rng or np.random.default_rng()
        self._last_dir = None

    def _free_run(self, position, direction, limit_m):
        """그 방향으로 벽에 막히기 전까지 갈 수 있는 거리(m)."""
        res = self.grid_map.config.resolution_m
        steps = max(int(limit_m / res), 1)
        for i in range(1, steps + 1):
            probe = position + direction * (i * res)
            try:
                row, col = self.grid_map.world_to_cell(*probe)
            except ValueError:
                return (i - 1) * res
            if not self.grid_map.in_bounds(row, col) or self.grid_map.is_obstacle(row, col):
                return (i - 1) * res
        return limit_m

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        position = target_state[:2]
        scored = []
        for direction in _DIRECTIONS:
            run = self._free_run(position, direction, self.openness_probe_m)
            if run < self.lookahead_m:
                continue                      # 바로 앞이 벽이면 후보에서 제외
            score = 0.0
            for robot_pos in robot_positions:
                away = position - robot_pos
                dist = float(np.linalg.norm(away))
                if 1e-6 < dist < self.flee_reaction_distance_m:
                    urgency = (self.flee_reaction_distance_m - dist) / self.flee_reaction_distance_m
                    score += 3.0 * urgency * float(np.dot(direction, away / dist))
            # 트인 쪽 선호 -- 구석으로 들어가는 선택에 벌점을 주는 효과.
            score += self.openness_weight * (run / self.openness_probe_m)
            if self._last_dir is not None:
                score += self.momentum_bonus * float(np.dot(direction, self._last_dir))
            scored.append((score, direction))

        if not scored:
            self._last_dir = None
            return np.zeros(2)
        if self.temperature <= 1e-9:
            chosen = max(scored, key=lambda sd: sd[0])[1]
        else:
            values = np.array([sd[0] for sd in scored]) / self.temperature
            weights = np.exp(values - values.max())
            chosen = scored[int(self.rng.choice(len(scored), p=weights / weights.sum()))][1]
        self._last_dir = chosen
        return chosen * self.max_speed_mps
