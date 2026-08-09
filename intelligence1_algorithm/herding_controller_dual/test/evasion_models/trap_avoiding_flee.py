# herding_controller_dual/test/evasion_models/trap_avoiding_flee.py
"""포획존을 **능동적으로 피하는** 표적 모델 -- 지금까지 없던 유형.

기존 모델의 한계(트러블슈팅 노트 16-9): 주 검증 모델 `ReactiveFlee`는
반응거리(0.42m) 안에 로봇이 없으면 **속도 0을 반환해 가만히 서 있고**, 로봇이
가까워지면 곧장 반대로 밀린다. `WallHugger`/`NoisyHuman`/`JukingFlee`는 전부
그 위에 벽 따라가기·잡음·삐끗을 얹은 것이고, `DecisiveFlee`는 전략적이긴
하지만 **포획존의 존재를 모른다**.

즉 어떤 모델도 "잡히지 않으려고 트랩을 피한다"를 하지 않는다. 표적이 밀면
밀리는 물체에 가까우니, 뒤에서 미는 로봇 한 대로 충분한 게 당연하다 --
로봇 두 대가 필요해지려면 표적이 실제로 "안 잡히려고" 움직여야 한다.

이 모델은 매 스텝 8방향 후보를 점수로 평가한다:
  - 로봇에서 멀어지는가 (반응거리 안의 로봇만, 가까울수록 큰 가중)
  - **포획존에서 멀어지는가** (`trap_avoid_weight`로 강도 조절)
  - 직전 방향과 같은가 (momentum -- 매 스텝 홱홱 뒤집히지 않게)
  - 벽에 막히면 후보에서 제외
그리고 `ReactiveFlee`와 달리 **항상 움직인다**(실제 쥐는 가만히 안 있는다).

`trap_avoid_weight`를 0으로 두면 트랩을 신경 쓰지 않는 기존 모델과 비슷해지고,
키우면 "잡히기 싫어하는" 정도가 세진다 -- 이 값을 스윕해서 "로봇 한 대로는
부족하지만 두 대로는 되는" 난이도 구간이 존재하는지 찾는 데 쓴다.
"""
import numpy as np

from test.evasion_models.base import EvasionModel

_N = 8
_DIRECTIONS = np.array([[np.cos(2 * np.pi * i / _N), np.sin(2 * np.pi * i / _N)] for i in range(_N)])


class TrapAvoidingFlee(EvasionModel):
    """로봇을 피하면서 **포획존도 피하는** 표적. 항상 움직인다."""

    def __init__(
        self,
        max_speed_mps: float,
        flee_reaction_distance_m: float,
        grid_map,
        trap_position,
        trap_avoid_weight: float = 1.0,
        lookahead_m: float = 0.6,
        momentum_bonus: float = 0.5,
        temperature: float = 0.0,
        rng=None,
    ) -> None:
        self.max_speed_mps = max_speed_mps
        self.flee_reaction_distance_m = flee_reaction_distance_m
        self.grid_map = grid_map
        self.trap_position = np.asarray(trap_position, dtype=float)
        self.trap_avoid_weight = trap_avoid_weight
        self.lookahead_m = lookahead_m
        self.momentum_bonus = momentum_bonus
        # 방향 선택의 무작위성. 0이면 항상 최고점(결정론적), 크면 점수 차이를
        # 무시하고 아무 방향이나 고른다. 실제 쥐는 완벽한 최적화 기계가
        # 아니므로 0보다 커야 현실적이다 -- 그리고 결정론적이면 회피 강도에
        # 따라 성공률이 100%/0%로만 갈리고 중간이 없다(실측).
        self.temperature = temperature
        self.rng = rng or np.random.default_rng()
        self._last_dir = None

    def _blocked(self, position, direction):
        """그 방향으로 lookahead_m 안에 벽이 있는지."""
        steps = max(int(self.lookahead_m / self.grid_map.config.resolution_m), 1)
        for i in range(1, steps + 1):
            probe = position + direction * (i * self.grid_map.config.resolution_m)
            try:
                row, col = self.grid_map.world_to_cell(*probe)
            except ValueError:
                return True
            if not self.grid_map.in_bounds(row, col) or self.grid_map.is_obstacle(row, col):
                return True
        return False

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        position = target_state[:2]
        trap_dist = float(np.linalg.norm(position - self.trap_position))

        scored = []
        for direction in _DIRECTIONS:
            if self._blocked(position, direction):
                continue
            score = 0.0
            # 로봇에서 멀어지기: 반응거리 안의 로봇만 신경 쓴다(가까울수록 크게).
            for robot_pos in robot_positions:
                away = position - robot_pos
                dist = float(np.linalg.norm(away))
                if 1e-6 < dist < self.flee_reaction_distance_m:
                    urgency = (self.flee_reaction_distance_m - dist) / self.flee_reaction_distance_m
                    score += 3.0 * urgency * float(np.dot(direction, away / dist))
            # 포획존에서 멀어지기 -- 이게 기존 모델에 없던 항이다.
            if trap_dist > 1e-6:
                away_trap = (position - self.trap_position) / trap_dist
                # 트랩에 가까울수록 더 강하게 벗어나려 한다(1m 안에서 급격히).
                proximity = float(np.clip(1.5 - trap_dist, 0.0, 1.5))
                score += self.trap_avoid_weight * (0.5 + proximity) * float(np.dot(direction, away_trap))
            if self._last_dir is not None:
                score += self.momentum_bonus * float(np.dot(direction, self._last_dir))
            scored.append((score, direction))

        if not scored:
            self._last_dir = None
            return np.zeros(2)
        if self.temperature <= 1e-9:
            best_dir = max(scored, key=lambda sd: sd[0])[1]
        else:
            # 소프트맥스 표집: 점수가 높은 방향을 더 자주 고르되 가끔 실수한다.
            values = np.array([sd[0] for sd in scored]) / self.temperature
            weights = np.exp(values - values.max())
            best_dir = scored[int(self.rng.choice(len(scored), p=weights / weights.sum()))][1]
        self._last_dir = best_dir
        return best_dir * self.max_speed_mps
