# herding_controller/test/evasion_models/decisive_flee.py
"""주기적으로 두 로봇을 모두 고려해 '지금 가장 트인 방향'을 전략적으로 정하고
그 방향을 한동안 유지하며 도망친다 -- 기존 회피 모델(ReactiveFlee/WallHugger/
NoisyHuman)은 전부 "그 순간 반응거리(flee_reaction_distance_m) 안에 있는
로봇으로부터 곧장 도망친다"는 순간적/연속적 벡터합만 쓴다. 이 모델은 그
대신 "지금 어느 로봇이 어디 있든, 벽에 안 막히면서 두 로봇 모두로부터 가장
멀어지는 방향으로 결정적으로 튄다"는 이산적(discrete) 선택을 한다 --
"쥐가 옆으로 새는" 시나리오를 테스트하기 위한 모델(트러블슈팅 노트 참고).
"""
import numpy as np

from test.evasion_models.base import EvasionModel

_N_DIRECTIONS = 8
_DIRECTIONS = np.array([
    [np.cos(2 * np.pi * i / _N_DIRECTIONS), np.sin(2 * np.pi * i / _N_DIRECTIONS)]
    for i in range(_N_DIRECTIONS)
])


class DecisiveFlee(EvasionModel):
    """일정 주기마다 방향을 전략적으로 재선택하고, 그 사이엔 방향을 고수한다."""

    def __init__(
        self,
        max_speed_mps: float,
        flee_reaction_distance_m: float,
        grid_map,
        decision_interval_sec: float = 0.6,
        lookahead_m: float = 1.0,
        momentum_bonus: float = 0.3,
        panic_fraction: float = 0.5,
        robot_hard_block: bool = False,
        robot_block_radius_m: float = 0.35,
    ) -> None:
        """속도 상한, 즉시반응 임계값(반응거리의 panic_fraction배), 그리드를 저장한다.

        decision_interval_sec: 이 주기마다 방향을 다시 계산한다(그 사이엔 유지).
        lookahead_m: 후보 방향이 벽으로 막혀 있는지 이 거리만큼 앞을 살펴 확인한다.
        momentum_bonus: 직전 방향과 같은 방향일수록 가산점을 줘서, 매 주기 홱홱
            뒤집히지 않고 대체로 일관된 경로를 타게 한다.
        panic_fraction: 로봇이 반응거리의 이 비율보다 더 가까이 붙으면(진짜 위급
            상황) 주기를 기다리지 않고 즉시 재평가한다.
        robot_hard_block: True면 로봇도 벽처럼 취급한다 -- 후보 방향의
            lookahead 경로가 로봇의 robot_block_radius_m 안을 지나가면 그
            방향은 (약한 가중치 페널티가 아니라) 후보에서 완전히 제외된다.
            기본은 False로, 기존 어블레이션 결과와의 재현성을 유지한다.
        robot_block_radius_m: robot_hard_block=True일 때, 로봇 주변에서
            "몸으로 막고 있다"고 볼 반경. 로봇 실측 풋프린트가 없어 대략적인
            값을 실험적으로 사용한다(캡처 반경 0.3m과 비슷한 수준).
        """
        self.max_speed_mps = max_speed_mps
        self.flee_reaction_distance_m = flee_reaction_distance_m
        self.grid_map = grid_map
        self.decision_interval_sec = decision_interval_sec
        self.lookahead_m = lookahead_m
        self.momentum_bonus = momentum_bonus
        self.panic_distance_m = flee_reaction_distance_m * panic_fraction
        self.robot_hard_block = robot_hard_block
        self.robot_block_radius_m = robot_block_radius_m
        self._committed_dir: np.ndarray | None = None
        self._elapsed_sec = decision_interval_sec  # 첫 스텝에서 바로 결정하도록

    def step(self, target_state: np.ndarray, robot_positions: list, obstacle_map, dt: float) -> np.ndarray:
        target_pos = target_state[:2]
        self._elapsed_sec += dt
        immediate_threat = any(
            np.linalg.norm(target_pos - r) < self.panic_distance_m for r in robot_positions
        )
        if self._committed_dir is None or immediate_threat or self._elapsed_sec >= self.decision_interval_sec:
            self._committed_dir = self._choose_direction(target_pos, robot_positions)
            self._elapsed_sec = 0.0
        if self._committed_dir is None:
            return np.zeros(2)
        return self._committed_dir * self.max_speed_mps

    def _choose_direction(self, target_pos: np.ndarray, robot_positions: list) -> np.ndarray | None:
        """8방향 후보 중, 벽에 안 막히면서 로봇들로부터 가장 멀어지는 방향을 고른다."""
        best_dir, best_score = None, -np.inf
        for direction in _DIRECTIONS:
            if self._blocked(target_pos, direction, robot_positions):
                continue
            score = 0.0
            for robot_pos in robot_positions:
                away = target_pos - robot_pos
                dist = np.linalg.norm(away)
                if dist < 1e-6:
                    continue
                # 하드 컷오프 없음(기존 모델과의 핵심 차이): 멀리 있는 로봇도
                # 약하게나마 계속 고려한다 -- "지금 안 보여도 저기 있다는 걸
                # 감안해서 경로를 고른다"는 전략적 판단을 표현한다.
                weight = 1.0 / (dist + 0.3)
                score += max(0.0, float(np.dot(direction, away / dist))) * weight
            if self._committed_dir is not None:
                score += self.momentum_bonus * float(np.dot(direction, self._committed_dir))
            if score > best_score:
                best_dir, best_score = direction, score
        return best_dir

    def _blocked(self, target_pos: np.ndarray, direction: np.ndarray, robot_positions: list) -> bool:
        """lookahead_m 앞까지 일정 간격으로 샘플링해 벽(+옵션에 따라 로봇)이
        있으면 그 방향을 배제한다."""
        steps = max(1, int(self.lookahead_m / max(self.grid_map.config.resolution_m, 0.01)))
        for i in range(1, steps + 1):
            probe = target_pos + direction * (self.lookahead_m * i / steps)
            try:
                row, col = self.grid_map.world_to_cell(*probe)
            except ValueError:
                return True  # 그리드 밖 -- 안전하게 막힌 것으로 취급
            if self.grid_map.is_obstacle(row, col):
                return True
            if self.robot_hard_block:
                for robot_pos in robot_positions:
                    if np.linalg.norm(probe - robot_pos) < self.robot_block_radius_m:
                        return True
        return False
