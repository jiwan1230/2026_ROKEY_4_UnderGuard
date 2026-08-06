# herding_controller_dual/herding_controller_dual/herding_planner.py
"""Driving Point(Driver 목표점)와 Blocking Point(Blocker 목표점)를 계산한다."""
from dataclasses import dataclass

import numpy as np

from herding_controller_dual.escape_model import EscapeEstimate
from herding_controller_dual.grid_map import GridMap


@dataclass
class PlannerConfig:
    """Driver가 타겟을 얼마나 공격적으로 압박할지를 제어하는 임계값들."""
    drive_distance_m: float
    panic_distance_m: float
    alignment_threshold: float
    drive_distance_ease_factor: float
    block_lookahead_m: float


@dataclass
class DrivingResult:
    """Driver의 목표점과 panic-distance 후퇴 중인지 여부."""
    point: np.ndarray
    is_panic: bool


def compute_driving_point(
    target_pos: np.ndarray,
    target_vel: np.ndarray,
    goal_pos: np.ndarray,
    robot_pos: np.ndarray,
    config: PlannerConfig,
) -> DrivingResult:
    """Driver의 목표점을 반환한다: 타겟 뒤쪽, 캡처 목표점의 반대 방향.

    핵심 기하: u = normalize(target_pos - goal_pos)는 "포획존에서 타겟을
    향하는" 단위벡터다. Driver의 목표점 = target_pos + drive_distance_m * u,
    즉 **타겟을 기준으로 포획존과 정반대편**에 위치한다. 표적은 접근하는
    로봇으로부터 도망치는 반응(reactive flee)을 하므로, Driver가 이
    "포획존 반대편" 지점에 서면 표적은 자연히 포획존 방향으로 밀려난다 —
    로봇이 표적을 직접 붙잡거나 미는 게 아니라, 표적 자신의 도주 본능을
    역이용해서 원하는 방향으로 유도하는 것이 이 알고리즘의 핵심 아이디어.
    """
    to_target = target_pos - robot_pos
    dist = np.linalg.norm(to_target)
    if dist < config.panic_distance_m:
        # Panic-retreat: Driver가 타겟에 panic_distance_m보다 가까이
        # 붙어버린 경우 (예: 급격한 방향전환으로 타겟이 로봇 쪽으로 순간
        # 이동했을 때). 이 상태에서 정상적인 "타겟 뒤쪽" 목표점을 계속
        # 요구하면 로봇이 타겟을 향해 더 다가가라는 명령을 받게 되어
        # 표적이 과도하게 겁먹고 예측 불가능하게 튈 수 있다. 대신 로봇
        # 자신의 현재 위치를 기준으로 타겟 반대쪽으로 즉시 물러나는 지점을
        # 준다 — "너무 가까워졌으니 우선 거리부터 벌리자."
        retreat_dir = -to_target / dist if dist > 1e-6 else np.array([1.0, 0.0])
        retreat_point = robot_pos + retreat_dir * (config.panic_distance_m - dist)
        return DrivingResult(point=retreat_point, is_panic=True)

    u = target_pos - goal_pos
    norm = np.linalg.norm(u)
    u = u / norm if norm > 1e-6 else np.array([1.0, 0.0])

    drive_distance = config.drive_distance_m
    to_goal = -u  # goal_pos 쪽을 향하는 단위벡터 (u의 반대)
    speed = np.linalg.norm(target_vel)
    if speed > 1e-6:
        # Alignment easing: 타겟이 이미 스스로 포획존 쪽으로 가고 있다면
        # (진행 방향과 to_goal의 내적이 alignment_threshold 이상이면),
        # Driver가 평소처럼 바짝 붙어 압박할 필요가 없다 — 오히려 너무
        # 가까이 붙으면 표적이 놀라서 엉뚱한 방향으로 급선회할 위험이
        # 있다. 이런 경우 drive_distance를 ease_factor(>1)만큼 늘려
        # 목표점을 타겟에서 더 멀리 물러나게 하고, 로봇은 "이미 잘
        # 가고 있으니 살짝만 압박"하는 셈이 된다.
        alignment = float(np.dot(target_vel / speed, to_goal))
        if alignment >= config.alignment_threshold:
            drive_distance *= config.drive_distance_ease_factor

    return DrivingResult(point=target_pos + drive_distance * u, is_panic=False)


def compute_blocking_point(
    target_pos: np.ndarray,
    goal_pos: np.ndarray,
    escape_estimate: EscapeEstimate,
    grid_map: GridMap,
    config: PlannerConfig,
) -> np.ndarray:
    """Blocker의 목표점을 반환한다: 목표 반구(goal hemisphere) 밖에서 가장 가능성 높은 도주 경로.

    4단계 알고리즘: (1) escape_model이 예측한 8방향 확률을 확률 내림차순으로
    순회하되, "포획존 쪽을 향하는" 방향(목표 반구, dots > 0)은 건너뛴다 —
    그 방향은 Driver가 이미 처리 중이므로 Blocker가 갈 필요가 없다.
    (2) 남은 후보 중 그 방향의 lookahead 지점이 장애물이면 건너뛴다 —
    자연적으로 막힌 도주로는 애초에 표적이 쓸 수 없으므로 지킬 필요가
    없다. (3) 살아남은 첫 후보(= 목표 반구 밖에서 가장 확률 높고 실제로
    갈 수 있는 도주로)를 Blocker의 목표점으로 채택. (4) 목표 반구 밖
    후보가 전부 막혀 있으면 반구 조건을 완화해 전체 8방향 중 최고확률
    빈 경로를 대신 취하고, 그마저 없으면(사방이 막힘) 제자리를 지킨다 —
    아래 코드의 세 개 루프가 각각 이 (3)/(4-완화)/(4-완전차단) 단계다.
    """
    to_goal = goal_pos - target_pos
    norm = np.linalg.norm(to_goal)
    to_goal = to_goal / norm if norm > 1e-6 else np.array([1.0, 0.0])

    # dots[i] > 0 은 방향 i가 to_goal과 같은 반구(포획존 쪽)에 있다는 뜻.
    dots = escape_estimate.directions @ to_goal
    candidate_order = np.argsort(escape_estimate.probabilities)[::-1]

    for index in candidate_order:
        if dots[index] > 0:
            continue  # 방향이 목표 반구 내부에 있으므로 건너뜀 (2-4 step 1)
        direction = escape_estimate.directions[index]
        point = target_pos + direction * config.block_lookahead_m
        try:
            row, col = grid_map.world_to_cell(*point)
        except ValueError:
            continue
        if grid_map.is_obstacle(row, col):
            continue  # 경로가 이미 자연적으로 막혀 있으므로 차선책 경로 시도 (2-4 step 4)
        return point

    # 목표 반구 밖의 후보가 모두 막혀 있거나 그리드 밖에 있음: 목표 반구
    # 선호도를 완화하여, 무효인 것으로 알려진 지점을 반환하는 대신 여전히
    # 장애물이 없고 범위 내에 있는(전체 8방향 중) 최고 확률 방향을 취한다.
    for index in candidate_order:
        direction = escape_estimate.directions[index]
        point = target_pos + direction * config.block_lookahead_m
        try:
            row, col = grid_map.world_to_cell(*point)
        except ValueError:
            continue
        if grid_map.is_obstacle(row, col):
            continue
        return point

    # 모든 방향(8방향 전부)이 장애물에 막혀 있거나 그리드 밖에 있음: 타겟이
    # 완전히 갇혀 있어 근처에 유효한 blocking point가 없다. Blocker를 벽이나
    # 그리드 밖으로 보내는 대신 제자리에 머문다.
    return target_pos.copy()
