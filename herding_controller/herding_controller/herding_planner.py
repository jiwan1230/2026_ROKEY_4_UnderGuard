# herding_controller/herding_controller/herding_planner.py
"""Driving Point(Driver 목표점)와 Blocking Point(Blocker 목표점)를 계산한다."""
from dataclasses import dataclass

import numpy as np

from herding_controller.escape_model import EscapeEstimate
from herding_controller.grid_map import GridMap


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
    """Driver의 목표점을 반환한다: 타겟 뒤쪽, 캡처 목표점의 반대 방향."""
    to_target = target_pos - robot_pos
    dist = np.linalg.norm(to_target)
    if dist < config.panic_distance_m:
        retreat_dir = -to_target / dist if dist > 1e-6 else np.array([1.0, 0.0])
        retreat_point = robot_pos + retreat_dir * (config.panic_distance_m - dist)
        return DrivingResult(point=retreat_point, is_panic=True)

    u = target_pos - goal_pos
    norm = np.linalg.norm(u)
    u = u / norm if norm > 1e-6 else np.array([1.0, 0.0])

    drive_distance = config.drive_distance_m
    to_goal = -u
    speed = np.linalg.norm(target_vel)
    if speed > 1e-6:
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
    """Blocker의 목표점을 반환한다: 목표 반구(goal hemisphere) 밖에서 가장 가능성 높은 도주 경로."""
    to_goal = goal_pos - target_pos
    norm = np.linalg.norm(to_goal)
    to_goal = to_goal / norm if norm > 1e-6 else np.array([1.0, 0.0])

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
