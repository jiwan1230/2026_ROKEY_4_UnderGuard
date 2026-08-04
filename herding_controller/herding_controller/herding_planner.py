# herding_controller/herding_controller/herding_planner.py
"""Computes Driving Point (Driver goal) and Blocking Point (Blocker goal)."""
from dataclasses import dataclass

import numpy as np

from herding_controller.escape_model import EscapeEstimate
from herding_controller.grid_map import GridMap


@dataclass
class PlannerConfig:
    """Thresholds controlling how aggressively the Driver pressures the target."""
    drive_distance_m: float
    panic_distance_m: float
    alignment_threshold: float
    drive_distance_ease_factor: float
    block_lookahead_m: float


@dataclass
class DrivingResult:
    """The Driver's goal point and whether it is in a panic-distance retreat."""
    point: np.ndarray
    is_panic: bool


def compute_driving_point(
    target_pos: np.ndarray,
    target_vel: np.ndarray,
    goal_pos: np.ndarray,
    robot_pos: np.ndarray,
    config: PlannerConfig,
) -> DrivingResult:
    """Return the Driver's goal: behind the target, opposite the capture goal."""
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
    """Return the Blocker's goal: the most likely escape route outside the goal hemisphere."""
    to_goal = goal_pos - target_pos
    norm = np.linalg.norm(to_goal)
    to_goal = to_goal / norm if norm > 1e-6 else np.array([1.0, 0.0])

    dots = escape_estimate.directions @ to_goal
    candidate_order = np.argsort(escape_estimate.probabilities)[::-1]

    for index in candidate_order:
        if dots[index] > 0:
            continue  # direction is within the goal hemisphere, skip (2-4 step 1)
        direction = escape_estimate.directions[index]
        point = target_pos + direction * config.block_lookahead_m
        try:
            row, col = grid_map.world_to_cell(*point)
        except ValueError:
            continue
        if grid_map.is_obstacle(row, col):
            continue  # path already naturally blocked, try next-best route (2-4 step 4)
        return point

    # Every non-hemisphere candidate was blocked or off-grid: relax the goal-hemisphere
    # preference and take the highest-probability direction (from ALL 8) that is still
    # obstacle-clear and in-bounds, rather than returning a point known to be invalid.
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

    # Every direction (all 8) is obstacle-blocked or off-grid: the target is fully
    # boxed in and there is no valid blocking point nearby. Stay in place rather than
    # send the Blocker into a wall or off the grid.
    return target_pos.copy()
