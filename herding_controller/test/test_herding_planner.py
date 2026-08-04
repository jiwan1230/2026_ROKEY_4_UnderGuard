# herding_controller/test/test_herding_planner.py
import numpy as np

from herding_controller.escape_model import EscapeEstimate
from herding_controller.grid_map import GridConfig, GridMap
from herding_controller.herding_planner import PlannerConfig, compute_blocking_point, compute_driving_point


def make_config():
    return PlannerConfig(
        drive_distance_m=0.8, panic_distance_m=0.35, alignment_threshold=0.7,
        drive_distance_ease_factor=1.3, block_lookahead_m=1.2,
    )


def test_driving_point_is_opposite_the_goal():
    config = make_config()
    target_pos = np.array([2.0, 2.0])
    goal_pos = np.array([5.0, 2.0])
    result = compute_driving_point(target_pos, np.zeros(2), goal_pos, np.array([1.0, 2.0]), config)
    # goal is to the +x side of target, so the driving point must be on the -x side
    assert result.point[0] < target_pos[0]
    assert result.is_panic is False


def test_panic_distance_triggers_retreat():
    config = make_config()
    target_pos = np.array([2.0, 2.0])
    robot_pos = np.array([2.1, 2.0])  # 0.1m away, inside panic_distance_m
    result = compute_driving_point(target_pos, np.zeros(2), np.array([5.0, 2.0]), robot_pos, config)
    assert result.is_panic is True
    # retreat point must be farther from the target than the robot currently is
    assert np.linalg.norm(result.point - target_pos) > np.linalg.norm(robot_pos - target_pos)


def test_blocking_point_excludes_goal_hemisphere():
    config = make_config()
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    target_pos = np.array([5.0, 5.0])
    goal_pos = np.array([8.0, 5.0])  # goal is due "E" of target
    directions = np.array(
        [[0, 1], [1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]], dtype=float
    )
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    probabilities = np.zeros(8)
    probabilities[2] = 1.0  # "E" (toward goal) has max probability but must be excluded
    probabilities[6] = 0.5  # "W" (away from goal) is the best allowed candidate
    estimate = EscapeEstimate(directions=directions, probabilities=probabilities, top_k_routes=[])
    point = compute_blocking_point(target_pos, goal_pos, estimate, grid, config)
    assert point[0] < target_pos[0]  # chosen route points away from the goal (west)
