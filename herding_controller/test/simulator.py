# herding_controller/test/simulator.py
"""헤드리스 2D 물리 시뮬레이터: 점질량 로봇 + 타겟 회피 모델, ROS 없이 동작."""
from dataclasses import dataclass, field

import numpy as np

from herding_controller.herding_core import HerdingConfig, HerdingCore, Observation
from herding_controller.state_machine import FSMState
from test.evasion_models.base import EvasionModel

# 아레나의 위쪽 경계는 배타적(exclusive)이다 (GridMap.world_to_cell이 내림 처리하므로),
# 그래서 살짝 안쪽으로 클램프하지 않으면 위/오른쪽 벽에 붙은 물체가 그리드 밖으로 벗어난다.
_EDGE_EPS = 1e-9


@dataclass
class SimulatorConfig:
    """2D 허딩 시뮬레이션의 물리적 한계와 시행 지속 시간."""
    robot_max_speed_mps: float = 0.3
    target_max_speed_mps: float = 0.4
    dt: float = 0.1
    max_sim_time_sec: float = 120.0
    robot_gain: float = 1.0


@dataclass
class TrialResult:
    """한 허딩 시행의 결과와 텔레메트리."""
    success: bool
    duration_sec: float
    panic_count: int
    role_swap_count: int
    mean_latency_ms: float
    target_trajectory: np.ndarray
    robot1_trajectory: np.ndarray
    robot2_trajectory: np.ndarray
    escape_snapshot: np.ndarray | None = None
    min_robot_target_dist: float = field(default=float("inf"))


def _move_toward(position: np.ndarray, goal: np.ndarray, max_speed: float, gain: float, dt: float) -> np.ndarray:
    """`position`을 `goal` 쪽으로 이동시킨다. 최대 max_speed * gain * dt 미터만큼 이동한다."""
    direction = goal - position
    dist = np.linalg.norm(direction)
    if dist < 1e-9:
        return position
    step = min(max_speed * gain * dt, dist)
    return position + (direction / dist) * step


def _update_heading(old_pos: np.ndarray, new_pos: np.ndarray, heading: np.ndarray) -> np.ndarray:
    """로봇의 변위 방향을 단위 벡터로 반환한다. 움직이지 않았다면 이전 방향을 그대로 반환한다.

    유효성 여부는 오직 로봇이 실제로 움직였는지에만 달려 있으며 -- 어디에 도착했는지와는
    무관하다 -- 그래서 맵의 x 음수 영역에 있는 로봇도 방향(heading) 갱신이 계속 이루어진다.
    """
    delta = np.asarray(new_pos, dtype=float) - np.asarray(old_pos, dtype=float)
    norm = float(np.linalg.norm(delta))
    if norm < 1e-9:
        return heading
    return delta / norm


def _arena_bounds(config: HerdingConfig) -> tuple[np.ndarray, np.ndarray]:
    """벽으로 둘러싸인 아레나의 (x, y) 하한/상한 모서리. 그리드 범위와 정확히 일치한다."""
    low = np.array([config.grid_origin_x_m, config.grid_origin_y_m], dtype=float)
    span = np.array([config.grid_width_cells, config.grid_height_cells], dtype=float)
    return low, low + span * config.grid_resolution_m


def _clamp_to_arena(position: np.ndarray, low: np.ndarray, high: np.ndarray) -> np.ndarray:
    """물체를 아레나 벽 안쪽에 머무르게 한다.

    이스케이프 모델은 이미 그리드 밖의 모든 인접 셀을 벽으로 취급하므로, 벽 처리가 없는
    시뮬레이터는 알고리즘이 불가능하다고 여기는 위치로 (그리고 이스케이프 분포가 전혀
    존재하지 않는 위치로) 타겟이 벗어나게 만들 수 있다.
    """
    return np.clip(np.asarray(position, dtype=float), low, high - _EDGE_EPS)


def _is_obstacle_at(core: HerdingCore, position: np.ndarray) -> bool:
    """`position`이 core 그리드의 점유된 셀 안에 있으면 True를 반환한다."""
    try:
        row, col = core.grid_map.world_to_cell(float(position[0]), float(position[1]))
    except (ValueError, TypeError):
        return False
    return bool(core.grid_map.obstacle_mask[row, col])


def _boundary_ring_mask(config: HerdingConfig) -> np.ndarray:
    """벽만 있는 빈 방의 점유 상태: 가장 바깥쪽 셀 링이 단단한 벽이다.

    아레나 벽은 단순히 클램핑으로 강제되는 것이 아니라 장애물 셀로서 *실제로 보여야*
    한다. WallHugger(그리고 이를 감싸는 NoisyHuman)는 붙어서 이동할 점유 셀을 찾고,
    없으면 정지한다. 또한 EscapeModel의 향촉성(thigmotaxis) 항도 따라갈 벽이 필요하다.
    모두 False인 마스크는 물리적으로는 닫혀 있다고 하는 방 안에서 둘 다 무한히 열린
    평면을 모델링하게 만든다.
    """
    mask = np.zeros((config.grid_height_cells, config.grid_width_cells), dtype=bool)
    mask[0, :] = mask[-1, :] = True
    mask[:, 0] = mask[:, -1] = True
    return mask


def _bind_model_to_arena(evasion_model: EvasionModel, grid_map) -> None:
    """그리드를 참조하는 모든 회피 모델이 이번 시행이 시뮬레이션하는 아레나를 가리키도록 다시 연결한다.

    WallHugger(그리고 이를 감싸는 NoisyHuman)는 step()의 `obstacle_map` 인자가 아니라
    생성 시 전달받은 GridMap을 통해 벽을 판별한다. 호출자는 별도의 프로브 core의
    그리드 맵으로 모델을 생성하는데 -- 이는 run_trial이 만드는 GridMap 객체와는 다른
    객체이다 -- 따라서 재연결하지 않으면 물리 엔진은 벽으로 둘러싸인 아레나에서 동작하는데
    모델은 빈 세계를 읽게 되어 아무것도 따라가지 못한다.
    """
    for candidate in (evasion_model, getattr(evasion_model, "_wall_hugger", None)):
        if candidate is not None and hasattr(candidate, "grid_map"):
            candidate.grid_map = grid_map


def _step_body(core: HerdingCore, position: np.ndarray, proposed: np.ndarray,
               low: np.ndarray, high: np.ndarray) -> np.ndarray:
    """물체를 `proposed` 위치로 이동시킨다. 아레나 안으로 클램프되고 장애물 셀에 막힌다.

    로봇과 타겟은 동일한 규칙을 따른다: 벽으로 걸어 들어간 물체는 벽을 통과하지 않고
    그 자리에서 멈춘다.
    """
    moved = _clamp_to_arena(proposed, low, high)
    if _is_obstacle_at(core, moved):
        return np.asarray(position, dtype=float).copy()
    return moved


def run_trial(
    herding_config: HerdingConfig,
    evasion_model: EvasionModel,
    seed: int,
    sim_config: SimulatorConfig = SimulatorConfig(),
    obstacle_mask: np.ndarray | None = None,
    control_mode: str = "algorithm",
) -> TrialResult:
    """허딩 시행 한 번을 시뮬레이션한다. control_mode: 'algorithm' | 'idle' | 'random'."""
    if control_mode not in ("algorithm", "idle", "random"):
        raise ValueError(f"unknown control_mode: {control_mode!r}")

    rng = np.random.default_rng(seed)
    core = HerdingCore(herding_config)
    if obstacle_mask is not None:
        # 호출자가 제공한 레이아웃은 그대로 사용되며, 조용히 벽으로 둘러싸이지 않는다.
        expected = (herding_config.grid_height_cells, herding_config.grid_width_cells)
        if tuple(np.shape(obstacle_mask)) != expected:
            raise ValueError(f"obstacle_mask shape {np.shape(obstacle_mask)} != expected {expected}")
        core.grid_map.obstacle_mask = np.asarray(obstacle_mask, dtype=bool)
    else:
        core.grid_map.obstacle_mask = _boundary_ring_mask(herding_config)
    _bind_model_to_arena(evasion_model, core.grid_map)

    low, high = _arena_bounds(herding_config)
    margin = herding_config.grid_resolution_m * 2
    spawn_low, spawn_high = low + margin, high - margin
    # target_state는 [x, y, vx, vy]이다: 회피 모델에는 위치와 속도가 함께 전달된다.
    target_state = np.array([
        rng.uniform(spawn_low[0], spawn_high[0]), rng.uniform(spawn_low[1], spawn_high[1]), 0.0, 0.0,
    ])
    robot1_pos = np.array([spawn_low[0], spawn_low[1]])
    robot2_pos = np.array([spawn_high[0], spawn_low[1]])
    robot1_heading = np.array([1.0, 0.0])
    robot2_heading = np.array([1.0, 0.0])

    target_traj, robot1_traj, robot2_traj = [], [], []
    panic_count, role_swap_count, latencies = 0, 0, []
    min_dist = float("inf")
    success, elapsed_sec = False, 0.0
    escape_snapshot = None

    steps = max(int(round(sim_config.max_sim_time_sec / sim_config.dt)), 0)
    for index in range(steps):
        sim_time_sec = index * sim_config.dt
        target_traj.append(target_state[:2].copy())
        robot1_traj.append(robot1_pos.copy())
        robot2_traj.append(robot2_pos.copy())

        # 최근접 거리는 주기 시작 시점뿐 아니라 주기의 모든 하위 스텝에서 샘플링된다:
        # 로봇이 먼저 움직이고 타겟이 나중에 반응하므로, 주기 중 가장 좁은 간격은 보통
        # (로봇은 움직였고 타겟은 아직 움직이지 않은) 중간 상태에서 발생하는데, 이는
        # 주기 시작 시점에만 측정하면 결코 볼 수 없다.
        tick_min = _closest_robot_distance(target_state[:2], robot1_pos, robot2_pos)

        observation = Observation(
            target_measurement=target_state[:2].copy(),  # 원시 센서 값: 위치 정보만
            robot1_pos=robot1_pos.copy(), robot2_pos=robot2_pos.copy(),
            robot1_heading=robot1_heading.copy(), robot2_heading=robot2_heading.copy(),
            occupancy=None, sim_time_sec=sim_time_sec, dt=sim_config.dt,
        )
        # 모든 제어 모드는 core를 한 스텝 진행시키므로 FSM, KF, occlusion 그리드, role
        # 상태는 동일하게 진행된다; 로봇이 목표를 가지고 무엇을 하는지만 다르다.
        output = core.step(observation)
        latencies.append(output.latency_ms)
        if output.role_swapped:
            role_swap_count += 1
        if output.escape_top3:
            escape_snapshot = np.array(output.escape_top3)

        elapsed_sec = sim_time_sec + sim_config.dt
        if output.fsm_state == FSMState.CAPTURED:
            success = True
            min_dist = min(min_dist, tick_min)
            if tick_min < herding_config.panic_distance_m:
                panic_count += 1
            break

        if control_mode == "algorithm":
            new_r1 = _move_toward(robot1_pos, output.robot1_goal, sim_config.robot_max_speed_mps,
                                  sim_config.robot_gain, sim_config.dt)
            new_r2 = _move_toward(robot2_pos, output.robot2_goal, sim_config.robot_max_speed_mps,
                                  sim_config.robot_gain, sim_config.dt)
        elif control_mode == "random":
            angle1, angle2 = rng.uniform(0, 2 * np.pi, size=2)
            travel = sim_config.robot_max_speed_mps * sim_config.robot_gain * sim_config.dt
            new_r1 = robot1_pos + np.array([np.cos(angle1), np.sin(angle1)]) * travel
            new_r2 = robot2_pos + np.array([np.cos(angle2), np.sin(angle2)]) * travel
        else:  # idle
            new_r1, new_r2 = robot1_pos.copy(), robot2_pos.copy()
        new_r1 = _step_body(core, robot1_pos, new_r1, low, high)
        new_r2 = _step_body(core, robot2_pos, new_r2, low, high)

        robot1_heading = _update_heading(robot1_pos, new_r1, robot1_heading)
        robot2_heading = _update_heading(robot2_pos, new_r2, robot2_heading)
        robot1_pos, robot2_pos = new_r1, new_r2
        tick_min = min(tick_min, _closest_robot_distance(target_state[:2], robot1_pos, robot2_pos))

        target_state = _advance_target(
            core, evasion_model, target_state, robot1_pos, robot2_pos, sim_config, low, high
        )
        tick_min = min(tick_min, _closest_robot_distance(target_state[:2], robot1_pos, robot2_pos))

        min_dist = min(min_dist, tick_min)
        # 타겟이 한 번이라도 panic 거리 이내에 들어온 제어 주기마다 한 번씩 카운트한다.
        # 주기 경계에 정확히 걸친 위반은 양쪽 주기 모두에 귀속된다; 안전 지표는
        # 놓치는 것보다 과다 집계하는 편이 낫다.
        if tick_min < herding_config.panic_distance_m:
            panic_count += 1

    return TrialResult(
        success=success, duration_sec=elapsed_sec, panic_count=panic_count,
        role_swap_count=role_swap_count,
        mean_latency_ms=float(np.mean(latencies)) if latencies else 0.0,
        target_trajectory=np.array(target_traj, dtype=float).reshape(-1, 2),
        robot1_trajectory=np.array(robot1_traj, dtype=float).reshape(-1, 2),
        robot2_trajectory=np.array(robot2_traj, dtype=float).reshape(-1, 2),
        escape_snapshot=escape_snapshot, min_robot_target_dist=min_dist,
    )


def _closest_robot_distance(target_pos: np.ndarray, robot1_pos: np.ndarray, robot2_pos: np.ndarray) -> float:
    """타겟으로부터 더 가까운 로봇까지의 거리."""
    return float(min(
        np.linalg.norm(target_pos - robot1_pos),
        np.linalg.norm(target_pos - robot2_pos),
    ))


def _advance_target(
    core: HerdingCore,
    evasion_model: EvasionModel,
    target_state: np.ndarray,
    robot1_pos: np.ndarray,
    robot2_pos: np.ndarray,
    sim_config: SimulatorConfig,
    low: np.ndarray,
    high: np.ndarray,
) -> np.ndarray:
    """회피 모델이 명령한 속도를 적분하여 새로운 [x, y, vx, vy] 상태를 만든다."""
    commanded = np.asarray(
        evasion_model.step(target_state, [robot1_pos, robot2_pos], core.grid_map.obstacle_mask, sim_config.dt),
        dtype=float,
    ).reshape(2)
    speed = float(np.linalg.norm(commanded))
    if speed > sim_config.target_max_speed_mps:
        commanded = commanded / speed * sim_config.target_max_speed_mps

    position = target_state[:2]
    proposed = _step_body(core, position, position + commanded * sim_config.dt, low, high)
    # 저장되는 속도는 실제로 "달성된" 속도이다. 따라서 벽에 막혀 감속된 타겟은 실제로
    # 일어난 움직임을 보고하게 되며 -- 이는 다음 주기의 회피 모델, 이스케이프 모델의
    # 모멘텀 항, KF가 모두 일치해야 하는 값이다.
    achieved = (proposed - position) / sim_config.dt
    return np.concatenate([proposed, achieved])
