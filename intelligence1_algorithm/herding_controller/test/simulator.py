# herding_controller/test/simulator.py
"""헤드리스 2D 물리 시뮬레이터: 점질량 로봇 + 타겟 회피 모델, ROS 없이 동작."""
from dataclasses import dataclass, field

import numpy as np

from herding_controller.herding_core import HerdingConfig, HerdingCore, Observation
from herding_controller.state_machine import FSMState
from test import real_map_arena
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
    # 로봇 1(Driver/로봇 A)이 "순찰하다가 표적을 방금 발견했다"고 가정할 때
    # 표적으로부터 떨어져 있는 거리 범위(m). 실제 운용에서 A는 발견한 순간의
    # 위치에서 추격을 시작하므로, 방 반대편 고정 스폰 지점에서 시작하는 것은
    # 현실을 반영하지 못한다 (트러블슈팅 노트 참고).
    driver_discovery_min_m: float = 0.3
    driver_discovery_max_m: float = 1.5


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
    # 실제 맵 시행(run_trial_real_map)에서만 채워짐: 표적을 순찰 중 처음
    # 발견한 시각과 어느 포획구역이 목표였는지.
    discovery_time_sec: float | None = None
    goal_name: str | None = None


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


def _spawn_driver_near_target(
    target_pos: np.ndarray, rng: np.random.Generator, low: np.ndarray, high: np.ndarray,
    sim_config: "SimulatorConfig",
) -> np.ndarray:
    """로봇 1(Driver/로봇 A)을 "방금 표적을 발견한 순찰 위치"에 스폰시킨다.

    로봇 2(Blocker/로봇 B)와 달리 로봇 1은 충전소 같은 고정 지점에서
    시작하지 않는다: 실제 운용에서 A는 순찰하다가 발견한 그 자리에서 곧장
    추격을 시작하므로, 표적으로부터 [driver_discovery_min_m,
    driver_discovery_max_m] 범위의 무작위 방향/거리에 스폰시켜 이를
    근사한다.
    """
    angle = rng.uniform(0.0, 2.0 * np.pi)
    dist = rng.uniform(sim_config.driver_discovery_min_m, sim_config.driver_discovery_max_m)
    point = target_pos + dist * np.array([np.cos(angle), np.sin(angle)])
    return _clamp_to_arena(point, low, high)


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
    # 로봇 2(Blocker/로봇 B)는 충전소 같은 고정 지점에서 대기하다 투입된다고
    # 가정해 고정 스폰을 유지한다. 로봇 1(Driver/로봇 A)은 순찰하다가 방금
    # 표적을 발견한 위치에서 시작해야 하므로 표적 근처에 스폰시킨다 (고정된
    # 먼 구석에서 시작하면, 역할이 "그때그때 유리한 로봇"이 아니라 항상
    # 로봇 1로 고정된 지금 구조에서 비현실적으로 나쁜 위치 관계를 강제하게
    # 된다).
    # spawn_low/spawn_high(마진 적용)로 클램프한다: low/high(아레나 전체)로
    # 클램프하면 로봇이 경계 벽 셀 바로 위/안쪽에 스폰될 수 있다.
    robot1_pos = _spawn_driver_near_target(target_state[:2], rng, spawn_low, spawn_high, sim_config)
    robot2_pos = np.array([spawn_high[0], spawn_low[1]])
    robot1_heading = np.array([1.0, 0.0])
    robot2_heading = np.array([1.0, 0.0])

    target_traj, robot1_traj, robot2_traj = [], [], []
    panic_count, role_swap_count, latencies = 0, 0, []
    min_dist = float("inf")
    success, elapsed_sec = False, 0.0
    escape_snapshot = None

    # 덫(포획존)에 닿은 표적은 물리적으로 걸려서 더 이상 움직이지 못한다
    # (2026-08-08). 그 전까지는 표적이 계속 움직일 수 있고 FSM이 "반경 안에
    # capture_hold_sec 동안 머무르기"만 요구해서, **스스로 멈추는 표적만**
    # 잡을 수 있었다 -- 주 검증 모델 ReactiveFlee가 로봇이 멀면 속도 0을
    # 반환해 가만히 서 있는 덕에 잡혔던 것이고, 실제 쥐처럼 계속 움직이는
    # 모델은 원리적으로 포획이 불가능했다(실측 0%). 덫은 닿으면 걸리는
    # 물건이므로 이렇게 모델링하는 게 물리적으로 맞다.
    goal_xy = np.array([herding_config.capture_zone_x_m, herding_config.capture_zone_y_m])
    trapped = False

    steps = max(int(round(sim_config.max_sim_time_sec / sim_config.dt)), 0)
    for index in range(steps):
        sim_time_sec = index * sim_config.dt
        if not trapped and float(np.linalg.norm(target_state[:2] - goal_xy)) <= herding_config.capture_radius_m:
            trapped = True
            target_state = np.array([target_state[0], target_state[1], 0.0, 0.0])
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

        if not trapped:
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
    step_fn=None,
) -> np.ndarray:
    """회피 모델이 명령한 속도를 적분하여 새로운 [x, y, vx, vy] 상태를 만든다.

    `step_fn`(기본 `_step_body`)으로 실제 이동 규칙을 바꿔 끼울 수 있다 --
    실제 맵처럼 좁은 문턱이 있는 아레나에서는 표적도 로봇과 마찬가지로
    막히면 옆으로 미끄러지는 이동이 필요하다(그렇지 않으면 회피 모델이
    벽을 향한 도주 방향을 계속 명령해도 물리 엔진이 매번 그 자리에
    멈춰 세워서, 표적이 구석에서 영원히 얼어붙는다 -- 트러블슈팅 노트의
    "케이스 A" 문턱 정지와 동일한 근본 원인).
    """
    if step_fn is None:
        step_fn = _step_body
    commanded = np.asarray(
        evasion_model.step(target_state, [robot1_pos, robot2_pos], core.grid_map.obstacle_mask, sim_config.dt),
        dtype=float,
    ).reshape(2)
    speed = float(np.linalg.norm(commanded))
    if speed > sim_config.target_max_speed_mps:
        commanded = commanded / speed * sim_config.target_max_speed_mps

    position = target_state[:2]
    proposed = step_fn(core, position, position + commanded * sim_config.dt, low, high)
    # 저장되는 속도는 실제로 "달성된" 속도이다. 따라서 벽에 막혀 감속된 타겟은 실제로
    # 일어난 움직임을 보고하게 되며 -- 이는 다음 주기의 회피 모델, 이스케이프 모델의
    # 모멘텀 항, KF가 모두 일치해야 하는 값이다.
    achieved = (proposed - position) / sim_config.dt
    return np.concatenate([proposed, achieved])


def run_trial_real_map(
    herding_config: HerdingConfig,
    evasion_model: EvasionModel,
    seed: int,
    sim_config: SimulatorConfig = SimulatorConfig(),
) -> TrialResult:
    """실제 SLAM 맵(`herding_controller/maps/room_map.pgm`) 위에서 허딩 시행 한 번을 시뮬레이션한다.

    `herding_config.capture_zone_x_m/y_m`는 검증 대상 트랩 좌표로 이미
    설정되어 있어야 한다(`run_real_map_trials`가 트랩별로 다른 config를
    만들어 넘긴다). 로봇 1(Driver가 *될* 로봇)은 발견 전까지
    `real_map_arena.PATROL_WAYPOINTS`를 순찰하고, 센서 반경
    (`SENSOR_RANGE_M`) 안에 표적이 들어오면 그 순간부터 `HerdingCore.step()`이
    계산하는 `robot1_goal`(참고 지점)을 따라 추격한다. 로봇 2(Blocker)는
    항상 `HerdingCore.step()`의 `robot2_goal`(실제 알고리즘 출력)을 따른다
    -- 발견 전에는 FSM이 SEARCH 상태라 제자리를 지킨다.

    벽 하나 없는 `run_trial()`의 단순 아레나와 달리 이 맵은 좁은 문턱이
    있으므로, 로봇 이동에는 `_step_body`가 아니라 `real_map_arena`의
    벽 회피(potential field) + 슬라이딩(16방향 국소 우회) 이동을 쓴다.
    """
    rng = np.random.default_rng(seed)
    obstacle_mask = real_map_arena.load_room_obstacle_mask()
    core = HerdingCore(herding_config)
    core.grid_map.obstacle_mask = obstacle_mask
    distance_field = real_map_arena.build_distance_field(obstacle_mask)
    # 미터 단위 거리장 -- 로봇/표적이 자기 몸체 반지름만큼 벽에서 떨어져 있는지
    # 판정하는 데 쓴다 (2026-08-08 추가, 그 전엔 둘 다 점으로 취급했다).
    clearance_m = real_map_arena.clearance_field_m(obstacle_mask)
    _bind_model_to_arena(evasion_model, core.grid_map)

    grid = core.grid_map.config
    low = np.array([grid.origin_x_m, grid.origin_y_m])
    high = low + np.array([grid.width_cells, grid.height_cells]) * grid.resolution_m

    target_spawn = real_map_arena.sample_free_spawn(
        core.grid_map, rng, min_clear_m=max(0.3, real_map_arena.TARGET_RADIUS_M),
        exclude_points=[real_map_arena.ROBOT_A_SPAWN, real_map_arena.ROBOT_B_SPAWN]
        + list(real_map_arena.TRAPS.values()),
        exclude_radius_m=0.6,
    )
    target_state = np.array([target_spawn[0], target_spawn[1], 0.0, 0.0])
    robot1_pos = real_map_arena.ROBOT_A_SPAWN.copy()
    robot2_pos = real_map_arena.ROBOT_B_SPAWN.copy()
    robot1_heading = np.array([1.0, 0.0])
    robot2_heading = np.array([1.0, 0.0])
    prev_robot1_pos = robot1_pos.copy()
    prev_robot2_pos = robot2_pos.copy()
    prev_target_pos = target_state[:2].copy()

    def _target_step_fn(core_arg, position, proposed, low_arg, high_arg):
        # 표적도 로봇처럼 막히면 미끄러지듯 우회해야 한다 -- 그렇지 않으면
        # 회피 모델이 매 스텝 같은(막힌) 방향을 명령할 때 표적이 구석에서
        # 완전히 얼어붙는다 (실측: 100초 넘게 좌표 완전 고정, 트러블슈팅
        # 노트 10-4 항목).
        return real_map_arena.step_body_sliding(
            core_arg.grid_map, position, proposed, low_arg, high_arg, avoid_point=prev_target_pos,
            body_radius_m=real_map_arena.TARGET_RADIUS_M, clearance_m=clearance_m,
        )

    target_traj, robot1_traj, robot2_traj = [], [], []
    panic_count, role_swap_count, latencies = 0, 0, []
    min_dist = float("inf")
    success, elapsed_sec = False, 0.0
    escape_snapshot = None
    discovered = False
    discovery_time_sec = None
    patrol_idx = 0

    # 덫(포획존)에 닿은 표적은 물리적으로 걸려서 더 이상 움직이지 못한다
    # (2026-08-08). 그 전까지는 표적이 계속 움직일 수 있고 FSM이 "반경 안에
    # capture_hold_sec 동안 머무르기"만 요구해서, **스스로 멈추는 표적만**
    # 잡을 수 있었다 -- 주 검증 모델 ReactiveFlee가 로봇이 멀면 속도 0을
    # 반환해 가만히 서 있는 덕에 잡혔던 것이고, 실제 쥐처럼 계속 움직이는
    # 모델은 원리적으로 포획이 불가능했다(실측 0%). 덫은 닿으면 걸리는
    # 물건이므로 이렇게 모델링하는 게 물리적으로 맞다.
    goal_xy = np.array([herding_config.capture_zone_x_m, herding_config.capture_zone_y_m])
    trapped = False

    steps = max(int(round(sim_config.max_sim_time_sec / sim_config.dt)), 0)
    for index in range(steps):
        sim_time_sec = index * sim_config.dt
        if not trapped and float(np.linalg.norm(target_state[:2] - goal_xy)) <= herding_config.capture_radius_m:
            trapped = True
            target_state = np.array([target_state[0], target_state[1], 0.0, 0.0])
        target_traj.append(target_state[:2].copy())
        robot1_traj.append(robot1_pos.copy())
        robot2_traj.append(robot2_pos.copy())

        tick_min = _closest_robot_distance(target_state[:2], robot1_pos, robot2_pos)

        if not discovered:
            dist_to_robot1 = float(np.linalg.norm(target_state[:2] - robot1_pos))
            if dist_to_robot1 <= real_map_arena.SENSOR_RANGE_M:
                discovered = True
                discovery_time_sec = sim_time_sec

        observation = Observation(
            target_measurement=target_state[:2].copy() if discovered else None,
            robot1_pos=robot1_pos.copy(), robot2_pos=robot2_pos.copy(),
            robot1_heading=robot1_heading.copy(), robot2_heading=robot2_heading.copy(),
            occupancy=None, sim_time_sec=sim_time_sec, dt=sim_config.dt,
        )
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

        if discovered:
            robot1_target = output.robot1_goal
        else:
            waypoint = real_map_arena.PATROL_WAYPOINTS[patrol_idx]
            if np.linalg.norm(robot1_pos - waypoint) <= real_map_arena.PATROL_WAYPOINT_TOLERANCE_M:
                patrol_idx = (patrol_idx + 1) % len(real_map_arena.PATROL_WAYPOINTS)
                waypoint = real_map_arena.PATROL_WAYPOINTS[patrol_idx]
            robot1_target = waypoint

        speed = sim_config.robot_max_speed_mps * sim_config.robot_gain
        new_r1_raw = real_map_arena.move_with_wall_avoidance(
            robot1_pos, robot1_target, distance_field, core.grid_map, speed, sim_config.dt
        )
        new_r2_raw = real_map_arena.move_with_wall_avoidance(
            robot2_pos, output.robot2_goal, distance_field, core.grid_map, speed, sim_config.dt
        )
        new_r1 = real_map_arena.step_body_sliding(
            core.grid_map, robot1_pos, new_r1_raw, low, high, avoid_point=prev_robot1_pos,
            body_radius_m=real_map_arena.ROBOT_BODY_CLEARANCE_M, clearance_m=clearance_m,
        )
        new_r2 = real_map_arena.step_body_sliding(
            core.grid_map, robot2_pos, new_r2_raw, low, high, avoid_point=prev_robot2_pos,
            body_radius_m=real_map_arena.ROBOT_BODY_CLEARANCE_M, clearance_m=clearance_m,
        )
        # 로봇끼리도 겹칠 수 없다 (2026-08-08). 그 전까지 몸체 크기를 벽에만
        # 적용해서, 두 로봇이 같은 자리를 차지하는 배치가 시뮬레이션에서
        # 허용됐다 -- 압박 선분 모드의 반각 15/30도가 "잘 됐던" 이유가 바로
        # 이것이었다(중심 간격 0.155~0.30m < 최소 0.342m, 즉 사실상 로봇
        # 한 대를 돌린 셈). 실제 터틀봇 두 대는 그럴 수 없으므로, 겹치면
        # 이번 스텝 이동을 취소한다.
        min_center_gap = 2.0 * real_map_arena.ROBOT_RADIUS_M
        if float(np.linalg.norm(new_r1 - new_r2)) < min_center_gap:
            if float(np.linalg.norm(new_r1 - robot2_pos)) >= min_center_gap:
                new_r2 = robot2_pos.copy()          # 로봇 1만 이동
            elif float(np.linalg.norm(robot1_pos - new_r2)) >= min_center_gap:
                new_r1 = robot1_pos.copy()          # 로봇 2만 이동
            else:
                new_r1, new_r2 = robot1_pos.copy(), robot2_pos.copy()  # 둘 다 정지
        prev_robot1_pos, prev_robot2_pos = robot1_pos, robot2_pos
        robot1_heading = _update_heading(robot1_pos, new_r1, robot1_heading)
        robot2_heading = _update_heading(robot2_pos, new_r2, robot2_heading)
        robot1_pos, robot2_pos = new_r1, new_r2
        tick_min = min(tick_min, _closest_robot_distance(target_state[:2], robot1_pos, robot2_pos))

        # avoid_point는 "이번 이동 시작 시점의 위치"(한 주기 전 위치)여야
        # 한다 -- _target_step_fn이 호출되는 동안에는 아직 갱신 전의(즉 지난
        # 주기의) prev_target_pos를 봐야 하므로, 대입은 _advance_target 호출
        # *이후*에 한다 (로봇 슬라이딩과 동일한 순서).
        pre_move_target_xy = target_state[:2].copy()
        if not trapped:
            target_state = _advance_target(
                core, evasion_model, target_state, robot1_pos, robot2_pos, sim_config, low, high,
                step_fn=_target_step_fn,
            )
        prev_target_pos = pre_move_target_xy
        tick_min = min(tick_min, _closest_robot_distance(target_state[:2], robot1_pos, robot2_pos))

        min_dist = min(min_dist, tick_min)
        if tick_min < herding_config.panic_distance_m:
            panic_count += 1

    goal_name, _ = real_map_arena.nearest_trap(
        np.array([herding_config.capture_zone_x_m, herding_config.capture_zone_y_m])
    )

    return TrialResult(
        success=success, duration_sec=elapsed_sec, panic_count=panic_count,
        role_swap_count=role_swap_count,
        mean_latency_ms=float(np.mean(latencies)) if latencies else 0.0,
        target_trajectory=np.array(target_traj, dtype=float).reshape(-1, 2),
        robot1_trajectory=np.array(robot1_traj, dtype=float).reshape(-1, 2),
        robot2_trajectory=np.array(robot2_traj, dtype=float).reshape(-1, 2),
        escape_snapshot=escape_snapshot, min_robot_target_dist=min_dist,
        discovery_time_sec=discovery_time_sec, goal_name=goal_name,
    )
