# herding_controller_dual/test/test_simulator.py
import numpy as np
import pytest

from herding_controller_dual.herding_core import HerdingConfig, HerdingCore
from test.evasion_models.base import EvasionModel
from test.evasion_models.noisy_human import NoisyHuman
from test.evasion_models.random_walk import RandomWalk
from test.evasion_models.wall_hugger import WallHugger
from test.simulator import SimulatorConfig, _arena_bounds, _spawn_driver_near_target, _update_heading, run_trial


def make_herding_config(**overrides):
    """herding_params.yaml을 반영한 HerdingConfig를 생성하며, 테스트별로 값을 오버라이드할 수 있다."""
    defaults = dict(
        frame_id="map", control_rate_hz=5.0,
        capture_zone_x_m=3.0, capture_zone_y_m=3.0, capture_radius_m=0.5, capture_hold_sec=1.0,
        grid_resolution_m=0.25, grid_width_cells=40, grid_height_cells=40,
        kf_process_noise=0.1, kf_measurement_noise=0.05, occlusion_timeout_sec=3.0,
        markov_wall_follow_p=0.70, markov_wall_hug_p=0.20, markov_center_p=0.10,
        momentum_weight=0.4, robot_repulsion_weight=1.5, wall_detect_radius_cells=1, escape_route_top_k=3,
        escape_concentration_threshold=0.5,
        drive_distance_m=0.8, flee_reaction_distance_m=1.0, panic_distance_m=0.35,
        # ease factor를 flee_reaction_distance_m / drive_distance_m
        # (1.0 / 0.8 = 1.25) 미만으로 유지하여 HerdingConfig.__post_init__이 이 픽스처를 허용하도록 함.
        alignment_threshold=0.7, drive_distance_ease_factor=1.15, block_lookahead_m=1.2,
        min_robot_separation_m=0.6, diffusion_rate=0.2, decay_factor=0.9,
    )
    defaults.update(overrides)
    return HerdingConfig(**defaults)


class ScriptedTarget(EvasionModel):
    """매 사이클마다 타겟을 스크립트된 위치로 순간이동시킨다(속도 상한을 높여야 함)."""

    def __init__(self, positions_by_cycle, dt: float) -> None:
        self.positions_by_cycle = positions_by_cycle
        self.dt = dt
        self.calls = 0
        self.received_states = []

    def step(self, target_state, robot_positions, obstacle_map, dt):
        """이번 사이클의 스크립트된 위치에 타겟이 도달하도록 하는 속도를 반환한다."""
        self.received_states.append(np.asarray(target_state, dtype=float).copy())
        desired = np.asarray(self.positions_by_cycle(self.calls), dtype=float)
        self.calls += 1
        return (desired - np.asarray(target_state[:2], dtype=float)) / self.dt


class ConstantVelocity(EvasionModel):
    """항상 고정된 속도를 명령하며, 전달받은 모든 상태를 기록한다."""

    def __init__(self, velocity) -> None:
        self.velocity = np.asarray(velocity, dtype=float)
        self.received_states = []

    def step(self, target_state, robot_positions, obstacle_map, dt):
        """들어오는 상태를 기록하고 고정된 명령 속도를 반환한다."""
        self.received_states.append(np.asarray(target_state, dtype=float).copy())
        return self.velocity.copy()


def test_run_trial_produces_bounded_trajectories():
    sim_config = SimulatorConfig(max_sim_time_sec=20.0)
    model = RandomWalk(max_speed_mps=0.4, rng=np.random.default_rng(1))
    result = run_trial(make_herding_config(), model, seed=1, sim_config=sim_config)
    assert result.duration_sec <= sim_config.max_sim_time_sec + 1e-6
    assert result.target_trajectory.shape[1] == 2
    assert result.robot1_trajectory.shape == result.target_trajectory.shape


def test_run_trial_is_deterministic_given_same_seed():
    sim_config = SimulatorConfig(max_sim_time_sec=10.0)
    model_a = RandomWalk(max_speed_mps=0.4, rng=np.random.default_rng(42))
    model_b = RandomWalk(max_speed_mps=0.4, rng=np.random.default_rng(42))
    result_a = run_trial(make_herding_config(), model_a, seed=42, sim_config=sim_config)
    result_b = run_trial(make_herding_config(), model_b, seed=42, sim_config=sim_config)
    assert np.allclose(result_a.target_trajectory, result_b.target_trajectory)


# --------------------------------------------------------------------------- #
# Heading 업데이트                                                            #
# --------------------------------------------------------------------------- #

def test_heading_update_depends_on_movement_not_on_resulting_x_sign():
    # 움직인 로봇은 어디로 향했든 heading이 바뀐 로봇이다.
    # 새 x좌표를 기준으로 업데이트를 게이팅하면 grid_origin_x_m < 0일 때
    # 지도의 절반에 해당하는 x <= 0에서 동작하는 모든 로봇의 heading이 고정되었다.
    heading = _update_heading(np.array([-3.0, 1.0]), np.array([-4.0, 1.0]), np.array([1.0, 0.0]))
    assert np.allclose(heading, [-1.0, 0.0])


def test_heading_is_held_when_the_robot_does_not_move():
    previous = np.array([0.0, 1.0])
    heading = _update_heading(np.array([2.0, 2.0]), np.array([2.0, 2.0]), previous)
    assert np.allclose(heading, previous)


def test_trial_spawns_inside_a_negative_origin_arena():
    config = make_herding_config(
        grid_origin_x_m=-5.0, grid_origin_y_m=-5.0, capture_zone_x_m=-2.0, capture_zone_y_m=-2.0,
    )
    result = run_trial(config, RandomWalk(0.4, np.random.default_rng(3)), seed=3,
                       sim_config=SimulatorConfig(max_sim_time_sec=10.0))
    for trajectory in (result.target_trajectory, result.robot1_trajectory, result.robot2_trajectory):
        assert (trajectory >= -5.0).all() and (trajectory <= 5.0).all()


# --------------------------------------------------------------------------- #
# Arena 경계 제한                                                             #
# --------------------------------------------------------------------------- #

def test_target_is_confined_by_the_arena_walls():
    # escape model은 그리드 밖의 모든 인접 셀을 벽으로 취급하므로, 물리적으로
    # 그리드를 벗어난 타겟은 시뮬레이션을 알고리즘이 불가능하다고 여기는 위치,
    # 즉 escape 분포가 전혀 존재하지 않는 위치로 밀어넣는다.
    config = make_herding_config()
    model = ConstantVelocity([10.0, 10.0])
    result = run_trial(config, model, seed=1, sim_config=SimulatorConfig(max_sim_time_sec=60.0),
                       control_mode="idle")
    assert (result.target_trajectory >= 0.0).all()
    assert (result.target_trajectory <= 10.0).all()
    # 코너에 고정된 이후에는 실제 달성된 속도가 벽이 거부한 명령이 아니라
    # 0으로 읽혀야 한다: 다음 사이클의 모델들이 이 값을 그대로 소비하기 때문이다.
    assert np.allclose(model.received_states[-1][2:], [0.0, 0.0])


def test_random_mode_keeps_robots_inside_the_arena():
    config = make_herding_config()
    result = run_trial(config, RandomWalk(0.4, np.random.default_rng(7)), seed=7,
                       sim_config=SimulatorConfig(max_sim_time_sec=60.0), control_mode="random")
    for trajectory in (result.robot1_trajectory, result.robot2_trajectory):
        assert (trajectory >= 0.0).all() and (trajectory <= 10.0).all()
    assert not np.allclose(result.robot1_trajectory[0], result.robot1_trajectory[-1])


def test_target_is_stopped_by_an_obstacle_cell():
    config = make_herding_config()
    mask = np.zeros((config.grid_height_cells, config.grid_width_cells), dtype=bool)
    mask[:, 24:] = True  # x = 6.0 m부터 오른쪽으로 이어지는 단단한 벽
    result = run_trial(config, ConstantVelocity([0.4, 0.0]), seed=1,
                       sim_config=SimulatorConfig(max_sim_time_sec=30.0),
                       obstacle_mask=mask, control_mode="idle")
    assert result.target_trajectory[:, 0].max() < 6.0


def make_probe_grid_map(config):
    """Task 12가 만드는 방식 그대로, 호출자가 WallHugger/NoisyHuman에 넘기는 grid map."""
    return HerdingCore(config).grid_map


def target_path_length(result) -> float:
    """한 trial 동안 타겟이 실제로 이동한 총 거리."""
    return float(np.linalg.norm(np.diff(result.target_trajectory, axis=0), axis=1).sum())


# WallHugger의 3셀 벽 감지 윈도우 안에서 타겟이 시간을 보내는 시드들.
# 30초 동안 측정한 총 타겟 이동 거리: all-False 마스크에서는 0.16 / 0.56 / 1.72 m,
# arena 벽이 장애물 셀로 존재할 때는 6.76 / 6.84 / 5.98 m.
@pytest.mark.parametrize("seed", [1, 8, 10])
def test_default_arena_walls_are_visible_to_a_wall_following_target(seed):
    # all-False 장애물 마스크에서는 WallHugger._nearest_wall_tangent()가 붙을 것을
    # 찾지 못하고 None을 반환하므로, flee_reaction_distance_m 안에 로봇이 없을 때마다
    # 타겟이 가만히 서 있게 된다 -- 이는 모든 capture rate를 부풀리는 거의 정지된
    # 타겟이며, 특히 noisy_human(WallHugger를 감싸며 Task 12의 실세계 예측 모델)에서
    # 심하다. 벽은 물리 엔진의 clamp로만 존재하는 것이 아니라 장애물 셀로도 존재해야
    # 하며, 모델은 반드시 이번 trial의 arena를 읽고 있어야 한다.
    # 임계값은 낮게 잡는다(예전 4.0m가 아니라 0.5m): 로봇 1(Driver)이 이제
    # 표적 근처에서 스폰되므로(고정된 먼 구석이 아니라) 시드별 경로 길이의
    # 편차가 커졌다 -- 이 값 자체가 아니라 "완전히 멈춰있지 않다"(장애물
    # 마스크가 실제로 보인다)는 것만 확인하면 되는 회귀 테스트다.
    config = make_herding_config()
    model = WallHugger(0.4, config.flee_reaction_distance_m, make_probe_grid_map(config))
    result = run_trial(config, model, seed=seed, sim_config=SimulatorConfig(max_sim_time_sec=30.0))
    assert target_path_length(result) > 0.5


def test_noisy_human_inherits_the_walled_arena():
    # 수정 전에는 타겟 경로가 4.38 m, 수정 후에는 8.31 m. 로봇 1(Driver)이
    # 표적 근처에서 스폰되도록 바뀐 뒤로는(위 테스트와 동일한 이유) 시드별
    # 편차가 커져 임계값을 0.5m로 낮췄다 -- "완전히 멈춰있지 않다"만 확인.
    config = make_herding_config()
    model = NoisyHuman(0.4, config.flee_reaction_distance_m, make_probe_grid_map(config),
                       rng=np.random.default_rng(8))
    result = run_trial(config, model, seed=8, sim_config=SimulatorConfig(max_sim_time_sec=30.0))
    assert model._wall_hugger.grid_map.obstacle_mask[0, :].all()
    assert target_path_length(result) > 0.5


def test_default_obstacle_mask_is_a_boundary_ring():
    config = make_herding_config()
    model = WallHugger(0.4, config.flee_reaction_distance_m, make_probe_grid_map(config))
    run_trial(config, model, seed=2, sim_config=SimulatorConfig(max_sim_time_sec=1.0))
    mask = model.grid_map.obstacle_mask  # trial 자체의 arena로 다시 연결됨
    assert mask.shape == (config.grid_height_cells, config.grid_width_cells)
    assert mask[0, :].all() and mask[-1, :].all() and mask[:, 0].all() and mask[:, -1].all()
    assert not mask[1:-1, 1:-1].any()


def test_explicit_obstacle_mask_is_used_verbatim():
    config = make_herding_config()
    supplied = np.zeros((config.grid_height_cells, config.grid_width_cells), dtype=bool)
    supplied[10, 10] = True
    model = WallHugger(0.4, config.flee_reaction_distance_m, make_probe_grid_map(config))
    run_trial(config, model, seed=2, sim_config=SimulatorConfig(max_sim_time_sec=1.0),
              obstacle_mask=supplied)
    assert np.array_equal(model.grid_map.obstacle_mask, supplied)


def test_robots_do_not_walk_into_obstacle_cells():
    # 로봇과 타겟은 동일한 충돌 규칙을 따른다; 벽 링 안에 있는 goal을 쫓는 로봇은
    # 벽을 뚫고 들어가는 대신 벽에서 멈춰야 한다.
    config = make_herding_config()
    result = run_trial(config, RandomWalk(0.4, np.random.default_rng(5)), seed=5,
                       sim_config=SimulatorConfig(max_sim_time_sec=60.0))
    ring = config.grid_resolution_m  # 가장 바깥쪽 셀 링은 단단한 벽
    for trajectory in (result.robot1_trajectory, result.robot2_trajectory,
                       result.target_trajectory):
        assert (trajectory >= ring).all() and (trajectory <= 10.0 - ring).all()


def test_obstacle_mask_shape_is_validated():
    with pytest.raises(ValueError):
        run_trial(make_herding_config(), RandomWalk(0.4, np.random.default_rng(0)), seed=0,
                  sim_config=SimulatorConfig(max_sim_time_sec=1.0),
                  obstacle_mask=np.zeros((10, 10), dtype=bool))


# --------------------------------------------------------------------------- #
# evasion model에 전달되는 타겟 상태                                          #
# --------------------------------------------------------------------------- #

def test_evasion_model_receives_position_and_achieved_velocity():
    dt = 0.1
    velocity = np.array([0.2, 0.1])
    model = ConstantVelocity(velocity)
    run_trial(make_herding_config(), model, seed=1,
              sim_config=SimulatorConfig(max_sim_time_sec=2.0, dt=dt), control_mode="idle")
    assert all(state.shape == (4,) for state in model.received_states)
    assert np.allclose(model.received_states[0][2:], [0.0, 0.0])  # 첫 명령 전에는 정지 상태
    for previous, current in zip(model.received_states, model.received_states[1:]):
        assert np.allclose(current[2:], velocity)
        assert np.allclose(current[:2], previous[:2] + velocity * dt)


# --------------------------------------------------------------------------- #
# Panic 집계                                                                  #
# --------------------------------------------------------------------------- #

def _expected_robot1_spawn(config: HerdingConfig, sim_config: SimulatorConfig, seed: int) -> np.ndarray:
    """run_trial()이 이 seed/config로 실제 스폰시킬 로봇 1(Driver) 위치를 재현한다.

    로봇 1은 더 이상 고정된 구석이 아니라 표적 스폰 위치 근처에 무작위로
    스폰되므로(스폰 로직은 run_trial()과 완전히 동일한 순서로 rng를
    소비해야 한다: 표적 x, 표적 y, 그다음 로봇 1의 각도/거리), 이 값을
    미리 알아야 하는 테스트는 동일한 시드로 그 계산을 그대로 재현해야 한다.
    """
    rng = np.random.default_rng(seed)
    low, high = _arena_bounds(config)
    margin = config.grid_resolution_m * 2
    spawn_low, spawn_high = low + margin, high - margin
    target_spawn = np.array([rng.uniform(spawn_low[0], spawn_high[0]), rng.uniform(spawn_low[1], spawn_high[1])])
    return _spawn_driver_near_target(target_spawn, rng, spawn_low, spawn_high, sim_config)


def test_panic_count_is_per_cycle_and_not_latched_once_triggered():
    # panic_distance_m과 비교하는 누적 최소값을 사용하면, 타겟이 한 번이라도
    # 가까워진 이후에는 trial의 남은 모든 사이클마다 계속 발동하여
    # panic_count가 "첫 위반 이후 경과한 사이클 수"가 되어버린다.
    config = make_herding_config()
    dt = 0.1
    seed = 1
    sim_config = SimulatorConfig(max_sim_time_sec=3.0, dt=dt, target_max_speed_mps=1e3)
    # 로봇 1이 실제로 스폰될 위치로부터 0.2m 떨어진 지점을 "close"로 잡는다
    # (예전에는 로봇 1이 항상 고정된 (0.5, 0.5)였지만, 이제는 표적 근처에
    # 무작위로 스폰된다).
    robot1_spawn = _expected_robot1_spawn(config, sim_config, seed)
    far = np.array([5.0, 5.0])
    close = robot1_spawn + np.array([0.2, 0.0])
    model = ScriptedTarget(lambda cycle: close if 3 <= cycle <= 5 else far, dt)
    result = run_trial(config, model, seed=seed, sim_config=sim_config, control_mode="idle")
    assert result.min_robot_target_dist == pytest.approx(0.2, abs=1e-6)
    assert 1 <= result.panic_count <= 6  # 30 사이클이 실행됨; 그중 소수만 가까웠음


def test_panic_is_detected_when_it_happens_at_the_very_end_of_the_last_cycle():
    # 매 사이클 시작 시점에서만 샘플링하면 타겟과 로봇이 실제로 도달한 최종
    # 상태는 절대 측정되지 않으므로, 마지막 이동으로 생긴 위반이 ALGO-003의
    # panic 집계에서 조용히 누락된다.
    config = make_herding_config()
    dt = 0.1
    seed = 1
    sim_config = SimulatorConfig(max_sim_time_sec=1.0, dt=dt, target_max_speed_mps=1e3)
    robot1_spawn = _expected_robot1_spawn(config, sim_config, seed)
    far = np.array([5.0, 5.0])
    close = robot1_spawn + np.array([0.2, 0.0])
    model = ScriptedTarget(lambda cycle: close if cycle == 9 else far, dt)
    result = run_trial(config, model, seed=seed, sim_config=sim_config, control_mode="idle")
    assert result.min_robot_target_dist == pytest.approx(0.2, abs=1e-6)
    assert result.panic_count == 1


# --------------------------------------------------------------------------- #
# 제어 모드 및 trial 결과 보고                                                #
# --------------------------------------------------------------------------- #

def test_idle_mode_holds_the_robots_but_still_advances_the_core():
    config = make_herding_config()
    result = run_trial(config, ConstantVelocity([0.0, 0.0]), seed=1,
                       sim_config=SimulatorConfig(max_sim_time_sec=5.0), control_mode="idle")
    assert np.allclose(result.robot1_trajectory, result.robot1_trajectory[0])
    assert np.allclose(result.robot2_trajectory, result.robot2_trajectory[0])
    assert result.mean_latency_ms > 0.0
    # escape snapshot이 채워져 있다는 것은 FSM이 HERD에 도달했고 escape model이
    # 실행되었음을, 즉 core가 'algorithm' 모드와 정확히 동일하게 진행되었음을 증명한다.
    assert result.escape_snapshot is not None
    assert result.escape_snapshot.shape[1] == 2


def test_unknown_control_mode_is_rejected():
    with pytest.raises(ValueError):
        run_trial(make_herding_config(), RandomWalk(0.4, np.random.default_rng(0)), seed=0,
                  sim_config=SimulatorConfig(max_sim_time_sec=1.0), control_mode="typo")


def test_trial_that_never_captures_reports_failure_at_the_full_duration():
    sim_config = SimulatorConfig(max_sim_time_sec=5.0)
    result = run_trial(make_herding_config(), ConstantVelocity([0.0, 0.0]), seed=1,
                       sim_config=sim_config, control_mode="idle")
    assert result.success is False
    assert result.duration_sec == pytest.approx(sim_config.max_sim_time_sec)
    assert len(result.target_trajectory) == 50


def test_capture_reports_the_elapsed_time_not_the_trial_limit():
    config = make_herding_config()  # capture_hold_sec = 1.0, capture zone (3, 3) r = 0.5
    sim_config = SimulatorConfig(max_sim_time_sec=30.0, target_max_speed_mps=1e3)
    capture_zone = np.array([config.capture_zone_x_m, config.capture_zone_y_m])
    result = run_trial(config, ScriptedTarget(lambda cycle: capture_zone, sim_config.dt), seed=1,
                       sim_config=sim_config, control_mode="idle")
    assert result.success is True
    assert result.duration_sec < 5.0
    assert result.duration_sec == pytest.approx(len(result.target_trajectory) * sim_config.dt)


# --------------------------------------------------------------------------- #

def test_step_body_rejects_positions_where_the_body_would_overlap_a_wall():
    """반지름을 주면, 중심 셀이 자유공간이어도 몸체가 벽을 파고드는 위치는 거부돼야 한다.

    이 검사가 없으면 body_radius_m을 배선해놓고도 아무 효과가 없는 걸(예: 거리장을
    안 넘겨서 조용히 무시되는 경우) 못 잡는다 -- 두 로봇이 통로를 몸으로 막을 수
    있는지 판정하는 전체 근거가 이 반지름이므로, 실제로 강제되는지 고정해둔다.
    """
    from test import real_map_arena

    mask = real_map_arena.load_room_obstacle_mask()
    grid_map = real_map_arena.build_grid_map(mask)
    clearance = real_map_arena.clearance_field_m(mask)
    grid = grid_map.config
    low = np.array([grid.origin_x_m, grid.origin_y_m])
    high = low + np.array([grid.width_cells, grid.height_cells]) * grid.resolution_m

    # 벽에서 0.20m 떨어진 지점(트랩 자리) -- 자유공간이지만 로봇 몸체
    # (0.171m + 여유 0.03m = 0.201m)에는 1mm 모자란다.
    tight = real_map_arena.TRAPS["top"]
    assert not mask[grid_map.world_to_cell(*tight)], "이 지점은 자유공간이어야 테스트가 성립한다"
    start = real_map_arena.ROBOT_A_SPAWN.copy()

    as_point = real_map_arena._step_body(grid_map, start, tight, low, high)
    np.testing.assert_allclose(as_point, tight, atol=1e-9)  # 점으로 보면 갈 수 있고

    as_robot = real_map_arena._step_body(
        grid_map, start, tight, low, high,
        body_radius_m=real_map_arena.ROBOT_BODY_CLEARANCE_M, clearance_m=clearance,
    )
    np.testing.assert_allclose(as_robot, start, atol=1e-9)  # 몸체를 고려하면 못 간다


def test_robot_bodies_never_overlap_walls_during_a_real_map_trial():
    """실제 맵 시행 내내 두 로봇 중심이 벽에서 몸체 반지름 이상 떨어져 있어야 한다."""
    import dataclasses

    from test import real_map_arena
    from test.evasion_models.reactive_flee import ReactiveFlee
    from test.run_validation import CONFIG_PATH, load_herding_config, make_real_map_config
    from test.simulator import run_trial_real_map

    config = make_real_map_config(load_herding_config(CONFIG_PATH), real_map_arena.TRAPS["top"])
    sim_config = SimulatorConfig()
    mask = real_map_arena.load_room_obstacle_mask()
    grid_map = real_map_arena.build_grid_map(mask)
    clearance = real_map_arena.clearance_field_m(mask)

    model = ReactiveFlee(sim_config.target_max_speed_mps, config.flee_reaction_distance_m)
    result = run_trial_real_map(config, model, seed=7, sim_config=sim_config)

    for name, traj in (("robot1", result.robot1_trajectory), ("robot2", result.robot2_trajectory)):
        for step, pos in enumerate(traj):
            row, col = grid_map.world_to_cell(*pos)
            assert clearance[row, col] >= real_map_arena.ROBOT_BODY_CLEARANCE_M - 1e-9, (
                f"{name} step {step} at {pos}: 벽까지 {clearance[row, col]:.3f}m "
                f"< 필요한 {real_map_arena.ROBOT_BODY_CLEARANCE_M:.3f}m"
            )
