# herding_controller/test/test_herding_core.py
import numpy as np
import pytest

from herding_controller.herding_core import HerdingConfig, HerdingCore, Observation
from herding_controller.state_machine import FSMState


def make_config(**overrides):
    """테스트용 HerdingConfig를 생성하며, 테스트별로 값을 오버라이드할 수 있다."""
    defaults = dict(
        frame_id="map", control_rate_hz=5.0,
        capture_zone_x_m=3.0, capture_zone_y_m=3.0, capture_radius_m=0.5, capture_hold_sec=0.4,
        grid_resolution_m=0.25, grid_width_cells=40, grid_height_cells=40,
        kf_process_noise=0.1, kf_measurement_noise=0.05, occlusion_timeout_sec=3.0,
        markov_wall_follow_p=0.70, markov_wall_hug_p=0.20, markov_center_p=0.10,
        momentum_weight=0.4, robot_repulsion_weight=1.5, wall_detect_radius_cells=1, escape_route_top_k=3,
        escape_concentration_threshold=0.5,
        drive_distance_m=0.8, flee_reaction_distance_m=1.0, panic_distance_m=0.35,
        # ease factor를 flee_reaction_distance_m / drive_distance_m
        # (1.0 / 0.8 = 1.25) 미만으로 유지하여 HerdingConfig.__post_init__이 이 픽스처를 허용하도록 함.
        alignment_threshold=0.7, drive_distance_ease_factor=1.15, block_lookahead_m=1.2,
        role_swap_margin=0.5, role_swap_cooldown_sec=2.0, min_robot_separation_m=0.6,
        role_cost_turn_weight=0.3, diffusion_rate=0.2, decay_factor=0.9,
    )
    defaults.update(overrides)
    return HerdingConfig(**defaults)


def test_no_rclpy_import_anywhere_in_core_chain():
    import herding_controller.herding_core as core_module
    with open(core_module.__file__) as f:
        assert "import rclpy" not in f.read()


def test_step_returns_search_state_with_no_observation():
    core = HerdingCore(make_config())
    obs = Observation(
        target_measurement=None, robot1_pos=np.array([0.0, 0.0]), robot2_pos=np.array([1.0, 0.0]),
        robot1_heading=np.array([1.0, 0.0]), robot2_heading=np.array([1.0, 0.0]),
        occupancy=None, sim_time_sec=0.0, dt=0.2,
    )
    output = core.step(obs)
    assert output.fsm_state == FSMState.SEARCH


def test_step_tracks_and_drives_toward_target_after_observations():
    core = HerdingCore(make_config())
    t = 0.0
    output = None
    for _ in range(20):
        obs = Observation(
            target_measurement=np.array([2.0, 2.0]), robot1_pos=np.array([0.0, 0.0]),
            robot2_pos=np.array([4.0, 4.0]), robot1_heading=np.array([1.0, 0.0]),
            robot2_heading=np.array([1.0, 0.0]), occupancy=None, sim_time_sec=t, dt=0.2,
        )
        output = core.step(obs)
        t += 0.2
    assert output.fsm_state in (FSMState.HERD, FSMState.CORNER, FSMState.CAPTURED)
    assert output.driver_id in (1, 2)
    assert output.blocker_id in (1, 2)
    assert output.driver_id != output.blocker_id


# --------------------------------------------------------------------------- #
# 통합 엣지 케이스                                                              #
# --------------------------------------------------------------------------- #


class Runner:
    """단조 증가하는 시계로 HerdingCore를 여러 사이클에 걸쳐 구동한다."""

    def __init__(self, config=None, dt=0.2):
        self.core = HerdingCore(config or make_config())
        self.dt = dt
        self.t = 0.0

    def run(self, n, measurement, r1=(0.0, 0.0), r2=(4.0, 4.0), occupancy=None):
        output = None
        for _ in range(n):
            output = self.core.step(Observation(
                target_measurement=None if measurement is None else np.array(measurement, dtype=float),
                robot1_pos=np.array(r1, dtype=float), robot2_pos=np.array(r2, dtype=float),
                robot1_heading=np.array([1.0, 0.0]), robot2_heading=np.array([1.0, 0.0]),
                occupancy=occupancy, sim_time_sec=self.t, dt=self.dt,
            ))
            self.t += self.dt
        return output

    def run_until(self, state, measurement, limit=40, **kwargs):
        for _ in range(limit):
            output = self.run(1, measurement, **kwargs)
            if output.fsm_state == state:
                return output
        raise AssertionError(f"never reached {state}")


def test_config_wires_every_subconfig_without_type_error():
    core = HerdingCore(make_config())
    assert core.grid_map.config.resolution_m == 0.25
    assert core.grid_map.config.width_cells == 40
    assert core.estimator.config.process_noise == 0.1
    assert core.estimator.config.occlusion_timeout_sec == 3.0
    assert core.escape_model.config.wall_follow_p == 0.70
    assert core.escape_model.config.escape_route_top_k == 3
    assert core.planner_config.drive_distance_m == 0.8
    assert core.planner_config.block_lookahead_m == 1.2
    assert core.role_assigner_config.role_cost_turn_weight == 0.3
    assert core.role_assigner_config.min_robot_separation_m == 0.6
    assert core.occlusion_grid.config.diffusion_rate == 0.2
    assert core.occlusion_grid.config.decay_factor == 0.9
    # escape model과 occlusion grid는 core의 단일 GridMap을 공유해야 한다.
    assert core.escape_model.grid_map is core.grid_map
    assert core.occlusion_grid.grid_map is core.grid_map


def test_escape_concentration_threshold_gates_the_corner_transition():
    """이 임계값은 하드코딩된 리터럴이 아니라 config에서 와야 하며, 실제로 게이팅 역할을 해야 한다."""
    import dataclasses
    # 타겟이 캡처 구역 안에 머무르며, CORNER를 관찰할 수 있을 만큼 충분히 긴 hold 시간을 갖는다.
    base = dataclasses.replace(make_config(), capture_hold_sec=1e6)

    never = Runner(dataclasses.replace(base, escape_concentration_threshold=1.1))
    assert never.run(10, (3.1, 3.0)).fsm_state == FSMState.HERD

    always = Runner(dataclasses.replace(base, escape_concentration_threshold=0.0))
    assert always.run(10, (3.1, 3.0)).fsm_state == FSMState.CORNER


def test_search_state_skips_escape_planner_and_role_logic():
    """관측이 이루어지기 전에는 KF 상태가 모두 0이므로, 이후 단계는 이를 기반으로 동작해서는 안 된다."""
    runner = Runner()
    output = runner.run(5, None)
    assert output.fsm_state == FSMState.SEARCH
    assert output.escape_top3 == []
    assert output.panic is False
    assert output.role_swapped is False
    # 로봇들은 위치를 유지하며, estimator는 한 번도 진행되지 않았다.
    np.testing.assert_allclose(output.robot1_goal, [0.0, 0.0])
    np.testing.assert_allclose(output.robot2_goal, [4.0, 4.0])
    assert runner.core.estimator.get_state().time_since_observation == 0.0


def test_lost_state_seeds_occlusion_grid_and_skips_escape_planner_role():
    runner = Runner()
    assert runner.run(5, (2.0, 2.0)).fsm_state == FSMState.HERD
    output = runner.run_until(FSMState.LOST, None)

    assert runner.core._occlusion_seeded is True
    assert runner.core._last_known_cell == runner.core.grid_map.world_to_cell(2.0, 2.0)
    assert runner.core.occlusion_grid.belief.sum() > 0.0
    # Occlusion grid는 LOST 상태에서만 사용된다: escape 분포도, panic도, swap도 없음.
    assert output.escape_top3 == []
    assert output.panic is False
    assert output.role_swapped is False
    # 두 로봇 모두 마지막으로 알려진 타겟 위치 근처를 탐색하지만, 같은 지점은 아니다.
    np.testing.assert_allclose(output.robot1_goal, [2.125, 2.125], atol=0.3)
    separation = float(np.linalg.norm(output.robot1_goal - output.robot2_goal))
    assert separation >= runner.core.config.min_robot_separation_m - 1e-6


def test_second_lost_episode_reseeds_from_new_position():
    runner = Runner()
    runner.run(5, (2.0, 2.0))
    runner.run_until(FSMState.LOST, None)
    first_cell = runner.core._last_known_cell

    # 복구 시 다음 에피소드가 새로 시작되도록 seed 상태를 지워야 한다.
    recovered = runner.run(1, (2.0, 2.0))
    assert recovered.fsm_state == FSMState.TRACK
    assert runner.core._occlusion_seeded is False
    assert runner.core._last_known_cell is None

    runner.run(12, (5.0, 5.0))
    runner.run_until(FSMState.LOST, None)
    second_cell = runner.core._last_known_cell
    assert second_cell is not None
    assert second_cell != first_cell
    # 오래된 위치가 아니라 새 위치 근처에 시딩됨.
    assert abs(second_cell[0] - 20) <= 6 and abs(second_cell[1] - 20) <= 6


def test_role_swapped_is_false_on_the_bootstrap_assignment():
    """첫 assign() 호출은 비용이 최적인 driver를 바로 선택한다; 이는 swap이 아니다."""
    runner = Runner()
    output = runner.run_until(FSMState.HERD, (2.0, 2.0), r1=(8.0, 8.0), r2=(1.5, 1.5))
    assert output.driver_id == 2  # robot 2가 비용이 훨씬 낮으므로 bootstrap이 이를 선택함
    assert output.role_swapped is False


def test_role_swapped_is_true_only_on_the_cycle_of_a_real_swap():
    runner = Runner()
    runner.run_until(FSMState.HERD, (2.0, 2.0), r1=(8.0, 8.0), r2=(1.5, 1.5))
    swaps = []
    drivers = []
    for _ in range(20):
        output = runner.run(1, (2.0, 2.0), r1=(1.5, 1.5), r2=(9.0, 9.0))
        drivers.append(output.driver_id)
        swaps.append(output.role_swapped)
    assert drivers[0] == 2 and drivers[-1] == 1, "cost-optimal driver must eventually flip"
    assert sum(swaps) == 1, "role_swapped must be True on exactly the flip cycle"
    flip_index = swaps.index(True)
    assert drivers[flip_index] == 1 and drivers[flip_index - 1] == 2


def test_role_assignment_candidate_is_not_biased_by_a_panicking_robot():
    """패닉 거리 안에 있는 로봇이 단지 가깝다는 이유만으로 Driver 역할을 얻어서는 안 된다.

    compute_driving_point()는 평가 대상 로봇 바로 옆의 후퇴 지점으로 수렴하므로,
    robot1의 결과를 role-assignment 후보로 사용하면 robot1이 panic_distance_m
    안에 있을 때마다 무조건 이기게 된다.
    """
    runner = Runner()
    # robot1은 타겟과 목표 사이에 위치하며(나쁜 Driver) panic 범위 안에 있다;
    # robot2는 타겟 뒤, 이상적인 driving point 바로 위에 위치한다(좋은 Driver).
    output = runner.run_until(FSMState.HERD, (2.0, 2.0), r1=(2.2, 2.2), r2=(1.4, 1.4))
    assert output.driver_id == 2
    assert output.panic is False  # robot2(Driver)는 panic 범위 안에 있지 않음


def test_panic_flag_propagates_from_the_driving_point():
    runner = Runner()
    output = runner.run_until(FSMState.HERD, (2.0, 2.0), r1=(1.9, 2.0), r2=(5.0, 5.0))
    assert output.driver_id == 1
    assert output.panic is True
    # Panic 후퇴는 robot 1을 타겟으로부터 직접 멀어지도록 이동시킨다.
    np.testing.assert_allclose(output.robot1_goal, [1.65, 2.0], atol=1e-6)


def test_captured_state_is_reached_and_holds_position():
    runner = Runner()
    output = runner.run_until(FSMState.CAPTURED, (3.1, 3.0), limit=60)
    assert output.fsm_state == FSMState.CAPTURED
    np.testing.assert_allclose(output.robot1_goal, [0.0, 0.0])
    np.testing.assert_allclose(output.robot2_goal, [4.0, 4.0])


def test_off_grid_target_does_not_crash_the_cycle():
    """world_to_cell()은 범위를 벗어나면 예외를 발생시킨다; core는 폭발하지 않고 성능이 저하되어야 한다."""
    runner = Runner()
    output = runner.run(6, (50.0, 50.0))
    assert output.fsm_state == FSMState.HERD
    assert output.escape_top3 == []
    # Driver는 계속 주행하며; blocking point를 계산할 수 없으므로 blocker는 대기한다.
    assert output.driver_id == 2
    np.testing.assert_allclose(output.robot1_goal, [0.0, 0.0])


def test_mismatched_occupancy_shape_is_ignored():
    runner = Runner()
    output = runner.run(3, (2.0, 2.0), occupancy=np.zeros((5, 5), dtype=np.int8))
    assert runner.core.grid_map.obstacle_mask.shape == (40, 40)
    assert output.fsm_state == FSMState.HERD


def test_occupancy_of_the_right_shape_is_applied():
    runner = Runner()
    occupancy = np.zeros((40, 40), dtype=np.int8)
    occupancy[0:4, 0:4] = 100
    runner.run(3, (2.0, 2.0), occupancy=occupancy)
    assert runner.core.grid_map.obstacle_mask[0, 0]
    assert not runner.core.grid_map.obstacle_mask[20, 20]


def test_output_goals_do_not_alias_the_observation_arrays():
    runner = Runner()
    robot1 = np.array([0.0, 0.0])
    output = runner.core.step(Observation(
        target_measurement=None, robot1_pos=robot1, robot2_pos=np.array([1.0, 0.0]),
        robot1_heading=np.array([1.0, 0.0]), robot2_heading=np.array([1.0, 0.0]),
        occupancy=None, sim_time_sec=0.0, dt=0.2,
    ))
    output.robot1_goal[0] = 99.0
    assert robot1[0] == 0.0


def test_no_module_in_the_core_import_chain_imports_rclpy():
    import herding_controller.herding_core  # noqa: F401
    import sys
    for name, module in list(sys.modules.items()):
        if not name.startswith("herding_controller."):
            continue
        if name.endswith(".herding_node"):
            # herding_node.py는 이 패키지의 유일한 ROS2 경계이므로
            # rclpy를 import해야 한다. 그 외의 모든 모듈은 오프라인에서
            # 알고리즘을 실행할 수 있도록 ROS에 의존하지 않아야 한다.
            continue
        path = getattr(module, "__file__", None)
        if not path or not path.endswith(".py"):
            continue
        with open(path) as handle:
            source = handle.read()
        assert "import rclpy" not in source, f"{name} imports rclpy"
        assert "from rclpy" not in source, f"{name} imports rclpy"


# --- 최종 검토 I1: drive/flee 기하학적 불변식이 코드에서 강제되는지 확인 --- #

def test_herding_config_rejects_drive_distance_that_exceeds_the_flee_reaction_radius():
    """0.8 * 1.3 = 1.04 >= 1.0: Driver가 타겟의 반응 반경 밖에서 멈추게 된다."""
    with pytest.raises(ValueError) as excinfo:
        make_config(drive_distance_m=0.8, drive_distance_ease_factor=1.3,
                    flee_reaction_distance_m=1.0)
    message = str(excinfo.value)
    # 메시지는 단순히 실패하는 것이 아니라 제약 조건과 문제가 된 값들을 명시해야 한다.
    assert "flee_reaction_distance_m" in message
    assert "drive_distance_ease_factor" in message
    assert "0.8" in message and "1.3" in message


def test_herding_config_rejects_the_exact_boundary_case():
    """동등한 경우도 위반이다: 반응 반경과 정확히 같으면 타겟이 도망치지 않는다."""
    with pytest.raises(ValueError):
        make_config(drive_distance_m=1.0, drive_distance_ease_factor=1.0,
                    flee_reaction_distance_m=1.0)


def test_herding_config_accepts_a_combination_with_margin():
    config = make_config(drive_distance_m=0.75, drive_distance_ease_factor=1.15,
                         flee_reaction_distance_m=1.0)
    assert config.drive_distance_m * config.drive_distance_ease_factor < config.flee_reaction_distance_m


def test_shipping_yaml_config_satisfies_the_invariant():
    """config/herding_params.yaml에 실제로 배포된 값들은 예외 없이 로드되어야 한다.

    실제 오프라인 config 로딩 경로인 run_validation.load_herding_config를 통해
    로드되므로, yaml이 언젠가 제약을 위반하는 조합으로 바뀌면 이 테스트가 실패한다
    (test_herding_node_imports.py의 노드 측 기본값 검사를 보완한다).
    """
    # 지연 import: run_validation은 matplotlib/scipy를 끌어오는데, 이 모듈의
    # 다른 부분에서는 필요하지 않다.
    from test.run_validation import CONFIG_PATH, load_herding_config

    config = load_herding_config(CONFIG_PATH)  # 예외가 발생하면 안 됨
    eased = config.drive_distance_m * config.drive_distance_ease_factor
    assert eased < config.flee_reaction_distance_m
