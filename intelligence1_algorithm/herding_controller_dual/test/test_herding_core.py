# herding_controller_dual/test/test_herding_core.py
import numpy as np
import pytest

from herding_controller_dual.herding_core import HerdingConfig, HerdingCore, Observation
from herding_controller_dual.state_machine import FSMState


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
        min_robot_separation_m=0.6, diffusion_rate=0.2, decay_factor=0.9,
    )
    defaults.update(overrides)
    return HerdingConfig(**defaults)


def test_no_rclpy_import_anywhere_in_core_chain():
    import herding_controller_dual.herding_core as core_module
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


# --------------------------------------------------------------------------- #
# Blocking Point 이력현상 -- blocking_point_commit_sec (트러블슈팅 노트 10-7)   #
# --------------------------------------------------------------------------- #


def test_blocking_point_commit_sec_defaults_to_one_second():
    """설정을 안 주면 기존 하드코딩 상수와 동일하게 1.0초를 유지해야 한다."""
    core = HerdingCore(make_config())
    assert core.config.blocking_point_commit_sec == 1.0


def test_stabilize_blocking_point_holds_previous_value_within_commit_window():
    core = HerdingCore(make_config(blocking_point_commit_sec=1.0))
    first = core._stabilize_blocking_point(np.array([1.0, 1.0]), now_sec=0.0)
    held = core._stabilize_blocking_point(np.array([9.0, 9.0]), now_sec=0.5)
    assert np.array_equal(held, first), "커밋 윈도우(1.0s) 안에서는 새 후보를 무시해야 한다"


def test_stabilize_blocking_point_adopts_new_candidate_after_commit_window():
    core = HerdingCore(make_config(blocking_point_commit_sec=1.0))
    core._stabilize_blocking_point(np.array([1.0, 1.0]), now_sec=0.0)
    updated = core._stabilize_blocking_point(np.array([9.0, 9.0]), now_sec=1.0)
    assert np.array_equal(updated, np.array([9.0, 9.0])), "커밋 윈도우가 지나면 새 후보로 갈아타야 한다"


def test_shorter_commit_window_adopts_new_candidates_sooner():
    """10-7의 핵심 동작: commit_sec을 줄이면 같은 흐름에서 더 빨리 새 후보를 채택한다."""
    responsive = HerdingCore(make_config(blocking_point_commit_sec=0.2))
    sluggish = HerdingCore(make_config(blocking_point_commit_sec=1.0))
    for core in (responsive, sluggish):
        core._stabilize_blocking_point(np.array([1.0, 1.0]), now_sec=0.0)

    at_half_second_responsive = responsive._stabilize_blocking_point(
        np.array([9.0, 9.0]), now_sec=0.5
    )
    at_half_second_sluggish = sluggish._stabilize_blocking_point(
        np.array([9.0, 9.0]), now_sec=0.5
    )
    assert np.array_equal(at_half_second_responsive, np.array([9.0, 9.0])), \
        "0.2초짜리는 0.5초 시점에 이미 새 후보로 갈아탔어야 한다"
    assert np.array_equal(at_half_second_sluggish, np.array([1.0, 1.0])), \
        "1.0초짜리는 0.5초 시점엔 아직 이전 값을 유지해야 한다"


def test_zero_commit_sec_adopts_every_candidate_immediately():
    core = HerdingCore(make_config(blocking_point_commit_sec=0.0))
    core._stabilize_blocking_point(np.array([1.0, 1.0]), now_sec=0.0)
    updated = core._stabilize_blocking_point(np.array([2.0, 2.0]), now_sec=0.01)
    assert np.array_equal(updated, np.array([2.0, 2.0]))
    assert core.escape_model.config.escape_route_top_k == 3
    assert core.planner_config.drive_distance_m == 0.8
    assert core.planner_config.block_lookahead_m == 1.2
    assert core.config.min_robot_separation_m == 0.6
    assert core.occlusion_grid.config.diffusion_rate == 0.2
    assert core.occlusion_grid.config.decay_factor == 0.9
    # escape model과 occlusion grid는 core의 단일 GridMap을 공유해야 한다.
    assert core.escape_model.grid_map is core.grid_map
    assert core.occlusion_grid.grid_map is core.grid_map


# --------------------------------------------------------------------------- #
# 교착(deadlock) 감지 -- Driver+Blocker가 포획존이 아닌 곳에서 표적을 우연히    #
# 가두는 문제 (트러블슈팅 노트 11-8)                                           #
# --------------------------------------------------------------------------- #


def test_deadlock_config_defaults():
    core = HerdingCore(make_config())
    assert core.config.deadlock_stall_sec == 3.0
    assert core.config.deadlock_drift_radius_m == 0.1
    assert core.config.deadlock_release_distance_m == 1.0


def test_update_deadlock_state_stays_false_until_stall_sec_elapses():
    core = HerdingCore(make_config(deadlock_stall_sec=1.0, deadlock_drift_radius_m=0.1))
    pos = np.array([2.0, 2.0])
    assert core._update_deadlock_state(pos, now_sec=0.0) is False  # 첫 호출: 기준점만 기록
    assert core._update_deadlock_state(pos, now_sec=0.6) is False  # 아직 1.0s 안 지남
    assert core._update_deadlock_state(pos, now_sec=1.1) is True  # 기준점에서 1.1s 경과


def test_update_deadlock_state_resets_when_target_leaves_drift_radius():
    core = HerdingCore(make_config(deadlock_stall_sec=1.0, deadlock_drift_radius_m=0.1))
    anchor = np.array([2.0, 2.0])
    core._update_deadlock_state(anchor, now_sec=0.0)
    core._update_deadlock_state(anchor, now_sec=0.9)  # 거의 도달 직전까지 누적
    moved = np.array([2.5, 2.0])  # drift_radius(0.1m)를 훌쩍 넘는 실제 이동
    assert core._update_deadlock_state(moved, now_sec=1.0) is False
    # 새 기준점이 잡혔으므로, 그 자리에서 다시 잠깐 있어서는 아직 안 걸림
    assert core._update_deadlock_state(moved, now_sec=1.3) is False


def test_update_deadlock_state_ignores_jitter_within_drift_radius():
    """벽 충돌회피 물리엔진의 스텝별 미세한 흔들림(수 cm)에는 리셋되지
    않아야 한다 -- seed=1200008에서 실측된, 순간속도 기준 구현이 놓쳤던
    바로 그 문제(트러블슈팅 노트 11-8)."""
    core = HerdingCore(make_config(deadlock_stall_sec=1.0, deadlock_drift_radius_m=0.1))
    jitter_positions = [
        np.array([1.95, -5.33]), np.array([1.95, -5.34]), np.array([1.94, -5.34]),
        np.array([1.94, -5.37]), np.array([1.96, -5.36]), np.array([1.95, -5.35]),
    ]
    result = None
    for i, pos in enumerate(jitter_positions):
        result = core._update_deadlock_state(pos, now_sec=i * 0.2)
    # 6번째 호출 시점(1.0s 경과)까지 전부 drift_radius(0.1m) 안의 흔들림이므로
    # 기준점이 한 번도 안 바뀌고 그대로 누적돼 있어야 한다.
    assert result is True


def test_deadlock_release_point_moves_blocker_away_along_its_own_bearing():
    core = HerdingCore(make_config(deadlock_release_distance_m=2.0))
    target_pos = np.array([0.0, 0.0])
    robot2_pos = np.array([1.0, 0.0])  # 표적의 +x 쪽에 서 있음
    release = core._deadlock_release_point(target_pos, robot2_pos)
    # 로봇이 서 있던 그 방향(+x) 그대로, 지정된 거리만큼 더 물러난 지점이어야 한다
    np.testing.assert_allclose(release, np.array([2.0, 0.0]))


def test_deadlock_release_point_falls_back_to_arbitrary_direction_when_coincident():
    """표적과 로봇 2가 (거의) 같은 위치라 방향을 못 정할 때도 죽지 않아야 한다."""
    core = HerdingCore(make_config(deadlock_release_distance_m=1.0))
    same_pos = np.array([3.0, 3.0])
    release = core._deadlock_release_point(same_pos, same_pos.copy())
    assert np.isfinite(release).all()
    assert np.linalg.norm(release - same_pos) == pytest.approx(1.0)


def test_herd_state_triggers_deadlock_release_when_target_stays_put_off_goal():
    """표적을 포획존과 먼 곳에 고정해두고 계속 같은 위치를 관측시키면,
    deadlock_stall_sec가 지난 뒤 HerdingOutput.deadlock_release가 True로
    바뀌고 로봇 2의 목표가 표적에서 확실히 멀어져야 한다."""
    config = make_config(deadlock_stall_sec=0.4, deadlock_drift_radius_m=0.1)
    runner = Runner(config=config, dt=0.2)
    target = (0.5, 0.5)  # capture_zone(3,3)에서 멀리 떨어진 위치
    output = runner.run_until(FSMState.HERD, measurement=target, limit=40)
    assert output.deadlock_release is False, "HERD에 막 진입한 시점엔 아직 정지 누적이 안 됐어야 한다"

    for _ in range(5):  # 0.2s * 5 = 1.0s >= deadlock_stall_sec(0.4s)
        output = runner.run(1, measurement=target)
        if output.deadlock_release:
            break
    assert output.deadlock_release is True
    dist_to_target = float(np.linalg.norm(output.robot2_goal - np.array(target)))
    assert dist_to_target >= config.deadlock_release_distance_m - 1e-6


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


def test_bootstrap_picks_the_geometrically_better_driver_candidate():
    """(플랜 B) 역할은 비용 비교로 동적 배정된다: 처음 HERD에 들어갈 때, 표적에
    더 가깝고 방향도 맞는 로봇이 부트스트랩 시점에 바로 Driver로 뽑혀야 한다
    -- 로봇 1이라는 슬롯 번호 자체에 편향돼서는 안 된다(플랜 A는 정반대로
    로봇 1이 항상 고정 Driver였다 -- herding_controller_트러블슈팅_노트.md
    10번/13번 항목 참고).
    """
    runner = Runner()
    output = runner.run_until(FSMState.HERD, (2.0, 2.0), r1=(8.0, 8.0), r2=(1.5, 1.5))
    assert output.driver_id == 2  # 표적(2,2)에 훨씬 가까운 로봇 2가 부트스트랩 채택됨
    assert output.blocker_id == 1
    # 최초 배정은 이전 배정으로부터의 "교체"가 아니라 부트스트랩이므로 swap이 아니다.
    assert output.role_swapped is False


def test_role_swaps_after_cooldown_when_geometry_reverses():
    """기하학적 상황이 뒤집히고 margin+cooldown을 넘기면 실제로 역할이 스왑된다."""
    runner = Runner()
    runner.run_until(FSMState.HERD, (2.0, 2.0), r1=(8.0, 8.0), r2=(1.5, 1.5))
    assert runner.core.role_assigner._driver_id == 2  # 부트스트랩 상태 확인

    swapped_at = None
    for i in range(20):
        output = runner.run(1, (2.0, 2.0), r1=(1.5, 1.5), r2=(9.0, 9.0))
        if output.role_swapped:
            swapped_at = i
            break
    assert swapped_at is not None, "role_swap_cooldown_sec(2.0s)를 넘겼는데도 스왑되지 않음"
    assert output.driver_id == 1  # 이제 로봇 1이 표적에 가까우므로 Driver로 넘어옴
    assert output.blocker_id == 2
    # 스왑 이후에는 다시 안정적으로 유지된다.
    for _ in range(5):
        output = runner.run(1, (2.0, 2.0), r1=(1.5, 1.5), r2=(9.0, 9.0))
        assert output.driver_id == 1


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
    """world_to_cell()은 범위를 벗어나면 예외를 발생시킨다; core는 폭발하지 않고 성능이 저하되어야 한다.

    역할 배정(_nominal_driving_point 기반 비용 비교)은 grid_map과 무관한
    순수 기하학이라 그리드 밖 표적에서도 계속 동작한다 -- 기본 스폰
    r1=(0,0)/r2=(4,4), 표적(50,50)에서는 로봇 2가 더 가까워 Driver로
    뽑힌다(플랜 A라면 항상 로봇 1이었을 상황). blocking point는 계산할 수
    없으므로(escape distribution 없음) Blocker는 자기 자리를 유지한다.
    """
    runner = Runner()
    output = runner.run(6, (50.0, 50.0))
    assert output.fsm_state == FSMState.HERD
    assert output.escape_top3 == []
    assert output.driver_id == 2
    assert output.blocker_id == 1
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
    import herding_controller_dual.herding_core  # noqa: F401
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


def test_robot_repulsion_activation_distance_defaults_to_unbounded():
    core = HerdingCore(make_config())
    assert core.escape_model.config.robot_repulsion_activation_distance_m == float("inf")


def test_robot_repulsion_activation_distance_wired_to_escape_model():
    core = HerdingCore(make_config(robot_repulsion_activation_distance_m=1.0))
    assert core.escape_model.config.robot_repulsion_activation_distance_m == 1.0
