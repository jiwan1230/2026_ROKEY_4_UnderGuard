import csv

import numpy as np
import pytest

from herding_controller_dual.grid_map import GridConfig, GridMap
from test.evasion_models.log_replay import LogReplay
from test.evasion_models.noisy_human import NoisyHuman
from test.evasion_models.random_walk import RandomWalk
from test.evasion_models.reactive_flee import ReactiveFlee
from test.evasion_models.wall_hugger import WallHugger


def test_reactive_flee_moves_away_from_nearby_robot():
    model = ReactiveFlee(max_speed_mps=0.4, flee_reaction_distance_m=1.0)
    target_state = np.array([5.0, 5.0, 0.0, 0.0])
    velocity = model.step(target_state, [np.array([4.5, 5.0])], obstacle_map=None, dt=0.1)
    assert velocity[0] > 0  # 타겟이 +x 방향으로 도망침, -x 쪽 로봇으로부터 멀어짐
    assert np.linalg.norm(velocity) <= 0.4 + 1e-9


def test_reactive_flee_stays_still_when_robot_is_far():
    model = ReactiveFlee(max_speed_mps=0.4, flee_reaction_distance_m=1.0)
    target_state = np.array([5.0, 5.0, 0.0, 0.0])
    velocity = model.step(target_state, [np.array([0.0, 0.0])], obstacle_map=None, dt=0.1)
    assert np.linalg.norm(velocity) < 1e-6


def test_wall_hugger_flees_when_robot_close():
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    model = WallHugger(max_speed_mps=0.4, flee_reaction_distance_m=1.0, grid_map=grid)
    target_state = np.array([5.0, 5.0, 0.0, 0.0])
    velocity = model.step(target_state, [np.array([4.5, 5.0])], obstacle_map=None, dt=0.1)
    assert np.linalg.norm(velocity) > 0


def test_random_walk_ignores_robot_but_moves():
    model = RandomWalk(max_speed_mps=0.4, rng=np.random.default_rng(0))
    target_state = np.array([5.0, 5.0, 0.0, 0.0])
    v1 = model.step(target_state, [np.array([5.01, 5.0])], obstacle_map=None, dt=0.1)
    assert np.linalg.norm(v1) <= 0.4 + 1e-9


def test_noisy_human_delays_reaction(monkeypatch):
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    rng = np.random.default_rng(0)
    model = NoisyHuman(
        max_speed_mps=0.4, flee_reaction_distance_m=1.0, grid_map=grid,
        reaction_delay_range=(1.0, 1.0), noise_std=0.0, rng=rng,
    )
    target_state = np.array([5.0, 5.0, 0.0, 0.0])
    # 1.0초 반응 지연이 경과하기 전에 발생한 첫 호출들 -> 속도 명령은 여전히 0
    v_early = model.step(target_state, [np.array([4.5, 5.0])], obstacle_map=None, dt=0.5)
    assert np.linalg.norm(v_early) < 1e-6
    v_late = model.step(target_state, [np.array([4.5, 5.0])], obstacle_map=None, dt=0.6)
    assert np.linalg.norm(v_late) > 0


def test_noisy_human_clips_noisy_command_to_max_speed():
    # 회귀 테스트: 기본 WallHugger 명령에 가우시안 노이즈를 추가하더라도
    # 반환되는 속도가 max_speed_mps를 초과해서는 안 된다. noise_std=0.5이고
    # 반응 지연이 거의 0에 가까울 때, 클리핑되지 않은 구현은 상한값을
    # 여러 배 초과하는 속도를 자주 생성한다 (0.4 m/s 상한 대비 최대 약 2.1 m/s 관측됨).
    grid = GridMap(GridConfig(resolution_m=0.25, width_cells=40, height_cells=40))
    rng = np.random.default_rng(42)
    model = NoisyHuman(
        max_speed_mps=0.4, flee_reaction_distance_m=1.0, grid_map=grid,
        reaction_delay_range=(0.01, 0.01), noise_std=0.5, rng=rng,
    )
    target_state = np.array([5.0, 5.0, 0.0, 0.0])
    for _ in range(500):
        velocity = model.step(target_state, [np.array([4.5, 5.0])], obstacle_map=None, dt=0.02)
        assert np.linalg.norm(velocity) <= 0.4 + 1e-9


def test_log_replay_reads_csv_and_returns_velocity_to_next_point(tmp_path):
    csv_path = tmp_path / "trace.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y"])
        writer.writerow([0.0, 0.0, 0.0])
        writer.writerow([1.0, 1.0, 0.0])
    model = LogReplay(str(csv_path))
    target_state = np.array([0.0, 0.0, 0.0, 0.0])
    velocity = model.step(target_state, [], obstacle_map=None, dt=0.1)
    assert velocity[0] > 0


def test_log_replay_clips_velocity_spike_when_dt_misaligns_with_sample_boundary(tmp_path):
    # 회귀 테스트: 시뮬레이터의 고정 dt가 CSV 샘플 경계와 정확히 맞아떨어지지 않으면,
    # 한 스텝이 샘플을 지나친 직후 `remaining = next_t - elapsed`가 거의 0으로 줄어들어
    # (next_pos - target_pos) / remaining 값이 정상 범위를 크게 벗어나 급증할 수 있다.
    # CSV 타임스탬프 0.00/0.05/0.11을 dt=0.1로 재생하면 이를 재현한다: 한 스텝 후
    # elapsed=0.10이 되어 모델이 t=0.05 샘플을 지나치고 remaining=0.11-0.10=0.01이 된다.
    csv_path = tmp_path / "trace.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", "x", "y"])
        writer.writerow([0.00, 0.0, 0.0])
        writer.writerow([0.05, 1.0, 0.0])
        writer.writerow([0.11, 2.0, 0.0])
    model = LogReplay(str(csv_path), max_speed_mps=0.4)
    # 타겟의 실제 위치가 다음에 기록된 샘플과 거리가 멀기 때문에, 단순한
    # (next_pos - target_pos) / remaining 나눗셈은 상한 없이 급증한다.
    target_state = np.array([0.0, 0.0, 0.0, 0.0])
    velocity = model.step(target_state, [], obstacle_map=None, dt=0.1)
    assert np.linalg.norm(velocity) <= 0.4 + 1e-9
