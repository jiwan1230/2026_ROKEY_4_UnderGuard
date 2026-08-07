"""blocker_contribution_ablation.py 스모크 테스트 -- 작은 trial 수로
run_paired_trap()이 구조적으로 일관된 결과를 내는지만 빠르게 확인한다.
(전체 스윕(N=150/트랩)은 이 테스트가 커버할 범위가 아니다 -- 그건 다음
세션의 실제 실험 실행이 담당한다.)
"""
import argparse
import os
import sys

import numpy as np

_EXPERIMENTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "experiments"
)
sys.path.insert(0, _EXPERIMENTS_DIR)

from blocker_contribution_ablation import _frozen_blocker, run_paired_trap  # noqa: E402
from test import real_map_arena  # noqa: E402
from test.evasion_models.reactive_flee import ReactiveFlee  # noqa: E402
from test.run_validation import (  # noqa: E402
    CONFIG_PATH,
    SIM_CONFIG,
    load_herding_config,
    make_real_map_config,
)
from test.simulator import run_trial_real_map  # noqa: E402


def _args(**overrides):
    defaults = dict(
        robot_repulsion_activation_distance=None,
        juke_probability=None, juke_duration=0.4,
        juke_angle_min=0.0, juke_angle_max=0.0,
    )
    defaults.update(overrides)
    return argparse.Namespace(**defaults)


def test_run_paired_trap_counts_are_internally_consistent():
    herding_config_base = load_herding_config(CONFIG_PATH)
    trap_name, trap_pos = next(iter(real_map_arena.TRAPS.items()))
    row = run_paired_trap(
        herding_config_base, trap_name, trap_pos, trials=3, seed_base=999_000, args=_args()
    )
    total = row["rescue"] + row["regression"] + row["both_success"] + row["both_fail"]
    assert total == 3
    assert 0.0 <= row["p_value"] <= 1.0


def test_run_paired_trap_accepts_activation_distance_override():
    herding_config_base = load_herding_config(CONFIG_PATH)
    trap_name, trap_pos = next(iter(real_map_arena.TRAPS.items()))
    row = run_paired_trap(
        herding_config_base, trap_name, trap_pos, trials=3, seed_base=999_100,
        args=_args(robot_repulsion_activation_distance=1.0),
    )
    total = row["rescue"] + row["regression"] + row["both_success"] + row["both_fail"]
    assert total == 3


def test_frozen_blocker_stays_at_spawn_even_during_deadlock_release():
    """_frozen_blocker()가 _stabilize_blocking_point만 패치했을 때는 이 테스트가
    실패했다 -- deadlock release가 blocking_point를 다시 덮어써서 로봇 B가
    스폰에서 최대 3m까지 실제로 이동했다(최종 브랜치 리뷰, seed=999000/999003
    실측). _deadlock_release_point도 함께 패치한 뒤로는 통과해야 한다.

    임계값 노트: step()의 resolve_separation()은 patched blocking point에도
    여전히 적용되므로(role_assigner.py), Driver의 driving point가 스폰에
    min_robot_separation_m(0.6m) 이내로 접근하면 로봇 2의 명령 목표가 스폰에서
    최대 그 정도(관측상 <0.2m) 밀려날 수 있다 -- 이는 알고리즘의 정상 동작이지
    이 fix가 잡으려는 버그가 아니다. deadlock release 버그는 스폰이 아니라
    *표적* 기준으로 최대 deadlock_release_distance_m(1.0m)만큼 떨어진 지점을
    반환하므로 표적이 스폰에서 멀리 떨어진 트랩에서는 훨씬 더 크게(실측 3.02m)
    벗어난다. 1.0m 임계값은 정상적인 resolve_separation 잡음은 여유 있게
    통과시키면서 deadlock release 버그(3.02m)는 확실히 잡아낸다.
    """
    herding_config_base = load_herding_config(CONFIG_PATH)
    trap_name, trap_pos = next(iter(real_map_arena.TRAPS.items()))
    config = make_real_map_config(herding_config_base, trap_pos)
    with _frozen_blocker():
        for seed in range(999_000, 999_012):
            model = ReactiveFlee(SIM_CONFIG.target_max_speed_mps, config.flee_reaction_distance_m)
            result = run_trial_real_map(config, model, seed, SIM_CONFIG)
            max_disp = max(
                float(np.linalg.norm(np.asarray(pos) - real_map_arena.ROBOT_B_SPAWN))
                for pos in result.robot2_trajectory
            )
            assert max_disp < 1.0, f"seed={seed}: frozen Blocker moved {max_disp:.2f}m from spawn"
            # 리뷰가 구체적으로 실측한 두 시드는 훨씬 더 엄격하게 검증한다:
            # 픽스 이전엔 이 두 시드에서 3.02m가 관측되었고, 픽스 이후엔
            # resolve_separation조차 걸리지 않아 사실상 0m여야 한다.
            if seed in (999_000, 999_003):
                assert max_disp < 0.05, (
                    f"seed={seed}: expected ~0m (no resolve_separation trigger observed "
                    f"pre-fix analysis), got {max_disp:.2f}m"
                )
