"""blocker_contribution_ablation.py 스모크 테스트 -- 작은 trial 수로
run_paired_trap()이 구조적으로 일관된 결과를 내는지만 빠르게 확인한다.
(전체 스윕(N=150/트랩)은 이 테스트가 커버할 범위가 아니다 -- 그건 다음
세션의 실제 실험 실행이 담당한다.)
"""
import argparse
import os
import sys

_EXPERIMENTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "experiments"
)
sys.path.insert(0, _EXPERIMENTS_DIR)

from blocker_contribution_ablation import run_paired_trap  # noqa: E402
from test import real_map_arena  # noqa: E402
from test.run_validation import CONFIG_PATH, load_herding_config  # noqa: E402


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
