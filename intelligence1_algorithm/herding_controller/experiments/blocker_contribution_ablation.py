"""로봇 B(Blocker) 기여도 페어드 rescue/regression 어블레이션 스크립트.

트러블슈팅 노트 11-9에서 처음 쓰였지만 저장소에 커밋되지 않아 이후 세션에서
재사용할 수 없었던 분석 방식을 다시 구현하고, 이번엔 반드시 커밋해서 남긴다.

`HerdingCore._stabilize_blocking_point`를 몽키패치해 Blocker를 스폰 위치에
완전히 고정한 "frozen" 조건과, 원래 알고리즘 그대로인 "active" 조건을
**동일 시드**로 짝지어 실행한 뒤,
- rescue: frozen 실패 -> active 성공
- regression: frozen 성공 -> active 실패
를 트랩별로 센다. 집계 성공률 차이만 보면 서로 상쇄되는 rescue/regression을
놓칠 수 있어서(11-3 vs 11-9), 반드시 시드 단위로 페어링해서 본다.

`--robot-repulsion-activation-distance`로 EscapeModel._robot_repulsion()의
Blocker 게이팅 임계값(m)을 오버라이드할 수 있다(기본값은 HerdingConfig
기본값 그대로, 즉 게이팅 없음). `--juke-probability`를 주면 reactive_flee
대신 그 파라미터로 튜닝한 JukingFlee를 evasion model로 쓴다(강화 시나리오
진단용, 트러블슈팅 노트 12번 항목 참고).

사용법:
    python3 blocker_contribution_ablation.py [--trials-per-trap N] \
        [--robot-repulsion-activation-distance D] \
        [--juke-probability P] [--juke-duration S] \
        [--juke-angle-min A] [--juke-angle-max A]
"""
import argparse
import dataclasses
import os
import sys
from contextlib import contextmanager

import numpy as np
from scipy.stats import binomtest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from herding_controller.herding_core import HerdingCore  # noqa: E402
from test import real_map_arena  # noqa: E402
from test.evasion_models.juking_flee import JukingFlee  # noqa: E402
from test.evasion_models.reactive_flee import ReactiveFlee  # noqa: E402
from test.run_validation import (  # noqa: E402
    CONFIG_PATH,
    SIM_CONFIG,
    load_herding_config,
    make_real_map_config,
)
from test.simulator import run_trial_real_map  # noqa: E402


@contextmanager
def _frozen_blocker():
    """HerdingCore._stabilize_blocking_point를 패치해 Blocker를 스폰 위치에 고정한다.

    패치된 함수는 (candidate, now_sec)를 무시하고 항상
    real_map_arena.ROBOT_B_SPAWN을 돌려주므로, resolve_separation()을
    거쳐도 로봇 2의 목표는 사실상 자기 스폰 위치 그대로다 -- "로봇 B가
    아예 관여하지 않았다면"을 근사한다(트러블슈팅 노트 11-3/11-9의 몽키패치
    방식 재구현).
    """
    original = HerdingCore._stabilize_blocking_point

    def frozen(self, candidate, now_sec):
        return real_map_arena.ROBOT_B_SPAWN.copy()

    HerdingCore._stabilize_blocking_point = frozen
    try:
        yield
    finally:
        HerdingCore._stabilize_blocking_point = original


def _make_evasion_model(herding_config, seed, args):
    rng = np.random.default_rng([seed, 4242])
    speed = SIM_CONFIG.target_max_speed_mps
    if args.juke_probability is None:
        return ReactiveFlee(speed, herding_config.flee_reaction_distance_m)
    return JukingFlee(
        speed, herding_config.flee_reaction_distance_m,
        juke_probability_per_sec=args.juke_probability,
        juke_duration_sec=args.juke_duration,
        juke_angle_range=(args.juke_angle_min, args.juke_angle_max),
        rng=rng,
    )


def run_paired_trap(herding_config_base, trap_name, trap_pos, trials, seed_base, args):
    """한 트랩에서 능동/frozen Blocker를 동일 시드로 페어링해 rescue/regression을 센다."""
    config = make_real_map_config(herding_config_base, trap_pos)
    if args.robot_repulsion_activation_distance is not None:
        config = dataclasses.replace(
            config, robot_repulsion_activation_distance_m=args.robot_repulsion_activation_distance,
        )

    active_results = []
    for i in range(trials):
        seed = seed_base + i
        model = _make_evasion_model(config, seed, args)
        active_results.append(run_trial_real_map(config, model, seed, SIM_CONFIG))

    frozen_results = []
    with _frozen_blocker():
        for i in range(trials):
            seed = seed_base + i
            model = _make_evasion_model(config, seed, args)
            frozen_results.append(run_trial_real_map(config, model, seed, SIM_CONFIG))

    rescue = sum(1 for a, f in zip(active_results, frozen_results) if a.success and not f.success)
    regression = sum(1 for a, f in zip(active_results, frozen_results) if f.success and not a.success)
    both_success = sum(1 for a, f in zip(active_results, frozen_results) if a.success and f.success)
    both_fail = sum(1 for a, f in zip(active_results, frozen_results) if not a.success and not f.success)

    n_discordant = rescue + regression
    p_value = (
        binomtest(min(rescue, regression), n_discordant, 0.5, alternative="two-sided").pvalue
        if n_discordant > 0 else 1.0
    )
    return {
        "trap": trap_name, "rescue": rescue, "regression": regression,
        "both_success": both_success, "both_fail": both_fail, "p_value": p_value,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials-per-trap", type=int, default=150)
    parser.add_argument("--seed-base", type=int, default=1_200_000)
    parser.add_argument("--robot-repulsion-activation-distance", type=float, default=None)
    parser.add_argument("--juke-probability", type=float, default=None)
    parser.add_argument("--juke-duration", type=float, default=0.4)
    parser.add_argument("--juke-angle-min", type=float, default=np.pi / 4)
    parser.add_argument("--juke-angle-max", type=float, default=np.pi / 2)
    args = parser.parse_args()

    herding_config_base = load_herding_config(CONFIG_PATH)

    rows = []
    for offset, (trap_name, trap_pos) in enumerate(real_map_arena.TRAPS.items()):
        row = run_paired_trap(
            herding_config_base, trap_name, trap_pos, args.trials_per_trap,
            args.seed_base + offset * 100_000, args,
        )
        rows.append(row)
        print(f"{trap_name:8s}: rescue={row['rescue']:3d} regression={row['regression']:3d} "
              f"both_success={row['both_success']:3d} both_fail={row['both_fail']:3d} "
              f"p={row['p_value']:.3f}")

    total_rescue = sum(r["rescue"] for r in rows)
    total_regression = sum(r["regression"] for r in rows)
    total_n = args.trials_per_trap * len(rows)
    total_discordant = total_rescue + total_regression
    total_p = (
        binomtest(min(total_rescue, total_regression), total_discordant, 0.5,
                  alternative="two-sided").pvalue
        if total_discordant > 0 else 1.0
    )
    print(f"{'전체':8s}: rescue={total_rescue:3d} regression={total_regression:3d} "
          f"(n={total_n}) p={total_p:.3f}")


if __name__ == "__main__":
    main()
