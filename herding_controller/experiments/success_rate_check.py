"""real_map_sim.py의 로봇 B(Blocker) 실질 기여도 소거(ablation) 실험 스크립트.

**2026-08-06부로 통계적 성공률의 정식 출처가 아니다.** N=100/트랩 규모의
공식 성공률 검증은 `run_validation.py: run_real_map_algo_suite()`가 담당한다
(`python3 test/run_validation.py 100`으로 실행, 트러블슈팅 노트 10번 항목
참고). 이 스크립트는 그걸로는 안 나오는 것 하나 -- "로봇 B(Blocker)를
완전히 꺼버리면 성공률이 실제로 얼마나 떨어지는가" -- 를 빠르게 재는 용도로만
남아 있다.

사용법:
    python3 success_rate_check.py [N_트라이얼_모델당]

로봇 A+B 정상 운용과 B를 무력화(blocker_active=False)한 대조군을 같은 시드로
돌려서 "B가 있을 때 vs 없을 때" 성공률 차이를 직접 비교한다.
"""
import sys
import time

import numpy as np

from real_map_sim import ROBOT_A_SPAWN, ROBOT_B_SPAWN, TRAPS, load_room_obstacle_mask, run_trial, sample_free_spawn
from test import real_map_arena
from test.run_validation import CONFIG_PATH, load_herding_config

MODELS = ["reactive_flee", "noisy_human"]


def _setup():
    herding_config = load_herding_config(CONFIG_PATH)
    obstacle_mask, pix, free = load_room_obstacle_mask()
    grid_map = real_map_arena.build_grid_map(obstacle_mask)
    distance_field = real_map_arena.build_distance_field(obstacle_mask)
    return herding_config, grid_map, distance_field


def run_batch(n_per_model, herding_config, grid_map, distance_field, blocker_active=True, base_seed=0):
    rng = np.random.default_rng(base_seed)
    results = []
    seed = base_seed
    for model_name in MODELS:
        for _ in range(n_per_model):
            mouse_spawn = sample_free_spawn(
                grid_map, rng, min_clear_m=0.3,
                exclude_points=[ROBOT_A_SPAWN, ROBOT_B_SPAWN] + list(TRAPS.values()),
                exclude_radius_m=0.6,
            )
            trial = run_trial(
                herding_config, grid_map, distance_field, model_name, seed, mouse_spawn,
                record_frames=False, blocker_active=blocker_active,
            )
            results.append(trial)
            seed += 1
    return results


def summarize(label, results):
    n = len(results)
    successes = [r for r in results if r["success"]]
    rate = 100.0 * len(successes) / n if n else 0.0
    print(f"\n=== {label} (n={n}) ===")
    print(f"성공률: {len(successes)}/{n} = {rate:.1f}%")
    for model_name in MODELS:
        sub = [r for r in results if r["model"] == model_name]
        sub_succ = [r for r in sub if r["success"]]
        sub_rate = 100.0 * len(sub_succ) / len(sub) if sub else 0.0
        print(f"  {model_name}: {len(sub_succ)}/{len(sub)} = {sub_rate:.1f}%")

    dtimes = [r["discovery_time"] for r in successes if r["discovery_time"] is not None]
    durs = [r["duration"] for r in successes]
    if dtimes:
        print(f"평균 발견 시각: {np.mean(dtimes):.1f}s, 평균 소요 시간(성공 시행): {np.mean(durs):.1f}s")

    min_dists = [r["min_blocker_dist_after_discovery"] for r in successes
                 if r["min_blocker_dist_after_discovery"] is not None]
    if min_dists:
        print(f"[B 기여도] 발견 이후 B<->표적 최소거리: 평균 {np.mean(min_dists):.2f}m, "
              f"중앙값 {np.median(min_dists):.2f}m, 최댓값 {np.max(min_dists):.2f}m")

    return rate


if __name__ == "__main__":
    n_per_model = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 25

    herding_config, grid_map, distance_field = _setup()

    t0 = time.time()
    results = run_batch(n_per_model, herding_config, grid_map, distance_field, blocker_active=True)
    rate = summarize("정상 운용 (로봇 A + 로봇 B)", results)
    print(f"\n(소요 시간: {time.time() - t0:.1f}s)")

    t1 = time.time()
    results_no_b = run_batch(n_per_model, herding_config, grid_map, distance_field, blocker_active=False)
    rate_no_b = summarize("소거 실험: 로봇 B 무력화 (로봇 A 단독)", results_no_b)
    print(f"\n(소요 시간: {time.time() - t1:.1f}s)")
    print(f"\n=== B의 순수 기여도 ===\nA+B: {rate:.1f}%  vs  A 단독: {rate_no_b:.1f}%  "
          f"(차이: {rate - rate_no_b:+.1f}%p)")
