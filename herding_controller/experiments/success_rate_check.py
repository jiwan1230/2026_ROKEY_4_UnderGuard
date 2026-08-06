"""real_map_sim.py의 통계적 성공률 + 로봇 B(Blocker) 실질 기여도 측정 스크립트.

`real_map_sim.py`의 main()은 GIF/Artifact 재생용으로 딱 4개 시행만 저장한다.
90% 성공률 목표를 검증하려면 훨씬 많은 시행이 필요하고, "로봇 B가 정말
기여했는가"도 숫자로 확인해야 한다 (사용자 지적: 포획 반경이 커서 로봇 A
혼자, B가 오기도 전에 성공해버리는 게 아닌지). 그래서 이 스크립트는
GIF/JSON을 만들지 않고(`record_frames=False`) 순수 통계만 빠르게 뽑는다.

사용법:
    python3 success_rate_check.py [N_트라이얼_모델당] [--ablation]

--ablation을 주면 로봇 B를 완전히 무력화(blocker_active=False)한 대조군도
같은 시드로 돌려서, "B가 있을 때 vs 없을 때" 성공률 차이를 직접 비교한다.
"""
import sys
import time

import numpy as np

from real_map_sim import (
    CAPTURE_RADIUS_M, ROBOT_A_SPAWN, ROBOT_B_SPAWN, TRAPS,
    load_room_obstacle_mask, make_target_model, run_trial, sample_free_spawn,
)
from herding_controller.grid_map import GridConfig, GridMap
from herding_controller.herding_planner import PlannerConfig
from test.run_validation import CONFIG_PATH, load_herding_config

RESOLUTION = 0.05
ORIGIN_X, ORIGIN_Y = -3.19, -9.03
MODELS = ["reactive_flee", "noisy_human"]


def _setup():
    herding_config = load_herding_config(CONFIG_PATH)
    obstacle_mask, pix, free = load_room_obstacle_mask()
    height_cells, width_cells = obstacle_mask.shape
    grid_map = GridMap(GridConfig(
        resolution_m=RESOLUTION, width_cells=width_cells, height_cells=height_cells,
        origin_x_m=ORIGIN_X, origin_y_m=ORIGIN_Y,
    ))
    grid_map.obstacle_mask = obstacle_mask
    from scipy import ndimage
    distance_field = ndimage.distance_transform_edt(~obstacle_mask)
    planner_config = PlannerConfig(
        drive_distance_m=herding_config.drive_distance_m, panic_distance_m=herding_config.panic_distance_m,
        alignment_threshold=herding_config.alignment_threshold,
        drive_distance_ease_factor=herding_config.drive_distance_ease_factor,
        block_lookahead_m=1.8,
    )
    return herding_config, grid_map, distance_field, planner_config


def run_batch(n_per_model, herding_config, grid_map, distance_field, planner_config, blocker_active=True,
              base_seed=0, use_geodesic=True):
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
                herding_config, planner_config, grid_map, distance_field, model_name, seed, mouse_spawn,
                record_frames=False, blocker_active=blocker_active, use_geodesic=use_geodesic,
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

    # 로봇 B(Blocker) 기여도: 발견 이후 표적에 가장 가까이 다가간 거리의 분포.
    # 이 값이 계속 크면(예: SENSOR_RANGE_M보다 훨씬 크면) B가 몰이에 거의
    # 관여하지 못했다는 뜻이고, 작으면 B가 실제로 도주로 근처까지 다가가
    # 압박했다는 뜻이다.
    min_dists = [r["min_blocker_dist_after_discovery"] for r in successes
                 if r["min_blocker_dist_after_discovery"] is not None]
    if min_dists:
        print(f"[B 기여도] 발견 이후 B<->표적 최소거리: 평균 {np.mean(min_dists):.2f}m, "
              f"중앙값 {np.median(min_dists):.2f}m, 최댓값 {np.max(min_dists):.2f}m")
        close_frac = 100.0 * sum(d <= 1.5 for d in min_dists) / len(min_dists)
        print(f"           1.5m(센서 반경) 이내로 접근한 성공 시행 비율: {close_frac:.0f}%")

    cap_dists = [r["blocker_dist_at_capture"] for r in successes if r["blocker_dist_at_capture"] is not None]
    if cap_dists:
        print(f"[B 기여도] 포획 순간 B<->표적 거리: 평균 {np.mean(cap_dists):.2f}m, 중앙값 {np.median(cap_dists):.2f}m")

    esc_max = [r["escape_max_at_capture"] for r in successes if r["escape_max_at_capture"] is not None]
    if esc_max:
        print(f"[게이트 확인] 포획 순간 도주확률 최댓값: 평균 {np.mean(esc_max):.2f} "
              f"(escape_concentration_threshold={herding_config_global.escape_concentration_threshold} 이상이어야 포획 인정됨)")

    # 실패 원인 분석: 실패한 시행 중 "반경 안에는 들어갔었는가"와 "도주확률이
    # 집중된 적은 있었는가"를 각각 따로 세어, 두 조건 중 어느 쪽이 병목인지 구분한다.
    failures = [r for r in results if not r["success"]]
    if failures:
        not_discovered = sum(1 for r in failures if not r["discovered"])
        in_radius_ever = sum(1 for r in failures if r["ever_in_radius"])
        concentrated_ever = sum(1 for r in failures if r["ever_concentrated"])
        both_ever = sum(1 for r in failures if r["ever_both"])
        n_f = len(failures)
        print(f"[실패 {n_f}건 분석] 발견조차 못함: {not_discovered} | "
              f"반경 안 진입 경험 있음: {in_radius_ever}/{n_f} | "
              f"도주확률 집중 경험 있음: {concentrated_ever}/{n_f} | "
              f"둘 다 동시 충족 경험 있음(=capture_hold_sec 3초 연속을 못 채워서만 실패): {both_ever}/{n_f}")
        closest = [r["min_dist_to_goal_ever"] for r in failures if r["min_dist_to_goal_ever"] is not None]
        if closest:
            print(f"[실패 {n_f}건] 목표 트랩까지 최근접 거리(m): 평균 {np.mean(closest):.2f}, "
                  f"중앙값 {np.median(closest):.2f}, 최솟값(가장 근접했던 시행) {np.min(closest):.2f} "
                  f"(포획 인정 기준 {CAPTURE_RADIUS_M}m)")

    return rate


if __name__ == "__main__":
    n_per_model = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 25
    ablation = "--ablation" in sys.argv
    compare_geodesic = "--compare-geodesic" in sys.argv

    herding_config, grid_map, distance_field, planner_config = _setup()
    herding_config_global = herding_config  # summarize()에서 참조

    t0 = time.time()
    results = run_batch(n_per_model, herding_config, grid_map, distance_field, planner_config, blocker_active=True)
    rate = summarize("정상 운용 (로봇 A + 로봇 B, geodesic 목표방향 적용)", results)
    print(f"\n(소요 시간: {time.time() - t0:.1f}s)")

    if compare_geodesic:
        t2 = time.time()
        results_euclid = run_batch(n_per_model, herding_config, grid_map, distance_field, planner_config,
                                   blocker_active=True, use_geodesic=False)
        rate_euclid = summarize("대조군: geodesic 없이 직선(유클리드) 목표방향", results_euclid)
        print(f"\n(소요 시간: {time.time() - t2:.1f}s)")
        print(f"\n=== geodesic 목표방향의 순수 효과 ===\ngeodesic: {rate:.1f}%  vs  유클리드(기존): {rate_euclid:.1f}%  "
              f"(차이: {rate - rate_euclid:+.1f}%p)")

    if ablation:
        t1 = time.time()
        results_no_b = run_batch(n_per_model, herding_config, grid_map, distance_field, planner_config,
                                 blocker_active=False)
        rate_no_b = summarize("소거 실험: 로봇 B 무력화 (로봇 A 단독)", results_no_b)
        print(f"\n(소요 시간: {time.time() - t1:.1f}s)")
        print(f"\n=== B의 순수 기여도 ===\nA+B: {rate:.1f}%  vs  A 단독: {rate_no_b:.1f}%  "
              f"(차이: {rate - rate_no_b:+.1f}%p)")
