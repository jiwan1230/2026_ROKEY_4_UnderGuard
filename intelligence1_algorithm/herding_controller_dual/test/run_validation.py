# herding_controller_dual/test/run_validation.py
"""ALGO-001~008 수용 시행, ALGO-008 대조 실험을 실행하고 플롯을 작성한다."""
import dataclasses
import os
import sys

import matplotlib

# pyplot import보다 앞서야 한다: 이 스크립트는 디스플레이 없이 렌더링해야 한다.
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from scipy.stats import chi2_contingency  # noqa: E402

# 그래프 라벨을 한글로 표기하기 위한 폰트 설정 (미설정 시 한글이 네모(tofu)로 깨짐).
# matplotlib은 .ttc 폰트 파일에서 첫 번째 서브패밀리 이름만 색인하기 때문에,
# 시스템에는 "Noto Sans CJK KR"이 있어도 matplotlib 폰트 매니저는 "Noto Sans CJK JP"라는
# 이름으로만 이 폰트를 찾을 수 있다 (글리프 자체는 두 이름 모두 동일한 한글 포함).
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
matplotlib.rcParams["axes.unicode_minus"] = False

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from herding_controller_dual.grid_map import GridConfig, GridMap  # noqa: E402
from herding_controller_dual.herding_core import (  # noqa: E402
    HerdingConfig,
    HerdingCore,
    Observation,
)
from herding_controller_dual.state_machine import FSMState  # noqa: E402
from test.evasion_models.noisy_human import NoisyHuman  # noqa: E402
from test.evasion_models.random_walk import RandomWalk  # noqa: E402
from test.evasion_models.reactive_flee import ReactiveFlee  # noqa: E402
from test.evasion_models.wall_hugger import WallHugger  # noqa: E402
from test import real_map_arena  # noqa: E402
from test.simulator import SimulatorConfig, run_trial, run_trial_real_map  # noqa: E402

# occlusion-recovery 검사는 타겟 측정값을 보류할 수 있는 제어 루프가 필요한데,
# run_trial()은 이를 노출하지 않는다. 그래서 (복제해서 어긋나게 만들기보다는)
# 시뮬레이터 자체의 물리 헬퍼 함수를 재사용한다.
from test.simulator import (  # noqa: E402
    _advance_target,
    _arena_bounds,
    _bind_model_to_arena,
    _boundary_ring_mask,
    _move_toward,
    _spawn_driver_near_target,
    _step_body,
    _update_heading,
)

# 이 파일을 기준으로 경로를 고정하여, cwd가 무엇이든 출력이 test/output/에 놓이도록 한다.
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "herding_params.yaml"
)

SIM_CONFIG = SimulatorConfig()
EVASION_MODEL_NAMES = ("reactive_flee", "wall_hugger", "noisy_human", "random_walk")

# --- ALGO-00x 임계값, 스펙의 수용 기준표에서 그대로 가져옴 --------------- #
ALGO_001_MIN_SUCCESS_RATE = 0.70
ALGO_002_MAX_MEAN_TIME_SEC = 60.0
ALGO_003_MAX_PANIC_RATE = 0.10
ALGO_004_MAX_ROLE_SWAPS = 5
ALGO_005_MAX_LATENCY_MS = 100.0
ALGO_006_MAX_RECOVERY_SEC = 5.0
ALGO_006_MIN_RECOVERY_RATE = 0.80
ALGO_008_MIN_DIFFERENCE_PP = 40.0
ALGO_008_MAX_P_VALUE = 0.05

# ALGO-006 센서 모델. 스펙은 복구 기한(5초)은 고정하지만 타겟이 *어떻게* 다시 보이게
# 되는지에 대해서는 아무 말도 하지 않는다. 스위트의 나머지 부분이 사용하는 완벽한
# 전역 센서를 그대로 쓰면 블랙아웃은 그냥 시간이 지나면 끝나버려서 -- 알고리즘이
# 아무것도 하지 않아도 즉시 "복구"된 것처럼 보일 수 있다. 그래서 재탐지는 로봇이
# 이 범위 안으로 접근해야 한다는 조건으로 모델링했으며, 이 조건이 있어야 이 검사가
# occluder의 지속 시간이 아니라 LOST 상태의 occlusion-grid 탐색을 실제로 측정하게 된다.
# 이는 이 검증 하니스의 가정일 뿐 스펙 파라미터가 아니다; 보고서를 참고할 것.
OCCLUSION_SENSOR_RANGE_M = 1.5

# run_trial()은 시행 시드로 자신만의 spawn RNG를 초기화하며, 그 첫 번째 추출값이
# 타겟의 spawn x이다. 따라서 회피 모델을 같은 값으로 시드하면 모델의 무작위성이
# spawn의 결정론적 함수가 되어 버린다: 200개의 시드로 측정한 결과 RandomWalk의 초기
# 방향(heading)이 정확히 2*pi*(spawn_x - lo)/(hi - lo)로 나타났고(상관계수 1.0),
# NoisyHuman의 반응 지연도 마찬가지였다. 같은 시드의 별도 스트림에서 모델을 뽑으면
# 모든 시행의 재현성은 유지하면서 둘을 진짜로 독립적으로 만들 수 있다.
MODEL_RNG_STREAM = 1

# 민감도 플롯을 위한 파라미터 스윕; 각 리스트에는 기준값(baseline)이 포함되어 있다
# (플롯이 이를 표시하므로, 설정된 값이 빠진 리스트는 라벨 없는 차트를 만들어낸다 --
# 이 값들을 config/herding_params.yaml과 계속 맞춰 둘 것).
#
# block_lookahead_m은 Task 15에서 추가되었다. 원래의 두 항목은 스펙이 예측한 가장
# 민감한 파라미터에서 왔지만 실측 결과는 달랐다: robot_repulsion_weight를 8배 범위로
# 스윕해도 성공률은 몇 포인트밖에 움직이지 않는 반면, block_lookahead_m은 성공률을
# 25%에서 82%까지 움직인다. 이를 민감도 그림에서 빼면 이 시스템이 실제로 민감하게
# 반응하는 유일한 파라미터를 숨기는 셈이 된다.
SENSITIVITY_SWEEPS = {
    # 2026-08-06: 실제 맵 기준값(drive_distance_m=0.3, block_lookahead_m=1.8)에
    # 맞춰 스윕 지점도 재조정 (트러블슈팅 노트 10번 항목). 여전히 상위 두 지점은
    # HerdingConfig 불변식(drive_distance_m * ease < flee_reaction_distance_m,
    # 0.42) 위반 지점을 의도적으로 포함한다: 0.45*1.15=0.5175, 0.6*1.15=0.69.
    "drive_distance_m": (0.15, 0.3, 0.45, 0.6),
    "block_lookahead_m": (0.9, 1.8, 2.7, 3.6),
    "robot_repulsion_weight": (0.5, 1.0, 1.5, 2.0),
}


def load_herding_config(yaml_path: str) -> HerdingConfig:
    """config/herding_params.yaml을 평평한(flat) HerdingConfig로 로드한다."""
    with open(yaml_path) as f:
        raw = yaml.safe_load(f)
    params = raw["herding_controller_dual"]["ros__parameters"]
    # 직접 작성한 kwargs 리스트가 아니라 이름으로 매핑한다: 이 스크립트가 작성된 이후
    # HerdingConfig에는 이미 필드가 추가되었으며, 낡은 리터럴 리스트를 쓰면 잘못된
    # 파라미터로 알고리즘을 조용히 검증해 버리게 된다.
    known = {field.name for field in dataclasses.fields(HerdingConfig)}
    unknown = sorted(set(params) - known)
    if unknown:
        raise ValueError(f"{yaml_path} has parameters HerdingConfig does not accept: {unknown}")
    return HerdingConfig(**params)


def _make_grid_map(herding_config: HerdingConfig) -> GridMap:
    """그리드를 참조하는 회피 모델을 위해, 설정과 일치하는 독립적인 GridMap을 생성한다."""
    return GridMap(GridConfig(
        resolution_m=herding_config.grid_resolution_m,
        width_cells=herding_config.grid_width_cells,
        height_cells=herding_config.grid_height_cells,
        origin_x_m=herding_config.grid_origin_x_m,
        origin_y_m=herding_config.grid_origin_y_m,
    ))


def _make_model(name: str, herding_config: HerdingConfig, seed: int):
    """이름으로 회피 모델 하나를 생성한다. 재현성을 위해 시드를 지정한다."""
    rng = np.random.default_rng([seed, MODEL_RNG_STREAM])
    speed = SIM_CONFIG.target_max_speed_mps
    if name == "reactive_flee":
        return ReactiveFlee(speed, herding_config.flee_reaction_distance_m)
    # WallHugger/NoisyHuman은 GridMap을 통해 벽을 판별한다; run_trial이 이들을 자신의
    # 아레나로 재연결해 주지만, 독립적으로도 동작하도록 여기서 유효한 GridMap을 제공한다.
    if name == "wall_hugger":
        return WallHugger(speed, herding_config.flee_reaction_distance_m, _make_grid_map(herding_config))
    if name == "noisy_human":
        return NoisyHuman(speed, herding_config.flee_reaction_distance_m,
                          _make_grid_map(herding_config), rng=rng)
    if name == "random_walk":
        return RandomWalk(speed, rng=rng)
    raise ValueError(f"unknown evasion model: {name}")


def make_real_map_config(herding_config_base: HerdingConfig, capture_zone) -> HerdingConfig:
    """실제 SLAM 맵(room_map.pgm) 격자 파라미터 + 지정된 트랩 좌표로 config를 만든다.

    grid_resolution_m/grid_width_cells/grid_height_cells/grid_origin_x_m/
    grid_origin_y_m은 `maps/room_map.yaml`의 실측값과 반드시 일치해야 한다
    (herding_node.py가 /map 수신 시 이 값들과 다르면 경고를 낸다).
    """
    obstacle_mask = real_map_arena.load_room_obstacle_mask()
    height_cells, width_cells = obstacle_mask.shape
    return dataclasses.replace(
        herding_config_base,
        grid_resolution_m=real_map_arena.RESOLUTION_M,
        grid_width_cells=width_cells, grid_height_cells=height_cells,
        grid_origin_x_m=real_map_arena.ORIGIN_X_M, grid_origin_y_m=real_map_arena.ORIGIN_Y_M,
        capture_zone_x_m=float(capture_zone[0]), capture_zone_y_m=float(capture_zone[1]),
    )


def run_real_map_model_trials(herding_config: HerdingConfig, model_name: str, trials: int, seed_base: int) -> list:
    """실제 맵 위에서 하나의 회피 모델에 대해 `trials`번 시행하고 TrialResults를 반환한다.

    `_make_model()`을 그대로 재사용한다: WallHugger/NoisyHuman이 생성 시 받는
    GridMap의 obstacle_mask는 어차피 `run_trial_real_map()` 내부의
    `_bind_model_to_arena()`가 실제 맵으로 재연결하므로 신경 쓸 필요 없다.
    """
    results = []
    for i in range(trials):
        seed = seed_base + i
        model = _make_model(model_name, herding_config, seed)
        results.append(run_trial_real_map(herding_config, model, seed, SIM_CONFIG))
    return results


# 실제 맵 검증은 reactive_flee(주 검증 모델)와 noisy_human(실물 근사)만 돈다 --
# wall_hugger/random_walk은 추상 아레나 쪽에서 이미 검증되고 있고, 실제 맵의
# 문턱 회피 로직과의 상호작용까지 추가로 검증할 필요성은 낮다고 판단했다.
REAL_MAP_MODEL_NAMES = ("reactive_flee", "noisy_human")


def run_real_map_algo_suite(herding_config_base: HerdingConfig, trials_per_trap: int = 100,
                            seed_base: int = 0) -> dict:
    """실제 SLAM 맵 위에서, 포획구역 후보 3곳 각각에 대해 ALGO-001/002/003/005를 검증한다.

    이 스위트가 "정식" 검증이다 (2026-08-06 정정, 트러블슈팅 노트 10번
    항목): 추상 정사각형 아레나(`run_algo_suite`)는 이제 빠른 회귀 테스트
    용도로만 남고, 실제 배포 성공률의 근거는 여기서 나온다. ALGO-004(역할
    진동, 역할이 고정이라 항상 0)/006(occlusion)/007(구조적)/008(대조군)은
    추상 아레나 쪽 검증을 그대로 채택한다 -- 006/008은 통제된 비교 실험이라
    단순한 환경에서 더 깨끗하게 측정되고, 004/007은 환경과 무관한 구조적
    보장이라 어느 쪽에서 측정해도 같다.
    """
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    trap_results = {}
    for offset, (trap_name, trap_pos) in enumerate(real_map_arena.TRAPS.items()):
        config = make_real_map_config(herding_config_base, trap_pos)
        model_results = {}
        for model_name in REAL_MAP_MODEL_NAMES:
            results = run_real_map_model_trials(
                config, model_name, trials_per_trap, seed_base + offset * 100_000
            )
            model_results[model_name] = (results, summarize(results))
        trap_results[trap_name] = model_results

    _, primary_summary = trap_results["top"]["reactive_flee"]
    algo_status = {
        "ALGO-001 (실제맵)": primary_summary["success_rate"] >= ALGO_001_MIN_SUCCESS_RATE,
        "ALGO-002 (실제맵)": primary_summary["mean_time_sec"] <= ALGO_002_MAX_MEAN_TIME_SEC,
        "ALGO-003 (실제맵)": primary_summary["panic_rate"] <= ALGO_003_MAX_PANIC_RATE,
        "ALGO-005 (실제맵)": primary_summary["mean_latency_ms"] <= ALGO_005_MAX_LATENCY_MS,
    }
    _write_real_map_report(trap_results, algo_status, trials_per_trap)
    return {"trap_results": trap_results, "algo_status": algo_status}


def _write_real_map_report(trap_results: dict, algo_status: dict, trials_per_trap: int) -> None:
    lines = ["=== 실제 맵 검증 (room_map.pgm, 정식) ==="]
    all_results_by_model = {name: [] for name in REAL_MAP_MODEL_NAMES}
    for trap_name, model_results in trap_results.items():
        lines.append(f"--- 트랩: {trap_name} ---")
        for model_name, (results, summary) in model_results.items():
            all_results_by_model[model_name].extend(results)
            discoveries = [r.discovery_time_sec for r in results if r.discovery_time_sec is not None]
            lines.append(
                f"  {model_name:13s}: 성공 {summary['success_rate']*100:5.1f}% | "
                f"평균 소요 {summary['mean_time_sec']:5.1f}s | panic {summary['panic_rate']*100:4.1f}% | "
                f"평균 발견시각 {np.mean(discoveries):.1f}s" if discoveries else
                f"  {model_name:13s}: 성공 {summary['success_rate']*100:5.1f}%"
            )
    lines.append("--- 트랩 전체 통합 (모델별) ---")
    for model_name, results in all_results_by_model.items():
        summary = summarize(results)
        lines.append(
            f"  {model_name:13s}: 성공 {summary['success_rate']*100:5.1f}% (n={summary['trials']}) | "
            f"평균 소요 {summary['mean_time_sec']:5.1f}s | panic {summary['panic_rate']*100:4.1f}%"
        )
    lines.append(f"=== SUMMARY (실제 맵, 트랩당 {trials_per_trap}회 × {len(REAL_MAP_MODEL_NAMES)}개 모델) ===")
    lines.append(" / ".join(f"{k} {'PASS' if v else 'FAIL'}" for k, v in algo_status.items()))
    report = "\n".join(lines)
    print(report)
    with open(os.path.join(OUTPUT_DIR, "real_map_validation_report.txt"), "w") as f:
        f.write(report + "\n")


def run_model_trials(herding_config: HerdingConfig, model_name: str, trials: int, seed_base: int) -> list:
    """하나의 회피 모델에 대해 `trials`번의 허딩 시뮬레이션을 실행하고 TrialResults를 반환한다."""
    results = []
    for i in range(trials):
        seed = seed_base + i
        results.append(run_trial(herding_config, _make_model(model_name, herding_config, seed),
                                 seed, SIM_CONFIG))
    return results


def summarize(results: list) -> dict:
    """TrialResults를 집계하여 ALGO-00x 지표로 만든다."""
    n = len(results)
    if n == 0:
        raise ValueError("summarize() needs at least one trial result")
    successes = [r for r in results if r.success]
    return {
        "trials": n,
        "success_rate": len(successes) / n,
        "mean_time_sec": float(np.mean([r.duration_sec for r in successes])) if successes else float("nan"),
        "panic_rate": float(np.mean([1 if r.panic_count > 0 else 0 for r in results])),
        "mean_role_swaps": float(np.mean([r.role_swap_count for r in results])),
        "max_role_swaps": max(r.role_swap_count for r in results),
        "mean_latency_ms": float(np.mean([r.mean_latency_ms for r in results])),
    }


def run_algo_suite(herding_config: HerdingConfig, trials: int = 100, seed_base: int = 0) -> dict:
    """모든 회피 모델의 시행을 실행하고 ALGO-001~008의 통과/실패 여부를 계산한다."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    model_results = {}
    for name in EVASION_MODEL_NAMES:
        results = run_model_trials(herding_config, name, trials, seed_base)
        model_results[name] = (results, summarize(results))

    _, primary_summary = model_results["reactive_flee"]
    algo_status = {
        "ALGO-001": primary_summary["success_rate"] >= ALGO_001_MIN_SUCCESS_RATE,
        "ALGO-002": primary_summary["mean_time_sec"] <= ALGO_002_MAX_MEAN_TIME_SEC,
        "ALGO-003": primary_summary["panic_rate"] <= ALGO_003_MAX_PANIC_RATE,
        "ALGO-004": primary_summary["max_role_swaps"] <= ALGO_004_MAX_ROLE_SWAPS,
        "ALGO-005": primary_summary["mean_latency_ms"] <= ALGO_005_MAX_LATENCY_MS,
    }

    # ALGO-006: occlusion 복구 -- 허딩 도중 강제로 센서 블랙아웃을 발생시키는 시행.
    occlusion_summary = _run_occlusion_recovery_check(
        herding_config, trials=max(trials // 3, 10), seed_base=seed_base + 10_000
    )
    algo_status["ALGO-006"] = occlusion_summary["passed"]
    # ALGO-007: 구조적으로 강제됨 -- 위의 모든 임계값은 herding_params.yaml에서 온다.
    algo_status["ALGO-007"] = True
    # ALGO-008: 대조 실험(idle / random / algorithm-on), 카이제곱 유의성 검정.
    control_summary = _run_control_experiment(herding_config, trials=trials, seed_base=seed_base + 20_000)
    algo_status["ALGO-008"] = (
        control_summary["difference_pp"] >= ALGO_008_MIN_DIFFERENCE_PP
        and control_summary["p_value"] is not None
        and control_summary["p_value"] < ALGO_008_MAX_P_VALUE
    )

    sensitivity = _run_sensitivity_sweep(
        herding_config, trials=max(trials // 10, 3), seed_base=seed_base + 30_000
    )

    _write_report(herding_config, model_results, algo_status, control_summary,
                  occlusion_summary, sensitivity)
    _write_plots(herding_config, model_results, sensitivity)
    return {
        "model_results": model_results, "algo_status": algo_status,
        "control_summary": control_summary, "occlusion_summary": occlusion_summary,
        "sensitivity": sensitivity,
    }


# ---------------------------------------------------------------------------- #
# ALGO-006: occlusion 복구                                                      #
# ---------------------------------------------------------------------------- #

def _target_visible(sim_time_sec: float, target_pos: np.ndarray, robot1_pos: np.ndarray,
                    robot2_pos: np.ndarray, blackout_start_sec: float, blackout_end_sec: float) -> bool:
    """ALGO-006 occlusion 모델 하에서 이번 주기에 타겟이 관측 가능하면 True를 반환한다."""
    if sim_time_sec < blackout_start_sec:
        return True  # occluder가 나타나기 전: 스위트의 나머지 부분과 동일한 전역 센서 사용
    if sim_time_sec < blackout_end_sec:
        return False  # 기하학적 배치와 무관하게 가려진 상태
    nearest = float(min(np.linalg.norm(target_pos - robot1_pos), np.linalg.norm(target_pos - robot2_pos)))
    return nearest <= OCCLUSION_SENSOR_RANGE_M


def _simulate_occlusion_episode(herding_config: HerdingConfig, seed: int) -> float | None:
    """블랙아웃 에피소드 하나를 실행한다. 재탐지까지 걸린 초를 반환하며, 재탐지되지 않으면 inf, 사용 불가능하면 None을 반환한다."""
    rng = np.random.default_rng(seed)
    core = HerdingCore(herding_config)
    core.grid_map.obstacle_mask = _boundary_ring_mask(herding_config)
    evasion_model = ReactiveFlee(SIM_CONFIG.target_max_speed_mps, herding_config.flee_reaction_distance_m)
    _bind_model_to_arena(evasion_model, core.grid_map)

    low, high = _arena_bounds(herding_config)
    margin = herding_config.grid_resolution_m * 2
    spawn_low, spawn_high = low + margin, high - margin
    target_state = np.array([
        rng.uniform(spawn_low[0], spawn_high[0]), rng.uniform(spawn_low[1], spawn_high[1]), 0.0, 0.0,
    ])
    # run_trial()과 동일한 근거: 로봇 1(Driver)은 표적을 방금 발견한 위치에서
    # 시작하고, 로봇 2(Blocker)는 고정된 대기 지점에서 시작한다. spawn_low/
    # spawn_high(마진 적용)로 클램프해서 경계 벽 셀 위에 스폰되지 않게 한다.
    robot1_pos = _spawn_driver_near_target(target_state[:2], rng, spawn_low, spawn_high, SIM_CONFIG)
    robot2_pos = np.array([spawn_high[0], spawn_low[1]])
    robot1_heading = np.array([1.0, 0.0])
    robot2_heading = np.array([1.0, 0.0])

    # 허딩이 실제로 진행 중일 때 블랙아웃을 시작하고, FSM이 실제로 LOST 상태에
    # 들어가 belief-grid 탐색을 수행하도록 occlusion_timeout_sec를 넘겨서 유지한다.
    blackout_start_sec = float(rng.uniform(5.0, 15.0))
    blackout_end_sec = blackout_start_sec + float(rng.uniform(
        herding_config.occlusion_timeout_sec + 0.5, herding_config.occlusion_timeout_sec + 3.0
    ))
    horizon_sec = min(blackout_end_sec + ALGO_006_MAX_RECOVERY_SEC + SIM_CONFIG.dt,
                      SIM_CONFIG.max_sim_time_sec)
    steps = max(int(round(horizon_sec / SIM_CONFIG.dt)), 0)

    for index in range(steps):
        sim_time_sec = index * SIM_CONFIG.dt
        visible = _target_visible(sim_time_sec, target_state[:2], robot1_pos, robot2_pos,
                                  blackout_start_sec, blackout_end_sec)
        output = core.step(Observation(
            target_measurement=target_state[:2].copy() if visible else None,
            robot1_pos=robot1_pos.copy(), robot2_pos=robot2_pos.copy(),
            robot1_heading=robot1_heading.copy(), robot2_heading=robot2_heading.copy(),
            occupancy=None, sim_time_sec=sim_time_sec, dt=SIM_CONFIG.dt,
        ))
        if output.fsm_state == FSMState.CAPTURED:
            # occluder가 영향을 주기 전에 이미 허딩이 끝남: 여기서는 측정할 복구가 없다.
            return None
        if sim_time_sec >= blackout_end_sec and visible:
            return sim_time_sec - blackout_end_sec

        new_r1 = _move_toward(robot1_pos, output.robot1_goal, SIM_CONFIG.robot_max_speed_mps,
                              SIM_CONFIG.robot_gain, SIM_CONFIG.dt)
        new_r2 = _move_toward(robot2_pos, output.robot2_goal, SIM_CONFIG.robot_max_speed_mps,
                              SIM_CONFIG.robot_gain, SIM_CONFIG.dt)
        new_r1 = _step_body(core, robot1_pos, new_r1, low, high)
        new_r2 = _step_body(core, robot2_pos, new_r2, low, high)
        robot1_heading = _update_heading(robot1_pos, new_r1, robot1_heading)
        robot2_heading = _update_heading(robot2_pos, new_r2, robot2_heading)
        robot1_pos, robot2_pos = new_r1, new_r2
        target_state = _advance_target(
            core, evasion_model, target_state, robot1_pos, robot2_pos, SIM_CONFIG, low, high
        )
    return float("inf")


def _run_occlusion_recovery_check(herding_config: HerdingConfig, trials: int, seed_base: int) -> dict:
    """ALGO-006: 강제 블랙아웃 에피소드 중 기한 내에 재탐지된 비율."""
    latencies = []
    for i in range(trials):
        latency = _simulate_occlusion_episode(herding_config, seed_base + i)
        if latency is not None:
            latencies.append(latency)
    episodes = len(latencies)
    recovered = [latency for latency in latencies if latency <= ALGO_006_MAX_RECOVERY_SEC]
    rate = len(recovered) / episodes if episodes else 0.0
    return {
        "attempted": trials,
        "episodes": episodes,
        "recovery_rate": rate,
        "mean_recovery_sec": float(np.mean(recovered)) if recovered else float("nan"),
        "passed": episodes > 0 and rate >= ALGO_006_MIN_RECOVERY_RATE,
    }


# ---------------------------------------------------------------------------- #
# ALGO-008: 대조 실험                                                           #
# ---------------------------------------------------------------------------- #

def _chi_square_p(contingency: np.ndarray) -> tuple[float | None, float | None]:
    """분할표(contingency table)에 대해 (p-value, 최소 기대 셀 값)을 반환한다; 퇴화된 경우 (None, None).

    p-value와 함께 최소 기대 빈도를 반환하는 이유는 그 값이 p-value를 신뢰할 수 있는지를
    말해주기 때문이다: 카이제곱은 점근적(asymptotic) 검정이며, 통상적인 경험칙은 모든
    기대 셀이 최소 5 이상이어야 한다는 것이다. 이 스위트가 실행하는 시행 횟수에서는
    표가 유의미하면서도 동시에 신뢰할 수 없는 경우가 쉽게 생길 수 있으므로, 호출자는
    p-value만 단독으로 보고하지 않고 두 값을 함께 보고한다.
    """
    table = np.asarray(contingency, dtype=float)
    # chi2_contingency는 행/열이 모두 0인 경우(기대 빈도 0) 예외를 발생시킨다.
    # 이는 N이 작을 때 쉽게 발생할 수 있다 -- 예를 들어 모든 조건이 모든 시행에서
    # 실패하는 경우 -- 그리고 이는 차이가 있다는 증거가 아니므로, "통과"가 아니라
    # "검정 불가"로 보고된다.
    if (table.sum(axis=0) == 0).any() or (table.sum(axis=1) == 0).any():
        return None, None
    _, p_value, _, expected = chi2_contingency(table)
    return float(p_value), float(np.min(expected))


def _run_control_experiment(herding_config: HerdingConfig, trials: int, seed_base: int) -> dict:
    """ALGO-008: 알고리즘 ON vs 로봇 idle vs 로봇 random을 카이제곱 검정으로 비교한다."""
    conditions = {"algorithm": [], "idle": [], "random": []}
    for mode in conditions:
        for i in range(trials):
            seed = seed_base + i  # 모드 간 동일한 시드 사용: 동일한 spawn, 쌍체(paired) 비교
            model = ReactiveFlee(SIM_CONFIG.target_max_speed_mps, herding_config.flee_reaction_distance_m)
            result = run_trial(herding_config, model, seed, SIM_CONFIG, control_mode=mode)
            conditions[mode].append(result.success)

    modes = list(conditions)
    success_counts = {mode: int(sum(conditions[mode])) for mode in modes}
    fail_counts = {mode: trials - success_counts[mode] for mode in modes}
    contingency = np.array([[success_counts[m], fail_counts[m]] for m in modes])
    p_value, min_expected = _chi_square_p(contingency)

    algo_rate = success_counts["algorithm"] / trials
    idle_rate = success_counts["idle"] / trials
    random_rate = success_counts["random"] / trials
    baseline_mode = "idle" if idle_rate >= random_rate else "random"
    baseline_rate = max(idle_rate, random_rate)
    # 진단 목적일 뿐: 스펙의 게이트는 위의 3x2 검정을 사용하여 세 조건이 조금이라도
    # 다른지를 묻지만, difference_pp는 알고리즘을 단일 최적 베이스라인과 비교한다.
    # 이 2x2 검정이 difference_pp와 일치하는 검정이다.
    pairwise_p, pairwise_min_expected = _chi_square_p(np.array([
        [success_counts["algorithm"], fail_counts["algorithm"]],
        [success_counts[baseline_mode], fail_counts[baseline_mode]],
    ]))
    return {
        "trials": trials, "algorithm_rate": algo_rate, "idle_rate": idle_rate, "random_rate": random_rate,
        "baseline_mode": baseline_mode,
        "difference_pp": (algo_rate - baseline_rate) * 100.0,
        "p_value": p_value, "min_expected": min_expected,
        "pairwise_p_value": pairwise_p, "pairwise_min_expected": pairwise_min_expected,
        "contingency": contingency,
    }


# ---------------------------------------------------------------------------- #
# 파라미터 민감도                                                                #
# ---------------------------------------------------------------------------- #

def _run_sensitivity_sweep(herding_config: HerdingConfig, trials: int, seed_base: int) -> dict:
    """Task 15의 튜닝을 위해, 단일 파라미터 스윕에 걸친 주 모델의 성공률.

    일부 스윕 지점은 의도적으로 HerdingConfig가 받아들이는 범위 밖에 위치한다: 이
    스윕은 HerdingConfig.__post_init__이 현재 거부하는, 데드락에 취약한 파라미터
    공간의 구석을 정확히 특성화하기 위해 존재한다 (예: drive_distance_m=1.05는
    출하 시 완화 계수(ease factor)를 적용하면 완화된 거리가 1.21 m가 되어, 타겟의
    1.0 m 반응 반경을 넘어선다 -- yaml 자체의 주석이 성공률이 2.5%로 붕괴한다고
    언급하는 바로 그 설정이다). dataclasses.replace()는 __post_init__을 실행하므로
    이런 지점은 ValueError를 발생시킨다; 이 경우 모든 시행 계산을 마친 뒤 전체
    검증 실행을 중단하는 대신, 해당 지점 하나만 거부(success_rate None)로 기록한다.
    """
    sweep = {}
    for offset, (param, values) in enumerate(SENSITIVITY_SWEEPS.items()):
        rates = []
        rejected = {}
        for value in values:
            try:
                variant = dataclasses.replace(herding_config, **{param: value})
            except ValueError as exc:
                # drive_distance_m뿐 아니라 스윕되는 어떤 파라미터도 설정 불변식에
                # 걸릴 수 있다 -- 그래서 이를 일반적으로 캐치하고 그 이유를
                # 보고서와 플롯까지 그대로 전달한다.
                rates.append(None)
                rejected[value] = str(exc)
                continue
            results = run_model_trials(variant, "reactive_flee", trials, seed_base + offset * 1_000)
            rates.append(summarize(results)["success_rate"])
        sweep[param] = {
            "values": list(values), "success_rates": rates, "rejected": rejected,
            "baseline": getattr(herding_config, param), "trials": trials,
        }
    return sweep


def _format_sensitivity_cells(data: dict) -> str:
    """스윕 지점들을 한 줄로 렌더링한다; 거부된 지점은 사라지지 않고 그 사실을 표시한다."""
    return "  ".join(
        f"{value:g}{'*' if value == data['baseline'] else ''}="
        + ("REJECTED" if rate is None else f"{rate*100:.0f}%")
        for value, rate in zip(data["values"], data["success_rates"])
    )


# ---------------------------------------------------------------------------- #
# 보고서 작성                                                                    #
# ---------------------------------------------------------------------------- #

def _spawned_inside_capture_zone(trial, herding_config: HerdingConfig) -> bool:
    """타겟이 포획 구역 안에서 스폰되어 시행에 허딩이 전혀 필요하지 않았다면 True를 반환한다.

    run_trial()은 타겟을 아레나 전체에 균일하게 스폰하며, 포획 구역도 포함된다. 이런
    시행은 로봇이 전혀 접근하지 않아도 capture_hold_sec 이후 성공으로 채점된다 --
    이는 버그가 아니라 스폰 정책의 실제 결과이지만, 알고리즘이 작동한다는 증거는
    아니다. 무엇을 성공으로 셀지 바꾸는 것은 Task 15의 결정 사항이므로, 대표
    success_rate는 의도적으로 이런 경우도 여전히 포함한다; 보고서는 그런 경우가
    몇 건이었는지만 명시하며, 궤적 그림은 그런 시행을 예시로 그리지 않는다.
    """
    if not len(trial.target_trajectory):
        return False
    goal = np.array([herding_config.capture_zone_x_m, herding_config.capture_zone_y_m])
    start = trial.target_trajectory[0]
    return float(np.linalg.norm(start - goal)) <= herding_config.capture_radius_m


CHI_SQUARE_MIN_EXPECTED = 5.0


def _format_p(p_value: float | None, min_expected: float | None = None) -> str:
    """신뢰성 단서와 함께 p-value를 렌더링한다. 퇴화된 경우 검정 불가로 표시한다."""
    if p_value is None:
        return "n/a (degenerate table)"
    # 이렇게 희소한 표에서 유의미해 보이는 p-value는 의미가 없으며, 이 파일의 숫자들은
    # 프로젝트의 공식 기록이다 -- 그래서 단서(caveat)는 다른 곳의 각주로 남기지 않고
    # 숫자와 함께 이동한다.
    if min_expected is not None and min_expected < CHI_SQUARE_MIN_EXPECTED:
        return (f"{p_value:.4f} (min expected cell {min_expected:.1f} -- unreliable below "
                f"{CHI_SQUARE_MIN_EXPECTED:.0f})")
    return f"{p_value:.4f}"


def _write_report(herding_config: HerdingConfig, model_results: dict, algo_status: dict,
                  control_summary: dict, occlusion_summary: dict, sensitivity: dict) -> None:
    """ALGO-001~008 보고서를 출력하고 test/output/validation_report.txt에 기록한다."""
    lines = []
    for name, (results, summary) in model_results.items():
        lines.append(f"=== Evasion Model: {name} ===")
        lines.append(
            f"  trials: {summary['trials']} | success: {summary['success_rate']*100:.1f}% | "
            f"mean time: {summary['mean_time_sec']:.1f} s | panic rate: {summary['panic_rate']*100:.1f}%"
        )
        lines.append(
            f"  role swaps/trial: {summary['mean_role_swaps']:.1f} (max {summary['max_role_swaps']}) | "
            f"mean latency: {summary['mean_latency_ms']:.1f} ms"
        )
        # 위의 성공률에는 이런 경우도 포함되어 있으므로, 발견하도록 놔두지 않고
        # 명시한다: 이것들은 알고리즘이 굳이 얻어낼 필요가 없었던 포획이다.
        successes = [r for r in results if r.success]
        trivial = sum(_spawned_inside_capture_zone(r, herding_config) for r in successes)
        lines.append(
            f"  of {len(successes)} successes, {trivial} spawned already inside the capture zone "
            "(counted in success rate; no herding required)"
        )
    lines.append("=== Model Comparison ===")
    for name, (_, summary) in model_results.items():
        tag = "   <- 실물 시연 예상치" if name == "noisy_human" else ("   <- 대조군" if name == "random_walk" else "")
        lines.append(f"  {name:13s}: {summary['success_rate']*100:5.1f}%{tag}")

    lines.append("=== Occlusion Recovery (ALGO-006) ===")
    lines.append(
        f"  episodes: {occlusion_summary['episodes']}/{occlusion_summary['attempted']} usable | "
        f"re-acquired <= {ALGO_006_MAX_RECOVERY_SEC:.0f}s: {occlusion_summary['recovery_rate']*100:.1f}% | "
        f"mean recovery: {occlusion_summary['mean_recovery_sec']:.2f} s"
    )
    lines.append(
        f"  (re-detection modelled at {OCCLUSION_SENSOR_RANGE_M:.1f} m robot-to-target range; "
        "harness assumption, not a spec parameter)"
    )

    lines.append("=== Control Experiment (ALGO-008) ===")
    lines.append(
        f"  algorithm ON : {control_summary['algorithm_rate']*100:.1f}%  |  "
        f"robots idle : {control_summary['idle_rate']*100:.1f}%  |  "
        f"robots random : {control_summary['random_rate']*100:.1f}%"
    )
    lines.append(
        # 소수점 0자리가 아니라 1자리: 게이트는 정확히 difference_pp >= 40.0이며,
        # FAIL 판정 옆에 반올림된 "+40 %p"가 찍히면 모순처럼 보인다.
        f"  difference   : {control_summary['difference_pp']:+.1f} %p vs best baseline "
        f"({control_summary['baseline_mode']})  |  chi-square p = "
        f"{_format_p(control_summary['p_value'], control_summary['min_expected'])}"
        f"  -> {'PASS' if algo_status['ALGO-008'] else 'FAIL'}"
    )
    lines.append(
        f"  (diagnostic) algorithm vs {control_summary['baseline_mode']} 2x2 chi-square p = "
        f"{_format_p(control_summary['pairwise_p_value'], control_summary['pairwise_min_expected'])}"
    )

    lines.append("=== Parameter Sensitivity ===")
    for param, data in sensitivity.items():
        lines.append(
            f"  {param} ({data['trials']} trials/point, * = baseline): "
            f"{_format_sensitivity_cells(data)}"
        )
        # 거부된 지점도 하나의 결과이지 빈틈이 아니다: 이는 그 설정이 HerdingConfig가
        # 받아들이는 범위 밖에 있다는 것을 말해주며, 이것이 바로 이 스윕의 목적이다.
        for value, reason in data.get("rejected", {}).items():
            lines.append(
                f"    {param}={value:g} REJECTED by config invariant, not run: {reason}"
            )

    lines.append("=== SUMMARY ===")
    lines.append(" / ".join(f"{k} {'PASS' if v else 'FAIL'}" for k, v in sorted(algo_status.items())))
    report = "\n".join(lines)
    print(report)
    with open(os.path.join(OUTPUT_DIR, "validation_report.txt"), "w") as f:
        f.write(report + "\n")


def _write_plots(herding_config: HerdingConfig, model_results: dict, sensitivity: dict) -> None:
    """궤적, 이스케이프 스냅샷, 민감도 그림을 test/output/에 기록한다."""
    primary_results, _ = model_results["reactive_flee"]
    goal = np.array([herding_config.capture_zone_x_m, herding_config.capture_zone_y_m])

    # "최고"가 아니라 "처음 발견한 것"을 사용한다: 이 그림은 실행 과정을 보여주는
    # 것이지 선별하는 것이 아니다. 유일한 예외는 이미 포획 구역 안에서 스폰된
    # 타겟이다 -- 이런 경우 로봇이 아무것도 하지 않아도 capture_hold_sec 이후
    # 성공으로 채점되므로, 그런 시행을 첫 번째로 고르면 빈 패널이 그려지고
    # 알고리즘에 대해 아무것도 보여주지 못한다. 이런 경우를 건너뛰는 것은 실제로
    # 허딩이 이루어진 시행을 고르는 것이지, 가장 좋은 시행을 고르는 것과는 다르다.
    successes = [r for r in primary_results if r.success]
    examples = (
        ("success", next((r for r in successes
                          if not _spawned_inside_capture_zone(r, herding_config)),
                         successes[0] if successes else None)),
        ("failure", next((r for r in primary_results if not r.success), None)),
    )

    # 결과마다 패널을 하나씩: 4초짜리 포획과 120초짜리 추격은 유용하게 공유할 스케일이
    # 없으며, 겹쳐 그리면 짧은 쪽이 완전히 가려진다.
    panel_labels_ko = {"success": "성공", "failure": "실패"}
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.5), squeeze=False)
    for ax, (label, trial) in zip(axes[0], examples):
        label_ko = panel_labels_ko[label]
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        if trial is None:
            ax.set_title(f"이번 실행에는 {label_ko} 사례 없음")
            continue
        ax.plot(*trial.target_trajectory.T, color="tab:red", label="표적")
        ax.plot(*trial.robot1_trajectory.T, "--", color="tab:blue", alpha=0.7, label="로봇1")
        ax.plot(*trial.robot2_trajectory.T, "--", color="tab:green", alpha=0.7, label="로봇2")
        for traj, color in ((trial.target_trajectory, "tab:red"),
                            (trial.robot1_trajectory, "tab:blue"),
                            (trial.robot2_trajectory, "tab:green")):
            if len(traj):
                ax.plot(traj[0, 0], traj[0, 1], "o", color=color, markersize=5)
        # 목표 지점이 없으면 이 플롯은 그저 세 개의 곡선일 뿐이다: 핵심은 타겟이
        # 어디로 밀려나고 있는지, 그리고 결국 포획 반경 안에 들어왔는지이다.
        ax.add_patch(plt.Circle(tuple(goal), herding_config.capture_radius_m,
                                color="tab:orange", alpha=0.25, label="포획 구역"))
        ax.plot(goal[0], goal[1], "*", color="tab:orange", markersize=12)
        ax.legend(fontsize=8)
        ax.set_title(f"{label_ko}: {trial.duration_sec:.1f}초, "
                     f"최근접 거리 {trial.min_robot_target_dist:.2f} m")
    fig.suptitle("표적 및 로봇 궤적 (reactive_flee, 동그라미 = 시작 지점)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, "trajectories.png"), dpi=120)
    plt.close(fig)

    snapshot_trial = next((r for r in primary_results if r.escape_snapshot is not None), None)
    if snapshot_trial is not None:
        fig, ax = plt.subplots(figsize=(5, 5))
        points = np.asarray(snapshot_trial.escape_snapshot, dtype=float).reshape(-1, 2)
        ax.scatter(points[:, 0], points[:, 1])
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title("도주 경로 스냅샷 (상위 K개)")
        fig.savefig(os.path.join(OUTPUT_DIR, "escape_heatmap_snapshot.png"), dpi=120)
        plt.close(fig)

    _write_sensitivity_plot(sensitivity)


def _write_sensitivity_plot(sensitivity: dict) -> None:
    """단일 파라미터 성공률 스윕들을 나란히 플롯한다.

    HerdingConfig 불변식에 의해 거부된 지점은 그냥 빠뜨리지 않고 명시적으로 라벨을
    붙인 높이 0의 빨간 막대로 그려서, 그림이 스윕 정의와 동일한 x축을 유지하고
    독자가 왜 거기에 숫자가 없는지 *알 수 있게* 한다.
    """
    params = list(sensitivity)
    fig, axes = plt.subplots(1, len(params), figsize=(5 * len(params), 4), squeeze=False)
    for ax, param in zip(axes[0], params):
        data = sensitivity[param]
        labels = [f"{value:g}" for value in data["values"]]
        heights = [0.0 if rate is None else rate * 100 for rate in data["success_rates"]]
        colors = []
        for value, rate in zip(data["values"], data["success_rates"]):
            if rate is None:
                colors.append("tab:red")
            elif value == data["baseline"]:
                colors.append("tab:orange")
            else:
                colors.append("tab:blue")
        ax.bar(labels, heights, color=colors)
        for index, rate in enumerate(data["success_rates"]):
            if rate is None:
                ax.text(index, 3, "설정 불변식에\n의해 거부됨", ha="center", va="bottom",
                        fontsize=7, color="tab:red", rotation=90)
        ax.set_title(f"{param}에 따른 성공률\n(지점당 {data['trials']}회 시행, 주황색 = 기준값)")
        ax.set_xlabel(param)
        ax.set_ylabel("성공률 [%]")
        ax.set_ylim(0, 100)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, "parameter_sensitivity.png"), dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    # 선택적인 argv[1]이 시행 횟수를 오버라이드하여, 스위트를 저렴하게 스모크 테스트할 수 있게 한다.
    # --abstract-only를 주면 추상 아레나(회귀용)만 돌리고 실제 맵 검증은 건너뛴다.
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    trial_count = int(args[0]) if args else 100
    herding_config = load_herding_config(CONFIG_PATH)
    run_algo_suite(herding_config, trials=trial_count)
    if "--abstract-only" not in sys.argv:
        run_real_map_algo_suite(herding_config, trials_per_trap=trial_count)
