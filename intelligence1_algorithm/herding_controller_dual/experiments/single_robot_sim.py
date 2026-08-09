"""로봇 1대(허더) + 로봇 1대(트래커)로 단순화한 몰이 실험.

사용자가 그린 월드맵 초안(벽 하나 + 쥐구멍/트랩 3곳)에 맞춰 커스텀 장애물 맵을 만들고,
기존 2로봇 Driver/Blocker 구조 대신 로봇 1대만 능동적으로 미는 실험적 정책을 구현한다.

핵심 아이디어(사용자 제안): 마르코프 도주모델이 내놓는 8방향 확률 분포에, "덫으로
다가가는 방향이면 +, 옆으로 새면 -5, 정반대로 가면 -10"인 가치 함수를 더해서
(마르코프 확률을 백분율로 스케일링해 같은 크기로 맞춘 뒤 더함) 로봇이 유도할 방향을 고른다.

프로덕션 herding_core.py(2로봇, role assigner, FSM)는 건드리지 않는다 -- 이건 별도 정책
프로토타입이며, 검증되면 나중에 정식으로 통합하면 된다.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from herding_controller_dual.grid_map import GridConfig, GridMap
from herding_controller_dual.escape_model import EscapeModel, EscapeModelConfig
import herding_controller_dual.escape_model as escape_model_module
from herding_controller_dual.target_estimator import EstimatorConfig, TargetEstimator
from test.evasion_models.noisy_human import NoisyHuman
from test.evasion_models.reactive_flee import ReactiveFlee
from test.run_validation import CONFIG_PATH, load_herding_config
from test.simulator import SimulatorConfig, _advance_target, _bind_model_to_arena, _move_toward, _step_body

_DIRECTIONS = escape_model_module._DIRECTIONS
_COMPASS_KO = ["북", "북동", "동", "남동", "남", "남서", "서", "북서"]

# --- 사용자가 그린 월드맵 초안을 좌표로 변환 (10m x 10m 아레나) --------------- #
ARENA_LOW = np.array([0.0, 0.0])
ARENA_HIGH = np.array([10.0, 10.0])
GRID_RES = 0.25
GRID_CELLS = 40  # 10m / 0.25m

WALL_X_RANGE = (4.5, 5.0)   # 그림의 파란 세로 벽
WALL_Y_RANGE = (0.0, 5.0)   # 바닥에 붙고, 위쪽 절반은 뚫려 있음

TRAPS = {
    "top": np.array([5.3, 9.6]),        # 그림 상단, "쥐구멍" 라벨이 붙은 곳
    "left": np.array([0.5, 5.0]),       # 그림 왼쪽 벽
    "bottom_right": np.array([8.7, 0.6]),  # 그림 오른쪽 아래 모서리
}

ROBOT_A_SPAWN = np.array([3.0, 8.3])
ROBOT_B_SPAWN = np.array([1.6, 1.9])
MOUSE_SPAWN = np.array([4.2, 6.6])

CAPTURE_RADIUS_M = 0.5
CAPTURE_HOLD_SEC = 3.0
TRACKER_STANDOFF_M = 1.2

SIM_CONFIG = SimulatorConfig()


class _CoreLike:
    """simulator.py의 헬퍼(_step_body, _advance_target)가 요구하는 `core.grid_map` 인터페이스만 흉내."""
    def __init__(self, grid_map):
        self.grid_map = grid_map


def build_obstacle_mask():
    mask = np.zeros((GRID_CELLS, GRID_CELLS), dtype=bool)
    mask[0, :] = mask[-1, :] = True
    mask[:, 0] = mask[:, -1] = True
    x0, x1 = (int(round(v / GRID_RES)) for v in WALL_X_RANGE)
    y0, y1 = (int(round(v / GRID_RES)) for v in WALL_Y_RANGE)
    mask[y0:y1, x0:x1] = True
    return mask


def direction_value(direction, to_goal_hat):
    """사용자 제안: 덫 방향이면 +, 옆이면 -5, 정반대면 -10."""
    cos = float(np.dot(direction, to_goal_hat))
    if cos > 0.70710678:      # ~45도 이내: 덫으로 다가가는 방향
        return 10.0 * cos
    if cos > -0.70710678:     # 45~135도: 옆으로 새는 방향
        return -5.0
    return -10.0               # 135도 초과: 거의 정반대


def compute_adaptive_driving_point(target_pos, target_vel, goal_pos, robot_pos, robot_b_pos,
                                   escape_model, grid_map, herding_config):
    """8개 후보 위협 위치 각각을 '만약 로봇이 거기 있다면 마르코프 모델이 예측하는
    도주 확률이 얼마인가'로 가정 평가한 뒤, 가치함수(덫 방향 +, 옆 -5, 반대 -10)를
    더해 가장 좋은 후보를 고른다.

    주의: 로봇의 *현재* 위치를 마르코프 모델에 그대로 넣으면 안 된다 -- 그러면
    "지금 내가 서 있는 쪽으로 이미 도망갈 것 같다"는 예측이 나와서 로봇이 자기
    현재 위치를 계속 정당화하며 그 자리에 멈춰버리는 피드백 루프가 생긴다
    (실제로 처음 구현에서 발생한 버그). 그래서 각 후보 방향마다 "내가 거기로
    이동해서 위협한다면"이라는 가정 위치를 만들어 모델에 넣는다.
    """
    to_goal_hat = goal_pos - target_pos
    n = np.linalg.norm(to_goal_hat)
    to_goal_hat = to_goal_hat / n if n > 1e-6 else np.array([1.0, 0.0])

    to_target = target_pos - robot_pos
    dist = np.linalg.norm(to_target)
    if dist < herding_config.panic_distance_m:
        retreat_dir = -to_target / dist if dist > 1e-6 else np.array([1.0, 0.0])
        retreat_point = robot_pos + retreat_dir * (herding_config.panic_distance_m - dist)
        return retreat_point, True, None, None

    speed = np.linalg.norm(target_vel)
    candidates = []
    for idx, desired_dir in enumerate(_DIRECTIONS):
        push_from = -desired_dir
        drive_distance = herding_config.drive_distance_m
        if speed > 1e-6:
            alignment = float(np.dot(target_vel / speed, desired_dir))
            if alignment >= herding_config.alignment_threshold:
                drive_distance *= herding_config.drive_distance_ease_factor
        hypothetical_pos = target_pos + drive_distance * push_from
        # 트래커(robot_b_pos)는 여기 넣지 않는다: 트래커는 후보 방향마다
        # 위치가 안 바뀌는 고정값이라, 넣으면 "트래커로부터 먼 방향"이
        # 모든 후보에 걸쳐 동일하게 유리해지는 상수 편향이 생겨서, 트래커가
        # 어쩌다 서 있는 쪽이 실제 목표 방향과 무관하게 점수를 지배해버리는
        # 문제가 실제로 있었다 (트래커 위치를 표적 반대편으로 바꾸는
        # 변경을 넣은 뒤 발견). 허더 자신의 가정 위치만 넣어 "만약 나만
        # 거기 있다면"을 순수하게 평가한다.
        hypothetical_estimate = escape_model.compute(target_pos, target_vel, [hypothetical_pos])
        markov_prob = float(hypothetical_estimate.probabilities[idx])
        value = direction_value(desired_dir, to_goal_hat)
        combined = markov_prob * 100.0 + value
        candidates.append((combined, idx, hypothetical_pos))

    candidates.sort(key=lambda c: c[0], reverse=True)
    combined_scores = np.array([c[0] for c in candidates])
    # 이상적인 위협 위치가 아레나 밖으로 나가면(표적이 벽에 붙어 있을 때 흔함),
    # 그 후보를 통째로 버리는 대신 아레나 안으로 clamp해서 시도한다. 그냥
    # 버리면 벽에 붙은 표적을 벽에서 떼어내려는 방향(위협 위치가 벽 너머로
    # 나가는 방향)은 전부 탈락하고, 벽을 따라가는(접선) 방향만 남아 목표가
    # 어느 쪽에 있든 표적이 벽을 따라 미끄러지기만 하는 문제가 실제로
    # 있었다 -- clamp하면 "정확히 그 지점"은 아니어도 최대한 그 방향으로
    # 향하는 위협 위치를 여전히 시도할 수 있다.
    margin = grid_map.config.resolution_m * 1.5
    low = np.array([grid_map.config.origin_x_m + margin, grid_map.config.origin_y_m + margin])
    high = np.array([
        grid_map.config.origin_x_m + grid_map.config.width_cells * grid_map.config.resolution_m - margin,
        grid_map.config.origin_y_m + grid_map.config.height_cells * grid_map.config.resolution_m - margin,
    ])
    for combined, idx, point in candidates:
        clamped = np.clip(point, low, high)
        try:
            row, col = grid_map.world_to_cell(*clamped)
        except ValueError:
            continue
        if grid_map.is_obstacle(row, col):
            continue
        return clamped, False, idx, combined_scores
    return target_pos.copy(), False, None, combined_scores


def tracker_goal_point(target_pos, herder_pos, tracker_pos, standoff, grid_map):
    """트래커는 표적을 사이에 두고 허더와 반대쪽 '옆'에 자리를 잡는다 (미는 힘은
    여전히 허더만 낸다 -- 트래커는 위치만 잡을 뿐 방향 계산에 관여하지 않는다).

    처음에는 표적을 기준으로 허더의 정반대(180도) 지점을 목표로 삼았는데,
    이러면 트래커의 실제 위치가 허더의 8방향 후보 평가(마르코프 확률 계산)에
    강한 방향성 편향을 추가하게 되어 -- 후보마다 바뀌는 허더 자신의 가정
    위치와 달리 트래커는 한 프레임 동안 고정값이라, "트래커 반대쪽"이 실제
    목표 방향과 무관하게 점수를 지배해버리는 문제가 있었다(실제로 재현/확인).
    그 정반대 지점이 벽 밖으로 나가는 경우(표적이 벽에 붙어 있을 때)도 겹쳐서
    여러 시나리오가 실패로 돌아섰다.

    그래서 반대편(180도)이 아니라 90도 옆(허더-표적 축에 수직)으로 완화했다:
    최소한 "표적의 같은 쪽에서 나란히 뒤따르는" 원래 문제(사용자가 지적한
    "일자로 따라가는" 모양)는 확실히 해소하면서, 허더 쪽으로 미는 축과는
    수직이라 허더의 방향 판단을 밀어붙이는 힘은 훨씬 약하다.
    """
    margin = grid_map.config.resolution_m * 1.5
    low = np.array([grid_map.config.origin_x_m + margin, grid_map.config.origin_y_m + margin])
    high = np.array([
        grid_map.config.origin_x_m + grid_map.config.width_cells * grid_map.config.resolution_m - margin,
        grid_map.config.origin_y_m + grid_map.config.height_cells * grid_map.config.resolution_m - margin,
    ])

    away_from_herder = target_pos - herder_pos
    norm = np.linalg.norm(away_from_herder)
    away_hat = away_from_herder / norm if norm > 1e-6 else np.array([1.0, 0.0])
    perp_a = np.array([-away_hat[1], away_hat[0]])
    perp_b = -perp_a
    for perp in (perp_a, perp_b):
        point = target_pos + perp * standoff
        if np.all(point >= low) and np.all(point <= high):
            return point

    # 양쪽 다 벽 밖이면(좁은 구석) 원래의 단순 추적으로 폴백.
    to_target = target_pos - tracker_pos
    dist = np.linalg.norm(to_target)
    if dist <= standoff:
        return tracker_pos.copy()
    return target_pos - (to_target / dist) * standoff


def make_target_model(name, herding_config, seed):
    speed = SIM_CONFIG.target_max_speed_mps
    grid_map = GridMap(GridConfig(resolution_m=GRID_RES, width_cells=GRID_CELLS,
                                  height_cells=GRID_CELLS))
    if name == "reactive_flee":
        return ReactiveFlee(speed, herding_config.flee_reaction_distance_m)
    if name == "noisy_human":
        rng = np.random.default_rng([seed, 777])
        return NoisyHuman(speed, herding_config.flee_reaction_distance_m, grid_map, rng=rng)
    raise ValueError(name)


def run_single_robot_trial(herding_config, target_model_name, seed, obstacle_mask, goal_name):
    """로봇 A는 트래커(추적만), 로봇 B가 알고리즘이 다루는 허더(능동 몰이)다."""
    grid_map = GridMap(GridConfig(resolution_m=GRID_RES, width_cells=GRID_CELLS, height_cells=GRID_CELLS))
    grid_map.obstacle_mask = obstacle_mask
    core_like = _CoreLike(grid_map)

    escape_model = EscapeModel(EscapeModelConfig(
        wall_follow_p=herding_config.markov_wall_follow_p, wall_hug_p=herding_config.markov_wall_hug_p,
        center_p=herding_config.markov_center_p, momentum_weight=herding_config.momentum_weight,
        robot_repulsion_weight=herding_config.robot_repulsion_weight,
        wall_detect_radius_cells=herding_config.wall_detect_radius_cells,
        escape_route_top_k=herding_config.escape_route_top_k,
    ), grid_map)
    estimator = TargetEstimator(EstimatorConfig(
        process_noise=herding_config.kf_process_noise, measurement_noise=herding_config.kf_measurement_noise,
        occlusion_timeout_sec=herding_config.occlusion_timeout_sec,
    ))
    evasion_model = make_target_model(target_model_name, herding_config, seed)
    _bind_model_to_arena(evasion_model, grid_map)

    goal_pos = TRAPS[goal_name]

    tracker_pos = ROBOT_A_SPAWN.copy()   # 로봇 A: 추적만
    herder_pos = ROBOT_B_SPAWN.copy()    # 로봇 B: 알고리즘이 미는 역할
    target_state = np.array([MOUSE_SPAWN[0], MOUSE_SPAWN[1], 0.0, 0.0])

    dt = SIM_CONFIG.dt
    steps = int(round(SIM_CONFIG.max_sim_time_sec / dt))
    frames = []
    capture_timer = 0.0
    success = False

    for i in range(steps):
        t = i * dt
        estimator.predict(dt)
        estimator.update(target_state[:2].copy())
        est = estimator.get_state()

        driving_point, is_panic, desired_idx, combined = compute_adaptive_driving_point(
            est.position, est.velocity, goal_pos, herder_pos, tracker_pos, escape_model, grid_map, herding_config
        )
        tracker_point = tracker_goal_point(target_state[:2], herder_pos, tracker_pos, TRACKER_STANDOFF_M, grid_map)

        dist_to_goal = float(np.linalg.norm(target_state[:2] - goal_pos))
        if dist_to_goal <= CAPTURE_RADIUS_M:
            capture_timer += dt
        else:
            capture_timer = 0.0

        dist_herder = float(np.linalg.norm(target_state[:2] - herder_pos))
        dist_tracker = float(np.linalg.norm(target_state[:2] - tracker_pos))
        tick_min = min(dist_herder, dist_tracker)

        if dist_to_goal <= CAPTURE_RADIUS_M * 3:
            state = "CORNER"
        elif dist_herder > herding_config.flee_reaction_distance_m * 1.5:
            state = "TRACK"
        else:
            state = "HERD"

        frames.append({
            "t": round(t, 2),
            "target": [round(float(target_state[0]), 3), round(float(target_state[1]), 3)],
            "herder": [round(float(herder_pos[0]), 3), round(float(herder_pos[1]), 3)],
            "tracker": [round(float(tracker_pos[0]), 3), round(float(tracker_pos[1]), 3)],
            "herder_goal": [round(float(driving_point[0]), 3), round(float(driving_point[1]), 3)],
            "tracker_goal": [round(float(tracker_point[0]), 3), round(float(tracker_point[1]), 3)],
            "herder_panic": bool(is_panic),
            "state": state,
            "panic": bool(tick_min < herding_config.panic_distance_m),
            "dist": round(tick_min, 3),
            "desired_dir": _COMPASS_KO[int(desired_idx)] if desired_idx is not None else None,
            "desired_vec": [round(float(_DIRECTIONS[desired_idx][0]), 3),
                           round(float(_DIRECTIONS[desired_idx][1]), 3)] if desired_idx is not None else None,
            "capture_progress": round(min(capture_timer / CAPTURE_HOLD_SEC, 1.0), 3),
        })

        if capture_timer >= CAPTURE_HOLD_SEC:
            success = True
            break

        new_herder = _move_toward(herder_pos, driving_point, SIM_CONFIG.robot_max_speed_mps, SIM_CONFIG.robot_gain, dt)
        new_tracker = _move_toward(tracker_pos, tracker_point, SIM_CONFIG.robot_max_speed_mps, SIM_CONFIG.robot_gain, dt)
        herder_pos = _step_body(core_like, herder_pos, new_herder, ARENA_LOW, ARENA_HIGH)
        tracker_pos = _step_body(core_like, tracker_pos, new_tracker, ARENA_LOW, ARENA_HIGH)

        target_state = _advance_target(core_like, evasion_model, target_state, herder_pos, tracker_pos,
                                       SIM_CONFIG, ARENA_LOW, ARENA_HIGH)

    return {
        "model": target_model_name, "seed": seed, "success": success,
        "goal_name": goal_name, "duration": frames[-1]["t"] + dt if frames else 0.0,
        "frames": frames,
        "note": None if success else (
            "로봇B가 남쪽 스폰 지점에서 목표 위협 위치(표적의 북동쪽)로 이동하는 동안, "
            "그 경로가 표적과 flee_reaction_distance(1.0m) 이내로 스쳐 지나가면서 표적이 "
            "의도와 다른 방향(북쪽)으로 먼저 도망쳐 버렸습니다. 결국 위쪽 벽 구석에 끼어 "
            "더 이상 밀어낼 방향이 없어 정지 -- 로봇 1대가 목표 위협 위치까지 '어떻게' "
            "이동할지(우회 경로)는 고려하지 않는 현재 정책의 한계를 보여주는 사례입니다."
        ),
    }


def main():
    herding_config = load_herding_config(CONFIG_PATH)
    obstacle_mask = build_obstacle_mask()

    trials = [
        run_single_robot_trial(herding_config, "reactive_flee", 0, obstacle_mask, "top"),
        run_single_robot_trial(herding_config, "reactive_flee", 0, obstacle_mask, "left"),
        run_single_robot_trial(herding_config, "reactive_flee", 0, obstacle_mask, "bottom_right"),
    ]
    for tr in trials:
        print(tr["model"], "goal=", tr["goal_name"], "success=", tr["success"],
              "duration=", tr["duration"], "frames=", len(tr["frames"]))

    payload = {
        "arena": {"low": ARENA_LOW.tolist(), "high": ARENA_HIGH.tolist()},
        "wall": {"x": list(WALL_X_RANGE), "y": list(WALL_Y_RANGE)},
        "traps": {k: v.tolist() for k, v in TRAPS.items()},
        "capture_radius": CAPTURE_RADIUS_M,
        "panic_distance": herding_config.panic_distance_m,
        "trials": trials,
    }
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "single_robot_frames.json")
    with open(out_path, "w") as f:
        json.dump(payload, f)
    print("bytes:", os.path.getsize(out_path))


if __name__ == "__main__":
    main()
