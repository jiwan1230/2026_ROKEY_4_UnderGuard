"""실제 room_map(pgm+yaml) 위에서 로봇 A(Driver=미는 역할) + 로봇 B(Blocker=경로 차단) 몰이 실험.

사용자 정정: 로봇 A가 미는 역할(Driver), 로봇 B가 마르코프 도주 예측을 이용해 도주로를
차단하는 역할(Blocker)이다. 이는 기존 2로봇 알고리즘(herding_planner.py)의
compute_driving_point/compute_blocking_point와 정확히 같은 개념이므로, 커스텀 로직을
새로 만들지 않고 검증된 그 두 함수를 그대로 재사용한다.
"""
import base64
import io
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from herding_controller.grid_map import GridConfig, GridMap
from herding_controller.escape_model import EscapeModel, EscapeModelConfig
from herding_controller.herding_planner import PlannerConfig, compute_blocking_point, compute_driving_point
from herding_controller.target_estimator import EstimatorConfig, TargetEstimator
from test.evasion_models.noisy_human import NoisyHuman
from test.evasion_models.reactive_flee import ReactiveFlee
from test.run_validation import CONFIG_PATH, load_herding_config
from test.simulator import SimulatorConfig, _advance_target, _bind_model_to_arena, _step_body

from geodesic_field import GeodesicField

_SLIDE_ANGLES = np.linspace(0, 2 * np.pi, 16, endpoint=False)


def _step_body_sliding(core_like, position, proposed, low, high, avoid_point=None):
    """직선 이동이 벽에 막히면 완전히 멈추는 대신, 우회 방향을 찾아 이동한다.

    실제 room_map에는 지그재그로 꺾인 좁은 문턱이 있어서, 목표 지점까지의 직선
    경로가 막히면 로봇이 완전히 얼어붙는 게 실제로 확인됐다. 16방향을 시도해
    원래 의도한 방향과 각도 차이가 가장 작은, 실제로 갈 수 있는 방향을 고른다.
    `avoid_point`(두 스텝 전 위치)로 되돌아가는 후보는 제외해 진동을 막는다.
    """
    direct = _step_body(core_like, position, proposed, low, high)
    if not np.array_equal(direct, position):
        return direct

    step_len = float(np.linalg.norm(proposed - position))
    if step_len < 1e-9:
        return position.copy()

    original_angle = np.arctan2(proposed[1] - position[1], proposed[0] - position[0])
    ranked = sorted(_SLIDE_ANGLES, key=lambda a: abs(((a - original_angle) + np.pi) % (2 * np.pi) - np.pi))
    for angle in ranked:
        candidate = position + step_len * np.array([np.cos(angle), np.sin(angle)])
        moved = _step_body(core_like, position, candidate, low, high)
        if np.array_equal(moved, position):
            continue
        if avoid_point is not None and np.linalg.norm(moved - avoid_point) < step_len * 0.5:
            continue
        return moved
    return position.copy()


WALL_AVOID_RADIUS_M = 0.4   # 이 거리 안으로 들어오면 벽 반발력이 작용하기 시작
WALL_AVOID_MAX_WEIGHT = 0.85  # 벽에 거의 붙었을 때, 목표 방향 대비 반발력에 두는 최대 비중


def _wall_repulsion_direction(position, distance_field, grid_map):
    """벽까지의 거리장(distance field)의 기울기를 구해, 가장 가까운 벽에서
    멀어지는 단위벡터와 현재 벽까지의 거리(m)를 반환한다.

    A* 같은 진짜 경로 계획은 이 시뮬레이터 범위를 넘어서지만("완전한 경로
    계획까지는 범위 밖"), 로봇이 벽에 붙어서야 반응하는 지금 방식은 실제
    로봇 연동 전에 너무 부실하다는 지적이 있어 추가한 임시 조치다. 거리장의
    기울기는 항상 "가장 가까운 장애물로부터 멀어지는 방향"을 가리키므로,
    미리 계산해두면 스텝마다 4번의 배열 조회만으로 저렴하게 반발 방향을 얻는다.
    """
    row, col = grid_map.world_to_cell(*position)
    h, w = distance_field.shape
    r0, r1 = max(row - 1, 0), min(row + 1, h - 1)
    c0, c1 = max(col - 1, 0), min(col + 1, w - 1)
    grad_x = (distance_field[row, c1] - distance_field[row, c0]) / 2.0
    grad_y = (distance_field[r1, col] - distance_field[r0, col]) / 2.0
    grad = np.array([grad_x, grad_y])
    norm = np.linalg.norm(grad)
    clearance_m = float(distance_field[row, col]) * grid_map.config.resolution_m
    if norm < 1e-9:
        return np.zeros(2), clearance_m
    return grad / norm, clearance_m


def _move_with_wall_avoidance(position, goal, distance_field, grid_map, speed, dt):
    """목표 방향과 '가장 가까운 벽에서 멀어지는 방향'을 섞어서 한 걸음 이동한다.

    벽에서 WALL_AVOID_RADIUS_M보다 멀면 순수하게 목표 방향으로만 움직이고
    (기존과 동일), 가까워질수록 반발 방향의 비중이 선형으로 커진다. 이렇게
    하면 로봇이 벽에 딱 붙기 전에 미리 커브를 틀게 되어, "막힌 걸 감지하고
    나서야 옆으로 피하는" 기존 방식보다 훨씬 자연스럽게 벽을 피해간다.
    최종 충돌 방지는 여전히 _step_body_sliding이 담당한다(이건 그 앞단의
    예방 조치일 뿐, 안전망을 대체하지 않는다).
    """
    to_goal = goal - position
    dist_to_goal = float(np.linalg.norm(to_goal))
    attract = to_goal / dist_to_goal if dist_to_goal > 1e-9 else np.zeros(2)

    repel_dir, clearance_m = _wall_repulsion_direction(position, distance_field, grid_map)
    weight = WALL_AVOID_MAX_WEIGHT * max(0.0, min(1.0, (WALL_AVOID_RADIUS_M - clearance_m) / WALL_AVOID_RADIUS_M))
    blended = (1.0 - weight) * attract + weight * repel_dir
    blended_norm = np.linalg.norm(blended)
    direction = blended / blended_norm if blended_norm > 1e-9 else attract

    step = min(speed * dt, dist_to_goal) if dist_to_goal > 1e-9 else 0.0
    return position + direction * step


PGM_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "maps", "room_map.pgm")
RESOLUTION = 0.05
ORIGIN_X, ORIGIN_Y = -3.19, -9.03
WALL_DETECT_RADIUS_CELLS = 5   # 0.25m 상당 (기존 튜닝: 1셀 @ 0.25m/셀)
CAPTURE_RADIUS_M = 0.3         # 방이 작아서 기존 0.5m는 트랩끼리/벽과 너무 겹침
CAPTURE_HOLD_SEC = 3.0
BLOCK_LOOKAHEAD_M = 1.8        # 기존 3.0m는 10x10 아레나 기준; "아레나 짧은 변의 1/3" 지침을
                                # 따라 5.3m 짧은 변 기준으로 재조정

ROBOT_A_SPAWN = np.array([-1.06, -3.60])   # 순찰 -> 발견하면 Driver(미는 역할)
ROBOT_B_SPAWN = np.array([-2.81, -3.35])   # 발견 전까지 대기 -> 발견되면 출발해 Blocker(경로 차단)
TRAPS = {
    # 이전에 "top"/"bottom" 좌표가 서로 뒤바뀌어 있었다 (사진에서 픽셀 좌표를
    # 역산할 때 라벨을 잘못 매칭한 것으로 보임): "top"에 배정된 좌표가 실제로는
    # 회전된 지도의 아래쪽에, "bottom"이 위쪽에 렌더링되고 있었다. 장애물 벽
    # 정렬을 겹쳐그리기로 먼저 검증해 좌표계 자체는 정상임을 확인한 뒤, 두
    # 좌표를 맞바꿔서 라벨과 실제 화면 위치가 일치하도록 고쳤다.
    "top": np.array([-2.81, -5.36]),
    "left": np.array([-2.17, -2.21]),
    "bottom": np.array([1.74, -6.36]),
}

# 로봇 A가 방 전체를 훑도록 순찰하는 경유점 (자유공간 중 벽에서 0.35m 이상 떨어진
# 지점으로 조정한 좌표). 순서대로 돌다가 마지막 지점 다음엔 처음으로 돌아간다.
PATROL_WAYPOINTS = [
    np.array([-1.51, -3.0]),   # 위쪽 방
    np.array([-0.76, -5.0]),   # 통로
    np.array([-2.12, -6.95]),  # 아래쪽 방 왼쪽
    np.array([1.14, -6.9]),    # 아래쪽 방 오른쪽
    np.array([-0.92, -8.21]),  # 아래쪽 방 하단
]
PATROL_WAYPOINT_TOLERANCE_M = 0.3
SENSOR_RANGE_M = 1.5   # 로봇이 쥐를 "발견"했다고 판정하는 센서 반경 (스펙에 없는 실험 가정)

SIM_CONFIG = SimulatorConfig()


class _CoreLike:
    def __init__(self, grid_map):
        self.grid_map = grid_map


def load_room_obstacle_mask():
    with open(PGM_PATH, "rb") as f:
        assert f.readline().strip() == b"P5"
        dims = f.readline().split()
        w, h = int(dims[0]), int(dims[1])
        f.readline()  # maxval
        pix = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    free = pix == 254
    # pgm 관례: row0 = 이미지 맨 위 = 월드 y 최댓값. 우리 그리드 관례(row0=origin_y, 하단)에
    # 맞추려면 위아래로 뒤집어야 한다.
    obstacle_mask = np.flipud(~free)
    return obstacle_mask, pix, free


def photo_oriented_map_data_uri(free):
    """사용자가 보는 사진과 동일한 방향(가로로 90도 회전한 방향)으로 맵 PNG를 만든다.

    시뮬레이션 자체는 room_map.yaml이 정의한 진짜 월드 좌표(origin/resolution)로
    동작해야 하지만, 사용자가 보내준 참고 사진은 그 pgm을 90도 회전시켜 놓은
    모양이다("맵이 똑바르지 않다"는 지적이 바로 이 방향 불일치였다). 그래서
    렌더링에서만 동일하게 회전시키고, worldToCanvas()도 이 회전에 맞춰 좌표를
    변환한다 (아래 build_canvas_transform 참고).
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    obstacle = np.rot90(~free, k=1)  # photo와 동일한 방향
    h, w = obstacle.shape
    rgba = np.zeros((h, w, 4), dtype=np.uint8)
    rgba[obstacle] = [90, 70, 200, 235]
    fig = plt.figure(figsize=(w / 100, h / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis("off")
    ax.imshow(rgba, origin="upper", interpolation="nearest")
    buf = io.BytesIO()
    fig.savefig(buf, format="png", transparent=True)
    plt.close(fig)
    buf.seek(0)
    return "data:image/png;base64," + base64.b64encode(buf.read()).decode("ascii")


def make_target_model(name, herding_config, seed, grid_map):
    speed = SIM_CONFIG.target_max_speed_mps
    if name == "reactive_flee":
        return ReactiveFlee(speed, herding_config.flee_reaction_distance_m)
    if name == "noisy_human":
        rng = np.random.default_rng([seed, 777])
        return NoisyHuman(speed, herding_config.flee_reaction_distance_m, grid_map, rng=rng)
    raise ValueError(name)


def sample_free_spawn(grid_map, rng, min_clear_m=0.3, exclude_points=(), exclude_radius_m=0.5):
    from scipy import ndimage
    free = ~grid_map.obstacle_mask
    clearance_cells = ndimage.distance_transform_edt(free) * grid_map.config.resolution_m
    candidates = np.argwhere(clearance_cells >= min_clear_m)
    while True:
        row, col = candidates[rng.integers(0, len(candidates))]
        x, y = grid_map.cell_to_world(row, col)
        point = np.array([x, y])
        if all(np.linalg.norm(point - ep) >= exclude_radius_m for ep in exclude_points):
            return point


def nearest_trap(point):
    name = min(TRAPS, key=lambda k: np.linalg.norm(TRAPS[k] - point))
    return name, TRAPS[name]


def run_trial(herding_config, planner_config, grid_map, distance_field, target_model_name, seed, mouse_spawn,
              record_frames=True, blocker_active=True, use_geodesic=True):
    core_like = _CoreLike(grid_map)
    escape_model = EscapeModel(EscapeModelConfig(
        wall_follow_p=herding_config.markov_wall_follow_p, wall_hug_p=herding_config.markov_wall_hug_p,
        center_p=herding_config.markov_center_p, momentum_weight=herding_config.momentum_weight,
        robot_repulsion_weight=herding_config.robot_repulsion_weight,
        wall_detect_radius_cells=WALL_DETECT_RADIUS_CELLS,
        escape_route_top_k=herding_config.escape_route_top_k,
    ), grid_map)
    estimator = TargetEstimator(EstimatorConfig(
        process_noise=herding_config.kf_process_noise, measurement_noise=herding_config.kf_measurement_noise,
        occlusion_timeout_sec=herding_config.occlusion_timeout_sec,
    ))
    evasion_model = make_target_model(target_model_name, herding_config, seed, grid_map)
    _bind_model_to_arena(evasion_model, grid_map)

    low = np.array([grid_map.config.origin_x_m, grid_map.config.origin_y_m])
    high = np.array([
        grid_map.config.origin_x_m + grid_map.config.width_cells * grid_map.config.resolution_m,
        grid_map.config.origin_y_m + grid_map.config.height_cells * grid_map.config.resolution_m,
    ])

    driver_pos = ROBOT_A_SPAWN.copy()   # 순찰 중 -> 발견하면 Driver
    blocker_pos = ROBOT_B_SPAWN.copy()  # 발견 전까지 대기 -> 발견되면 출발
    prev_driver_pos = driver_pos.copy()
    prev_blocker_pos = blocker_pos.copy()
    target_state = np.array([mouse_spawn[0], mouse_spawn[1], 0.0, 0.0])

    dt = SIM_CONFIG.dt
    steps = int(round(SIM_CONFIG.max_sim_time_sec / dt))
    frames = []
    capture_timer = 0.0
    success = False

    # 발견 전: 로봇 A만 경유점을 순회하며 순찰하고, 로봇 B는 대기한다. 표적의
    # 진짜 위치는 아직 estimator에 넣지 않는다 -- 실제로는 로봇이 보지 못한
    # 관측을 KF에 넣을 수 없고, 어떤 쥐구멍이 목표인지도 발견 전엔 알 방법이
    # 없다(그래서 goal도 발견 시점에야 정해진다).
    discovered = False
    patrol_idx = 0
    goal_name, goal_pos = None, None
    geo_field = None
    discovery_time = None
    min_blocker_dist_after_discovery = float("inf")
    blocker_dist_at_capture = None
    escape_max_at_capture = None
    last_t = 0.0
    ever_in_radius = False
    ever_concentrated = False
    ever_both = False
    min_dist_to_goal_ever = float("inf")

    for i in range(steps):
        t = i * dt
        last_t = t

        if not discovered:
            dist_to_mouse = float(np.linalg.norm(target_state[:2] - driver_pos))
            if dist_to_mouse <= SENSOR_RANGE_M:
                discovered = True
                discovery_time = t
                estimator.update(target_state[:2].copy())
                goal_name, goal_pos = nearest_trap(target_state[:2])
                if use_geodesic:
                    # 목표는 발견 시점에 딱 한 번만 정해지므로, geodesic
                    # 필드도 여기서 한 번만 계산한다 (매 스텝 재계산 X --
                    # Dijkstra는 그리드 전체를 도는 계산이라 스텝마다 돌리기엔
                    # 비싸고, 애초에 목표(trap)가 바뀌지 않는 한 다시 계산할
                    # 이유가 없다).
                    goal_row, goal_col = grid_map.world_to_cell(*goal_pos)
                    geo_field = GeodesicField(grid_map, goal_row, goal_col)

        escape_estimate = None
        if discovered:
            estimator.predict(dt)
            estimator.update(target_state[:2].copy())
            est = estimator.get_state()

            # 벽을 고려한 "진짜 목표 방향"을 구해서, compute_driving_point/
            # compute_blocking_point에는 실제 트랩 좌표 대신 이 방향으로
            # 만든 가상의 근접 목표점을 goal_pos로 넘긴다. 두 함수는
            # normalize(target_pos - goal_pos)로 방향만 뽑아 쓰므로,
            # 좌표값 자체가 아니라 "그 방향이 벽을 피해 실제로 트랩과
            # 가까워지는 방향인가"만 맞으면 된다 (geodesic_field.py
            # virtual_goal_point 참고). geodesic 필드가 없거나(off-grid 등)
            # use_geodesic=False면 기존처럼 순수 직선 방향으로 폴백한다.
            direction_goal = goal_pos
            if geo_field is not None:
                virtual_goal = geo_field.virtual_goal_point(est.position)
                if virtual_goal is not None:
                    direction_goal = virtual_goal

            driving = compute_driving_point(est.position, est.velocity, direction_goal, driver_pos, planner_config)
            escape_estimate = escape_model.compute(est.position, est.velocity, [driver_pos, blocker_pos])
            blocking_point = compute_blocking_point(
                est.position, direction_goal, escape_estimate, grid_map, planner_config
            )
            driver_goal_point, driver_panic = driving.point, driving.is_panic
            # blocker_active=False는 "로봇 B가 아예 없거나 손 놓고 있으면
            # 어떻게 되는가"를 재는 소거(ablation) 실험용 스위치다. 정상
            # 운용에서는 항상 True.
            blocker_goal_point = blocking_point if blocker_active else ROBOT_B_SPAWN
        else:
            waypoint = PATROL_WAYPOINTS[patrol_idx]
            if np.linalg.norm(driver_pos - waypoint) <= PATROL_WAYPOINT_TOLERANCE_M:
                patrol_idx = (patrol_idx + 1) % len(PATROL_WAYPOINTS)
                waypoint = PATROL_WAYPOINTS[patrol_idx]
            driver_goal_point, driver_panic = waypoint, False
            blocker_goal_point = ROBOT_B_SPAWN  # 대기

        dist_to_goal = float(np.linalg.norm(target_state[:2] - goal_pos)) if discovered else float("inf")
        # 정식 알고리즘(state_machine.py의 HERD->CORNER 전이)과 동일하게,
        # "포획반경 안"이라는 위치 조건만으로는 부족하고 "도주 확률이 한
        # 방향으로 집중되어 있다"(escape_prob_concentrated)는 조건까지 함께
        # 요구한다. 이 게이트가 없으면 표적이 그저 우연히 트랩 근처를
        # 스쳐 지나가기만 해도 포획으로 잡히므로, Blocker(로봇 B)가 실제로
        # 도주로를 막았는지와 무관하게 "성공"이 나올 수 있다 -- 정확히
        # 사용자가 지적한 "쥐구멍 근처만 가도 성공" 문제의 원인이었다.
        escape_concentrated = bool(
            escape_estimate is not None
            and escape_estimate.probabilities.max() >= herding_config.escape_concentration_threshold
        )
        if discovered:
            min_dist_to_goal_ever = min(min_dist_to_goal_ever, dist_to_goal)
        in_radius = dist_to_goal <= CAPTURE_RADIUS_M
        if in_radius:
            ever_in_radius = True
        if escape_concentrated:
            ever_concentrated = True
        if in_radius and escape_concentrated:
            ever_both = True
            capture_timer += dt
        else:
            capture_timer = 0.0

        dist_driver = float(np.linalg.norm(target_state[:2] - driver_pos))
        dist_blocker = float(np.linalg.norm(target_state[:2] - blocker_pos))
        tick_min = min(dist_driver, dist_blocker)
        if discovered:
            min_blocker_dist_after_discovery = min(min_blocker_dist_after_discovery, dist_blocker)

        if not discovered:
            state = "SEARCH"
        elif dist_to_goal <= CAPTURE_RADIUS_M * 3:
            state = "CORNER"
        elif dist_driver > herding_config.flee_reaction_distance_m * 1.5:
            state = "TRACK"
        else:
            state = "HERD"

        if record_frames:
            frames.append({
                "t": round(t, 2),
                "target": [round(float(target_state[0]), 3), round(float(target_state[1]), 3)],
                "driver": [round(float(driver_pos[0]), 3), round(float(driver_pos[1]), 3)],
                "blocker": [round(float(blocker_pos[0]), 3), round(float(blocker_pos[1]), 3)],
                "driver_goal": [round(float(driver_goal_point[0]), 3), round(float(driver_goal_point[1]), 3)],
                "blocker_goal": [round(float(blocker_goal_point[0]), 3), round(float(blocker_goal_point[1]), 3)],
                "driver_panic": bool(driver_panic),
                "state": state,
                "discovered": discovered,
                "panic": bool(discovered and tick_min < herding_config.panic_distance_m),
                "dist": round(tick_min, 3),
                "capture_progress": round(min(capture_timer / CAPTURE_HOLD_SEC, 1.0), 3),
            })

        if capture_timer >= CAPTURE_HOLD_SEC:
            success = True
            blocker_dist_at_capture = dist_blocker
            escape_max_at_capture = float(escape_estimate.probabilities.max()) if escape_estimate is not None else None
            break

        speed = SIM_CONFIG.robot_max_speed_mps * SIM_CONFIG.robot_gain
        new_driver = _move_with_wall_avoidance(driver_pos, driver_goal_point, distance_field, grid_map, speed, dt)
        new_blocker = _move_with_wall_avoidance(blocker_pos, blocker_goal_point, distance_field, grid_map, speed, dt)
        next_driver = _step_body_sliding(core_like, driver_pos, new_driver, low, high, avoid_point=prev_driver_pos)
        next_blocker = _step_body_sliding(core_like, blocker_pos, new_blocker, low, high, avoid_point=prev_blocker_pos)
        prev_driver_pos, prev_blocker_pos = driver_pos, blocker_pos
        driver_pos, blocker_pos = next_driver, next_blocker

        target_state = _advance_target(core_like, evasion_model, target_state, driver_pos, blocker_pos,
                                       SIM_CONFIG, low, high)

    return {
        "model": target_model_name, "seed": seed, "success": success,
        "goal_name": goal_name, "mouse_spawn": mouse_spawn.tolist(),
        "discovery_time": discovery_time,
        "duration": (frames[-1]["t"] + dt) if frames else last_t + dt,
        "frames": frames,
        "min_blocker_dist_after_discovery": (
            None if min_blocker_dist_after_discovery == float("inf") else round(min_blocker_dist_after_discovery, 3)
        ),
        "blocker_dist_at_capture": None if blocker_dist_at_capture is None else round(blocker_dist_at_capture, 3),
        "escape_max_at_capture": escape_max_at_capture,
        "discovered": discovered,
        "ever_in_radius": ever_in_radius,
        "ever_concentrated": ever_concentrated,
        "ever_both": ever_both,
        "min_dist_to_goal_ever": None if min_dist_to_goal_ever == float("inf") else round(min_dist_to_goal_ever, 3),
    }


def main():
    herding_config = load_herding_config(CONFIG_PATH)
    obstacle_mask, pix, free = load_room_obstacle_mask()
    height_cells, width_cells = obstacle_mask.shape

    grid_map = GridMap(GridConfig(
        resolution_m=RESOLUTION, width_cells=width_cells, height_cells=height_cells,
        origin_x_m=ORIGIN_X, origin_y_m=ORIGIN_Y,
    ))
    grid_map.obstacle_mask = obstacle_mask

    from scipy import ndimage
    distance_field = ndimage.distance_transform_edt(~obstacle_mask)  # 셀 단위 (미터 아님)

    planner_config = PlannerConfig(
        drive_distance_m=herding_config.drive_distance_m, panic_distance_m=herding_config.panic_distance_m,
        alignment_threshold=herding_config.alignment_threshold,
        drive_distance_ease_factor=herding_config.drive_distance_ease_factor,
        block_lookahead_m=BLOCK_LOOKAHEAD_M,
    )

    rng = np.random.default_rng(0)
    trials = []
    seed = 0
    attempts = 0
    while len(trials) < 4 and attempts < 40:
        attempts += 1
        mouse_spawn = sample_free_spawn(
            grid_map, rng, min_clear_m=0.3,
            exclude_points=[ROBOT_A_SPAWN, ROBOT_B_SPAWN] + list(TRAPS.values()),
            exclude_radius_m=0.6,
        )
        model_name = "reactive_flee" if len(trials) % 2 == 0 else "noisy_human"
        trial = run_trial(herding_config, planner_config, grid_map, distance_field, model_name, seed, mouse_spawn)
        seed += 1
        trials.append(trial)
        print(model_name, "goal=", trial["goal_name"], "spawn=", mouse_spawn, "success=", trial["success"],
              "duration=", trial["duration"], "frames=", len(trial["frames"]))

    map_data_uri = photo_oriented_map_data_uri(free)
    y_max = ORIGIN_Y + height_cells * RESOLUTION
    x_max = ORIGIN_X + width_cells * RESOLUTION

    payload = {
        # 사진과 같은 방향의 캔버스: 가로축(canvas x) = world y (뒤집힘), 세로축(canvas y) = world x (뒤집힘)
        "photo_frame": {"y_low": ORIGIN_Y, "y_high": y_max, "x_low": ORIGIN_X, "x_high": x_max},
        "map_image": map_data_uri,
        "traps": {k: v.tolist() for k, v in TRAPS.items()},
        "capture_radius": CAPTURE_RADIUS_M,
        "panic_distance": herding_config.panic_distance_m,
        "sensor_range": SENSOR_RANGE_M,
        "trials": trials,
    }
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "real_map_frames.json")
    with open(out_path, "w") as f:
        json.dump(payload, f)
    print("bytes:", os.path.getsize(out_path))


if __name__ == "__main__":
    main()
