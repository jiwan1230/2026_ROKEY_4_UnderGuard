# herding_controller_dual/test/real_map_arena.py
"""실제 SLAM 맵(room_map.pgm) 기반 검증 아레나 — 정식 ALGO-001~008 검증용.

2026-08-06 이전에는 `test/simulator.py`가 벽 하나 없는 단순한 정사각형
아레나(경계 링만 장애물)에서 검증을 돌렸다. 실제 배포 환경은 이 정사각형이
아니라 `herding_controller/maps/room_map.pgm`로 이미 SLAM된 실제 방이고,
"로봇 A가 순찰하다가 표적을 발견하면 그 순간 Driver로 배정된다"는 시나리오도
정식 운용의 일부라는 걸 확인한 뒤(트러블슈팅 노트 10번 항목), "정식 검증"
자체를 이 실제 맵 기준으로 다시 세웠다.

이 모듈은 그 재구축에 필요한 것들을 모은다: 실제 맵 로딩, 순찰 경유점,
포획구역(트랩) 후보, 그리고 실제 맵의 좁은 문턱에서 로봇이 얼어붙지 않게
하는 벽 회피 이동 헬퍼. 원래 `herding_controller/experiments/real_map_sim.py`
(비검증 프로토타입)에서 개발/검증된 로직을 정식 검증 하네스로 옮겨온 것이다.
"""
import os

import numpy as np
from scipy import ndimage

from herding_controller_dual.grid_map import GridConfig, GridMap

MAPS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "maps")
PGM_PATH = os.path.join(MAPS_DIR, "room_map.pgm")

# room_map.yaml에서 그대로 가져온 값 (SLAM 산출물, config/herding_params.yaml과
# 반드시 일치해야 한다 -- 실제 로봇에서는 herding_node.py가 /map 메시지의
# resolution/origin이 이 값과 다르면 경고를 낸다).
#
# 2026-08-09: 로봇 파트가 방을 다시 SLAM 했다(main 병합). 맵이 106x147
# origin(-3.19,-9.03) -> 109x149 origin(-2.68,-6.08)으로 바뀌면서 월드 좌표계가
# 통째로 이동했다. 아래 덫/스폰/순찰 좌표는 전부 새 맵 기준으로 다시 잡은 것이며,
# 옛 좌표는 새 맵에서 방 밖이라 그대로 쓰면 로봇이 엉뚱한 곳으로 간다.
RESOLUTION_M = 0.05
ORIGIN_X_M, ORIGIN_Y_M = -2.68, -6.08

# 로봇 스폰/대기 위치. ROBOT_B_SPAWN은 충전소 같은 고정 대기 지점(로봇
# 2/Blocker), ROBOT_A_SPAWN은 순찰을 시작하는 초기 위치(로봇 1/Driver-가
# 될 로봇 -- 발견 전까지는 아직 "Driver"가 아니라 그냥 순찰 중인 로봇이다).
#
# ROBOT_A_SPAWN은 로봇 파트 patrol_waypoints.yaml의 첫 경유점을 그대로 쓴다
# (실제로 순찰이 거기서 시작하므로).
ROBOT_A_SPAWN = np.array([-2.00, 0.74])
# 대기 지점은 벽 여유(ROBOT_BODY_CLEARANCE_M)를 만족하면서 순찰 경로에서
# 가장 멀리 떨어진 하단부 지점으로 골랐다 -- 충전소 성격이라 위치 자체에
# 의미는 없고, 순찰을 방해하지 않는 것만 중요하다.
ROBOT_B_SPAWN = np.array([0.62, -5.58])

# --- 물체의 물리적 크기 (2026-08-08 추가) -------------------------------- #
# 그 전까지 로봇과 표적을 모두 "점"으로 취급했다. 두 로봇이 통로를 몸으로
# 막을 수 있는지(봉인)를 판정하려면 실제 크기가 반드시 필요하다.
#
# TurtleBot 4 실측 스펙: 342 x 339 x 351 mm -> 반지름 0.171m.
# 2026-08-09: 로봇 파트 config/nav2.yaml의 robot_radius가 0.175라 그 값에 맞춘다.
# Nav2가 치명 영역을 0.175로 잡으므로, 우리가 그보다 작게 보면 Nav2가 거부할
# 목표점을 유효하다고 판정하게 된다. 참고로 nav2.yaml의 inflation_radius는
# 0.25로 우리 여유(0.205)보다 크다 -- 목표가 거부되진 않지만 비용이 높은
# 자리이므로, 실기에서 접근이 나쁘면 아래 벽 여유를 0.075로 올려 0.25에 맞춘다.
ROBOT_RADIUS_M = 0.175
# 벽에 이 정도만 여유를 두면 된다고 확인받은 값 (팀 확인, 2026-08-08).
ROBOT_WALL_CLEARANCE_M = 0.03
# 로봇 중심이 벽에서 최소한 떨어져 있어야 하는 거리.
ROBOT_BODY_CLEARANCE_M = ROBOT_RADIUS_M + ROBOT_WALL_CLEARANCE_M
# RC카(표적) 실측: 세로 18cm x 가로 9cm. 어느 방향으로 놓이든 안전하도록
# 외접원 반지름(대각선의 절반)을 쓴다: sqrt(0.09^2 + 0.045^2) ~= 0.101m.
TARGET_RADIUS_M = 0.101

# 포획구역(트랩) 후보 3곳 -- 2026-08-09 재-SLAM 맵 기준으로 다시 선정했다.
#
# 선정 기준 두 가지가 서로 상충한다:
#   (a) 포위도 -- 몰아넣었을 때 빠져나가기 어려운 곳(구석)일수록 몰이가 쉽다.
#   (b) 협공 성립률 -- 덫 주변 0.35~0.8m 링에 표적을 두고 compute_endgame_pincer가
#       해를 주는 비율. 구석일수록 로봇이 좌우로 벌릴 자리가 없어 오히려 낮아진다.
# 완전한 구석((+2.37,-5.43): 포위도 0.88 / 협공 28.6%)은 협공이 안 서고,
# 트인 곳((-1.58,-2.58): 포위도 0.31 / 협공 66%)은 몰이가 안 된다. 아래 셋은
# 둘 다 중간 이상이면서 방 세 구역(상단 팔 / 좌측 벽 / 하단 우측)에 흩어지도록
# 고른 것이다. 방이 L자라 세 곳의 성격이 실제로 다르다.
TRAPS = {
    # 상단 팔 끝. 포위도 0.69 / 협공 52.0% / 벽까지 0.35m
    "top": np.array([-0.23, 0.72]),
    # 좌측 벽 오목한 곳. 포위도 0.75 / 협공 39.6% / 벽까지 0.15m
    "left": np.array([-2.28, -1.48]),
    # 하단 우측. 포위도 0.75 / 협공 64.7% / 벽까지 0.15m
    "bottom": np.array([1.57, -4.38]),
}

# 로봇 A가 순찰하며 방 전체를 훑는 경유점. 마지막 지점 다음엔 처음으로 순환.
# 2026-08-09: 로봇 파트 src/turtle_project/resource/patrol_waypoints.yaml을
# 그대로 옮겼다(36점). 전에는 우리가 임의로 잡은 5점이었는데, 실제 순찰 경로를
# 쓰는 편이 "표적을 언제 발견하는가"를 현실에 맞게 만든다.
PATROL_WAYPOINTS = [
    np.array([-2.00, 0.74]),
    np.array([-0.15, 0.74]),
    np.array([-0.15, 0.34]),
    np.array([-2.00, 0.34]),
    np.array([-2.06, -0.06]),
    np.array([-0.10, -0.06]),
    np.array([-0.10, -0.46]),
    np.array([-2.10, -0.46]),
    np.array([-1.60, -0.85]),
    np.array([-0.10, -0.85]),
    np.array([-0.10, -1.25]),
    np.array([-1.25, -1.25]),
    np.array([-2.10, -1.66]),
    np.array([-0.10, -1.66]),
    np.array([-0.10, -2.06]),
    np.array([-2.06, -2.06]),
    np.array([-2.06, -2.46]),
    np.array([-0.06, -2.46]),
    np.array([2.29, -2.85]),
    np.array([0.84, -2.85]),
    np.array([0.20, -2.85]),
    np.array([-2.00, -2.85]),
    np.array([-2.00, -3.25]),
    np.array([2.25, -3.25]),
    np.array([2.25, -3.65]),
    np.array([-2.00, -3.65]),
    np.array([-1.55, -4.05]),
    np.array([0.84, -4.05]),
    np.array([1.65, -4.05]),
    np.array([2.25, -4.05]),
    np.array([0.69, -4.46]),
    np.array([-1.16, -4.46]),
    np.array([-0.81, -4.86]),
    np.array([1.50, -4.86]),
    np.array([2.29, -5.25]),
    np.array([-0.46, -5.25]),
]
PATROL_WAYPOINT_TOLERANCE_M = 0.3

# 로봇이 순찰 중 표적을 "발견"했다고 판정하는 센서 반경. 스펙에 없는 검증
# 하네스 가정값 -- 실제로는 Detection 파트의 인식 범위에 대응한다.
SENSOR_RANGE_M = 1.5

# 벽 회피(potential field) 파라미터. 완전한 경로 계획(A*)은 이 검증 하네스의
# 범위를 넘어선다 -- Nav2가 실제 배포에서 담당할 부분을, 시뮬레이터 안에서는
# 최소한의 근사로 대신한다.
WALL_AVOID_RADIUS_M = 0.4
WALL_AVOID_MAX_WEIGHT = 0.85

_SLIDE_ANGLES = np.linspace(0, 2 * np.pi, 16, endpoint=False)


def load_room_obstacle_mask():
    """room_map.pgm을 읽어 grid_map 규약(row0=origin_y, 하단)에 맞는 장애물 마스크를 반환한다."""
    with open(PGM_PATH, "rb") as f:
        assert f.readline().strip() == b"P5"
        dims = f.readline().split()
        w, h = int(dims[0]), int(dims[1])
        f.readline()  # maxval
        pix = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    free = pix == 254
    # pgm 관례: row0 = 이미지 맨 위 = 월드 y 최댓값. grid_map 규약(row0=origin_y,
    # 하단)에 맞추려면 위아래로 뒤집어야 한다.
    return np.flipud(~free)


def build_grid_map(obstacle_mask: np.ndarray) -> GridMap:
    height_cells, width_cells = obstacle_mask.shape
    grid_map = GridMap(GridConfig(
        resolution_m=RESOLUTION_M, width_cells=width_cells, height_cells=height_cells,
        origin_x_m=ORIGIN_X_M, origin_y_m=ORIGIN_Y_M,
    ))
    grid_map.obstacle_mask = obstacle_mask
    return grid_map


def build_distance_field(obstacle_mask: np.ndarray) -> np.ndarray:
    """가장 가까운 벽까지의 거리장(셀 단위, 미터 아님) -- 벽 회피용, 1회만 계산."""
    return ndimage.distance_transform_edt(~obstacle_mask)


def sample_free_spawn(grid_map, rng, min_clear_m=0.3, exclude_points=(), exclude_radius_m=0.5):
    """벽에서 min_clear_m 이상 떨어진 자유공간 중 무작위 지점 (exclude_points 근처는 제외)."""
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


def _wall_repulsion_direction(position, distance_field, grid_map, radius_cells=6):
    """가장 가까운 벽에서 멀어지는 단위벡터와 현재 벽까지의 거리(m)를 반환한다.

    `radius_cells`(기본 6칸=0.3m)만큼 떨어진 이웃으로 중심차분을 구한다.
    바로 옆 1칸만 보면(예전 방식), 두 벽이 가까이 마주친 좁은 구석에서는
    "가장 가까운 벽"이 로봇이 1cm만 움직여도 반대쪽 벽으로 뒤집힐 수 있어
    반발 방향이 매 제어 주기 거의 정반대로 진동한다 -- 실측: 이 좁은
    구석에서 로봇이 3cm 박스 안을 벗어나지 못하고 제자리걸음만 함
    (트러블슈팅 노트 10-6 항목). geodesic_field.py의
    `gradient_toward_goal()`에서 똑같은 문제를 똑같은 방법(더 넓게 평균)으로
    이미 고친 적이 있다 -- 여기서도 동일한 해법을 적용한다.
    """
    row, col = grid_map.world_to_cell(*position)
    h, w = distance_field.shape
    r0, r1 = max(row - radius_cells, 0), min(row + radius_cells, h - 1)
    c0, c1 = max(col - radius_cells, 0), min(col + radius_cells, w - 1)
    grad_x = (distance_field[row, c1] - distance_field[row, c0]) / 2.0
    grad_y = (distance_field[r1, col] - distance_field[r0, col]) / 2.0
    grad = np.array([grad_x, grad_y])
    norm = np.linalg.norm(grad)
    clearance_m = float(distance_field[row, col]) * grid_map.config.resolution_m
    if norm < 1e-9:
        return np.zeros(2), clearance_m
    return grad / norm, clearance_m


def clearance_field_m(obstacle_mask: np.ndarray, resolution_m: float = RESOLUTION_M) -> np.ndarray:
    """각 셀에서 가장 가까운 장애물까지의 거리(미터). 물체 반지름 충돌 판정용.

    `build_distance_field()`는 셀 단위를 반환하는 벽 회피 전용이라 그대로
    쓸 수 없다 -- 여기서는 미터 단위여야 반지름(m)과 직접 비교할 수 있다.
    """
    return ndimage.distance_transform_edt(~obstacle_mask) * resolution_m


def _step_body(grid_map, position, proposed, low, high, body_radius_m=0.0, clearance_m=None):
    """물체를 proposed 위치로 이동시킨다. 아레나 안으로 클램프되고 벽에 막힌다.

    `body_radius_m`이 0이면 예전처럼 물체를 점으로 취급한다(하위호환 -- 추상
    아레나 검증 등 크기가 의미 없는 곳은 그대로 둔다). 0보다 크면 "중심이
    벽에서 body_radius_m 이상 떨어져 있어야 한다"로 판정하므로, 몸체가 벽을
    파고드는 위치는 거부된다. `clearance_m`(미터 단위 거리장)은
    `clearance_field_m()`으로 시행 시작 시 1회만 계산해서 넘긴다.
    """
    moved = np.clip(np.asarray(proposed, dtype=float), low, high - 1e-9)
    row, col = grid_map.world_to_cell(*moved)
    if grid_map.obstacle_mask[row, col]:
        return np.asarray(position, dtype=float).copy()
    if body_radius_m > 0.0 and clearance_m is not None:
        if clearance_m[row, col] < body_radius_m:
            return np.asarray(position, dtype=float).copy()
    return moved


def step_body_sliding(grid_map, position, proposed, low, high, avoid_point=None,
                      body_radius_m=0.0, clearance_m=None):
    """직선 이동이 벽에 막히면, 원래 방향과 각도 차이가 가장 작은 실제로 갈 수 있는 방향을 찾는다.

    실제 room_map에는 지그재그로 꺾인 좁은 문턱이 있어서, 직선 경로가
    막히면 로봇이 완전히 얼어붙는 게 실제로 확인됐다 (`experiments/`에서
    개발/검증된 로직). `avoid_point`(두 스텝 전 위치)로 되돌아가는 후보는
    제외해 진동을 막는다.
    """
    direct = _step_body(grid_map, position, proposed, low, high, body_radius_m, clearance_m)
    if not np.array_equal(direct, position):
        return direct

    step_len = float(np.linalg.norm(np.asarray(proposed) - position))
    if step_len < 1e-9:
        return position.copy()

    original_angle = np.arctan2(proposed[1] - position[1], proposed[0] - position[0])
    ranked = sorted(_SLIDE_ANGLES, key=lambda a: abs(((a - original_angle) + np.pi) % (2 * np.pi) - np.pi))
    for angle in ranked:
        candidate = position + step_len * np.array([np.cos(angle), np.sin(angle)])
        moved = _step_body(grid_map, position, candidate, low, high, body_radius_m, clearance_m)
        if np.array_equal(moved, position):
            continue
        if avoid_point is not None and np.linalg.norm(moved - avoid_point) < step_len * 0.5:
            continue
        return moved
    return position.copy()


def move_with_wall_avoidance(position, goal, distance_field, grid_map, speed, dt):
    """목표 방향과 '가장 가까운 벽에서 멀어지는 방향'을 섞어서 한 걸음 이동한다.

    벽에서 WALL_AVOID_RADIUS_M보다 멀면 순수하게 목표 방향으로만, 가까워질수록
    반발 방향의 비중이 선형으로 커진다. 최종 충돌 방지는 `step_body_sliding`이
    담당 -- 이건 그 앞단의 예방 조치일 뿐이다.
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
