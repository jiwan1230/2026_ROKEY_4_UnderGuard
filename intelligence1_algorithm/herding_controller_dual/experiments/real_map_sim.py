"""실제 room_map(pgm+yaml) 위에서 로봇 A(Driver) + 로봇 B(Blocker) 몰이 GIF 시각화 스크립트.

**2026-08-06부로 통계 검증 용도가 아니다.** 이 스크립트는 GIF 시각화 전용이고,
통계적 성공률 검증은 `test/real_map_arena.py` + `test/simulator.py:
run_trial_real_map()` + `run_validation.py: run_real_map_algo_suite()`가
정식으로 담당한다 (자세한 경위는 트러블슈팅 노트 10번 항목). 이 스크립트는
`HerdingCore` 전체를 그대로 사용해서(더는 개별 함수를 손으로 조합하지 않음)
그 정식 검증과 동일한 알고리즘/파라미터로 GIF를 뽑는다 — 지도/트랩/순찰경유점/
벽 회피 이동도 전부 `test/real_map_arena.py`에서 그대로 가져와서, 이 스크립트가
정식 검증과 몰래 어긋나는 일(예: 예전엔 `wall_detect_radius_cells`나
`capture_hold_sec`을 여기서만 다른 값으로 하드코딩해뒀었다)이 없게 했다.
"""
import base64
import io
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from herding_controller_dual.herding_core import HerdingCore, Observation
from herding_controller_dual.state_machine import FSMState
from test import real_map_arena
from test.evasion_models.noisy_human import NoisyHuman
from test.evasion_models.reactive_flee import ReactiveFlee
from test.run_validation import CONFIG_PATH, load_herding_config, make_real_map_config
from test.simulator import SimulatorConfig, _advance_target, _bind_model_to_arena, _update_heading

# 지도/트랩/순찰경유점/벽 회피 이동은 전부 정식 검증 하네스(test/real_map_arena.py)의
# 것을 그대로 쓴다 -- 아래는 이 스크립트(사진 방향 렌더링, GIF 페이로드 구성)에서
# 짧게 쓰기 위한 별칭일 뿐이다.
PGM_PATH = real_map_arena.PGM_PATH
RESOLUTION = real_map_arena.RESOLUTION_M
ORIGIN_X, ORIGIN_Y = real_map_arena.ORIGIN_X_M, real_map_arena.ORIGIN_Y_M
ROBOT_A_SPAWN = real_map_arena.ROBOT_A_SPAWN
ROBOT_B_SPAWN = real_map_arena.ROBOT_B_SPAWN
TRAPS = real_map_arena.TRAPS
PATROL_WAYPOINTS = real_map_arena.PATROL_WAYPOINTS
PATROL_WAYPOINT_TOLERANCE_M = real_map_arena.PATROL_WAYPOINT_TOLERANCE_M
SENSOR_RANGE_M = real_map_arena.SENSOR_RANGE_M
sample_free_spawn = real_map_arena.sample_free_spawn
nearest_trap = real_map_arena.nearest_trap

SIM_CONFIG = SimulatorConfig()


def load_room_obstacle_mask():
    """(obstacle_mask, pix, free) 3-tuple로 반환 -- 이 스크립트의 사진 방향 렌더링용.

    `real_map_arena.load_room_obstacle_mask()`는 obstacle_mask 하나만 반환하므로,
    `photo_oriented_map_data_uri()`가 필요로 하는 `free`(원본 픽셀 기준, 반전 없음)를
    pgm에서 다시 읽어 함께 돌려준다.
    """
    with open(PGM_PATH, "rb") as f:
        assert f.readline().strip() == b"P5"
        dims = f.readline().split()
        w, h = int(dims[0]), int(dims[1])
        f.readline()  # maxval
        pix = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    free = pix == 254
    obstacle_mask = real_map_arena.load_room_obstacle_mask()
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


def run_trial(herding_config, grid_map, distance_field, target_model_name, seed, mouse_spawn,
              record_frames=True, blocker_active=True):
    evasion_model = make_target_model(target_model_name, herding_config, seed, grid_map)
    _bind_model_to_arena(evasion_model, grid_map)

    low = np.array([grid_map.config.origin_x_m, grid_map.config.origin_y_m])
    high = np.array([
        grid_map.config.origin_x_m + grid_map.config.width_cells * grid_map.config.resolution_m,
        grid_map.config.origin_y_m + grid_map.config.height_cells * grid_map.config.resolution_m,
    ])

    driver_pos = ROBOT_A_SPAWN.copy()   # 순찰 중 -> 발견하면 Driver(참고용, 이 패키지가 조종 안 함)
    blocker_pos = ROBOT_B_SPAWN.copy()  # 발견 전까지 대기 -> 발견되면 HerdingCore가 목표를 계산
    driver_heading = np.array([1.0, 0.0])
    blocker_heading = np.array([1.0, 0.0])
    prev_driver_pos = driver_pos.copy()
    prev_blocker_pos = blocker_pos.copy()
    target_state = np.array([mouse_spawn[0], mouse_spawn[1], 0.0, 0.0])
    prev_target_pos = target_state[:2].copy()

    def _target_step_fn(core_arg, position, proposed, low_arg, high_arg):
        # 표적도 로봇처럼 막히면 미끄러지듯 우회해야 한다 -- 안 그러면 벽
        # 앞에서 완전히 얼어붙는다 (트러블슈팅 노트 10-4 항목, run_trial_real_map()과 동일 로직).
        return real_map_arena.step_body_sliding(
            core_arg.grid_map, position, proposed, low_arg, high_arg, avoid_point=prev_target_pos
        )

    dt = SIM_CONFIG.dt
    steps = int(round(SIM_CONFIG.max_sim_time_sec / dt))
    frames = []
    success = False

    # 발견 전까지는 HerdingCore를 아예 만들지 않는다: 어느 트랩이 목표인지
    # "발견한 위치에서 가장 가까운 트랩"으로 그때 가서 정하기 때문이다
    # (design spec §1의 "고정 포획구역 1곳"은 미션 시작 전에 이미 정해져
    # 있다는 뜻이므로, 이 발견-시점 선택은 이 GIF 데모의 편의를 위한 것이지
    # 프로덕션 계약은 아니다 -- 정식 통계 검증(run_real_map_algo_suite)은
    # 트랩을 미리 고정하고 돈다).
    discovered = False
    patrol_idx = 0
    goal_name, goal_pos = None, None
    core = None
    discovery_time = None
    min_blocker_dist_after_discovery = float("inf")
    last_t = 0.0

    for i in range(steps):
        t = i * dt
        last_t = t

        if not discovered:
            dist_to_mouse = float(np.linalg.norm(target_state[:2] - driver_pos))
            if dist_to_mouse <= SENSOR_RANGE_M:
                discovered = True
                discovery_time = t
                goal_name, goal_pos = nearest_trap(target_state[:2])
                trial_config = make_real_map_config(herding_config, goal_pos)
                core = HerdingCore(trial_config)
                core.grid_map.obstacle_mask = grid_map.obstacle_mask

        if discovered:
            observation = Observation(
                target_measurement=target_state[:2].copy(),
                robot1_pos=driver_pos.copy(), robot2_pos=blocker_pos.copy(),
                robot1_heading=driver_heading.copy(), robot2_heading=blocker_heading.copy(),
                occupancy=None, sim_time_sec=t, dt=dt,
            )
            output = core.step(observation)
            driver_goal_point, driver_panic = output.robot1_goal, output.panic
            # blocker_active=False는 "로봇 B가 아예 없거나 손 놓고 있으면
            # 어떻게 되는가"를 재는 소거(ablation) 실험용 스위치다. 정상
            # 운용에서는 항상 True.
            blocker_goal_point = output.robot2_goal if blocker_active else ROBOT_B_SPAWN
            state = output.fsm_state.name
            capture_hold_elapsed = core.fsm._capture_hold_elapsed_sec
            capture_progress = min(capture_hold_elapsed / core.config.capture_hold_sec, 1.0)
            success = output.fsm_state == FSMState.CAPTURED
        else:
            waypoint = PATROL_WAYPOINTS[patrol_idx]
            if np.linalg.norm(driver_pos - waypoint) <= PATROL_WAYPOINT_TOLERANCE_M:
                patrol_idx = (patrol_idx + 1) % len(PATROL_WAYPOINTS)
                waypoint = PATROL_WAYPOINTS[patrol_idx]
            driver_goal_point, driver_panic = waypoint, False
            blocker_goal_point = ROBOT_B_SPAWN  # 대기
            state = "SEARCH"
            capture_progress = 0.0

        dist_driver = float(np.linalg.norm(target_state[:2] - driver_pos))
        dist_blocker = float(np.linalg.norm(target_state[:2] - blocker_pos))
        tick_min = min(dist_driver, dist_blocker)
        if discovered:
            min_blocker_dist_after_discovery = min(min_blocker_dist_after_discovery, dist_blocker)

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
                "capture_progress": round(capture_progress, 3),
            })

        if success:
            break

        speed = SIM_CONFIG.robot_max_speed_mps * SIM_CONFIG.robot_gain
        new_driver_raw = real_map_arena.move_with_wall_avoidance(
            driver_pos, driver_goal_point, distance_field, grid_map, speed, dt
        )
        new_blocker_raw = real_map_arena.move_with_wall_avoidance(
            blocker_pos, blocker_goal_point, distance_field, grid_map, speed, dt
        )
        next_driver = real_map_arena.step_body_sliding(
            grid_map, driver_pos, new_driver_raw, low, high, avoid_point=prev_driver_pos
        )
        next_blocker = real_map_arena.step_body_sliding(
            grid_map, blocker_pos, new_blocker_raw, low, high, avoid_point=prev_blocker_pos
        )
        prev_driver_pos, prev_blocker_pos = driver_pos, blocker_pos
        driver_heading = _update_heading(driver_pos, next_driver, driver_heading)
        blocker_heading = _update_heading(blocker_pos, next_blocker, blocker_heading)
        driver_pos, blocker_pos = next_driver, next_blocker

        pre_move_target_xy = target_state[:2].copy()
        # core가 아직 없으면(발견 전) _target_step_fn이 grid_map만 있으면
        # 되므로 임시로 grid_map을 core처럼 넘긴다(.grid_map 속성 필요).
        core_like = core if core is not None else type("_Arena", (), {"grid_map": grid_map})()
        target_state = _advance_target(core_like, evasion_model, target_state, driver_pos, blocker_pos,
                                       SIM_CONFIG, low, high, step_fn=_target_step_fn)
        prev_target_pos = pre_move_target_xy

    return {
        "model": target_model_name, "seed": seed, "success": success,
        "goal_name": goal_name, "mouse_spawn": mouse_spawn.tolist(),
        "discovery_time": discovery_time,
        "duration": (frames[-1]["t"] + dt) if frames else last_t + dt,
        "frames": frames,
        "min_blocker_dist_after_discovery": (
            None if min_blocker_dist_after_discovery == float("inf") else round(min_blocker_dist_after_discovery, 3)
        ),
        "discovered": discovered,
    }


def main(seed_base=None):
    """4개 시행을 만들어 real_map_frames.json에 저장한다.

    `seed_base`가 없으면(기본값) 매번 다른 무작위 시드를 뽑아서 쓴다 --
    예전에는 `np.random.default_rng(0)`으로 고정되어 있어서, 다시 실행해도
    항상 똑같은 쥐 스폰 위치/시행이 나왔다("쥐 생성 위치가 랜덤이 아닌가?"
    라는 지적이 정확했다). 특정 시행을 재현하고 싶으면
    `python3 real_map_sim.py <seed_base>`처럼 정수를 직접 넘기면 된다 --
    실행 시 출력되는 seed_base를 적어뒀다가 나중에 그대로 넘기면 동일한
    결과가 다시 나온다.
    """
    if seed_base is None:
        seed_base = int(np.random.default_rng().integers(0, 1_000_000))
    print(f"seed_base = {seed_base}  (다시 재현하려면: python3 real_map_sim.py {seed_base})")

    herding_config = load_herding_config(CONFIG_PATH)
    obstacle_mask, pix, free = load_room_obstacle_mask()
    height_cells, width_cells = obstacle_mask.shape

    grid_map = real_map_arena.build_grid_map(obstacle_mask)
    distance_field = real_map_arena.build_distance_field(obstacle_mask)

    rng = np.random.default_rng(seed_base)
    trials = []
    seed = seed_base * 1000
    attempts = 0
    while len(trials) < 4 and attempts < 40:
        attempts += 1
        mouse_spawn = sample_free_spawn(
            grid_map, rng, min_clear_m=0.3,
            exclude_points=[ROBOT_A_SPAWN, ROBOT_B_SPAWN] + list(TRAPS.values()),
            exclude_radius_m=0.6,
        )
        model_name = "reactive_flee" if len(trials) % 2 == 0 else "noisy_human"
        trial = run_trial(herding_config, grid_map, distance_field, model_name, seed, mouse_spawn)
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
        "capture_radius": herding_config.capture_radius_m,
        "panic_distance": herding_config.panic_distance_m,
        "sensor_range": SENSOR_RANGE_M,
        "trials": trials,
    }
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "real_map_frames.json")
    with open(out_path, "w") as f:
        json.dump(payload, f)
    print("bytes:", os.path.getsize(out_path))


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else None)
