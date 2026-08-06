"""시행 하나를 화면 녹화 없이 GIF로 렌더링한다 (트러블슈팅 문서용).

브라우저로 Artifact를 열어 직접 화면 녹화하는 대신, 이미 저장된 시행 프레임 데이터
(real_map_frames.json / single_robot_frames.json)를 그대로 matplotlib 애니메이션으로
렌더링해서 GIF 파일로 저장한다. ffmpeg가 없어도 되도록 PillowWriter를 쓴다.

지도 옆에 Artifact 재생 페이지의 텔레메트리 패널(상태, 경과 시간, 역할, 다음 목표 좌표,
거리, 패닉, 포획 진행도)과 동일한 정보를 다크 테마로 같이 그린다.
"""
import argparse
import base64
import io
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import FancyBboxPatch, Rectangle
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))

# Artifact HTML(다크 테마)과 맞춘 팔레트
BG = "#14171A"
PANEL = "#1B1F22"
INK = "#E8EAE6"
MUTED = "#99A09B"
LINE = "#333A3D"
COLORS = {
    "target": "#FF6B52", "driver": "#6FA8DC", "blocker": "#C9CE93",
    "capture": "#F2A33D", "sensor": "#4FC2BB", "warn": "#FF6B52",
}
STATE_KO = {"SEARCH": "순찰", "TRACK": "접근", "HERD": "몰이", "CORNER": "구석 압박", "CAPTURED": "포획 완료"}
STATE_COLOR = {
    "SEARCH": "#E0A93F", "TRACK": "#6FA8DC", "HERD": "#9C97E8",
    "CORNER": "#F2A33D", "CAPTURED": "#6FCF87",
}
TRAP_KO = {"top": "상단 쥐구멍", "left": "좌측 쥐구멍", "bottom": "하단 쥐구멍", "bottom_right": "우측 하단 쥐구멍"}


def _line_crosses_wall(p1, p2, grid_map, n=20):
    """world 좌표 p1->p2 직선이 장애물 셀을 지나가면 True.

    로봇의 실제 이동은 `real_map_arena.step_body_sliding`/
    `move_with_wall_avoidance`가 항상 벽을 피해가지만(실측: 로봇 실제
    좌표가 장애물 셀에 있었던 적은 0번), "다음 목표"를 점선으로 그릴 때는
    로봇의 현재 위치와 목표점을 그냥 일직선으로 잇는다. 목표점이 벽
    너머(예: 문턱 반대편)에 있으면 이 점선이 화면에서 벽을 뚫고 지나가는
    것처럼 보여서, 로봇이 벽을 피해 실제로 돌아가고 있는데도 "벽을 뚫고
    가려 한다"는 오해를 준다 (사용자 피드백으로 확인됨). 이 함수로 그
    상황을 감지해서, 호출부가 점선을 다르게(흐리게/설명 붙여) 그리게 한다.
    """
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    for t in np.linspace(0.0, 1.0, n):
        point = p1 * (1 - t) + p2 * t
        try:
            row, col = grid_map.world_to_cell(*point)
        except ValueError:
            continue
        if grid_map.obstacle_mask[row, col]:
            return True
    return False


def render_gif(data_path, trial_index, out_path, max_seconds=None, subsample=3, fps=15):
    with open(data_path) as f:
        data = json.load(f)
    trial = data["trials"][trial_index]
    frames = trial["frames"]
    if max_seconds is not None:
        frames = [f for f in frames if f["t"] <= max_seconds]
    frames = frames[::subsample]

    is_real_map = "photo_frame" in data
    wall_grid_map = None
    if is_real_map:
        # 목표 점선이 벽을 관통하는지 판정하기 위한 장애물 격자. GIF
        # 렌더링에만 쓰이며, 시뮬레이션 자체(이미 frames에 다 기록됨)에는
        # 영향이 없다.
        from test import real_map_arena
        wall_grid_map = real_map_arena.build_grid_map(real_map_arena.load_room_obstacle_mask())
    driver_key = "driver" if "driver" in frames[0] else "herder"
    blocker_key = "blocker" if "blocker" in frames[0] else "tracker"
    if driver_key == "driver":
        role_a_label, role_b_label = "로봇 A — Driver(미는 역할)", "로봇 B — Blocker(경로 차단)"
    else:
        role_a_label, role_b_label = "로봇 B — 허더(능동, 알고리즘 제어)", "로봇 A — 트래커(추적만)"

    if is_real_map:
        pf = data["photo_frame"]
        map_bytes = base64.b64decode(data["map_image"].split(",", 1)[1])
        map_img = Image.open(io.BytesIO(map_bytes))

        def world_to_plot(x, y):
            # plot_y는 x_high - x가 아니라 x - x_low여야 한다. B(사용자가 확인한 회전
            # 방향)의 픽셀 좌표를 직접 역산해서 교차검증한 결과, 이전 공식은 세로
            # 방향으로 반사(mirror)까지 들어가 있어서 지도 배경은 맞아도 로봇/트랩
            # 점들이 자유공간과 반대되는 위치(벽 위)에 찍히는 원인이었다.
            return pf["y_high"] - y, x - pf["x_low"]

        xlim = (0, pf["y_high"] - pf["y_low"])
        ylim = (0, pf["x_high"] - pf["x_low"])
    else:
        lx, ly = data["arena"]["low"]
        hx, hy = data["arena"]["high"]

        def world_to_plot(x, y):
            return x, y

        xlim = (lx, hx)
        ylim = (ly, hy)

    fig = plt.figure(figsize=(11, 6.2), facecolor=BG)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.35, 1], wspace=0.04, left=0.02, right=0.98, top=0.94, bottom=0.03)
    ax = fig.add_subplot(gs[0, 0])
    ax_t = fig.add_subplot(gs[0, 1])

    ax.set_facecolor(PANEL)
    ax.set_xlim(*xlim); ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color(LINE)
    if is_real_map:
        # extent는 정상 순서 (bottom=ylim[0], top=ylim[1])여야 한다.
        #
        # 이 한 줄을 두 번 반대로 고쳤었다 -- 둘 다 "장애물 좌표를 겹쳐 그려서
        # 검증"까지 해놓고도 틀렸는데, 원인은 검증에 쓴 ground truth 배열이
        # 잘못됐기 때문이었다: `real_map_sim.py`의 grid_map.obstacle_mask는
        # `np.flipud(~free)`로, 물리 연산(world_to_cell 등)의 row0=하단 규약을
        # 맞추려고 pgm 원본에 위아래 반전이 이미 한 번 들어가 있다. 그 배열
        # 기준으로 겹쳐그리기 검증을 하면 "자기 자신과는" 잘 맞아떨어지므로
        # 통과하지만, 실제 눈에 보이는 방(사용자가 확인해준 회전 방향)과는
        # 반대 결론을 내리게 된다.
        #
        # 진짜 ground truth는 pgm에서 바로 뽑은 `free`/`~free`를 회전만 시킨
        # 배열이다 (grid_map 규약의 추가 반전이 섞이지 않은 것). 이 배열을
        # 기본 imshow(추가 옵션 없이)로 그린 모양이 사용자가 직접 확인해준
        # "우리 진짜 월드맵" 모양과 정확히 일치하며, 그 배열을 이 extent로
        # 배치했을 때만 로봇 스폰 좌표도 자유공간에 올바르게 찍힌다.
        # (검증 스크립트는 커밋 메시지 참고 -- 다시 만들 필요 없이 재사용할 것.)
        ax.imshow(map_img, extent=[xlim[0], xlim[1], ylim[0], ylim[1]], origin="upper")
    else:
        wx0, wx1 = data["wall"]["x"]
        wy0, wy1 = data["wall"]["y"]
        ax.add_patch(Rectangle((wx0, wy0), wx1 - wx0, wy1 - wy0, color="#7C8CE0"))

    goal_name = trial.get("goal_name")
    for name, (tx, ty) in data.get("traps", {}).items():
        is_goal = name == goal_name
        px, py = world_to_plot(tx, ty)
        ax.add_patch(plt.Circle((px, py), data["capture_radius"], color=COLORS["capture"] if is_goal else MUTED,
                                alpha=0.22 if is_goal else 0.12))
        ax.plot(px, py, "*" if is_goal else "s", color=COLORS["capture"] if is_goal else MUTED,
               markersize=14 if is_goal else 8, zorder=5)

    trail_lines, dots = {}, {}
    for name, color in (("target", COLORS["target"]), (driver_key, COLORS["driver"]), (blocker_key, COLORS["blocker"])):
        trail_lines[name], = ax.plot([], [], "-", color=color, alpha=0.6, linewidth=2)
        dots[name], = ax.plot([], [], "o", color=color, markersize=9, zorder=6)

    # 다음 목표 좌표: 로봇 -> 목표까지 점선 + 목표 지점에 X 표시 (Artifact와 동일)
    goal_links, goal_marks = {}, {}
    for name, color in ((driver_key, COLORS["driver"]), (blocker_key, COLORS["blocker"])):
        goal_links[name], = ax.plot([], [], ":", color=color, alpha=0.7, linewidth=1.5, zorder=4)
        goal_marks[name], = ax.plot([], [], "x", color=color, markersize=10, markeredgewidth=2.5, zorder=7)

    sensor_circle = plt.Circle((0, 0), 0, fill=False, linestyle="--", color=COLORS["sensor"], alpha=0.6)
    ax.add_patch(sensor_circle)
    panic_ring = plt.Circle((0, 0), 22 / 122 * (xlim[1] - xlim[0]), fill=False, color=COLORS["warn"],
                            linewidth=2, alpha=0.0)
    ax.add_patch(panic_ring)
    ax.set_title("", color=INK, fontsize=11)
    if is_real_map:
        ax.text(0.5, -0.03, "점선이 흐려지면: 목표 방향은 맞지만 그 직선이 벽을 지남 -- 로봇은 실제로 벽을 피해 돌아감",
               color=MUTED, fontsize=7.5, ha="center", va="top", transform=ax.transAxes)

    # ---- 텔레메트리 패널 (Artifact의 오른쪽 사이드바와 동일한 정보) ----
    ax_t.set_facecolor(PANEL)
    ax_t.set_xlim(0, 1); ax_t.set_ylim(0, 1)
    ax_t.axis("off")

    rows = [
        "state", "elapsed", "discover", "role_a", "role_b",
        "driver_goal", "blocker_goal", "dist", "panic", "capture",
    ]
    if not is_real_map:
        rows.remove("discover")
    y_positions = {}
    y = 0.96
    for key in rows:
        y_positions[key] = y
        y -= 0.095

    labels = {
        "state": "상태", "elapsed": "경과 시간", "discover": "발견 상태",
        "role_a": None, "role_b": None,
        "driver_goal": f"{role_a_label.split(chr(8212))[-1].strip()} 다음 목표",
        "blocker_goal": f"{role_b_label.split(chr(8212))[-1].strip()} 다음 목표",
        "dist": "표적 최근접 거리", "panic": "패닉(과근접)", "capture": "포획 유지 진행도",
    }
    label_texts = {}
    value_texts = {}
    state_pill_bg = FancyBboxPatch((0, 0), 0.001, 0.001, boxstyle="round,pad=0.01,rounding_size=0.02",
                                   linewidth=0, facecolor=STATE_COLOR.get("SEARCH", MUTED))
    ax_t.add_patch(state_pill_bg)
    for key in rows:
        if key in ("role_a", "role_b"):
            continue
        ax_t.text(0.0, y_positions[key], labels[key], color=MUTED, fontsize=9.5,
                  va="top", ha="left", transform=ax_t.transAxes)
        value_texts[key] = ax_t.text(0.95, y_positions[key], "", color=INK, fontsize=12.5,
                                     va="top", ha="right", weight="bold",
                                     transform=ax_t.transAxes)
    state_pill_text = ax_t.text(0.95, y_positions["state"] + 0.005, "", color=BG, fontsize=11,
                                va="top", ha="right", weight="bold", zorder=10,
                                transform=ax_t.transAxes)

    role_dot_a = plt.Circle((0.02, 0), 0.012, color=COLORS["driver"], transform=ax_t.transAxes, clip_on=False)
    role_dot_b = plt.Circle((0.02, 0), 0.012, color=COLORS["blocker"], transform=ax_t.transAxes, clip_on=False)
    ax_t.add_patch(role_dot_a); ax_t.add_patch(role_dot_b)
    role_dot_a.center = (0.015, y_positions["role_a"] - 0.01)
    role_dot_b.center = (0.015, y_positions["role_b"] - 0.01)
    ax_t.text(0.05, y_positions["role_a"], role_a_label, color=INK, fontsize=9.5, va="top", ha="left",
              transform=ax_t.transAxes)
    ax_t.text(0.05, y_positions["role_b"], role_b_label, color=INK, fontsize=9.5, va="top", ha="left",
              transform=ax_t.transAxes)

    bar_y = y_positions["capture"] - 0.045
    bar_bg = Rectangle((0.0, bar_y), 1.0, 0.02, facecolor=LINE, transform=ax_t.transAxes)
    bar_fill = Rectangle((0.0, bar_y), 0.0, 0.02, facecolor="#6FCF87", transform=ax_t.transAxes)
    ax_t.add_patch(bar_bg); ax_t.add_patch(bar_fill)

    trial_label = f"{trial['model']} · 목표: {TRAP_KO.get(goal_name, goal_name) if goal_name else '미발견'}"
    ax_t.text(0.0, 0.03, trial_label, color=MUTED, fontsize=9, va="bottom", ha="left", transform=ax_t.transAxes)

    trail_hist = {k: [] for k in (driver_key, blocker_key, "target")}

    def update(i):
        f = frames[i]
        for name in (driver_key, blocker_key, "target"):
            wx, wy = f[name]
            px, py = world_to_plot(wx, wy)
            trail_hist[name].append((px, py))
            if len(trail_hist[name]) > 60:
                trail_hist[name] = trail_hist[name][-60:]
            xs, ys = zip(*trail_hist[name])
            trail_lines[name].set_data(xs, ys)
            dots[name].set_data([px], [py])

        goal_field = {driver_key: "driver_goal" if "driver_goal" in f else "herder_goal",
                     blocker_key: "blocker_goal" if "blocker_goal" in f else "tracker_goal"}
        show_goals = f["state"] != "CAPTURED"
        for name in (driver_key, blocker_key):
            rx, ry = world_to_plot(*f[name])
            gx, gy = world_to_plot(*f[goal_field[name]])
            if show_goals:
                goal_links[name].set_data([rx, gx], [ry, gy])
                goal_marks[name].set_data([gx], [gy])
                if wall_grid_map is not None and _line_crosses_wall(f[name], f[goal_field[name]], wall_grid_map):
                    # 목표까지 일직선을 그으면 벽을 관통한다: 로봇은 실제로
                    # 이 직선이 아니라 벽을 피해 돌아가므로("케이스 A/B"
                    # 수정 이후 실측상 로봇이 장애물 셀에 있었던 적은 없음),
                    # 이 점선을 "실제 이동 경로"로 오해하지 않도록 훨씬
                    # 흐리게 그린다 -- 점선은 여전히 "그 방향이 목표"라는
                    # 정보는 담고 있지만, 로봇이 벽을 뚫고 가려 한다는
                    # 인상은 주지 않는다.
                    goal_links[name].set_alpha(0.18)
                else:
                    goal_links[name].set_alpha(0.7)
            else:
                goal_links[name].set_data([], [])
                goal_marks[name].set_data([], [])

        discovered = f.get("discovered", True)
        if not discovered:
            px, py = world_to_plot(*f[driver_key])
            sensor_circle.center = (px, py)
            sensor_circle.set_radius(data.get("sensor_range", 0))
            sensor_circle.set_visible(True)
        else:
            sensor_circle.set_visible(False)

        if f.get("panic"):
            tpx, tpy = world_to_plot(*f["target"])
            panic_ring.center = (tpx, tpy)
            panic_ring.set_alpha(0.8)
        else:
            panic_ring.set_alpha(0.0)

        state = f["state"]
        ax.set_title(f"t={f['t']:.1f}s", color=INK, fontsize=11)

        state_label = f"{state} · {STATE_KO.get(state, state)}"
        state_pill_text.set_text(state_label)
        color = STATE_COLOR.get(state, MUTED)
        # 대략적인 텍스트 폭에 맞춰 알약 배경 크기를 갱신
        w = 0.028 * len(state_label) + 0.03
        state_pill_bg.set_bounds(0.95 - w, y_positions["state"] - 0.055, w, 0.075)
        state_pill_bg.set_facecolor(color)

        value_texts["elapsed"].set_text(f"{f['t']:.1f} s")
        if "discover" in value_texts:
            if discovered:
                dt = trial.get("discovery_time")
                value_texts["discover"].set_text(f"발견됨 (t={dt:.1f}s)" if dt is not None else "발견됨")
            else:
                value_texts["discover"].set_text("순찰 중")

        dgx, dgy = f["driver_goal" if "driver_goal" in f else "herder_goal"]
        bgx, bgy = f["blocker_goal" if "blocker_goal" in f else "tracker_goal"]
        panic_flag = f.get("driver_panic", f.get("herder_panic", False))
        value_texts["driver_goal"].set_text(f"({dgx:.2f}, {dgy:.2f})" + (" ·후퇴" if panic_flag else ""))
        value_texts["blocker_goal"].set_text(f"({bgx:.2f}, {bgy:.2f})")

        value_texts["dist"].set_text(f"{f['dist']:.2f} m")
        value_texts["dist"].set_color(COLORS["warn"] if f.get("panic") else INK)
        value_texts["panic"].set_text("PANIC" if f.get("panic") else "정상")
        value_texts["panic"].set_color(COLORS["warn"] if f.get("panic") else INK)

        pct = f["capture_progress"]
        value_texts["capture"].set_text(f"{pct*100:.0f}%")
        bar_fill.set_width(pct)

        artists = list(trail_lines.values()) + list(dots.values()) + list(goal_links.values()) + list(goal_marks.values())
        artists += [sensor_circle, panic_ring, state_pill_bg, state_pill_text, bar_fill]
        artists += list(value_texts.values())
        return artists

    anim = FuncAnimation(fig, update, frames=len(frames), blit=False)
    anim.save(out_path, writer=PillowWriter(fps=fps), savefig_kwargs={"facecolor": BG})
    plt.close(fig)
    print(f"saved {out_path} ({len(frames)} frames, {os.path.getsize(out_path)/1024:.0f} KB)")


def _next_numbered_path(path: str) -> str:
    """path가 이미 존재하면 파일명 뒤에 _001, _002, ...를 붙여 절대 겹치지 않는 경로를 찾는다.

    `--number` 플래그가 켜졌을 때만 쓰인다. 매번 같은 out_gif 경로로 재실행하면
    이전 GIF가 조용히 덮어써지는("자꾸 초기화된다") 문제를, 이전 산출물을
    그대로 둔 채 새 번호로 저장하는 방식으로 해결한다.
    """
    if not os.path.exists(path):
        return path
    root, ext = os.path.splitext(path)
    n = 1
    while True:
        candidate = f"{root}_{n:03d}{ext}"
        if not os.path.exists(candidate):
            return candidate
        n += 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("data_json")
    parser.add_argument("trial_index", type=int)
    parser.add_argument("out_gif")
    parser.add_argument("--max-seconds", type=float, default=None)
    parser.add_argument("--subsample", type=int, default=3)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument(
        "--number", action="store_true",
        help="out_gif가 이미 존재하면 덮어쓰지 않고 _001, _002... 번호를 붙여 새 파일로 저장",
    )
    args = parser.parse_args()
    out_path = _next_numbered_path(args.out_gif) if args.number else args.out_gif
    render_gif(args.data_json, args.trial_index, out_path,
              max_seconds=args.max_seconds, subsample=args.subsample, fps=args.fps)
