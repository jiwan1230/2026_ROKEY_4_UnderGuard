"""시행 하나를 화면 녹화 없이 GIF로 렌더링한다 (트러블슈팅 문서용).

브라우저로 Artifact를 열어 직접 화면 녹화하는 대신, 이미 저장된 시행 프레임 데이터
(real_map_frames.json / single_robot_frames.json)를 그대로 matplotlib 애니메이션으로
렌더링해서 GIF 파일로 저장한다. ffmpeg가 없어도 되도록 PillowWriter를 쓴다.
"""
import argparse
import base64
import io
import json
import os

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))

COLORS = {
    "target": "#C1432B", "driver": "#2F6FB0", "blocker": "#8A8F5C",
    "capture": "#C77A1F", "sensor": "#1F7A76",
}


def render_gif(data_path, trial_index, out_path, max_seconds=None, subsample=3, fps=15):
    with open(data_path) as f:
        data = json.load(f)
    trial = data["trials"][trial_index]
    frames = trial["frames"]
    if max_seconds is not None:
        frames = [f for f in frames if f["t"] <= max_seconds]
    frames = frames[::subsample]

    is_real_map = "photo_frame" in data
    if is_real_map:
        pf = data["photo_frame"]
        map_bytes = base64.b64decode(data["map_image"].split(",", 1)[1])
        map_img = Image.open(io.BytesIO(map_bytes))

        def world_to_plot(x, y):
            return pf["y_high"] - y, pf["x_high"] - x  # (plot_x, plot_y)

        xlim = (0, pf["y_high"] - pf["y_low"])
        ylim = (0, pf["x_high"] - pf["x_low"])
    else:
        lx, ly = data["arena"]["low"]
        hx, hy = data["arena"]["high"]

        def world_to_plot(x, y):
            return x, y

        xlim = (lx, hx)
        ylim = (ly, hy)

    fig, ax = plt.subplots(figsize=(7, 7 * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])))
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    if is_real_map:
        ax.imshow(map_img, extent=[xlim[0], xlim[1], ylim[1], ylim[0]], origin="upper")
    else:
        ax.set_facecolor("#EDEFEC")
        wx0, wx1 = data["wall"]["x"]
        wy0, wy1 = data["wall"]["y"]
        ax.add_patch(plt.Rectangle((wx0, wy0), wx1 - wx0, wy1 - wy0, color="#3B4C9E"))

    goal_name = trial.get("goal_name")
    for name, (tx, ty) in data.get("traps", {}).items():
        is_goal = name == goal_name
        px, py = world_to_plot(tx, ty)
        r = data["capture_radius"] * (1 if is_real_map else 1)
        ax.add_patch(plt.Circle((px, py), r, color=COLORS["capture"] if is_goal else "#999",
                                alpha=0.2 if is_goal else 0.1))
        ax.plot(px, py, "*" if is_goal else "s", color=COLORS["capture"] if is_goal else "#999",
               markersize=14 if is_goal else 8, zorder=5)

    driver_key = "driver" if "driver" in frames[0] else "herder"
    blocker_key = "blocker" if "blocker" in frames[0] else "tracker"

    trail_lines = {}
    dots = {}
    for name, color in (("target", COLORS["target"]), (driver_key, COLORS["driver"]), (blocker_key, COLORS["blocker"])):
        trail_lines[name], = ax.plot([], [], "-", color=color, alpha=0.6, linewidth=2)
        dots[name], = ax.plot([], [], "o", color=color, markersize=9, zorder=6)
    sensor_circle = plt.Circle((0, 0), 0, fill=False, linestyle="--", color=COLORS["sensor"], alpha=0.6)
    ax.add_patch(sensor_circle)
    title = ax.text(0.5, 1.03, "", transform=ax.transAxes, ha="center", fontsize=12)

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
        if not f.get("discovered", True):
            px, py = world_to_plot(*f[driver_key])
            sensor_circle.center = (px, py)
            sensor_circle.set_radius(data.get("sensor_range", 0))
            sensor_circle.set_visible(True)
        else:
            sensor_circle.set_visible(False)
        state_ko = {"SEARCH": "순찰", "TRACK": "접근", "HERD": "몰이", "CORNER": "구석 압박", "CAPTURED": "포획 완료"}
        title.set_text(f"t={f['t']:.1f}s  {f['state']} ({state_ko.get(f['state'], f['state'])})")
        return list(trail_lines.values()) + list(dots.values()) + [sensor_circle, title]

    anim = FuncAnimation(fig, update, frames=len(frames), blit=False)
    anim.save(out_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    print(f"saved {out_path} ({len(frames)} frames, {os.path.getsize(out_path)/1024:.0f} KB)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("data_json")
    parser.add_argument("trial_index", type=int)
    parser.add_argument("out_gif")
    parser.add_argument("--max-seconds", type=float, default=None)
    parser.add_argument("--subsample", type=int, default=3)
    parser.add_argument("--fps", type=int, default=15)
    args = parser.parse_args()
    render_gif(args.data_json, args.trial_index, args.out_gif,
              max_seconds=args.max_seconds, subsample=args.subsample, fps=args.fps)
