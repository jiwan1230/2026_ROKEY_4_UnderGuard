#!/usr/bin/env python3
"""엔드게임 협공이 '눈에 보이게' 나온 시행을 찾아서 GIF로 뽑는다.

협공은 표적이 덫 0.8m 안에 들어와 3초간 안 잡힐 때만 발동하고, 대부분
몇 초 만에 끝난다. 그래서 아무 시드나 렌더링하면 두 로봇이 갈라지는
장면이 짧게 스쳐 지나가 "결국 한 대로 민 것"처럼 보인다. 이 스크립트는
협공이 오래 유지되고 두 로봇이 실제로 표적 양옆에 선 시행만 골라낸다.

사용 예
-------
    # 시드 40개 무작위로 돌려서 잘 나온 순으로 보기 (렌더링 없음, 빠름)
    python3 -m experiments.pincer_demo --trials 40

    # 상위 2개를 GIF로 저장
    python3 -m experiments.pincer_demo --trials 40 --render 2

    # 특정 시드만 다시 렌더링
    python3 -m experiments.pincer_demo --seed 2000017 --trap left --render 1

    # 매번 다른 결과를 보고 싶으면 (시드 기준점을 시계로)
    python3 -m experiments.pincer_demo --trials 30 --seed-base random --render 1

점수 기준
---------
협공이 실제로 성립한 장면인지를 세 가지로 본다.
  1. 지속 시간 -- 협공 상태가 몇 초나 유지됐는가
  2. 벌어진 각도 -- 표적에서 본 두 로봇의 사잇각. 협공이 제대로 서면
     ±60도이므로 120도에 가깝다. 한 대가 멀리 있으면 작아진다.
  3. 두 로봇이 실제로 붙었는가 -- 목표점만 받고 못 갔으면 협공이 아니다
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from test import real_map_arena as A                                  # noqa: E402
from test.evasion_models.cornering_aware_flee import CorneringAwareFlee  # noqa: E402
from test.evasion_models.noisy_human import NoisyHuman                # noqa: E402
from test.evasion_models.reactive_flee import ReactiveFlee            # noqa: E402
from test.run_validation import (CONFIG_PATH, load_herding_config,    # noqa: E402
                                 make_real_map_config)
from test.simulator import SimulatorConfig, run_trial_real_map        # noqa: E402

BG, INK = "#10161A", "#E7EDEC"
R1C, R2C, RAT, GOAL = "#E0700F", "#19A7B4", "#D1436A", "#3FA05E"
FIRE = "#FFD166"          # 협공 발동 중 강조색


def make_target(kind: str, cfg, grid_map, sim: SimulatorConfig, seed: int):
    """표적 모델 하나 생성. 시드를 주어 같은 시드면 같은 움직임이 나오게 한다."""
    rng = np.random.default_rng([seed, 7])
    if kind == "cornering":
        return CorneringAwareFlee(sim.target_max_speed_mps, cfg.flee_reaction_distance_m,
                                  grid_map, openness_weight=3.0, rng=rng)
    if kind == "noisy":
        return NoisyHuman(sim.target_max_speed_mps, cfg.flee_reaction_distance_m,
                          grid_map, rng=rng)
    if kind == "reactive":
        return ReactiveFlee(sim.target_max_speed_mps, cfg.flee_reaction_distance_m)
    raise SystemExit(f"모르는 표적 종류: {kind}")


def score_trial(result, trap_xy: np.ndarray) -> dict:
    """협공이 얼마나 뚜렷하게 보이는 시행인지 점수화한다."""
    flags = result.pincer_active
    if flags is None or not flags.any():
        return {"fired": False, "score": -1.0}

    idx = np.flatnonzero(flags)
    tgt = result.target_trajectory[idx]
    v1 = result.robot1_trajectory[idx] - tgt
    v2 = result.robot2_trajectory[idx] - tgt
    d1 = np.linalg.norm(v1, axis=1)
    d2 = np.linalg.norm(v2, axis=1)
    ok = (d1 > 1e-6) & (d2 > 1e-6)
    if not ok.any():
        return {"fired": True, "score": -1.0}

    cos = np.einsum("ij,ij->i", v1[ok], v2[ok]) / (d1[ok] * d2[ok])
    angles = np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))

    duration = float(len(idx)) * 0.1
    best_angle = float(np.max(angles))
    # 두 로봇이 모두 표적 근처(0.6m 이내)에 있으면서 가장 크게 벌어진 순간
    both_near = (d1[ok] < 0.6) & (d2[ok] < 0.6)
    framed = float(np.max(angles[both_near])) if both_near.any() else 0.0

    # 120도에 가까울수록, 오래 유지될수록, 성공했을수록 좋은 장면
    score = (framed / 120.0) * 2.0 + min(duration, 8.0) / 8.0 + (1.0 if result.success else 0.0)
    return {"fired": True, "score": score, "duration": duration,
            "max_angle": best_angle, "framed_angle": framed,
            "peak_frame": int(idx[int(np.argmax(angles))])}


def render(result, trap_name: str, seed: int, cfg, out_path: Path) -> None:
    """협공 순간이 보이도록 한 시행을 GIF로 그린다."""
    import matplotlib
    matplotlib.use("Agg")
    matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    mask = A.load_room_obstacle_mask()
    h, w = mask.shape
    ext = [A.ORIGIN_X_M, A.ORIGIN_X_M + w * A.RESOLUTION_M,
           A.ORIGIN_Y_M, A.ORIGIN_Y_M + h * A.RESOLUTION_M]
    trap = A.TRAPS[trap_name]
    flags = result.pincer_active
    n = len(result.target_trajectory)

    fig, ax = plt.subplots(figsize=(7.0, 8.6), facecolor=BG)
    ax.set_facecolor(BG)
    ax.imshow(~mask, extent=ext, origin="lower", cmap="gray", vmin=0, vmax=1, alpha=.85)
    ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3]); ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)

    ax.add_patch(plt.Circle(trap, cfg.capture_radius_m, color=GOAL, alpha=.35, zorder=3))
    ring = plt.Circle(trap, cfg.endgame_trigger_radius_m, ec=GOAL, fc="none",
                      ls=":", lw=1.3, alpha=.65, zorder=2)
    ax.add_patch(ring)
    ax.plot(*trap, "*", color=GOAL, ms=18, zorder=5)

    trail, = ax.plot([], [], "-", color=RAT, alpha=.5, lw=2.0)
    leg1, = ax.plot([], [], "-", color=FIRE, lw=2.2, alpha=.0, zorder=6)
    leg2, = ax.plot([], [], "-", color=FIRE, lw=2.2, alpha=.0, zorder=6)
    rat, = ax.plot([], [], "o", color=RAT, ms=11, zorder=8)
    r1, = ax.plot([], [], "o", color=R1C, ms=16, zorder=9, mec=BG, mew=1.8)
    r2, = ax.plot([], [], "o", color=R2C, ms=16, zorder=9, mec=BG, mew=1.8)

    banner = ax.text(.5, .965, "", transform=ax.transAxes, ha="center", va="top",
                     color=BG, fontsize=13, fontweight="bold",
                     bbox=dict(boxstyle="round,pad=0.35", fc=FIRE, ec="none", alpha=0.0))
    clock = ax.text(.015, .985, "", transform=ax.transAxes, ha="left", va="top",
                    color=INK, fontsize=12)
    status = ax.text(.5, -.025, "", transform=ax.transAxes, ha="center", va="top",
                     color=INK, fontsize=12.5)
    fig.text(.5, .012,
             f"{trap_name} 트랩 · seed={seed} · 점선 = 협공 발동 반경 "
             f"{cfg.endgame_trigger_radius_m:.1f}m   (주황=로봇1, 청록=로봇2, 자주=쥐)",
             color="#9AA5A2", ha="center", fontsize=9.5)
    fig.subplots_adjust(top=.97, bottom=.055, left=.02, right=.98)

    def update(i):
        j = min(i, n - 1)
        lo = max(0, j - 45)
        trail.set_data(result.target_trajectory[lo:j + 1, 0], result.target_trajectory[lo:j + 1, 1])
        t = result.target_trajectory[j]
        p1 = result.robot1_trajectory[j]
        p2 = result.robot2_trajectory[j]
        rat.set_data([t[0]], [t[1]])
        r1.set_data([p1[0]], [p1[1]])
        r2.set_data([p2[0]], [p2[1]])
        clock.set_text(f"t = {j * 0.1:.1f}s")

        firing = bool(flags[j]) if flags is not None and j < len(flags) else False
        if firing:
            leg1.set_data([t[0], p1[0]], [t[1], p1[1]]); leg1.set_alpha(.85)
            leg2.set_data([t[0], p2[0]], [t[1], p2[1]]); leg2.set_alpha(.85)
            v1, v2 = p1 - t, p2 - t
            d1, d2 = np.linalg.norm(v1), np.linalg.norm(v2)
            ang = (np.degrees(np.arccos(np.clip(np.dot(v1, v2) / (d1 * d2), -1, 1)))
                   if d1 > 1e-6 and d2 > 1e-6 else 0.0)
            banner.set_text(f"협공 발동 — 사잇각 {ang:.0f}°")
            banner.get_bbox_patch().set_alpha(.92)
            banner.set_color(BG)
        else:
            leg1.set_alpha(0.0); leg2.set_alpha(0.0)
            banner.set_text("")
            banner.get_bbox_patch().set_alpha(0.0)

        if i >= n - 1:
            if result.success:
                status.set_text(f"✓ 포획 성공 {result.duration_sec:.1f}s"); status.set_color("#5FD08A")
            else:
                near = float(np.min(np.linalg.norm(result.target_trajectory - trap, axis=1)))
                status.set_text(f"✕ 실패 — 트랩 {near:.2f}m 앞에서 놓침"); status.set_color("#FF8FA6")
        return []

    frames = min(n, 700)
    FuncAnimation(fig, update, frames=frames, interval=100).save(
        out_path, writer=PillowWriter(fps=10))
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="엔드게임 협공이 뚜렷하게 나온 시행을 찾아 GIF로 뽑는다.")
    ap.add_argument("--trials", type=int, default=30, help="돌려볼 시행 수 (기본 30)")
    ap.add_argument("--seed", type=int, default=None, help="이 시드 하나만 실행")
    ap.add_argument("--seed-base", default="2000000",
                    help="시드 시작값. 'random'이면 매번 다른 구간 (기본 2000000)")
    ap.add_argument("--trap", default="all", choices=["all", "top", "left", "bottom"])
    ap.add_argument("--target", default="cornering",
                    choices=["cornering", "noisy", "reactive"],
                    help="표적 모델. 협공은 cornering에서 잘 나온다 (기본)")
    ap.add_argument("--render", type=int, default=0, metavar="K",
                    help="점수 상위 K개를 GIF로 저장 (기본 0 = 저장 안 함)")
    ap.add_argument("--outdir", default=str(Path.home() / "Downloads"),
                    help="GIF 저장 폴더 (기본 ~/Downloads)")
    args = ap.parse_args()

    if args.seed_base == "random":
        base = int(time.time()) % 9_000_000
        print(f"시드 기준점: {base} (무작위)")
    else:
        base = int(args.seed_base)

    sim = SimulatorConfig()
    grid_map = A.build_grid_map(A.load_room_obstacle_mask())
    cfg_base = load_herding_config(CONFIG_PATH)
    traps = ["top", "left", "bottom"] if args.trap == "all" else [args.trap]

    jobs = ([(args.trap if args.trap != "all" else "left", args.seed)] if args.seed is not None
            else [(traps[i % len(traps)], base + i) for i in range(args.trials)])

    rows = []
    for trap_name, seed in jobs:
        cfg = make_real_map_config(cfg_base, A.TRAPS[trap_name])
        target = make_target(args.target, cfg, grid_map, sim, seed)
        result = run_trial_real_map(cfg, target, seed, sim)
        s = score_trial(result, A.TRAPS[trap_name])
        rows.append((s["score"], trap_name, seed, result, s))

    rows.sort(key=lambda r: -r[0])
    fired = sum(1 for r in rows if r[4]["fired"])
    ok = sum(1 for r in rows if r[3].success)
    print(f"\n시행 {len(rows)} | 성공 {ok} ({ok/len(rows):.0%}) | 협공 발동 {fired} ({fired/len(rows):.0%})")
    print(f"{'순위':>3} {'트랩':<7} {'시드':>9} {'결과':<5} {'협공지속':>7} {'최대사잇각':>9} {'양옆각':>7}")
    for rank, (score, trap_name, seed, result, s) in enumerate(rows[:15], 1):
        if not s["fired"]:
            print(f"{rank:>3} {trap_name:<7} {seed:>9} {'성공' if result.success else '실패':<5}"
                  f" {'발동안함':>9}")
            continue
        print(f"{rank:>3} {trap_name:<7} {seed:>9} {'성공' if result.success else '실패':<5}"
              f" {s['duration']:>6.1f}s {s['max_angle']:>8.0f}° {s['framed_angle']:>6.0f}°")

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    for rank, (score, trap_name, seed, result, s) in enumerate(rows[:args.render], 1):
        if not s["fired"]:
            print("협공이 발동한 시행이 더 없어 렌더링을 중단한다.")
            break
        cfg = make_real_map_config(cfg_base, A.TRAPS[trap_name])
        path = outdir / f"협공_{rank}_{trap_name}_{seed}.gif"
        render(result, trap_name, seed, cfg, path)
        print(f"저장: {path}  (사잇각 최대 {s['max_angle']:.0f}°, {s['duration']:.1f}s 지속)")


if __name__ == "__main__":
    main()
