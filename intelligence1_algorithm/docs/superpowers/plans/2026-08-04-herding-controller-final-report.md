# herding_controller — Final Validation Report

**Package:** `herding_controller` (ROS2, two-robot cooperative target herding)
**Date:** 2026-08-05
**Plan:** `docs/superpowers/plans/2026-08-04-herding-controller.md` (Task 15)
**Generated artifact:** `herding_controller/test/output/validation_report.txt`
**Command:** `cd herding_controller && python3 test/run_validation.py 100`

Unit tests: **133 passed, 0 failed**
(`python3 -m pytest test/ -q -p no:anyio`, the whole suite including the
`test_herding_node_imports.py` node-adapter tests and `test_run_validation.py`,
added when the final-review fix wave gave the validation harness its first
direct test coverage).
Note: `-p no:anyio` is required on this machine — a broken system-wide `anyio` pytest
plugin crashes a bare `python3 -m pytest`.

---

## 1. Headline result

**7 of 8 acceptance gates pass.** ALGO-006 (occlusion recovery) fails, and the analysis
in §6 shows it is *not* reachable by parameter tuning — it needs an algorithm change.

| ID | Metric | Threshold | Measured (N=100) | Verdict |
|---|---|---|---|---|
| ALGO-001 | Success rate (`reactive_flee`) | ≥ 70% | **83.0%** | **PASS** |
| ALGO-002 | Mean capture time (successes) | ≤ 60 s | **50.1 s** | **PASS** |
| ALGO-003 | Panic rate (trials with any approach < 0.35 m) | ≤ 10% | **6.0%** | **PASS** |
| ALGO-004 | Max role swaps per trial | ≤ 5 | **1** (mean 0.1) | **PASS** |
| ALGO-005 | Mean control-cycle latency | ≤ 100 ms | **0.3 ms** | **PASS** |
| ALGO-006 | Occlusion re-acquisition ≤ 5 s | ≥ 80% of episodes | **79.3%** (29 usable episodes) | **FAIL** |
| ALGO-007 | All thresholds sourced from yaml | — | structural | **PASS** |
| ALGO-008 | Algorithm vs best baseline | ≥ +40 %p, p < 0.05 | **+76.0 %p**, p < 0.0001 | **PASS** |

ALGO-001~005 are measured on `reactive_flee`, the spec's primary evasion model.

---

## 2. Per-model comparison (N=100 each)

| Evasion model | Success | Mean capture time | Panic rate | Role swaps (mean / max) | Mean latency |
|---|---|---|---|---|---|
| `reactive_flee` (primary) | **83.0%** | 50.1 s | 6.0% | 0.1 / 1 | 0.3 ms |
| `wall_hugger` | **89.0%** | 51.1 s | 8.0% | 0.2 / 5 | 0.4 ms |
| `noisy_human` (real-world proxy) | **90.0%** | 57.0 s | 13.0% | 0.3 / 6 | 0.4 ms |
| `random_walk` (control model) | **0.0%** | n/a | 68.0% | 0.7 / 3 | 0.5 ms |

**Real-world expected success rate = 90.0%** (the `noisy_human` model, per the spec's
"실물 시연 예상치" definition).

Two caveats on that number, both of which push the honest field expectation *below* 90%:

- `noisy_human` also has the highest panic rate of any cooperative model (13.0%, above
  the 10% ALGO-003 bar). Its extra motion noise makes closest-approach violations more
  likely, which is exactly what would happen with a real person.
- Its mean capture time (57.0 s) is only 3 s inside the 60 s ALGO-002 bar. A real
  demonstration with slower robots or added perception latency would likely exceed it.

**`random_walk` scoring 0% is expected, not a defect.** It is the spec's designated
control model (대조군): it ignores the robots entirely, so a reaction-driven herding
scheme has nothing to push against. A `random_walk` target is only ever captured by
coincidence. This is worth confirming with the spec owner (see §8).

**Trivial successes:** exactly 1 of each model's successes was a target that spawned
already inside the capture zone and was scored a success after `capture_hold_sec` with
the robots never moving. These are counted in the headline rate (Task 12 deliberately
left the call to Task 15; I kept them for continuity with the baseline). Excluding them,
`reactive_flee` is 82.0% and `noisy_human` 89.0% — still comfortably over the bar, so
the verdict does not depend on the choice. See §8, item C-6.

---

## 3. ALGO-008 control experiment (N=100 per condition)

Identical spawn seeds across all three conditions (paired comparison).

| Condition | Success rate | Successes / trials |
|---|---|---|
| **Algorithm ON** | **76.0%** | 76 / 100 |
| Robots idle | 0.0% | 0 / 100 |
| Robots random-walking | 0.0% | 0 / 100 |

- **Difference vs best baseline: +76.0 %p** (baseline = `idle`; both baselines tied at 0%).
  Requirement is ≥ +40 %p → **PASS** with a 36 %p margin.
- **Chi-square (3×2, the gated test): p < 0.0001** → PASS (requirement p < 0.05).
- **Diagnostic chi-square (2×2, algorithm vs `idle`): p < 0.0001.**
- **Reliability caveat: not triggered.** The suite prints a "min expected cell ... —
  unreliable below 5" warning whenever any expected cell count falls under 5, because a
  chi-square p-value from a sparse table cannot be believed. At N=100 with a 76/0/0
  split, every expected cell clears the threshold and **no warning fired**, so the
  p-value here is trustworthy.
  For contrast, the pre-tuning baseline run *did* fire it (`p = 0.0001 (min expected
  cell 3.0 — unreliable below 5)`) — that FAIL rested on a p-value that was not
  statistically dependable in the first place.

The algorithm ON rate here (76.0%) is lower than ALGO-001's 83.0% because the control
experiment uses a different seed block (`seed_base + 20000`); it is a different sample
of spawns, not a different configuration.

---

## 4. Parameter tuning performed

### 4.1 Why tuning was needed

Pre-tuning full N=100 run: **4 of 8 gates failing.**

| ID | Before (N=100) | Threshold | Was |
|---|---|---|---|
| ALGO-001 | 16.0% | ≥ 70% | FAIL |
| ALGO-002 | 73.7 s | ≤ 60 s | FAIL |
| ALGO-003 | 17.0% | ≤ 10% | FAIL |
| ALGO-008 | +9.0 %p | ≥ +40 %p | FAIL |

### 4.2 Root cause (diagnosed before changing anything)

Instrumenting individual trials showed the dominant failure was not "slow herding" but
**total deadlock**. Classifying 50 baseline trials:

| Outcome | Count |
|---|---|
| Deadlocked (target motionless for the final 20 s) | **36** |
| Real success | 10 |
| Entered capture zone but never held 3 s | 2 |
| Trivial success (spawned in zone) | 1 |
| Never reached the zone, still moving | 1 |

Instrumenting the deadlock cause over 40 trials:

| Stall cause | Count |
|---|---|
| **Driving Point inside a wall cell** | **25** |
| (captured) | 8 |
| Target pinned / other | 5 |
| Driver parked outside the flee radius | 2 |

**62.5% of all trials deadlocked because the Driving Point was unreachable.**
`compute_driving_point()` returns `target_pos + drive_distance_m · unit(target − goal)`
**unconditionally** — it has no obstacle check, while its sibling
`compute_blocking_point()` does test `grid_map.is_obstacle()` and falls through to the
next-best bearing. When the target rests within `drive_distance_m` of a wall on the far
side from the goal, the Driver's goal lands inside the wall ring. The simulator's
collision model is all-or-nothing (no wall sliding), so the Driver freezes against the
wall roughly 1.0 m from the target — just outside `flee_reaction_distance_m` — and the
target never flees again. Nothing moves for the rest of the trial.

A representative frozen state (seed 1, t = 44 s onward, unchanged to 120 s):

```
target = (3.778, 9.023)     free space ends at y = 9.75 (0.25 m wall ring)
Driver = (4.477, 9.749)     pinned against the wall
Driver goal (Driving Point) = (3.881, 9.816)   <- inside the wall
Blocker = (4.978, 9.023)    parked at exactly block_lookahead_m = 1.2 m
```

A second structural fact fell out of the same trace: the Driver converges *onto* its
driving point and stops there. So if
`drive_distance_m × drive_distance_ease_factor ≥ flee_reaction_distance_m`, the Driver
parks outside the target's reaction radius and the system deadlocks *everywhere*, not
just near walls. That is a hard constraint on the parameter set.

### 4.3 What the plan predicted vs what measurement showed

The plan predicted `drive_distance_m` and `robot_repulsion_weight` would be the two most
sensitive knobs. Measurement says that was half right:

| Parameter | Range swept | Success-rate span | Verdict |
|---|---|---|---|
| `drive_distance_m` | 0.4 → 1.2 | 2% → 36% | Real lever, but ceilings at ~36% alone |
| `robot_repulsion_weight` | 0.5 → 4.0 (8×) | 27.5% → 35% | **Essentially inert** |
| `alignment_threshold` | 0.3 → 1.01 | 35% → 35% | **Exactly inert** |
| `block_lookahead_m` | 1.2 → 3.0 | **25% → 82.5%** \* | **Highest-leverage — not predicted by the plan** |

\* **Only once the ratio constraint in §4.4 is satisfied.** This span was measured
holding `drive_distance_m` = 0.7 and `drive_distance_ease_factor` = 1.0. On a config
that violates the constraint, the *same* `block_lookahead_m` change runs 20% → 2.5%
instead. "Highest-leverage" here means "biggest effect once the system is in a working
regime," **not** "safe to change on its own." See the warning box in §4.4.

`robot_repulsion_weight` only feeds the escape distribution, which only selects the
Blocker's *bearing*. It never touches the Driver — and the Driver was what deadlocked.
That is why an 8× sweep barely moves the number. It was left at its original value
rather than changed for the sake of changing something.

### 4.4 Changes made

All three changes are in `herding_controller/config/herding_params.yaml`.

> ### ⚠️ The three changes are NOT independent. Do not apply them separately.
>
> There is a **necessary condition** on the parameter set:
>
> ```
> drive_distance_m × drive_distance_ease_factor  <  flee_reaction_distance_m
> ```
>
> The Driver converges *onto* its driving point and stops there. If the eased drive
> distance is not strictly inside the target's reaction radius, the Driver parks where
> the target does not react, no pressure is ever applied, and the system deadlocks
> everywhere. **The pre-tuning config violated this** (0.8 × 1.3 = 1.04 ≥ 1.0).
>
> Raising `block_lookahead_m` on its own — treating it as "the dominant lever" — makes
> things **much worse**, not better. Measured directly (N=40, `reactive_flee`):
>
> | Configuration | eased distance | Success |
> |---|---|---|
> | pre-tuning (dd 0.8, ease 1.3, bl 1.2) | 1.04 ✗ | 20.0% |
> | **`block_lookahead_m` → 3.0 alone** (dd 0.8, ease 1.3) | 1.04 ✗ | **2.5%** |
> | ratio fixed alone (dd 0.75, ease 1.15, bl 1.2) | 0.86 ✓ | 25.0% |
> | **both together** (dd 0.75, ease 1.15, bl 3.0) | 0.86 ✓ | **82.5%** |
>
> Neither change is worth much alone (20% → 2.5% or 25%); together they give 82.5%.
> This is a strong interaction, not a sum of two independent effects. **Satisfy the
> ratio constraint first, then tune `block_lookahead_m`** — see C-1 for the arena
> re-tuning procedure.

| Parameter | Before | After | Rationale |
|---|---|---|---|
| `block_lookahead_m` | 1.2 | **3.0** | Reduces how often either robot **transits through** the target's 1.0 m reaction radius on its way to its goal (see mechanism below). Only effective once the ratio constraint above holds. |
| `drive_distance_m` | 0.8 | **0.75** | Mid-plateau of the measured 0.70–0.80 optimum (80–87%), so it is a robust pick rather than a sweep maximum. Keeps clearance over `panic_distance_m` = 0.35 — at 0.4 the panic rate hits 62%. |
| `drive_distance_ease_factor` | 1.3 | **1.15** | Brings the eased drive distance back inside the reaction radius: 0.75 × 1.15 = 0.86, comfortably under 1.0, where the old 0.8 × 1.3 = 1.04 **violated** the constraint. This is the change that makes the other two work. |

#### Mechanism: what `block_lookahead_m` actually does

An earlier draft of this report claimed the Blocker at 1.2 m was "close enough to rotate
the summed flee vector off the goal line." **That was wrong**, and the correction
matters for anyone re-tuning. `ReactiveFlee.step()` gates on a **hard cutoff**, not a
soft falloff:

```python
if dist < self.flee_reaction_distance_m and dist > 1e-6:
    flee_dir += (away / dist) * (self.flee_reaction_distance_m - dist)
```

A robot at 1.2 m contributes **exactly zero** to the flee sum. A Blocker parked at 3.0 m
likewise cannot influence an agent that only reacts within 1.0 m — so "flanking wide to
cut the escape corridor" is *not* the mechanism either.

The real mechanism is **removal of transit interference**. The Blocker is not static; it
is continuously chasing a goal that moves with the target, and at a short lookahead its
path repeatedly crosses *inside* the reaction radius, injecting flee impulses that fight
the Driver's push. Measured over N=40 (fraction of HERD/CORNER cycles with at least one
robot inside the 1.0 m radius):

| Configuration | steps with a robot inside the reaction radius |
|---|---|
| pre-tuning (bl 1.2) | 34.9% |
| tuned (bl 3.0) | **21.0%** |

Raising the lookahead parks the Blocker outside the interaction zone entirely, leaving
the Driver as the sole source of pressure and making the target's flee direction a clean
function of the Driver's bearing. **The Blocker's contribution in this regime is
geometric containment of where the target can be pushed, not force applied to it.**

#### Supporting sweep

`block_lookahead_m` sweep, N=40, `reactive_flee`, **holding `drive_distance_m` = 0.7 and
`drive_distance_ease_factor` = 1.0 fixed** (eased distance 0.70, i.e. the ratio
constraint already satisfied — these numbers do **not** transfer to a config that
violates it, per the warning box above):

| `block_lookahead_m` | 0.6 | 0.8 | 1.0 | **1.2 (old)** | 1.5 | 2.0 | 2.5 | **3.0 (new)** | 4.0 | 6.0 | 8.0 | 12.0 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Success | 7.5% | 15% | 27.5% | 25% | 42.5% | 62.5% | 72.5% | **82.5%** | 80% | 75% | 62.5% | 35% |
| Panic | 67.5% | 47.5% | 20% | 17.5% | 17.5% | 7.5% | 7.5% | **7.5%** | 7.5% | 7.5% | 12.5% | **82.5%** |

**3.0 m is a genuine interior optimum, not a "disable the Blocker" degenerate value.**
Past ~8 m every candidate bearing leaves the 10×10 m grid, `compute_blocking_point()`
exhausts both of its loops and falls through to `return target_pos.copy()`, so the
Blocker drives *straight at the target* — the panic rate jumps to 82.5% and success
collapses. The optimum is bounded on both sides.

**A warning about proxy metrics.** The `block_lookahead_m` → 3.0-alone config (2.5%
success) actually scores *better* than the shipping config on both geometric proxies —
10.4% transit interference and 17.4% invalid Driver goals, versus 21.0% and 22.8%
shipping. Neither proxy predicts success on its own, because both are irrelevant when
the ratio constraint is violated and no pressure is applied at all. Do not tune against
these numbers directly; tune against the success rate.

### 4.5 Before / after

| ID | Before | After | Threshold |
|---|---|---|---|
| ALGO-001 success rate | 16.0% | **83.0%** | ≥ 70% |
| ALGO-002 mean time | 73.7 s | **50.1 s** | ≤ 60 s |
| ALGO-003 panic rate | 17.0% | **6.0%** | ≤ 10% |
| ALGO-004 max role swaps | 3 | **1** | ≤ 5 |
| ALGO-005 mean latency | 0.3 ms | **0.3 ms** | ≤ 100 ms |
| ALGO-006 recovery rate | 80.6% * | **79.3%** * | ≥ 80% |
| ALGO-008 difference | +9.0 %p | **+76.0 %p** | ≥ +40 %p |
| `wall_hugger` success | 15.0% | **89.0%** | — |
| `noisy_human` success | 41.0% | **90.0%** | — |

\* Both ALGO-006 figures are small-sample artefacts — see §6.

---

## 5. Parameter sensitivity (ALGO-004 supporting analysis)

The plan flagged the sensitivity figure as a known placeholder. It is now a real
multi-point sweep, and `block_lookahead_m` was **added** to
`SENSITIVITY_SWEEPS` in `test/run_validation.py`, because the original two-parameter
list omitted the one parameter the system is genuinely sensitive to. As generated by the
final run (`*` = shipping value):

```
drive_distance_m       (10 trials/point):  0.6=50%   0.75*=70%  0.9=10%  1.05=0%
block_lookahead_m      (10 trials/point):  1.2=30%   2=60%      3*=50%   4=50%
robot_repulsion_weight (10 trials/point):  0.5=80%   1=80%      1.5*=80% 2=80%
```

Two things to read from this, and one trap:

- `drive_distance_m` shows the deadlock cliff cleanly: 0.9 → 10% and 1.05 → 0%, exactly
  where the eased drive distance crosses `flee_reaction_distance_m` = 1.0.
- `robot_repulsion_weight` is flat at 80% across its whole range — independent
  confirmation of §4.3 that it is inert.
- **Trap:** the sweep runs at `trials // 10` = 10 trials per point, which is far too
  noisy to rank neighbouring values — it puts `block_lookahead_m` = 2.0 (60%) above the
  shipping 3.0 (50%), where the 40-trial sweep in §4.4 measured 62.5% vs 82.5%. Treat
  this figure as a shape, not a measurement, and use the §4 sweeps for actual values.
  See C-15.

Regenerated figures in `herding_controller/test/output/`: `trajectories.png`,
`escape_heatmap_snapshot.png`, `parameter_sensitivity.png`.

---

## 6. ALGO-006 — the one failing gate, and why tuning cannot fix it

**Measured: 79.3% re-acquisition within 5 s, over 29 usable episodes. Threshold ≥ 80%.**

This gate fails by a single episode at the suite's default sample size (the suite runs
`trials // 3` = 33 occlusion episodes at N=100). That is far too few to resolve an 80%
threshold, so I re-measured at 300 episodes:

| Config | 33 episodes (suite default) | **300 episodes** |
|---|---|---|
| Pre-tuning baseline | 80.6% → PASS | **71.2% → FAIL** |
| Tuned (shipping) | 79.3% → FAIL | **69.8% → FAIL** |

**Two conclusions:**

1. **The tuning did not cause this.** The gap between configs is ~1.4 %p, inside noise.
   The pre-tuning "PASS" at 80.6% was small-sample luck — the true baseline rate was
   ~71%, also failing. **ALGO-006 has been failing all along**; the suite's 33-episode
   sample was simply too small to reveal it.
2. **It is not tunable.** Sweeping every parameter that touches the LOST-state search,
   at 150 episodes each:

   | Parameter | Range swept | Recovery rate |
   |---|---|---|
   | `diffusion_rate` | 0.0 → 2.0 | 76.4% at *every* value |
   | `decay_factor` | 0.5 → 1.0 | 76.4% at *every* value |
   | `min_robot_separation_m` | 0.6 → 5.0 | 76.4% at *every* value |

   Identical to three significant figures across the entire range.

**Episode-selection bias — read before comparing the two configs.**
`_simulate_occlusion_episode()` returns `None` (discarding the episode) when the target
is captured *before* the blackout matters, and those episodes are excluded from the
denominator. The better-herding tuned config therefore discards **more** episodes than
the baseline (29/33 usable vs 31/33): the trials it throws away are precisely the easy
ones it herded home quickly. **The tuned config's surviving sample is systematically
harder than the baseline's**, so the baseline-vs-tuned comparison above is not
apples-to-apples and, if anything, understates the tuned config. This does not change
the conclusion — both fail at ~70% over 300 episodes — but it does mean the ~1.4 %p gap
should not be read as "tuning slightly hurt recovery."

**Structural root cause.** `OcclusionGrid.step()` applies isotropic, mass-conserving
4-neighbour diffusion plus uniform decay. Under normal settings isotropic diffusion
preserves the argmax at the seed cell, so **`best_guess_cell()` returns the target's
last-known cell, essentially regardless of how the diffusion and decay rates are set.**
There is no motion model: the belief never drifts toward where the target actually went,
even though the KF holds a perfectly good last-known velocity. The LOST search therefore
degenerates to "both robots drive to where you last saw it and wait." Because the target
moves at 0.4 m/s and the robots at 0.3 m/s, once it leaves that neighbourhood it cannot
be re-acquired. The ~70% that *do* recover are the episodes where the target happened to
stay nearby.

*Precision note:* "the argmax never moves" is the practical behaviour, not an airtight
theorem. The retained centre weight is `1 − 4·min(diffusion_rate·dt, 0.25)`, which
reaches exactly zero at the clamp, so at extreme `diffusion_rate × dt` the argmax can
tie and jump to a neighbouring cell. That is a one-cell (0.25 m) wobble, not a motion
model, which is why the measured recovery rate is unchanged across the whole swept
range — but the claim is "no useful motion model," not "mathematically pinned forever."

**A knob that moves the number, and why I did not turn it.** `occlusion_timeout_sec`
does shift the result (1.0 s → 72.9%, 2.0 s → 75.5%, 5.0 s → 81.0%, i.e. a "PASS" at
5.0). But this is confounded, not an improvement: the ALGO-006 harness *derives the
blackout duration from that same parameter*
(`blackout_end = blackout_start + uniform(occlusion_timeout_sec + 0.5, + 3.0)`), so
raising it lengthens the blackout and gives the robots more travel time before the
measurement window even opens. It measures a different experiment, and it also makes the
FSM wait a full 5 s before admitting the target is lost — a real behavioural
regression. Turning it to claim a green ALGO-006 would be tuning the metric rather than
the system, so **ALGO-006 is reported honestly as FAIL.**

**Recommended fix (algorithm change, out of scope for a tuning pass):** propagate the
occlusion belief with the target's last-known KF velocity — advect the belief along the
last heading instead of diffusing it isotropically — so the search point leads the
target rather than marking its grave. Alternatively, have the two robots sweep outward
from the seed instead of both converging on one cell.

Note also that ALGO-006's measured value depends on `OCCLUSION_SENSOR_RANGE_M = 1.5`, a
**harness assumption, not a spec parameter** — see §8, item C-4.

---

## 7. Integration checklist — what the team must wire

This package **never calls Nav2 directly.** It consumes poses and a map, and emits goal
poses that the mission manager is responsible for forwarding to navigation. All `~/`
topics are node-relative and resolve inside this node's namespace.

### Subscribed — must be published *into this node's namespace* by others

| Topic | Type | Owner | Notes |
|---|---|---|---|
| `~/target_pose` | `geometry_msgs/PoseStamped` | **Perception team** | The tracked target. Missing for longer than `occlusion_timeout_sec` (3.0 s) drives the FSM into LOST. |
| `~/robot1_pose` | `geometry_msgs/PoseStamped` | **Localization / mission manager** | Orientation is used, not just position — yaw feeds `role_cost_turn_weight` in role assignment. Do not publish identity quaternions. |
| `~/robot2_pose` | `geometry_msgs/PoseStamped` | **Localization / mission manager** | As above. |
| `/map` | `nav_msgs/OccupancyGrid` | **SLAM / `map_server`** | Global (not namespaced). Subscribed with `TRANSIENT_LOCAL` durability, so a latched publisher is required. Shape must equal `grid_height_cells × grid_width_cells` (40×40) or the update is ignored with a warning. |

### Published — must be consumed by others

| Topic | Type | Consumer | Notes |
|---|---|---|---|
| `~/robot1_goal` | `geometry_msgs/PoseStamped` | **Mission manager → Nav2** | Withheld until that robot's pose has been received at least once. |
| `~/robot2_goal` | `geometry_msgs/PoseStamped` | **Mission manager → Nav2** | As above. |
| `~/herding_state` | `std_msgs/String` | Mission manager / operator HMI | FSM state name: `IDLE`/`SEARCH`/`TRACK`/`HERD`/`CORNER`/`CAPTURED`/`LOST`. |
| `~/escape_probability` | `nav_msgs/OccupancyGrid` | Operator HMI / RViz | **Read C-9 below before consuming this.** |
| `~/capture_result` | `std_msgs/Bool` | Mission manager | Capture success signal. |

### ⚠️ Do not forward goal poses to Nav2 without validating them

**The mission manager MUST validate/clamp `~/robot1_goal` and `~/robot2_goal` against
the occupancy grid before passing them to Nav2.**

Goal validity is asymmetric by design in this package:

- **Blocker goals are obstacle-checked.** `compute_blocking_point()` tests
  `grid_map.is_obstacle()` and falls through to the next-best escape bearing.
- **Driver goals are NOT checked at all.** `compute_driving_point()` returns
  `target_pos + drive_distance_m · unit(target − goal)` unconditionally.

Measured on the shipping config, **~23–28% of published Driver goals fall inside an
occupied cell or off-grid** (C-3). Forwarded straight to Nav2, those become rejected or
aborted navigation requests — the failure will look like flaky navigation, not like a
herding bug, so it is worth knowing where it comes from.

Recommended handling: project an invalid goal to the nearest free cell (or hold the
robot's previous goal) and count the occurrences. Which robot is the Driver on any given
cycle is visible from `~/herding_state` plus the goal topics, but the safe assumption is
that **either** goal may be invalid.

### Wiring preconditions

- **Frame:** everything is in `frame_id` = `map` (configurable). Poses in any other frame
  must be transformed by the publisher — this node does no TF lookups.
- **Grid agreement:** `grid_resolution_m` (0.25), `grid_width_cells`/`grid_height_cells`
  (40×40) and `grid_origin_x_m`/`grid_origin_y_m` must match the real `/map`, or the
  occupancy update is silently dropped.
- **Rate:** the node runs its control loop at `control_rate_hz` (5.0 Hz). Pose inputs
  should arrive at ≥ 5 Hz.
- **Capture zone:** `capture_zone_x_m`/`capture_zone_y_m` (3.0, 3.0) must be set to the
  real zone before the field trial (see C-1).

---

## 8. Confirm-needed before this ships

Prioritized. Items marked **[new]** were discovered during this task; the rest are
carried forward from the project ledger
(`.superpowers/sdd/2026-08-04-herding-controller/progress.md`).

### Blocking — a human must resolve these before a field trial

- **C-1. `block_lookahead_m` = 3.0 is scaled to the 10×10 m validation arena. [new]**
  Confirm the real field arena's dimensions and re-tune to roughly a third of its
  shorter side. This is not cosmetic: in a small arena, every candidate bearing at 3.0 m
  falls off the grid, `compute_blocking_point()` falls through to `target_pos`, and the
  Blocker **drives straight at the target** — measured 82.5% panic rate in that
  degenerate regime. Same check applies to `capture_zone_x_m`/`capture_zone_y_m` (3.0,
  3.0) and the 40×40-cell grid, which are all still validation-arena values.

  **Re-tuning procedure — follow this order, the parameters interact:**

  1. **First**, establish the real target's reaction distance and set
     `flee_reaction_distance_m` to it.
  2. **Then** check the necessary condition
     `drive_distance_m × drive_distance_ease_factor < flee_reaction_distance_m`,
     with margin (the shipping config leaves ~14%). If this is violated, nothing else
     you tune will matter — the Driver parks outside the reaction radius and applies no
     pressure at all.
  3. **Only then** sweep `block_lookahead_m`. Changing it first, or alone, is how you
     get 2.5% instead of 82.5% (§4.4).
  4. Keep `drive_distance_m` comfortably above `panic_distance_m` — at 0.4 vs 0.35 the
     panic rate hit 62%.

- **C-2. `CAPTURE_ZONE_CANDIDATES` in `test/field_logger.py` is 4 placeholder
  coordinates.** The field protocol's blinding depends on randomizing over the *real*
  four candidate zones. Ship with placeholders and the blinding is void.

- **C-3. Driver goals are never obstacle-checked — ~23–28% of published
  `~/robot1_goal`/`~/robot2_goal` poses land inside occupied cells. [new]**
  `compute_driving_point()` returns its point unconditionally;
  `compute_blocking_point()` tests `is_obstacle()` and falls back. Instrumenting the
  actual returned points against the live obstacle mask across the N=40 runs:

  | Config | Driver goals inside an obstacle / off-grid |
  |---|---|
  | pre-tuning | ~51–56% of HERD/CORNER cycles |
  | **tuned (shipping)** | **~23–28% of HERD/CORNER cycles** |

  (Two independent measurements bracket each figure — they differ on whether off-grid
  points and non-HERD cycles are counted — but agree that tuning roughly halved it and
  that the residual is **not rare**. Adding interior clutter barely moves it, 25–27%.)

  **The real risk is ROS integration, not simulator deadlock.** In simulation these bad
  goals mostly waste a cycle, because bare point-mass robots slide up to the wall and
  stop. On real hardware a Nav2-routed robot handed a goal pose inside an occupied cell
  will **reject or abort the navigation request** — and roughly a quarter of this
  package's published Driver goals are in that category. See the §7 warning: the mission
  manager must validate/clamp goals rather than forwarding them blindly.

  A prototype of the obvious fix (giving `compute_driving_point()` the same
  obstacle-aware fallback its sibling has) was measured to move **no ALGO metric**
  (+1.6 pp, inside noise), which is why no code change was made in this pass. The fix is
  still worth doing — its value is in goal *validity* for the ROS integration, not in
  the simulated success rate, so do not judge it by the acceptance numbers.

- **C-4. ALGO-006's sensor model is a harness assumption, not a spec parameter.**
  `OCCLUSION_SENSOR_RANGE_M = 1.5` in `run_validation.py` was invented by the test
  harness. The spec fixes the 5 s deadline but says nothing about *how* the target
  becomes visible again; with the suite's otherwise-perfect global sensor, a blackout
  would end by wall clock alone and the algorithm could do nothing and still "recover".
  Confirm 1.5 m matches the real perception stack's re-acquisition range — the ALGO-006
  number moves with it.

- **C-5. ALGO-006 fails at ~70%, and always did. [new]** See §6. Not caused by tuning,
  not fixable by tuning. Someone must decide: accept a ~70% occlusion-recovery rate,
  relax the gate, or fund the belief-advection fix.

### Important — affects how much the numbers should be trusted

- **C-6. Trivial successes are counted in the headline rate.** `run_trial()` spawns the
  target uniformly over the whole arena, capture zone included, and such a trial scores
  a success after `capture_hold_sec` with the robots never moving. Task 12 deliberately
  deferred the call to Task 15; I kept them in for continuity with the baseline and
  report the corrected figures alongside (§2). Confirm which is the official metric.
  The verdict does not change either way.

- **C-7. `random_walk` scores 0%. [new]** Believed correct-by-design — it is the
  designated control model and ignores the robots entirely, so a reaction-driven scheme
  has nothing to push against. Confirm with the spec owner that 0% is the expected
  outcome and not a defect.

- **C-8. ALGO-007 is asserted `True`, not measured.** `run_algo_suite()` hardcodes it.
  A yaml-vs-dataclass consistency check does exist at config-load time, but the gate
  itself does not re-verify against the live `HerdingConfig`.

- **C-9. `~/escape_probability` semantics.** It publishes an `OccupancyGrid`
  rasterization of the **8-direction escape-probability distribution** — *not* the
  occlusion belief grid (publishing the latter was a Task 13 bug, since fixed). Two
  residual quirks for consumers: its origin uses the configured `grid_origin_*` rather
  than the live `/map` origin (can misalign in RViz), and diagonal rays paint 2 cells
  where cardinal rays paint 3 (cosmetic asymmetry).

- **C-10. Field-protocol violation detection only checks the *closest* robot.**
  `detect_rule_violations()` misses a target closing on a second, farther-but-still-
  panic-close robot. Correct per the current spec wording, but it is a genuine
  safety-relevant hole in the 20%-exclusion analysis. Owner of the design spec should
  rule on it.

- **C-11. `kf_converged` does not check KF covariance convergence.** It means "first
  observation seen and not lost". Faithful to the brief, but the name misleads anyone
  reading the FSM transition table.

- **C-12. Nothing in `test/` imports `run_validation.py`.** Its occlusion-recovery loop
  duplicates `simulator.py`'s physics via private helpers, so the two can silently drift
  apart with no test catching it. Given ALGO-006 is the failing gate, this is worth a
  regression test.

- **C-17. `herding_node.py`'s `_sim_time` is a synthetic dt-counter, not the ROS clock.**
  It increments by a fixed `1 / control_rate_hz` each timer callback rather than reading
  the actual clock, so it drifts from wall time whenever the timer jitters or a callback
  runs long. **This directly affects the one failing gate:** `occlusion_timeout_sec` —
  the LOST-entry threshold — is measured against this counter in the live node. Under
  real ROS timer jitter the node's idea of "3 seconds without a detection" will not
  match 3 real seconds, so field occlusion-recovery behaviour can differ from the
  simulated ALGO-006 number in either direction. Should be switched to the ROS clock
  before drawing field conclusions about occlusion handling.

- **C-18. `mean_recovery_sec` averages only the episodes that recovered.** The generated
  report line reads `re-acquired <= 5s: 79.3% | mean recovery: 0.79 s`, and the 0.79 s is
  computed over the recovered 79.3% only — the ~21% that never recovered are excluded,
  not counted as slow. Read naively, "79.3% recovered, mean 0.79 s" suggests a system
  that is nearly always fast; the honest reading is "when it works it is fast (0.79 s),
  and one episode in five it never works at all." Worth relabelling in the generator.

### Minor — bookkeeping

- **C-13.** `test/evasion_models/` file-list mismatch between spec §3-1 and §3-2,
  resolved in favour of §3-2 (5 models). Confirm with the spec owner.
- **C-14.** Three yaml parameters now exist beyond the original spec table:
  `drive_distance_ease_factor`, `role_cost_turn_weight`, and
  `escape_concentration_threshold`.
- **C-15.** The plan's "sensitivity plot is a placeholder" concern is **resolved** — it
  is a real multi-point sweep and now includes `block_lookahead_m`. But it runs only
  `trials // 10` = 10 trials per point, which is noisy enough to mislead (the pre-tuning
  run's sweep suggested `drive_distance_m` = 1.2 was best; a 50-trial sweep showed it
  was the *worst* value tested). Raise trials-per-point before quoting it.
- **C-16.** `herding_node.py` still runs `core.step()` against (0, 0) placeholder
  positions while poses are missing — goal *publishing* is correctly withheld, but the
  FSM, role assigner and escape model advance on fake data in the meantime.

---

## 9. Bottom line

The two-robot Driver/Blocker herding controller **meets 7 of its 8 acceptance criteria**
at N=100, with a real-world expected success rate of **90%** on the `noisy_human` model
and a **+76 %p, p < 0.0001** advantage over both control conditions.

The single failure, **ALGO-006 (occlusion recovery, ~70% vs an 80% bar)**, is a
pre-existing structural limitation that was masked by the suite's 33-episode sample —
not a regression from tuning, and not reachable by any parameter. It needs a motion
model in the occlusion belief grid.

Two things must not get lost on the way to the field:

- **The three tuned parameters are interdependent (§4.4).** `drive_distance_m ×
  drive_distance_ease_factor` must stay below `flee_reaction_distance_m`, or the Driver
  applies no pressure at all. Applying the highest-leverage change
  (`block_lookahead_m` → 3.0) *without* that constraint holding drops success to 2.5%.
  Anyone re-tuning for a different arena must fix the ratio first.
- **C-3: Driver goals are never obstacle-checked.** Roughly 23–28% of published
  `~/robot1_goal`/`~/robot2_goal` poses land inside occupied cells. This is largely
  invisible in simulation but becomes rejected/aborted Nav2 requests on real hardware,
  so the mission manager must validate goals before forwarding them (§7).
