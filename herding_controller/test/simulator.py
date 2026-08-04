# herding_controller/test/simulator.py
"""Headless 2D physics simulator: point-mass robots + a target evasion model, no ROS."""
from dataclasses import dataclass, field

import numpy as np

from herding_controller.herding_core import HerdingConfig, HerdingCore, Observation
from herding_controller.state_machine import FSMState
from test.evasion_models.base import EvasionModel

# The arena's upper edge is exclusive (GridMap.world_to_cell floors), so clamp a hair
# inside it or a body pinned to the top/right wall would fall off the grid.
_EDGE_EPS = 1e-9


@dataclass
class SimulatorConfig:
    """Physical limits and trial duration for the 2D herding simulation."""
    robot_max_speed_mps: float = 0.3
    target_max_speed_mps: float = 0.4
    dt: float = 0.1
    max_sim_time_sec: float = 120.0
    robot_gain: float = 1.0


@dataclass
class TrialResult:
    """Outcome and telemetry of one herding trial."""
    success: bool
    duration_sec: float
    panic_count: int
    role_swap_count: int
    mean_latency_ms: float
    target_trajectory: np.ndarray
    robot1_trajectory: np.ndarray
    robot2_trajectory: np.ndarray
    escape_snapshot: np.ndarray | None = None
    min_robot_target_dist: float = field(default=float("inf"))


def _move_toward(position: np.ndarray, goal: np.ndarray, max_speed: float, gain: float, dt: float) -> np.ndarray:
    """Step `position` toward `goal`, travelling at most max_speed * gain * dt meters."""
    direction = goal - position
    dist = np.linalg.norm(direction)
    if dist < 1e-9:
        return position
    step = min(max_speed * gain * dt, dist)
    return position + (direction / dist) * step


def _update_heading(old_pos: np.ndarray, new_pos: np.ndarray, heading: np.ndarray) -> np.ndarray:
    """Unit heading along the robot's displacement, or the previous heading if it did not move.

    Validity depends solely on whether the robot actually moved -- never on where it
    ended up -- so a robot on the negative-x side of the map keeps updating its heading.
    """
    delta = np.asarray(new_pos, dtype=float) - np.asarray(old_pos, dtype=float)
    norm = float(np.linalg.norm(delta))
    if norm < 1e-9:
        return heading
    return delta / norm


def _arena_bounds(config: HerdingConfig) -> tuple[np.ndarray, np.ndarray]:
    """Lower/upper (x, y) corners of the walled arena, which is exactly the grid extent."""
    low = np.array([config.grid_origin_x_m, config.grid_origin_y_m], dtype=float)
    span = np.array([config.grid_width_cells, config.grid_height_cells], dtype=float)
    return low, low + span * config.grid_resolution_m


def _clamp_to_arena(position: np.ndarray, low: np.ndarray, high: np.ndarray) -> np.ndarray:
    """Keep a body inside the arena walls.

    The escape model already treats every off-grid neighbour cell as a wall, so an
    unwalled simulator would let the target wander somewhere the algorithm believes
    is impossible (and where no escape distribution exists at all).
    """
    return np.clip(np.asarray(position, dtype=float), low, high - _EDGE_EPS)


def _is_obstacle_at(core: HerdingCore, position: np.ndarray) -> bool:
    """True if `position` falls inside an occupied cell of the core's grid."""
    try:
        row, col = core.grid_map.world_to_cell(float(position[0]), float(position[1]))
    except (ValueError, TypeError):
        return False
    return bool(core.grid_map.obstacle_mask[row, col])


def _boundary_ring_mask(config: HerdingConfig) -> np.ndarray:
    """Occupancy of a bare walled room: the outermost cell ring is solid wall.

    The arena walls have to be *visible* as obstacle cells, not merely enforced by
    clamping. WallHugger (and NoisyHuman, which wraps it) look for occupied cells to
    hug and stand still when they find none, and EscapeModel's thigmotaxis term needs
    a wall to follow. An all-False mask leaves both of them modelling an infinite open
    plane inside a room that the physics says is closed.
    """
    mask = np.zeros((config.grid_height_cells, config.grid_width_cells), dtype=bool)
    mask[0, :] = mask[-1, :] = True
    mask[:, 0] = mask[:, -1] = True
    return mask


def _bind_model_to_arena(evasion_model: EvasionModel, grid_map) -> None:
    """Re-point every grid-consulting evasion model at the arena this trial simulates.

    WallHugger (and NoisyHuman, which wraps one) resolves walls through the GridMap it
    was constructed with, not through the `obstacle_map` argument of step(). Callers
    build their models from a separate probe core's grid map -- a different GridMap
    object from the one run_trial creates -- so without rebinding, the model reads an
    empty world while the physics runs in a walled arena, and it never hugs anything.
    """
    for candidate in (evasion_model, getattr(evasion_model, "_wall_hugger", None)):
        if candidate is not None and hasattr(candidate, "grid_map"):
            candidate.grid_map = grid_map


def _step_body(core: HerdingCore, position: np.ndarray, proposed: np.ndarray,
               low: np.ndarray, high: np.ndarray) -> np.ndarray:
    """Move a body to `proposed`, clamped into the arena and blocked by obstacle cells.

    Robots and the target obey the same rule: a body that walks into a wall stops dead
    rather than tunnelling through it.
    """
    moved = _clamp_to_arena(proposed, low, high)
    if _is_obstacle_at(core, moved):
        return np.asarray(position, dtype=float).copy()
    return moved


def run_trial(
    herding_config: HerdingConfig,
    evasion_model: EvasionModel,
    seed: int,
    sim_config: SimulatorConfig = SimulatorConfig(),
    obstacle_mask: np.ndarray | None = None,
    control_mode: str = "algorithm",
) -> TrialResult:
    """Simulate one herding trial. control_mode: 'algorithm' | 'idle' | 'random'."""
    if control_mode not in ("algorithm", "idle", "random"):
        raise ValueError(f"unknown control_mode: {control_mode!r}")

    rng = np.random.default_rng(seed)
    core = HerdingCore(herding_config)
    if obstacle_mask is not None:
        # A caller-supplied layout is used verbatim; it is not silently walled in.
        expected = (herding_config.grid_height_cells, herding_config.grid_width_cells)
        if tuple(np.shape(obstacle_mask)) != expected:
            raise ValueError(f"obstacle_mask shape {np.shape(obstacle_mask)} != expected {expected}")
        core.grid_map.obstacle_mask = np.asarray(obstacle_mask, dtype=bool)
    else:
        core.grid_map.obstacle_mask = _boundary_ring_mask(herding_config)
    _bind_model_to_arena(evasion_model, core.grid_map)

    low, high = _arena_bounds(herding_config)
    margin = herding_config.grid_resolution_m * 2
    spawn_low, spawn_high = low + margin, high - margin
    # target_state is [x, y, vx, vy]: the evasion models are fed position AND velocity.
    target_state = np.array([
        rng.uniform(spawn_low[0], spawn_high[0]), rng.uniform(spawn_low[1], spawn_high[1]), 0.0, 0.0,
    ])
    robot1_pos = np.array([spawn_low[0], spawn_low[1]])
    robot2_pos = np.array([spawn_high[0], spawn_low[1]])
    robot1_heading = np.array([1.0, 0.0])
    robot2_heading = np.array([1.0, 0.0])

    target_traj, robot1_traj, robot2_traj = [], [], []
    panic_count, role_swap_count, latencies = 0, 0, []
    min_dist = float("inf")
    success, elapsed_sec = False, 0.0
    escape_snapshot = None

    steps = max(int(round(sim_config.max_sim_time_sec / sim_config.dt)), 0)
    for index in range(steps):
        sim_time_sec = index * sim_config.dt
        target_traj.append(target_state[:2].copy())
        robot1_traj.append(robot1_pos.copy())
        robot2_traj.append(robot2_pos.copy())

        # Closest approach is sampled at every sub-step of the cycle, not just at its
        # start: the robots move first and the target reacts afterwards, so the tightest
        # gap of the cycle is normally the intermediate (robots moved, target has not)
        # state that a start-of-cycle-only measurement never sees.
        tick_min = _closest_robot_distance(target_state[:2], robot1_pos, robot2_pos)

        observation = Observation(
            target_measurement=target_state[:2].copy(),  # raw sensor reading: position only
            robot1_pos=robot1_pos.copy(), robot2_pos=robot2_pos.copy(),
            robot1_heading=robot1_heading.copy(), robot2_heading=robot2_heading.copy(),
            occupancy=None, sim_time_sec=sim_time_sec, dt=sim_config.dt,
        )
        # Every control mode steps the core, so the FSM, KF, occlusion grid and role
        # state advance identically; only what the robots do with the goals differs.
        output = core.step(observation)
        latencies.append(output.latency_ms)
        if output.role_swapped:
            role_swap_count += 1
        if output.escape_top3:
            escape_snapshot = np.array(output.escape_top3)

        elapsed_sec = sim_time_sec + sim_config.dt
        if output.fsm_state == FSMState.CAPTURED:
            success = True
            min_dist = min(min_dist, tick_min)
            if tick_min < herding_config.panic_distance_m:
                panic_count += 1
            break

        if control_mode == "algorithm":
            new_r1 = _move_toward(robot1_pos, output.robot1_goal, sim_config.robot_max_speed_mps,
                                  sim_config.robot_gain, sim_config.dt)
            new_r2 = _move_toward(robot2_pos, output.robot2_goal, sim_config.robot_max_speed_mps,
                                  sim_config.robot_gain, sim_config.dt)
        elif control_mode == "random":
            angle1, angle2 = rng.uniform(0, 2 * np.pi, size=2)
            travel = sim_config.robot_max_speed_mps * sim_config.robot_gain * sim_config.dt
            new_r1 = robot1_pos + np.array([np.cos(angle1), np.sin(angle1)]) * travel
            new_r2 = robot2_pos + np.array([np.cos(angle2), np.sin(angle2)]) * travel
        else:  # idle
            new_r1, new_r2 = robot1_pos.copy(), robot2_pos.copy()
        new_r1 = _step_body(core, robot1_pos, new_r1, low, high)
        new_r2 = _step_body(core, robot2_pos, new_r2, low, high)

        robot1_heading = _update_heading(robot1_pos, new_r1, robot1_heading)
        robot2_heading = _update_heading(robot2_pos, new_r2, robot2_heading)
        robot1_pos, robot2_pos = new_r1, new_r2
        tick_min = min(tick_min, _closest_robot_distance(target_state[:2], robot1_pos, robot2_pos))

        target_state = _advance_target(
            core, evasion_model, target_state, robot1_pos, robot2_pos, sim_config, low, high
        )
        tick_min = min(tick_min, _closest_robot_distance(target_state[:2], robot1_pos, robot2_pos))

        min_dist = min(min_dist, tick_min)
        # One count per control cycle in which the target came within panic distance at
        # any point. A violation sitting exactly on a cycle boundary is attributed to
        # both neighbouring cycles; over-reporting a safety metric beats missing it.
        if tick_min < herding_config.panic_distance_m:
            panic_count += 1

    return TrialResult(
        success=success, duration_sec=elapsed_sec, panic_count=panic_count,
        role_swap_count=role_swap_count,
        mean_latency_ms=float(np.mean(latencies)) if latencies else 0.0,
        target_trajectory=np.array(target_traj, dtype=float).reshape(-1, 2),
        robot1_trajectory=np.array(robot1_traj, dtype=float).reshape(-1, 2),
        robot2_trajectory=np.array(robot2_traj, dtype=float).reshape(-1, 2),
        escape_snapshot=escape_snapshot, min_robot_target_dist=min_dist,
    )


def _closest_robot_distance(target_pos: np.ndarray, robot1_pos: np.ndarray, robot2_pos: np.ndarray) -> float:
    """Distance from the target to whichever robot is nearer."""
    return float(min(
        np.linalg.norm(target_pos - robot1_pos),
        np.linalg.norm(target_pos - robot2_pos),
    ))


def _advance_target(
    core: HerdingCore,
    evasion_model: EvasionModel,
    target_state: np.ndarray,
    robot1_pos: np.ndarray,
    robot2_pos: np.ndarray,
    sim_config: SimulatorConfig,
    low: np.ndarray,
    high: np.ndarray,
) -> np.ndarray:
    """Integrate the evasion model's commanded velocity into a new [x, y, vx, vy] state."""
    commanded = np.asarray(
        evasion_model.step(target_state, [robot1_pos, robot2_pos], core.grid_map.obstacle_mask, sim_config.dt),
        dtype=float,
    ).reshape(2)
    speed = float(np.linalg.norm(commanded))
    if speed > sim_config.target_max_speed_mps:
        commanded = commanded / speed * sim_config.target_max_speed_mps

    position = target_state[:2]
    proposed = _step_body(core, position, position + commanded * sim_config.dt, low, high)
    # The stored velocity is the ACHIEVED one, so a target braked by a wall reports the
    # motion that actually happened -- which is what the next cycle's evasion model,
    # escape-model momentum term and KF all have to agree with.
    achieved = (proposed - position) / sim_config.dt
    return np.concatenate([proposed, achieved])
