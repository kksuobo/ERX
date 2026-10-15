"""Closed-loop episode runner for the local-planning benchmark."""

import math
import random
from time import perf_counter

import numpy as np

from .geometry import clamp, dist, unicycle_step
from .obstacles import Obstacle
from .risk import INF, time_to_collision
from .scenarios import build_scenario
from .state import Control, EgoState


def _observe(obstacles, rng, sensor):
    """Return the obstacle set as perceived by the planner.

    ``sensor`` may add Gaussian position/velocity error and random dropout;
    ground truth is always used for stepping and metric computation.
    """
    pos_std = sensor.get("pos_std", 0.0)
    vel_std = sensor.get("vel_std", 0.0)
    dropout = sensor.get("dropout", 0.0)
    if pos_std == 0.0 and vel_std == 0.0 and dropout == 0.0:
        return obstacles
    out = []
    for o in obstacles:
        if dropout and rng.random() < dropout:
            continue
        obs = Obstacle(o.x + rng.gauss(0.0, pos_std),
                       o.y + rng.gauss(0.0, pos_std), o.r, "static")
        obs.vx = o.vx + rng.gauss(0.0, vel_std)
        obs.vy = o.vy + rng.gauss(0.0, vel_std)
        out.append(obs)
    return out


def _fleet_clearance(ego, obstacles, r_ego):
    return min((dist(ego.x, ego.y, o.x, o.y) - o.r - r_ego for o in obstacles),
               default=INF)


def _fleet_ttc(ego, obstacles, r_ego):
    vx = ego.v * math.cos(ego.theta)
    vy = ego.v * math.sin(ego.theta)
    best = INF
    for o in obstacles:
        t = time_to_collision(ego.x, ego.y, vx, vy, o.x, o.y, o.vx, o.vy,
                              r_ego + o.r)
        if t < best:
            best = t
    return best


def step_plant(ego, ctrl, cfg):
    """Advance the plant one control cycle with the actuator envelope.

    The limits are applied by the plant, independently of the planner, so every
    planner faces identical dynamics.  Single source of truth for the update
    used by the simulator, the debug traces, and the method figures.
    """
    e = cfg["ego"]
    dt = cfg["sim"]["dt"]
    a = clamp((ctrl.v - ego.v) / dt, e["a_dec"], e["a_acc"])
    ego.v = clamp(ego.v + a * dt, e["v_min"], e["v_max"])
    alpha = clamp((ctrl.omega - ego.omega) / dt, -e["alpha"], e["alpha"])
    ego.omega = clamp(ego.omega + alpha * dt, -e["omega_max"], e["omega_max"])
    ego.x, ego.y, ego.theta = unicycle_step(ego.x, ego.y, ego.theta,
                                            ego.v, ego.omega, dt)
    return ego


def run_episode(planner, scenario_name, seed, cfg, record=False):
    """Run one episode on a named scenario (start/goal/obstacles from the suite)."""
    start, goal, obstacles = build_scenario(scenario_name, seed, cfg)
    return run_episode_custom(planner, start, goal, obstacles, cfg, seed, record)


def run_episode_custom(planner, start, goal, obstacles, cfg, seed=1,
                       record=False):
    """Run one episode from an explicit start, goal and obstacle set."""
    dt = cfg["sim"]["dt"]
    max_time = cfg["sim"]["max_time"]
    goal_tol = cfg["sim"]["goal_tol"]
    r_ego = cfg["ego"]["r"]

    theta0 = math.atan2(goal[1] - start[1], goal[0] - start[0])
    ego = EgoState(start[0], start[1], theta0, 0.0, 0.0)
    planner.reset(goal, random.Random(seed * 1000003 + 17))
    sensor = cfg.get("sensor", {})
    sensor_rng = random.Random(seed * 1000003 + 99991)
    delay = int(cfg["sim"].get("actuation_delay", 0))
    cmd_noise = float(cfg["sim"].get("command_noise", 0.0))
    cmd_hist = []

    log = {"t": [], "x": [], "y": [], "theta": [], "v": [], "omega": [],
           "clearance": [], "ttc": [], "state": [], "risk": []}
    latencies = []
    status = "timeout"
    t_collision = None
    t_goal = None
    t = 0.0

    while t < max_time:
        tic = perf_counter()
        perceived = _observe(obstacles, sensor_rng, sensor)
        ctrl = planner.plan(ego, perceived, t)
        latencies.append(perf_counter() - tic)

        # Actuation is applied after a delay and with optional command noise;
        # with both disabled this is exactly the planner's current command.
        cmd_hist.append((ctrl.v, ctrl.omega))
        av, aw = (cmd_hist[-1 - delay] if len(cmd_hist) > delay else cmd_hist[0])
        if cmd_noise:
            av += sensor_rng.gauss(0.0, cmd_noise)
            aw += sensor_rng.gauss(0.0, cmd_noise)
        step_plant(ego, Control(av, aw), cfg)
        t += dt
        for o in obstacles:
            o.step(dt, t)

        clear = _fleet_clearance(ego, obstacles, r_ego)
        ttc = _fleet_ttc(ego, obstacles, r_ego)
        log["t"].append(t)
        log["x"].append(ego.x)
        log["y"].append(ego.y)
        log["theta"].append(ego.theta)
        log["v"].append(ego.v)
        log["omega"].append(ego.omega)
        log["clearance"].append(clear)
        log["ttc"].append(ttc)
        log["state"].append(ctrl.state)
        log["risk"].append(ctrl.risk)

        if clear <= 0.0:
            status = "collision"
            t_collision = t
            break
        if dist(ego.x, ego.y, goal[0], goal[1]) <= goal_tol:
            status = "success"
            t_goal = t
            break

    v_arr = np.asarray(log["v"], dtype=float)
    w_arr = np.asarray(log["omega"], dtype=float)
    lat = np.asarray(latencies, dtype=float)

    if len(v_arr) >= 2:
        acc = np.diff(v_arr) / dt
        jerk = np.diff(acc) / dt if len(acc) >= 2 else np.zeros(1)
        mean_abs_jerk = float(np.mean(np.abs(jerk))) if len(jerk) else 0.0
    else:
        mean_abs_jerk = 0.0

    # Path length along the recorded waypoints.
    xs, ys = np.asarray(log["x"]), np.asarray(log["y"])
    path_length = float(np.sum(np.hypot(np.diff(xs), np.diff(ys)))) if len(xs) > 1 else 0.0

    finite_ttc = [x for x in log["ttc"] if x != INF]
    min_ttc = float(min(finite_ttc)) if finite_ttc else float("nan")
    min_clearance = float(np.min(log["clearance"])) if log["clearance"] else float("nan")

    mean_speed = float(np.mean(v_arr)) if len(v_arr) else 0.0
    deadlock = bool(status == "timeout" and mean_speed < 1.0)

    out = {
        "status": status,
        "success": int(status == "success"),
        "collision": int(status == "collision"),
        "timeout": int(status == "timeout"),
        "deadlock": int(deadlock),
        "time_to_goal": (t_goal if t_goal is not None else float("nan")),
        "completion_time": float(t),
        "path_length": path_length,
        "min_clearance": min_clearance,
        "min_ttc": min_ttc,
        "mean_abs_jerk": mean_abs_jerk,
        "omega_var": float(np.var(w_arr)) if len(w_arr) else 0.0,
        "mean_speed": mean_speed,
        "lat_mean_ms": float(np.mean(lat)) * 1e3 if len(lat) else 0.0,
        "lat_p95_ms": float(np.percentile(lat, 95)) * 1e3 if len(lat) else 0.0,
        "lat_max_ms": float(np.max(lat)) * 1e3 if len(lat) else 0.0,
        "n_cycles": len(v_arr),
    }
    if record:
        out["_log"] = log
        out["_goal"] = goal
    return out
