"""Method figures: Fig. 2 (risk field + cost-coloured candidates) and
Fig. 3 (weight schedule, admissible-speed modulation, state timeline)."""

import _bootstrap  # noqa: F401
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from radwa.config import load_config
from radwa.planners.common import roll_trajectory
from radwa.planners.radwa import RADWA
from radwa.risk import INF, risk_from_ttc, time_to_collision
from radwa.scenarios import build_scenario
from radwa.simulator import run_episode, step_plant
from radwa.state import EgoState

ROOT = _bootstrap.ROOT
STATE_COLOR = {"free": "#cfe8cf", "follow": "#ffe7b0",
               "conflict": "#ffc9a3", "emergency": "#f6b0b0"}


def replay(cfg, scenario, seed, planner, stop_at=None):
    """Replay an episode, returning per-cycle (ego, obstacles, control).

    When ``stop_at`` is given, stop right after planning that cycle so the
    planner's debug buffer matches the returned frame.
    """
    start, goal, obstacles = build_scenario(scenario, seed, cfg)
    dt = cfg["sim"]["dt"]
    theta0 = math.atan2(goal[1] - start[1], goal[0] - start[0])
    ego = EgoState(start[0], start[1], theta0, 0.0, 0.0)
    planner.reset(goal)
    frames = []
    t = 0.0
    while t < cfg["sim"]["max_time"]:
        ctrl = planner.plan(ego, obstacles, t)
        frames.append((EgoState(ego.x, ego.y, ego.theta, ego.v, ego.omega),
                       [(o.x, o.y, o.r, o.vx, o.vy) for o in obstacles], ctrl))
        if stop_at is not None and len(frames) - 1 >= stop_at:
            break
        step_plant(ego, ctrl, cfg)
        t += dt
        for o in obstacles:
            o.step(dt, t)
    return frames


def risk_field(cfg, ego, obs):
    r_ego = cfg["ego"]["r"]
    p = cfg["radwa"]
    vx = ego.v * math.cos(ego.theta)
    vy = ego.v * math.sin(ego.theta)
    xs = np.linspace(ego.x - 8, ego.x + 26, 220)
    ys = np.linspace(-10, 10, 140)
    Z = np.zeros((len(ys), len(xs)))
    for j, yy in enumerate(ys):
        for i, xx in enumerate(xs):
            best = INF
            for (ox, oy, orr, ovx, ovy) in obs:
                ttc = time_to_collision(xx, yy, vx, vy, ox, oy, ovx, ovy,
                                        r_ego + orr + p["d_safe_margin"])
                if ttc < best:
                    best = ttc
            Z[j, i] = risk_from_ttc(best, p["T0"], p.get("risk_cutoff", INF))
    return xs, ys, Z


def fig_risk_field(cfg, scenario, seed):
    def dmin(f):
        _, obs, _ = f
        return min(math.hypot(f[0].x - ox, f[0].y - oy) - orr - r_ego
                   for (ox, oy, orr, _, _) in obs)

    r_ego = cfg["ego"]["r"]
    probe = replay(cfg, scenario, seed, RADWA(cfg))
    moving = [i for i, f in enumerate(probe) if f[0].v > 1.5] or list(range(len(probe)))
    idx = min(moving, key=lambda i: dmin(probe[i]))

    planner = RADWA(cfg)
    planner.debug = True
    frames = replay(cfg, scenario, seed, planner, stop_at=idx)
    ego, obs, ctrl = frames[idx]
    xs, ys, Z = risk_field(cfg, ego, obs)

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))
    ax = axes[0]
    im = ax.imshow(Z, origin="lower", extent=[xs[0], xs[-1], ys[0], ys[-1]],
                   aspect="equal", cmap="magma", vmin=0, vmax=1, alpha=0.9)
    for (ox, oy, orr, _, _) in obs:
        ax.add_patch(plt.Circle((ox, oy), orr, color="0.85", ec="k", lw=0.6))
    ax.plot(ego.x, ego.y, "o", color="cyan", ms=9, mec="k")
    ax.arrow(ego.x, ego.y, 2 * math.cos(ego.theta), 2 * math.sin(ego.theta),
             color="cyan", width=0.08)
    ax.set_title(f"(a) normalized TTC risk field  (t={idx * cfg['sim']['dt']:.1f}s,"
                 f" state={ctrl.state})")
    ax.set_xlabel("x [m]"), ax.set_ylabel("y [m]")
    fig.colorbar(im, ax=ax, fraction=0.046, label="risk R")

    ax = axes[1]
    cands = planner._last_candidates
    costs = np.array([c[0] for c in cands])
    lo, hi = np.percentile(costs, 5), np.percentile(costs, 95)
    norm = plt.Normalize(lo, hi)
    cmap = plt.cm.viridis_r
    best_i = int(np.argmin(costs))
    for k, (cost, v, w, *_ ) in enumerate(cands):
        traj = roll_trajectory(ego.x, ego.y, ego.theta, v, w,
                               cfg["radwa"]["horizon"], cfg["sim"]["dt"])
        tx = [p[1] for p in traj]
        ty = [p[2] for p in traj]
        ax.plot(tx, ty, color=cmap(norm(min(cost, hi))), lw=0.7, alpha=0.7)
    bc = cands[best_i]
    traj = roll_trajectory(ego.x, ego.y, ego.theta, bc[1], bc[2],
                           cfg["radwa"]["horizon"], cfg["sim"]["dt"])
    ax.plot([p[1] for p in traj], [p[2] for p in traj], "r-", lw=2.4,
            label="selected")
    for (ox, oy, orr, _, _) in obs:
        ax.add_patch(plt.Circle((ox, oy), orr, color="0.8", ec="k", lw=0.6))
    ax.plot(ego.x, ego.y, "o", color="cyan", ms=9, mec="k")
    ax.set_title(f"(b) sampled (v,omega) roll-outs by cost  "
                 f"(v*={bc[1]:.1f} m/s, w*={bc[2]:.2f} rad/s)")
    ax.set_xlabel("x [m]"), ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=8)
    sm = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
    fig.colorbar(sm, ax=ax, fraction=0.046, label="cost")
    fig.tight_layout()
    out = os.path.join(ROOT, "figures", "fig2_risk_field.png")
    fig.savefig(out, dpi=170)
    plt.close(fig)
    print("wrote", out)


def fig_schedule(cfg, scenario, seed):
    p = cfg["radwa"]
    th = p["thresholds"]
    w = p["weights"]
    rhos = np.linspace(0.0, 1.0, 400)

    def state_of(rho):
        if rho >= th["theta3"]:
            return "emergency"
        if rho >= th["theta2"]:
            return "conflict"
        if rho >= th["theta1"]:
            return "follow"
        return "free"

    fig, axes = plt.subplots(1, 3, figsize=(14, 3.9))
    # (a) weight schedule
    for key, color in [("progress", "#1f77b4"), ("risk", "#d62728"),
                       ("clear", "#2ca02c")]:
        axes[0].plot(rhos, [w[state_of(r)][key] for r in rhos], color=color,
                     label=f"w_{key}")
    for thr in (th["theta1"], th["theta2"], th["theta3"]):
        axes[0].axvline(thr, color="0.5", ls=":", lw=1)
    axes[0].set_xlabel("risk rho"), axes[0].set_ylabel("weight")
    axes[0].set_title("(a) risk-adaptive weight schedule"), axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=8)

    # (b) admissible-speed modulation
    gain = p["speed_risk_gain"]
    vmax = cfg["ego"]["v_max"]
    vmax_eff = [vmax * (1 - gain * r) if state_of(r) in ("conflict", "emergency")
                else vmax for r in rhos]
    axes[1].plot(rhos, vmax_eff, color="#8c564b")
    axes[1].set_xlabel("risk rho"), axes[1].set_ylabel("$v_{max,eff}$ [m/s]")
    axes[1].set_title("(b) admissible-speed modulation"), axes[1].grid(alpha=0.3)

    # (c) state timeline from a real episode
    res = run_episode(RADWA(cfg), scenario, seed, cfg, record=True)
    log = res["_log"]
    t = log["t"]
    risk = log["risk"]
    states = log["state"]
    for i in range(len(t)):
        axes[2].axvspan(t[i] - 0.05, t[i] + 0.05,
                        color=STATE_COLOR.get(states[i], "white"), lw=0)
    axes[2].plot(t, risk, "k-", lw=1.4)
    axes[2].set_xlabel("t [s]"), axes[2].set_ylabel("rho")
    axes[2].set_title(f"(c) state timeline: {scenario} (seed {seed})")
    axes[2].grid(alpha=0.3)
    handles = [plt.Rectangle((0, 0), 1, 1, color=STATE_COLOR[s]) for s in
               ("free", "follow", "conflict", "emergency")]
    axes[2].legend(handles, ("free", "follow", "conflict", "emergency"),
                   fontsize=7, ncol=2)
    fig.tight_layout()
    out = os.path.join(ROOT, "figures", "fig3_schedule_window_state.png")
    fig.savefig(out, dpi=170)
    plt.close(fig)
    print("wrote", out)


def main():
    cfg = load_config(os.path.join(ROOT, "configs", "experiment.json"))
    fig_risk_field(cfg, "crossing", 4)
    fig_schedule(cfg, "intersection", 1)


if __name__ == "__main__":
    main()
