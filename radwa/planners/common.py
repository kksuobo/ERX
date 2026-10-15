"""Shared roll-out and cost helpers for the sampling planners."""

import math

from ..geometry import unicycle_step
from ..risk import INF, risk_from_clearance, risk_from_ttc, time_to_collision

DEFAULT_SENSOR_RANGE = 30.0


def roll_trajectory(x, y, theta, v, omega, horizon, dt):
    """Forward-simulate a constant (v, omega) command."""
    traj = []
    n = max(1, int(round(horizon / dt)))
    t = 0.0
    for _ in range(n):
        x, y, theta = unicycle_step(x, y, theta, v, omega, dt)
        t += dt
        traj.append((t, x, y, theta, v, omega))
    return traj


def ctr_state(o, omega, tau):
    """Constant-turn-rate prediction of an obstacle ``tau`` seconds ahead."""
    s = math.hypot(o.vx, o.vy)
    if s < 1e-6 or abs(omega) < 1e-4:
        return o.x + o.vx * tau, o.y + o.vy * tau, o.vx, o.vy
    phi = math.atan2(o.vy, o.vx)
    radius = s / omega
    cx = o.x - radius * math.sin(phi)
    cy = o.y + radius * math.cos(phi)
    ang = phi + omega * tau
    return (cx + radius * math.sin(ang), cy - radius * math.cos(ang),
            s * math.cos(ang), s * math.sin(ang))


def min_ttc_at(x, y, vx, vy, obstacles, d_safe_extra, r_ego, sensor_range,
               t_obs=0.0, omegas=None):
    """Minimum TTC from (x, y, vx, vy) to any obstacle.

    ``t_obs`` advances each obstacle along its predicted motion before the TTC
    is evaluated. Prediction is constant-velocity, or constant-turn-rate when
    per-obstacle turn rates are supplied in ``omegas``.
    """
    best = INF
    s2 = sensor_range * sensor_range
    for i, o in enumerate(obstacles):
        if omegas is not None:
            ox, oy, ovx, ovy = ctr_state(o, omegas[i], t_obs)
        else:
            ox = o.x + o.vx * t_obs
            oy = o.y + o.vy * t_obs
            ovx, ovy = o.vx, o.vy
        dx = ox - x
        dy = oy - y
        if dx * dx + dy * dy > s2:
            continue
        ttc = time_to_collision(x, y, vx, vy, ox, oy, ovx, ovy,
                                r_ego + o.r + d_safe_extra)
        if ttc < best:
            best = ttc
    return best


def trajectory_risk(traj, obstacles, T0, d_safe_extra, r_ego, sensor_range,
                    cutoff=float("inf"), omegas=None):
    """Integral of the normalized TTC risk along a rolled-out trajectory."""
    total = 0.0
    prev = 0.0
    for (t, x, y, th, v, w) in traj:
        vx = v * math.cos(th)
        vy = v * math.sin(th)
        ttc = min_ttc_at(x, y, vx, vy, obstacles, d_safe_extra, r_ego,
                         sensor_range, t_obs=t, omegas=omegas)
        total += risk_from_ttc(ttc, T0, cutoff) * (t - prev)
        prev = t
    return total


def trajectory_risk_distance(traj, obstacles, d_risk, r_ego, moving=True,
                             omegas=None):
    """Distance-based risk integral (ablation: TTC risk vs pure distance)."""
    total = 0.0
    prev = 0.0
    for (t, x, y, th, v, w) in traj:
        m = INF
        for i, o in enumerate(obstacles):
            if omegas is not None:
                ox, oy, _, _ = ctr_state(o, omegas[i], t)
            elif moving:
                ox, oy = o.x + o.vx * t, o.y + o.vy * t
            else:
                ox, oy = o.x, o.y
            d = math.hypot(x - ox, y - oy) - o.r - r_ego
            if d < m:
                m = d
        total += risk_from_clearance(m, d_risk) * (t - prev)
        prev = t
    return total


def trajectory_min_clearance(traj, obstacles, r_ego, moving=False, omegas=None):
    m = INF
    for (t, x, y, _, _, _) in traj:
        for i, o in enumerate(obstacles):
            if omegas is not None:
                ox, oy, _, _ = ctr_state(o, omegas[i], t)
            elif moving:
                ox, oy = o.x + o.vx * t, o.y + o.vy * t
            else:
                ox, oy = o.x, o.y
            d = math.hypot(x - ox, y - oy) - o.r - r_ego
            if d < m:
                m = d
    return m


def obstacle_penalty(traj, obstacles, r_pen, r_ego, moving=False):
    """Classic DWA obstacle term: accumulate intrusion below ``r_pen``.

    ``moving`` propagates each obstacle along its measured velocity, giving the
    motion-aware variant used as a control baseline (DWA-P).
    """
    pen = 0.0
    for (t, x, y, _, _, _) in traj:
        for o in obstacles:
            ox = o.x + (o.vx * t if moving else 0.0)
            oy = o.y + (o.vy * t if moving else 0.0)
            d = math.hypot(x - ox, y - oy) - o.r - r_ego
            if d < r_pen:
                pen += (r_pen - d)
    return pen
