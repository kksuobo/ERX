"""Baseline: MPPI-lite (sampling-based) local planner.

Stochastic sampling around a nominal (v, omega) with soft-min weighting. Uses a
geometric obstacle cost (no TTC risk field), so it is the "sampling-based
learner-free" reference for RA-DWA.
"""

import math

from ..geometry import angle_diff, clamp, dist
from ..state import Control
from .base import Planner
from .common import obstacle_penalty, roll_trajectory


class MPPI(Planner):
    name = "MPPI"

    def __init__(self, cfg):
        super().__init__(cfg)
        self.p = cfg["mppi"]
        self.ego_cfg = cfg["ego"]
        self.dt = cfg["sim"]["dt"]
        self.r_ego = cfg["ego"]["r"]
        self.rng = None
        self.v0 = 0.0
        self.w0 = 0.0

    def reset(self, goal, rng=None):
        super().reset(goal, rng)
        self.rng = rng
        self.v0 = 0.0
        self.w0 = 0.0

    def plan(self, ego, obstacles, t):
        p = self.p
        e = self.ego_cfg
        gx, gy = self.goal
        bearing = math.atan2(gy - ego.y, gx - ego.x)
        W = p["weights"]

        samples = []
        for _ in range(p["n_samples"]):
            dv = self.rng.gauss(0.0, p["sigma_v"])
            dw = self.rng.gauss(0.0, p["sigma_w"])
            v = clamp(self.v0 + dv, e["v_min"], e["v_max"])
            w = clamp(self.w0 + dw, -e["omega_max"], e["omega_max"])
            traj = roll_trajectory(ego.x, ego.y, ego.theta, v, w,
                                   p["horizon"], self.dt)
            _, ex, ey, eth, _, _ = traj[-1]
            c = (W["progress"] * dist(ex, ey, gx, gy)
                 + W["heading"] * abs(angle_diff(eth, bearing))
                 + W["obstacle"] * obstacle_penalty(traj, obstacles,
                                                    p["obstacle_radius"], self.r_ego)
                 + W["smooth"] * (abs(v - self.v0) + abs(w - self.w0)))
            samples.append((v, w, c))

        cmin = min(s[2] for s in samples)
        temp = p["temperature"]
        weights = [math.exp(-(c - cmin) / temp) for (_, _, c) in samples]
        z = sum(weights) or 1.0
        v_out = sum(wi * s[0] for wi, s in zip(weights, samples)) / z
        w_out = sum(wi * s[1] for wi, s in zip(weights, samples)) / z
        self.v0, self.w0 = v_out, w_out
        return Control(v_out, w_out, state="mppi")
