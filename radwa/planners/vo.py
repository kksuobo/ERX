"""Baseline: velocity-obstacle (VO) sampling planner.

Samples (v, omega) in the dynamic window, discards candidates whose roll-out
enters the velocity obstacle of any neighbour (i.e. predicted clearance <= 0),
then returns the collision-free candidate closest to the preferred velocity
(towards the goal). If every candidate is forbidden, the one with the largest
predicted clearance is returned.
"""

import math

from ..geometry import angle_diff, clamp, dist, linspace
from ..state import Control
from .base import Planner
from .common import roll_trajectory, trajectory_min_clearance
from .dwa import dynamic_window


class VO(Planner):
    name = "VO"

    def __init__(self, cfg):
        super().__init__(cfg)
        self.p = cfg["vo"]
        self.ego_cfg = cfg["ego"]
        self.dt = cfg["sim"]["dt"]
        self.r_ego = cfg["ego"]["r"]

    def plan(self, ego, obstacles, t):
        p = self.p
        e = self.ego_cfg
        gx, gy = self.goal
        bearing = math.atan2(gy - ego.y, gx - ego.x)

        v_lo, v_hi, w_lo, w_hi = dynamic_window(ego, e, self.dt)
        v_lo = max(v_lo, 0.0)
        vs = linspace(v_lo, v_hi, p["n_v"])
        ws = linspace(w_lo, w_hi, p["n_w"])

        best, best_cost = None, float("inf")
        fallback, fallback_clear = None, -float("inf")
        for v in vs:
            for w in ws:
                traj = roll_trajectory(ego.x, ego.y, ego.theta, v, w,
                                       p["horizon"], self.dt)
                min_clear = trajectory_min_clearance(traj, obstacles, self.r_ego,
                                                     moving=True)
                if min_clear > fallback_clear:
                    fallback_clear = min_clear
                    fallback = Control(v, w, state="vo")
                if min_clear <= p["margin"]:
                    continue
                _, ex, ey, eth, _, _ = traj[-1]
                cost = (p["wa"] * abs(angle_diff(eth, bearing))
                        + p["wp"] * dist(ex, ey, gx, gy)
                        + p["wv"] * abs(v - p["v_ref"]))
                if cost < best_cost:
                    best_cost = cost
                    best = Control(v, w, state="vo")
        return best if best is not None else fallback
